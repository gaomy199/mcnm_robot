/**
 *********************************************************
 * ESP32-WROOM-32E 麦轮micro-ROS控制程序
 *********************************************************
 * 实现过程：
 *   1. micro-ROS 通过 WiFi 连接 Agent
 *   2. 订阅cmd_vel
 *   3. 麦轮逆运动学vx, vy, wz 转换成4个电机转速指令
 *   4. 通过 I2C 向驱动板写入转速指令
 *
 * 底盘电机位置（俯视，车头朝前）：
 *       M1(左前)    M3(右前)
 *       M2(左后)    M4(右后)
 *********************************************************
 * 编码器读取的每10ms脉冲数与指令值一致。最大指令50下转速为75.8RPM
 * 其余的值按照线性映射就行
 *********************************************************
 */

#include <Arduino.h>                 // Arduino标准头文件
#include <Wire.h>                    // I2C 头文件
#include <WiFi.h>                    // WiFi 头文件
#include <math.h>
#include <micro_ros_platformio.h>    // micro-ROS 头文件
#include <rcl/rcl.h>                 // ROS2 头文件
#include <rclc/rclc.h>               // ROS2 控制器头文件
#include <rclc/executor.h>           // ROS2 执行器头文件
#include <geometry_msgs/msg/twist.h> // geometry_msgs/msg/Twist头文件
#include <Kinematics.h>              // 麦轮运动学库
#include <nav_msgs/msg/odometry.h>   // nav_msgs/msg/odometry头文件
#include <micro_ros_utilities/string_utilities.h> // 用于时间同步
#include <freertos/semphr.h>         // loop() 和 odom_timer_callback 
                                     // 同时访问 Wire 总线造成里程计跳变，因此加I2C互锁
static SemaphoreHandle_t i2c_mutex = NULL;

// WiFi 和 Agent 配置
#define WIFI_SSID "Xiaomi50"      // WiFi 名称
#define WIFI_PASSWORD "88888888"  // WiFi 密码
#define AGENT_IP "192.168.31.139" // Agent IP，即电脑的ipv4地址
#define AGENT_PORT 8888           // Agent 端口

// I2C 配置
/* 如果采用其他esp32引脚，因为芯片自带上拉电阻是45k欧姆，不建议使用，建议外加上拉电阻4.7k欧姆
   上拉电阻即在引脚上引出一根电线，经过4.7k电阻后接入3.33V，则引脚为3.3V，实现上拉效果。*/
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19
#define I2C_ADDR 0x34 // 驱动板I2C地址

// 驱动板寄存器地址
#define MOTOR_TYPE_ADDR 20             // 电机地址
#define MOTOR_ENCODER_POLARITY_ADDR 21 // 电机编码器极性
#define MOTOR_FIXED_SPEED_ADDR 51      // 闭环控制
#define MOTOR_TYPE_JGB 3               // 电机类型JBG
#define MOTOR_ENCODER_TOTAL_ADDR 60    // 编码器累计脉冲寄存器

// 底盘物理参数，单位米
#define WHEEL_DIAMETER 0.075f                                    // 轮子直径
#define LENGTH_HALF 0.12f                                        // 底盘A轴长度，半个轴距
#define WIDTH_HALF 0.135f                                        // 底盘B轴长度，半个轮距
#define ENCODER_LINES 11                                         // 磁环个数
#define GEAR_RATIO 90                                            // 减速比
#define QUADRATURE 4                                             // AB双相四倍频
#define PULSES_PER_REV (ENCODER_LINES * QUADRATURE * GEAR_RATIO) // 轮圈编码器脉冲数3960pulse/r

// 控制参数
#define CONTROL_PERIOD_MS 50
#define CMD_TIMEOUT_MS 500

// 全局对象

// 运动学实例（构造时传入底盘参数）
Kinematics kinematics(WHEEL_DIAMETER, LENGTH_HALF, WIDTH_HALF, PULSES_PER_REV);

// I2C 通信函数
bool WireWriteByte(uint8_t val)
{
    // 不加锁，只是内部辅助函数
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}
 
bool WireWriteDataArray(uint8_t reg, uint8_t *val, unsigned int len)
{
    if (i2c_mutex) xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    for (unsigned int i = 0; i < len; i++) {
        Wire.write(val[i]);
    }
    bool ret = (Wire.endTransmission() == 0);
    if (i2c_mutex) xSemaphoreGive(i2c_mutex);
    return ret;
}
 
int WireReadDataArray(uint8_t reg, uint8_t *val, unsigned int len)
{
    if (i2c_mutex) xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    unsigned char i = 0;
    if (!WireWriteByte(reg)) {
        if (i2c_mutex) xSemaphoreGive(i2c_mutex);
        return -1;
    }
    Wire.requestFrom(I2C_ADDR, (int)len);
    while (Wire.available())
    {
        if (i >= len) {
            if (i2c_mutex) xSemaphoreGive(i2c_mutex);
            return -1;
        }
        val[i] = Wire.read();
        i++;
    }
    if (i2c_mutex) xSemaphoreGive(i2c_mutex);
    return i;
}

// 编码器读取
/**
 * 从寄存器地址 60 读取4个电机的累计脉冲值
 * 每个电机 4 字节（int32，小端序），共16字节
 * 本次把位运算改成了先转 uint32_t 再位移，
 * 避免 uint8_t << 24 在某些情况下发生符号扩展的未定义行为
 */
bool readEncoders(int32_t encoder_vals[4])
{
    uint8_t buf[16] = {0};  // 显式初始化
    int read_len = WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, buf, 16);
    if (read_len != 16) {
        return false;
    }
    for (int i = 0; i < 4; i++)
    {
        encoder_vals[i] = (int32_t)((uint32_t)buf[i * 4]
                        | ((uint32_t)buf[i * 4 + 1] << 8)
                        | ((uint32_t)buf[i * 4 + 2] << 16)
                        | ((uint32_t)buf[i * 4 + 3] << 24));
    }
    return true;
}

// micro-ROS 全局对象
rcl_allocator_t allocator;             // 创建分配器
rclc_support_t support;                // 创建支持
rclc_executor_t executor;              // 创建执行器
rcl_node_t node;                       // 创建节点
// 订阅 /cmd_vel
rcl_subscription_t cmd_vel_sub;        // 创建订阅
geometry_msgs__msg__Twist cmd_vel_msg; // 创建cmd_vel消息
// 发布 /odom
rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;
// 定时器（定期发布里程计）
rcl_timer_t odom_timer;

volatile float cmd_vx = 0.0f;
volatile float cmd_vy = 0.0f;
volatile float cmd_wz = 0.0f;
unsigned long last_cmd_time = 0;

// 回调函数cmd_vel_callback
void cmd_vel_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    cmd_vx = msg->linear.x;
    cmd_vy = msg->linear.y;
    cmd_wz = msg->angular.z;
    last_cmd_time = millis();
}

/**
 * 欧拉角(yaw)转换为四元数
 */
void euler_to_quat(float theta, double &qx, double &qy, double &qz, double &qw)
{
    qx = 0.0;
    qy = 0.0;
    qz = sin(theta / 2.0);
    qw = cos(theta / 2.0);
}
 
/**
 * 里程计定时器回调，读取编码器数据，根据编码器的变动差来更新里程计，并发布 /odom
 */
void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    int32_t encoder_now[4];
    if (!readEncoders(encoder_now)) return;
    /*
    Serial.printf("enc:[%ld,%ld,%ld,%ld]\n",
    encoder_now[0], encoder_now[1], encoder_now[2], encoder_now[3]); //此段输出用于实时监测底盘的编码器波动，检测信号干扰情况
    */
    kinematics.odom_update(encoder_now, millis());

    Odom_t odom = kinematics.odom_get();

    // 用同步后的时间戳
    int64_t stamp = rmw_uros_epoch_millis();
    odom_msg.header.stamp.sec = (int32_t)(stamp / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1e6);

    odom_msg.pose.pose.position.x = odom.x;
    odom_msg.pose.pose.position.y = odom.y;
    odom_msg.pose.pose.position.z = 0.0;

    double qx, qy, qz, qw;
    euler_to_quat(odom.theta, qx, qy, qz, qw);
    odom_msg.pose.pose.orientation.x = qx;
    odom_msg.pose.pose.orientation.y = qy;
    odom_msg.pose.pose.orientation.z = qz;
    odom_msg.pose.pose.orientation.w = qw;

    odom_msg.twist.twist.linear.x = odom.vx;
    odom_msg.twist.twist.linear.y = odom.vy;
    odom_msg.twist.twist.angular.z = odom.wz;

    rcl_publish(&odom_pub, &odom_msg, NULL);
}

// micro-ROS 任务
void micro_ros_task(void *parameter)
{
    IPAddress agent_ip;
    agent_ip.fromString(AGENT_IP);
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, agent_ip, AGENT_PORT);
    delay(2000);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "mecanum_base_controller", "", &support);

    // 创建 /cmd_vel 订阅
    rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    unsigned int num_handles = 0 + 2;
    rclc_executor_init(&executor, &support.context, num_handles, &allocator);
    odom_msg.header.frame_id = 
        micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
    odom_msg.child_frame_id = 
        micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_footprint");
    // 创建 /odom 发布者（用 best_effort，不用 reliable）
    rclc_publisher_init_best_effort(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),"odom");

    // 时间同步，在创建定时器之前完成
    while (!rmw_uros_epoch_synchronized())
    {
        rmw_uros_sync_session(1000);
        delay(10);
    }
    // 预热 I2C，丢弃前几次读取结果
    int32_t dummy[4];
    for (int i = 0; i < 10; i++) {
    readEncoders(dummy);
    delay(20);}
    // 创建里程计定时器
    rclc_timer_init_default(
        &odom_timer, &support, RCL_MS_TO_NS(CONTROL_PERIOD_MS), odom_timer_callback);

    // 初始化执行器：1个订阅 + 1个定时器 = 2个 handle
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                   &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &odom_timer);

    rclc_executor_spin(&executor);
}

// setup部分
void setup()
{
    Serial.begin(115200);
    Serial.println("Mecanum Base Controller Starting...");

    // 加上I2C互锁，减少编码器数据的跳变，具体原因看头文件
    i2c_mutex = xSemaphoreCreateMutex();
    // I2C 初始化
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    delay(200);

    // 驱动板初始化
    uint8_t motor_type = MOTOR_TYPE_JGB;
    uint8_t encoder_polarity = 0;
    WireWriteDataArray(MOTOR_TYPE_ADDR, &motor_type, 1);
    delay(5);
    WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &encoder_polarity, 1);
    delay(2000);
    // 驱动板初始化之后，对编码器数据寄存器数据清零
    uint8_t zero_buf[16] = {0};
    WireWriteDataArray(MOTOR_ENCODER_TOTAL_ADDR, zero_buf, 16);
    delay(10);
    Serial.println("Motor driver initialized.");

    // 启动 micro-ROS 任务
    xTaskCreate(micro_ros_task, "micro_ros", 10240, NULL, 1, NULL);
}

// loop 部分
void loop()
{
    delay(CONTROL_PERIOD_MS);

    // 超时停车
    float vx, vy, wz;
    if (millis() - last_cmd_time > CMD_TIMEOUT_MS)  //每次指令只执行500ms
    {
        vx = vy = wz = 0.0f;
    }
    else
    {
        vx = cmd_vx;
        vy = cmd_vy;
        wz = cmd_wz;
    }

    // 调用运动学库做逆解
    int8_t motor_cmd[4];
    kinematics.kinematics_inverse(vx, vy, wz, motor_cmd);

    // I2C 发送给驱动板
    WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t *)motor_cmd, 4);

    // 用于调试包含里程计信息的输出
    Odom_t odom = kinematics.odom_get();
    
    /*Serial.printf("cmd:[%d,%d,%d,%d] odom:(%.3f, %.3f, %.1f deg)\n",
                  motor_cmd[0], motor_cmd[1], motor_cmd[2], motor_cmd[3],
                  odom.x, odom.y, odom.theta * 180.0f / M_PI);
    */
}