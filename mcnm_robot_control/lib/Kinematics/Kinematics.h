#ifndef KINEMATICS_H // 头文件保护符，防止KINEMATICS_H头文件被重复包含，必须加
#define KINEMATICS_H // 头文件定义
#include <Arduino.h>  //也可以使用#include <stdint.h>但是Arduino.h包含了stdint.h，并且还有许多其他的库文件

/**
 * 定义一个odom结构体用来存储里程计信息
 */
struct Odom_t
{
    float x;        // 世界坐标系 x 位置（米）
    float y;        // 世界坐标系 y 位置（米）
    float theta;    // 航向角（弧度）
    float vx;       // 车体坐标系 前后速度（米/秒）
    float vy;       // 车体坐标系 左右速度（米/秒）
    float wz;       // 角速度（弧度/秒）
};

/**
*下面的代码定义了一个麦克纳姆轮运动学类
   底盘电机位置示意图：
   M1(左前)    M3(右前)
   M2(左后)    M4(右后)

*由于电机控制板自身的设置问题，以下是俯视视角下底盘的电机在给予正代码数值时的前进情况
   M1  M2  M3  M4 = 正  负  负  正
   逆运动学和正运动学中已做取反处理。
*/
class Kinematics
{
public:
    /**
     构造函数
     * @param wheel_diameter   麦轮直径，单位：m
     * @param length_half      底盘中心到前/后轮轴的纵向半距，单位：m
     * @param width_half       底盘中心到左/右轮的横向半距，单位：m
     * @param pulse_per_rev    每圈脉冲数
     */
    Kinematics(float wheel_diameter, float length_half, float width_half,
               int pulses_per_rev);

    /**
     * 逆运动学公式，由底盘速度来推算出所需要的4个电机驱动板转速指令
     * @param vx    前后线速度(m/s)
     * @param vy    左右线速度(m/s)
     * @param wz    角速度(rad/s)，俯视视角下正值逆时针
     * @param out   输出4个电机指令值 [M1, M2, M3, M4]，范围 [-50, 50]
     */
    void kinematics_inverse(float vx, float vy, float wz, int8_t out[4]);

    /**
     * 获取每脉冲对应的行进距离，外部计算里程计时需要用到
     */
    float getPulseToMeter() const;

    /**
     * 里程计更新函数
     * 在函数内部完成里程计工作过程 脉冲差 -> 轮子位移 -> 正运动学 -> 累计位姿
     * 直接从脉冲差计算位移，不依赖时间间隔，精度更高
     * @param current_pulse  4个电机当前累计脉冲
     * @param current_time   当前时间 (ms)，用于计算速度
     */
    void odom_update(int32_t current_pulse[4], uint64_t current_time);
 
    /** 获取里程计数据 */
    Odom_t odom_get() const;
 
    /** 重置里程计 */
    void odom_reset();

private:
    float wheel_diameter_;      // 轮子直径
    float chassis_lw_;          // wheelbase_half + track_half
    int pulses_per_rev_;        // 每圈总脉冲数
    float pulse_to_meter_;      // 每脉冲对应米数
    float mps_to_cmd_;          // m/s -> 驱动板指令值的换算系数
    /*odom_update使用的内部变量,避免与update_motor_speed干扰*/
    uint64_t odom_last_time_;
    int32_t odom_last_pulse_[4];
    bool odom_first_update_;
    Odom_t odom_;
};

#endif // KINEMATICS_H