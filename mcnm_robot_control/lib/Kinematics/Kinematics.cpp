#include "Kinematics.h"
#include <math.h>

Kinematics::Kinematics(float wheel_diameter, float length_half, float width_half,
                       int pulses_per_rev)
    : wheel_diameter_(wheel_diameter)
    , chassis_lw_(length_half + width_half)
    , pulses_per_rev_(pulses_per_rev)
    , pulse_to_meter_(M_PI * wheel_diameter / (float)(pulses_per_rev))
    , mps_to_cmd_((float)(pulses_per_rev) / (M_PI * wheel_diameter * 100.0f))
      // 这里mps_to_cmd_是个转换系数，将每个轮子的转动角速度转换为控制指令[-50, 50]
    , odom_last_time_(0)   //上次调用里程计的时间
    , odom_first_update_(true) //标记第一次调用里程计
{
    // 初始化上一次脉冲数组为0
    for (int i = 0; i < 4; i++){
        odom_last_pulse_[i] = 0;
    }
    odom_.x = 0.0f;
    odom_.y = 0.0f;
    odom_.theta = 0.0f;
    odom_.vx = 0.0f;
    odom_.vy = 0.0f;
    odom_.wz = 0.0f;
}

void Kinematics::kinematics_inverse(float vx, float vy, float wz, int8_t out[4])
{
    // 麦轮逆运动学公式，M2/M3 取反匹配电机安装方向
    float wheel[4];
    wheel[0] =  (vx - vy - chassis_lw_ * wz) * mps_to_cmd_;  // M1 左前
    wheel[1] = -(vx + vy - chassis_lw_ * wz) * mps_to_cmd_;  // M2 左后
    wheel[2] = -(vx + vy + chassis_lw_ * wz) * mps_to_cmd_;  // M3 右前
    wheel[3] =  (vx - vy + chassis_lw_ * wz) * mps_to_cmd_;  // M4 右后

    // 限幅 [-50, 50] 并转为 int8_t
    for (int i = 0; i < 4; i++)
    {
        if (wheel[i] > 30.0f)
            wheel[i] = 30.0f;
        if (wheel[i] < -30.0f)
            wheel[i] = -30.0f;
        out[i] = (int8_t)wheel[i];
    }
}

void Kinematics::odom_update(int32_t current_pulse[4], uint64_t current_time)
{
    // 里程计更新
    // 首次调用只记录初始值，不计算
    if (odom_first_update_)
    {
        for (int i = 0; i < 4; i++)
            odom_last_pulse_[i] = current_pulse[i];
        odom_last_time_ = current_time;
        odom_first_update_ = false;
        return;
    }
    // 计算时间间隔
    float dt = (float)(current_time - odom_last_time_) / 1000.0f;
    if (dt <= 0.0f) return;

    // 计算脉冲差
    int32_t delta_pulse[4];
    bool jump_detected = false;
    for (int i = 0; i < 4; i++) {
        delta_pulse[i] = current_pulse[i] - odom_last_pulse_[i];
        // 最大速度 30 pulse/10ms = 150 pulse/50ms
        // 设阈值 500，有 3 倍余量，足够过滤几十万的幽灵值
        if (delta_pulse[i] > 500 || delta_pulse[i] < -500) {
            jump_detected = true;
        }
    }
    
    if (jump_detected) {
        // 丢弃本次异常值，但不更新基准（让下一次正常读取重新同步）
        Serial.printf(">>> JUMP FILTERED @%lu ms: dp=[%ld,%ld,%ld,%ld]\n",
        (unsigned long)millis(),
        delta_pulse[0], delta_pulse[1], delta_pulse[2], delta_pulse[3]);
        // 注意：不更新 odom_last_pulse_！保留上次正常基准
        // 这样下一帧如果恢复正常，delta 会是 正常增量 + 一帧的小累积，依然合理
        return;
    }
    // 更新基准
    for (int i = 0; i < 4; i++)
        odom_last_pulse_[i] = current_pulse[i];
    odom_last_time_ = current_time;

    // 原有的累积计算
    float d[4];
    d[0] =  (float)delta_pulse[0] * pulse_to_meter_;
    d[1] = -(float)delta_pulse[1] * pulse_to_meter_;
    d[2] = -(float)delta_pulse[2] * pulse_to_meter_;
    d[3] =  (float)delta_pulse[3] * pulse_to_meter_;
    
    float dx  = ( d[0] + d[1] + d[2] + d[3]) / 4.0f;
    float dy  = (-d[0] + d[1] + d[2] - d[3]) / 4.0f;
    float dth = (-d[0] - d[1] + d[2] + d[3]) / (4.0f * chassis_lw_);
    
    odom_.x += dx * cos(odom_.theta) - dy * sin(odom_.theta);
    odom_.y += dx * sin(odom_.theta) + dy * cos(odom_.theta);
    odom_.theta += dth;
    
    odom_.vx = dx / dt;
    odom_.vy = dy / dt;
    odom_.wz = dth / dt;
}

// 里程计获取和重置
Odom_t Kinematics::odom_get() const
{
    return odom_;
}
 
void Kinematics::odom_reset()
{
    odom_.x = 0.0f;
    odom_.y = 0.0f;
    odom_.theta = 0.0f;
    odom_.vx = 0.0f;
    odom_.vy = 0.0f;
    odom_.wz = 0.0f;
}
float Kinematics::getPulseToMeter() const
{
    return pulse_to_meter_;
}