#pragma once

#include <AP_Common/AP_Common.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

class AR_AttitudeControl {
public:

    // 构造函数
    AR_AttitudeControl();

    // 禁止拷贝
    CLASS_NO_COPY(AR_AttitudeControl);

    static AR_AttitudeControl *get_singleton() { return _singleton; }

    //
    // 转向控制器
    //

    // 根据期望横向加速度（m/s/s）返回转向舵机输出。
    // 正的横向加速度为向右。dt 通常应为主循环周期。
    // 返回值通常在 -1.0 到 +1.0 之间，但可能更高或更低
    float get_steering_out_lat_accel(float desired_accel, bool motor_limit_left, bool motor_limit_right, float dt);

    // 根据航向（弧度）返回转向舵机输出
    // 将 rate_max_rads 设置为非零值以限制期望转向率
    // 返回值通常在 -1.0 到 +1.0 之间，但可能更高或更低
    float get_steering_out_heading(float heading_rad, float rate_max_rads, bool motor_limit_left, bool motor_limit_right, float dt);

    // 根据期望航向（弧度）返回期望转向率
    // 结果通常随后传入 get_steering_out_rate
    float get_turn_rate_from_heading(float heading_rad, float rate_max_rads) const;

    // 根据期望偏航角速度（弧度/秒）返回转向舵机输出。
    // 正向偏航为向右
    // 返回值通常在 -1.0 到 +1.0 之间，但可能更高或更低
    // 同时设置 steering_limit_left 和 steering_limit_right 标志
    float get_steering_out_rate(float desired_rate, bool motor_limit_left, bool motor_limit_right, float dt);

    // 获取调用 get_steering_out_rate 期间记录的最新期望转向率（rad/s）。仅供上报
    float get_desired_turn_rate() const;

    // 获取调用 get_steering_out_lat_accel 期间记录的最新期望横向加速度（m/s/s）。仅供上报
    float get_desired_lat_accel() const;

    // 获取实际横向加速度（m/s/s）。成功返回 true。仅供上报
    bool get_lat_accel(float &lat_accel) const;

    // 根据横向加速度（m/s/s）和速度（m/s）计算转向率（rad/s）
    float get_turn_rate_from_lat_accel(float lat_accel, float speed) const;

    // 获取横向加速度限制（m/s/s）。返回至少 0.1G 或约 1 m/s/s
    float get_turn_lat_accel_max() const { return MAX(_turn_lateral_G_max, 0.1f) * GRAVITY_MSS; }

    // 若转向已被限制则返回 true，可能由物理转向面
    // 到达其物理极限（即电机极限）或应用了加速度/转向率限制引起
    bool steering_limit_left() const { return _steering_limit_left; }
    bool steering_limit_right() const { return _steering_limit_right; }

    //
    // 油门 / 速度控制器
    //

    // 设置油门控制器使用的限制
    //   前后向最大加速度（m/s/s）
    //   前后向最大减速度（m/s/s）
    void set_throttle_limits(float throttle_accel_max, float throttle_decel_max);

    // 根据期望速度（m/s）返回 -1 到 +1 的油门输出（负速度表示倒车）
    //   desired_speed 参数应已通过 get_desired_speed_accel_limited 函数
    //   若电机已达上/下限，motor_limit 应为 true
    //   巡航速度单位为 m/s，巡航油门为 -1 到 +1 的数字
    float get_throttle_out_speed(float desired_speed, bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt);

    // 返回 -1 到 +1 的油门输出以执行受控停车。停车完成后 stopped 被设置为 true
    float get_throttle_out_stop(bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt, bool &stopped);

    // 平衡机器人俯仰到油门控制器
    // 根据期望俯仰角（弧度）返回 -1 到 +1 的油门输出
    // pitch_max 应为用户定义的最大俯仰角（弧度）
    // 若电机已达上/下限，motor_limit 应为 true
    float get_throttle_out_from_pitch(float desired_pitch, float pitch_max, bool motor_limit, float dt);

    // 若俯仰角已被限制以防止翻倒则返回 true
    // 俯仰限制保护在 get_throttle_out_from_pitch 内部实现
    bool pitch_limited() const { return _pitch_limited; }

    // 获取最新期望俯仰角（弧度），仅供上报
    float get_desired_pitch() const;

    // 帆船横倾（滚转）角控制器，释放帆以保持最大横倾角
    float get_sail_out_from_heel(float desired_heel, float dt);

    // 低层控制访问器，用于上报和日志
    AC_P& get_steering_angle_p() { return _steer_angle_p; }
    AC_PID& get_steering_rate_pid() { return _steer_rate_pid; }
    AC_PID& get_pitch_to_throttle_pid() { return _pitch_to_throttle_pid; }
    AC_PID& get_sailboat_heel_pid() { return _sailboat_heel_pid; }
    const AP_PIDInfo& get_throttle_speed_pid_info() const { return _throttle_speed_pid_info; }

    // 设置 PID 陷波采样率
    void set_notch_sample_rate(float sample_rate);

    // 获取速度和转向的转换速率值，供 lua 脚本进行振荡检测
    void get_srate(float &steering_srate, float &speed_srate);

    // 获取前进速度（m/s）（大地坐标系水平速度，但仅沿车辆 x 轴）。成功返回 true
    bool get_forward_speed(float &speed) const;

    // 获取油门/速度控制器最大加速度（也用于减速）
    float get_accel_max() const { return MAX(_throttle_accel_max, 0.0f); }

    // 获取油门/速度控制器最大减速度
    float get_decel_max() const;

    // 检查速度控制器是否激活
    bool speed_control_active() const;

    // 获取调用 get_throttle_out_speed 期间记录的最新期望速度。仅供上报
    float get_desired_speed() const;

    // 获取受加速度限制的期望速度
    float get_desired_speed_accel_limited(float desired_speed, float dt) const;

    // 根据速度（m/s）获取最小停车距离（米）
    float get_stopping_distance(float speed) const;

    // 获取车辆被认为已停止的速度下限（m/s）
    float get_stop_speed() const { return MAX(_stop_speed, 0.0f); }

    // 放松油门和转向控制器的 I 项
    void relax_I();

    // 参数变量表
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AR_AttitudeControl *_singleton;

    // 参数
    AC_P     _steer_angle_p;        // 转向角控制器
    AC_PID   _steer_rate_pid;       // 转向率控制器
    AC_PID   _throttle_speed_pid;   // 油门速度控制器
    AC_PID   _pitch_to_throttle_pid;// 平衡机器人俯仰控制器
    AP_Float _pitch_to_throttle_ff; // 平衡机器人由当前俯仰角的前馈
    AP_Float _pitch_limit_tc;       // 平衡机器人俯仰限制保护时间常数
    AP_Float _pitch_limit_throttle_thresh;  // 平衡机器人俯仰限制油门阈值（范围 0 到 1.0）

    AP_Float _throttle_accel_max;   // 速度/油门控制最大加速度（和减速度）（m/s/s）。0 表示禁用限制
    AP_Float _throttle_decel_max;    // 速度/油门控制最大减速度（m/s/s）。0 表示减速使用 ATC_ACCEL_MAX
    AP_Int8  _brake_enable;         // 速度控制刹车使能/禁用。若设为 1，向电机输出反向信号以减慢车辆
    AP_Float _stop_speed;           // 速度控制停车速度。车辆速度低于此值后电机输出归零
    AP_Float _steer_accel_max;      // 转向角最大加速度（deg/s/s）
    AP_Float _steer_decel_max;      // 转向角最大减速度（deg/s/s）
    AP_Float _steer_rate_max;       // 转向率控制最大速率（deg/s）
    AP_Float _turn_lateral_G_max;   // 转向最大横向加速度限制（'G'）

    // 转向控制
    uint32_t _steer_lat_accel_last_ms;  // 上次调用横向加速度控制器（即 get_steering_out_lat_accel）的系统时间
    uint32_t _steer_turn_last_ms;   // 上次调用转向率控制器的系统时间
    float    _desired_lat_accel;    // 最近一次调用 get_steering_out_lat_accel 得到的期望横向加速度（m/s/s）（仅供上报）
    float    _desired_turn_rate;    // 期望转向率（弧度/秒），来自外部调用者或横向加速度控制器
    bool     _steering_limit_left;  // 转向控制到达左极限时为 true（例如电机到达极限或应用了加速度/转向率限制）
    bool     _steering_limit_right; // 转向控制到达右极限时为 true（例如电机到达极限或应用了加速度/转向率限制）

    // 油门控制
    uint32_t _speed_last_ms;        // 上次调用 get_throttle_out_speed 的系统时间
    float    _desired_speed;        // 最后记录的期望速度
    uint32_t _stop_last_ms;         // 车辆完全停止时的系统时间
    bool     _throttle_limit_low;   // 油门输出被限制过低（用于减少 I 项累积）
    bool     _throttle_limit_high;  // 油门输出被限制过高（用于减少 I 项累积）
    AP_PIDInfo _throttle_speed_pid_info;   // throttle_speed 控制器 PID 信息的本地副本，用于上报异常 FF

    // 平衡机器人俯仰控制
    uint32_t _balance_last_ms = 0;  // 上次调用 get_throttle_out_from_pitch 的系统时间
    float _pitch_limit_low = 0;     // 最小期望俯仰角（弧度），用于防止翻倒
    float _pitch_limit_high = 0;    // 最大期望俯仰角（弧度），用于防止翻倒
    bool _pitch_limited = false;    // 上次调用 get_throttle_out_from_pitch 时俯仰被限制则为 true

    // 帆船横倾控制
    AC_PID   _sailboat_heel_pid;    // 帆船横倾角 PID 控制器
    uint32_t _heel_controller_last_ms = 0;
};
