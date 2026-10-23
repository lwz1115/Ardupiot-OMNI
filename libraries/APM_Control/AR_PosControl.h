#pragma once

#include <AP_Common/AP_Common.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <AC_PID/AC_P_2D.h>            // P 库（二维）
#include <AC_PID/AC_PID_2D.h>          // PID 库（二维）

class AR_PosControl {
public:

    // 构造函数
    AR_PosControl(AR_AttitudeControl& atc);

    // 禁止拷贝
    CLASS_NO_COPY(AR_PosControl);

    static AR_PosControl *get_singleton() { return _singleton; }

    // 更新导航
    void update(float dt);

    // 最近是否调用过 update
    bool is_active() const;

    // 设置速度、加速度和加加速度限制
    void set_limits(float speed_max, float accel_max, float lat_accel_max, float jerk_max);

    // 设置器：允许载具代码向本库提供转向相关参数值（应定期更新）
    void set_turn_params(float turn_radius, bool pivot_possible);

    // 设置反向
    void set_reversed(bool reversed) { _reversed = reversed; }

    // 返回 _reversed 的访问器
    bool get_reversed() { return _reversed; }

    // 获取限制
    float get_speed_max() const { return _speed_max; }
    float get_accel_max() const { return _accel_max; }
    float get_lat_accel_max() const { return _lat_accel_max; }
    float get_jerk_max() const { return _jerk_max; }

    // 将位置控制器初始化为当前位置、速度、加速度和姿态
    // 在使用输入整形方法之前应调用本函数
    // 成功返回 true，目标无法初始化时返回 false
    bool init();

    // 使用输入整形平滑调整位置、速度和加速度目标
    // pos 为目标位置，为相对 EKF 原点的偏移（米）
    // vel 为目标速度（m/s）。accel 为目标加速度（m/s/s）
    // dt 为更新周期（秒）
    // 在使用这些方法之前应调用一次 init
    void input_pos_target(const Vector2p &pos, float dt);
    void input_pos_vel_target(const Vector2p &pos, const Vector2f &vel, float dt);
    void input_pos_vel_accel_target(const Vector2p &pos, const Vector2f &vel, const Vector2f &accel, float dt);

    // 设置目标位置、期望速度和加速度。这些应来自外部生成的路径，不经过"输入整形"
    void set_pos_vel_accel_target(const Vector2p &pos, const Vector2f &vel, const Vector2f &accel);

    // 获取前后速度（m/s）、横向速度（m/s）和转向率（rad/s）输出
    float get_desired_speed() const { return _desired_speed; }
    float get_desired_turn_rate_rads() const { return _desired_turn_rate_rads; }
    float get_desired_lat_accel() const { return _desired_lat_accel; }

    // 获取位置目标
    const Vector2p& get_pos_target() const { return _pos_target; }

    // 返回东北坐标系下的期望速度向量（即前馈），单位 m/s
    Vector2f get_desired_velocity() const;

    // 返回东北坐标系下的速度目标（m/s）。该目标包含位置
    // P 控制器对位置误差的修正，叠加在 S 曲线的
    // 前馈速度之上，即"瞄准终点、修正横向误差"
    // 的速度。用于直接在世界坐标系中行驶的全向载具。
    Vector2f get_vel_target() const { return _vel_target; }

    // 返回东北坐标系下速度控制器的输出（m/s/s），不含
    // 独立的 S 曲线加速度前馈。结果包含速度
    // PID 的 P/I/D/FF 项，适合作为全向驱动混控的
    // 有界反馈修正。
    Vector2f get_accel_feedback() const {
        return _accel_target - (_accel_desired_valid ? _accel_desired : Vector2f{});
    }

    // 返回东北坐标系下的期望加速度向量（m/s）
    Vector2f get_desired_accel() const;

    /// 获取位置误差，即从当前位置指向目标位置的向量
    Vector2p get_pos_error() const;

    // 获取 PID 控制器
    AC_P_2D& get_pos_p() { return _p_pos; }
    AC_PID_2D& get_vel_pid() { return _pid_vel; }

    // 写入 PSC 日志
    void write_log();

    // 参数变量表
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AR_PosControl *_singleton;

    // 初始化并检查 EKF 位置重置
    void init_ekf_xy_reset();
    void handle_ekf_xy_reset();

    // 引用
    AR_AttitudeControl &_atc;       // 地面车姿态控制库

    // 参数
    AC_P_2D   _p_pos;               // 位置 P 控制器，将距离误差转换为期望速度
    AC_PID_2D _pid_vel;             // 速度 PID 控制器，将速度误差转换为期望加速度

    // 限制
    float _speed_max;               // 最大前进速度（m/s）
    float _accel_max;               // 最大前进/后退加速度（m/s/s）
    float _lat_accel_max;           // 最大横向加速度（m/s/s）
    float _jerk_max;                // 最大加加速度（m/s/s/s）（用于前后向和横向输入整形）
    float _turn_radius;             // 车辆转弯半径（米）

    // 位置和速度目标
    Vector2p _pos_target;           // 位置目标，为相对 EKF 原点的偏移（米）
    Vector2f _vel_desired;          // 东北坐标系下的期望速度（m/s）。为 S 曲线提供的前馈
    Vector2f _vel_target;           // 东北坐标系下的速度目标（m/s）
    Vector2f _accel_desired;        // 东北坐标系下的期望加速度（m/s/s）。为 S 曲线提供的前馈
    Vector2f _accel_target;         // 东北坐标系下的加速度目标（m/s/s）
    bool _pos_target_valid;         // _pos_target 有效则为 true
    bool _vel_desired_valid;        // _vel_desired 有效则为 true
    bool _accel_desired_valid;      // _accel_desired 有效则为 true

    // 导航用变量
    uint32_t _last_update_ms;       // 上次调用 update 的系统时间
    bool _reversed;                 // 车辆应倒车驶向目标则为 true

    // 主要输出
    float _desired_speed;           // 期望前后速度（m/s）
    float _desired_turn_rate_rads;  // 期望转向率（rad/s）（逆时针为负，顺时针为正）
    float _desired_lat_accel;       // 期望横向加速度（仅供上报）

    // EKF 重置处理
    uint16_t _ekf_xy_reset_count;      // EKF XY 位置重置次数
};
