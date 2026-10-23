/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AR_PosControl.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AC_Avoidance/AC_Avoid.h>
#include <AC_AttitudeControl/AC_PosControl.h>

#define AR_POSCON_TIMEOUT_MS            100     // 0.1 秒后超时
#define AR_POSCON_POS_P                 0.2f    // 默认位置 P 增益
#define AR_POSCON_VEL_P                 1.0f    // 默认速度 P 增益
#define AR_POSCON_VEL_I                 0.0f    // 默认速度 I 增益
#define AR_POSCON_VEL_D                 0.0f    // 默认速度 D 增益
#define AR_POSCON_VEL_FF                0.0f    // 默认速度 FF 增益
#define AR_POSCON_VEL_IMAX              1.0f    // 默认速度积分上限
#define AR_POSCON_VEL_FILT              5.0f    // 默认速度滤波器
#define AR_POSCON_VEL_FILT_D            5.0f    // 默认速度 D 项滤波器
#define AR_POSCON_DT                    0.02f   // PID 控制器默认周期

extern const AP_HAL::HAL& hal;

AR_PosControl *AR_PosControl::_singleton;

const AP_Param::GroupInfo AR_PosControl::var_info[] = {

    // @Param: _POS_P
    // @DisplayName: 位置控制器 P 增益
    // @Description: 位置控制器 P 增益。将到目标位置的距离转换为期望速度，然后传递给悬停横向速率控制器
    // @Range: 0.500 2.000
    // @User: 标准
    AP_SUBGROUPINFO(_p_pos, "_POS_", 1, AR_PosControl, AC_P_2D),

    // @Param: _VEL_P
    // @DisplayName: 速度（水平）P 增益
    // @Description: 速度（水平）P 增益。将期望速度与实际速度之差转换为目标加速度
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: 高级

    // @Param: _VEL_I
    // @DisplayName: 速度（水平）I 增益
    // @Description: 速度（水平）I 增益。修正期望与实际速度之间的长期差异，转换为目标加速度
    // @Range: 0.00 1.00
    // @Increment: 0.01
    // @User: 高级

    // @Param: _VEL_D
    // @DisplayName: 速度（水平）D 增益
    // @Description: 速度（水平）D 增益。修正速度的短期变化
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: 高级

    // @Param: _VEL_IMAX
    // @DisplayName: 速度（水平）积分器上限
    // @Description: 速度（水平）积分器上限。限制 I 增益输出的目标加速度
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: 高级

    // @Param: _VEL_FLTE
    // @DisplayName: 速度（水平）输入滤波器
    // @Description: 速度（水平）输入滤波器。此滤波器（单位 Hz）应用于 P 项和 I 项的输入
    // @Range: 0 100
    // @Units: Hz
    // @User: 高级

    // @Param: _VEL_FLTD
    // @DisplayName: 速度（水平）输入滤波器
    // @Description: 速度（水平）输入滤波器。此滤波器（单位 Hz）应用于 D 项的输入
    // @Range: 0 100
    // @Units: Hz
    // @User: 高级

    // @Param: _VEL_FF
    // @DisplayName: 速度（水平）前馈增益
    // @Description: 速度（水平）前馈增益。将期望速度的差值转换为目标加速度
    // @Range: 0 6
    // @Increment: 0.01
    // @User: 高级
    AP_SUBGROUPINFO(_pid_vel, "_VEL_", 2, AR_PosControl, AC_PID_2D),

    AP_GROUPEND
};

AR_PosControl::AR_PosControl(AR_AttitudeControl& atc) :
    _atc(atc),
    _p_pos(AR_POSCON_POS_P),
    _pid_vel(AR_POSCON_VEL_P, AR_POSCON_VEL_I, AR_POSCON_VEL_D, AR_POSCON_VEL_FF, AR_POSCON_VEL_IMAX, AR_POSCON_VEL_FILT, AR_POSCON_VEL_FILT_D)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// 更新导航
void AR_PosControl::update(float dt)
{
    // 若无当前位置、终点或未解锁则立即退出
    Vector3p curr_pos_NED_m;
    Vector3f curr_vel_NED;
    if (!hal.util->get_soft_armed() || !AP::ahrs().get_relative_position_NED_origin(curr_pos_NED_m) ||
        !AP::ahrs().get_velocity_NED(curr_vel_NED)) {
        _desired_speed = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        return;
    }

    // 检查 EKF XY 位置重置
    handle_ekf_xy_reset();

    // 若近期无调用则重置速度控制器
    if (!is_active()) {
        _pid_vel.reset_I();
        _pid_vel.reset_filter();
    }
    _last_update_ms = AP_HAL::millis();

    // 计算位置误差并转换为期望速度
    _vel_target.zero();
    if (_pos_target_valid) {
        Vector2p pos_target = _pos_target;
        _vel_target = _p_pos.update_all(pos_target, curr_pos_NED_m.xy());
    }

    // 计算速度误差
    bool stopping = false;
    if (_vel_desired_valid) {
        // 将位置误差产生的期望速度叠加目标速度
        _vel_target += _vel_desired;
        stopping = _vel_desired.is_zero();
    }

    // 将速度限制到最大速度
    _vel_target.limit_length(get_speed_max());

    // 限制速度以防止围栏违规
    bool backing_up = false;
#if AP_AVOIDANCE_ENABLED
    AC_Avoid *avoid = AP::ac_avoid();
    if (avoid != nullptr) {
        Vector3f vel_3d_cms{_vel_target.x * 100.0f, _vel_target.y * 100.0f, 0.0f};
        const float accel_max_cmss = MIN(_accel_max, _lat_accel_max) * 100.0;
        avoid->adjust_velocity(vel_3d_cms, backing_up, _p_pos.kP(), accel_max_cmss, _p_pos.kP(), accel_max_cmss, dt);
        _vel_target.x = vel_3d_cms.x * 0.01;
        _vel_target.y = vel_3d_cms.y * 0.01;
    }
#endif  // AP_AVOIDANCE_ENABLED

    // 基于转向限位计算限制向量
    Vector2f steering_limit_vec;
    if (_atc.steering_limit_left()) {
        steering_limit_vec = AP::ahrs().body_to_earth2D(Vector2f{0, _reversed ? 1.0f : -1.0f});
    } else if (_atc.steering_limit_right()) {
        steering_limit_vec = AP::ahrs().body_to_earth2D(Vector2f{0, _reversed ? -1.0f : 1.0f});
    }

    // 计算期望加速度
    _accel_target = _pid_vel.update_all(_vel_target, curr_vel_NED.xy(), dt, steering_limit_vec);
    if (_accel_desired_valid) {
        _accel_target += _accel_desired;
    }
    // 速度控制器的 I 项在前后方向被清零
    const Vector2f lat_vec_ef = AP::ahrs().body_to_earth2D(Vector2f{0, 1});
    const Vector2f vel_i = _pid_vel.get_i().projected(lat_vec_ef);
    _pid_vel.set_integrator(vel_i);

    // 将期望加速度转换为期望前后速度、期望横向速度和期望转向率

    // 使用当前航向将加速度旋转到机体坐标系
    const Vector2f accel_target_FR = AP::ahrs().earth_to_body2D(_accel_target);

    // 计算最小转弯速度，即车辆在给定转弯半径和最大横向加速度一半
    // 的情况下能够转弯通过弯道的最大速度
    // todo: 修复 safe_sqrt 后移除 MAX of zero
    float turn_speed_min = MAX(safe_sqrt(_atc.get_turn_lat_accel_max() * 0.5 * _turn_radius), 0);

    // 将目标速度从大地坐标系旋转到机体坐标系
    const Vector2f vel_target_FR = AP::ahrs().earth_to_body2D(_vel_target);

    // 期望速度通常是目标速度的前向分量（仅限前向分量）
    float des_speed = vel_target_FR.x;
    if (!stopping) {
        // 除非车辆正在减速，否则不允许目标速度低于最小转弯速度
        const float abs_des_speed_min = MIN(_vel_target.length(), turn_speed_min);
        if (_reversed != backing_up) {
            // 若反向或倒车，期望速度为负
            des_speed = MIN(-abs_des_speed_min, vel_target_FR.x);
        } else {
            des_speed = MAX(abs_des_speed_min, vel_target_FR.x);
        }
    }
    _desired_speed = _atc.get_desired_speed_accel_limited(des_speed, dt);

    // 由期望横向加速度计算转向率
    _desired_lat_accel = stopping ? 0 : accel_target_FR.y;
    _desired_turn_rate_rads = _atc.get_turn_rate_from_lat_accel(_desired_lat_accel, _desired_speed);
}

// 最近是否调用过 update
bool AR_PosControl::is_active() const
{
    return ((AP_HAL::millis() - _last_update_ms) < AR_POSCON_TIMEOUT_MS);
}

// 设置限制
void AR_PosControl::set_limits(float speed_max, float accel_max, float lat_accel_max, float jerk_max)
{
    _speed_max = MAX(speed_max, 0);
    _accel_max = MAX(accel_max, 0);
    _lat_accel_max = MAX(lat_accel_max, 0);
    _jerk_max = MAX(jerk_max, 0);

    // 设置位置 P 控制器限制
    _p_pos.set_limits(_speed_max, MIN(_accel_max, _lat_accel_max), _jerk_max);
}

// 设置器：允许载具代码向本库提供转向相关参数值（应定期更新）
void AR_PosControl::set_turn_params(float turn_radius, bool pivot_possible)
{
    if (pivot_possible) {
        _turn_radius = 0;
    } else {
        _turn_radius = turn_radius;
    }
}

// 将位置控制器初始化为当前位置、速度、加速度和姿态
// 在使用输入整形方法之前应调用本函数
bool AR_PosControl::init()
{
    // 从 AHRS 获取当前位置和速度
    Vector2f pos_NE;
    Vector3f vel_NED;
    if (!AP::ahrs().get_relative_position_NE_origin_float(pos_NE) || !AP::ahrs().get_velocity_NED(vel_NED)) {
        return false;
    }

    // 将目标位置设置为当前位置
    _pos_target.x = pos_NE.x;
    _pos_target.y = pos_NE.y;

    // 设置目标速度和加速度
    _vel_desired = vel_NED.xy();
    _vel_target.zero();
    _accel_desired = AP::ahrs().get_accel_ef().xy();
    _accel_target.zero();

    // 清除反向设置
    _reversed = false;

    // 初始化 EKF XY 重置处理
    init_ekf_xy_reset();

    return true;
}

// 使用输入整形平滑调整位置、速度和加速度目标
// pos 为目标位置，为相对 EKF 原点的偏移（米）
// vel 为目标速度（m/s）。accel 为目标加速度（m/s/s）
// dt 为更新周期（秒）
// 在使用这些方法之前应调用一次 init
void AR_PosControl::input_pos_target(const Vector2p &pos, float dt)
{
    Vector2f vel;
    Vector2f accel;
    input_pos_vel_accel_target(pos, vel, accel, dt);
}

// 使用输入整形平滑调整位置、速度和加速度目标
// pos 为目标位置，为相对 EKF 原点的偏移（米）
// vel 为目标速度（m/s）。accel 为目标加速度（m/s/s）
// dt 为更新周期（秒）
// 在使用这些方法之前应调用一次 init
void AR_PosControl::input_pos_vel_target(const Vector2p &pos, const Vector2f &vel, float dt)
{
    Vector2f accel;
    input_pos_vel_accel_target(pos, vel, accel, dt);
}

// 使用输入整形平滑调整位置、速度和加速度目标
// pos 为目标位置，为相对 EKF 原点的偏移（米）
// vel 为目标速度（m/s）。accel 为目标加速度（m/s/s）
// dt 为更新周期（秒）
// 在使用这些方法之前应调用一次 init
void AR_PosControl::input_pos_vel_accel_target(const Vector2p &pos, const Vector2f &vel, const Vector2f &accel, float dt)
{
    // 将目标位置、速度和加速度向前推进 dt
    update_pos_vel_accel_xy(_pos_target, _vel_desired, _accel_desired, dt, Vector2f(), Vector2f(), Vector2f());

    // 调用 shape_pos_vel_accel_xy 将目标拉向最终目的地
    const float accel_max = MIN(_accel_max, _lat_accel_max);
    shape_pos_vel_accel_xy(pos, vel, accel, _pos_target, _vel_desired, _accel_desired,
                           _speed_max, accel_max, _jerk_max, dt, false);

    // 设置标志位，使 update 消费目标位置、期望速度和期望加速度
    _pos_target_valid = true;
    _vel_desired_valid = true;
    _accel_desired_valid = true;
}

// 设置目标位置、期望速度和加速度。这些应来自外部生成的路径，不经过"输入整形"
void AR_PosControl::set_pos_vel_accel_target(const Vector2p &pos, const Vector2f &vel, const Vector2f &accel)
{
    _pos_target = pos;
    _vel_desired = vel;
    _accel_desired = accel;
    _pos_target_valid = true;
    _vel_desired_valid = true;
    _accel_desired_valid = true;
}

// 返回纬度和经度方向上的期望速度向量（即前馈），单位 cm/s
Vector2f AR_PosControl::get_desired_velocity() const
{
    if (_vel_desired_valid) {
        return _vel_desired;
    }
    return Vector2f();
}

// 返回纬度和经度方向上的期望加速度向量（m/s）
Vector2f AR_PosControl::get_desired_accel() const
{
    if (_accel_desired_valid) {
        return _accel_desired;
    }
    return Vector2f();
}

/// 获取位置误差，即从当前位置指向目标位置的向量
Vector2p AR_PosControl::get_pos_error() const
{
    // 若未激活或无数位置估计，返回零误差
    Vector2f curr_pos_NE;
    if (!is_active() ||!AP::ahrs().get_relative_position_NE_origin_float(curr_pos_NE)) {
        return Vector2p{};
    }

    // 获取当前位置
    return (_pos_target - curr_pos_NE.topostype());
}

#if HAL_LOGGING_ENABLED
// 写入 PSC 日志
void AR_PosControl::write_log()
{
    // 若未激活则立即退出
    if (!is_active()) {
        return;
    }

    // 若无数位置或速度估计则立即退出
    Vector3f curr_pos_NED;
    Vector3f curr_vel_NED;
    if (!AP::ahrs().get_relative_position_NED_origin_float(curr_pos_NED) || !AP::ahrs().get_velocity_NED(curr_vel_NED)) {
        return;
    }

    // 获取加速度
    const Vector3f curr_accel_NED = AP::ahrs().get_accel_ef() * 100.0;

    // 将位置转换为所需格式
    Vector2f pos_target_2d_cm = get_pos_target().tofloat() * 100.0;

    // 复用 AC_PosControl 的日志记录：
    AC_PosControl::Write_PSCN(0.0,                  // 期望位置
                            pos_target_2d_cm.x,     // 位置目标
                            curr_pos_NED.x * 100.0, // 位置
                            _vel_desired.x * 100.0, // 期望速度
                            _vel_target.x * 100.0,  // 目标速度
                            curr_vel_NED.x * 100.0, // 速度
                            _accel_desired.x * 100.0,   // 期望加速度
                            _accel_target.x * 100.0,    // 目标加速度
                            curr_accel_NED.x);      // 加速度
    AC_PosControl::Write_PSCE(0.0,                  // 期望位置
                            pos_target_2d_cm.y,     // 位置目标
                            curr_pos_NED.y * 100.0, // 位置
                            _vel_desired.y * 100.0, // 期望速度
                            _vel_target.y * 100.0,  // 目标速度
                            curr_vel_NED.y * 100.0, // 速度
                            _accel_desired.y * 100.0,   // 期望加速度
                            _accel_target.y * 100.0,    // 目标加速度
                            curr_accel_NED.y);      // 加速度
}
#endif

/// 初始化 EKF XY 位置重置检查
void AR_PosControl::init_ekf_xy_reset()
{
    _ekf_xy_reset_count = AP::ahrs().get_position_NE_reset_count();
}

/// handle_ekf_xy_reset - 检查 EKF 位置重置并调整悬停或刹车目标位置
void AR_PosControl::handle_ekf_xy_reset()
{
    // 检查位置重置
    const uint16_t reset_count = AP::ahrs().get_position_NE_reset_count();
    if (reset_count != _ekf_xy_reset_count) {
        Vector2p pos_ne_m;
        if (!AP::ahrs().get_relative_position_NE_origin(pos_ne_m)) {
            return;
        }
        _pos_target = pos_ne_m + _p_pos.get_error().topostype();

        Vector3f vel_NED;
        if (!AP::ahrs().get_velocity_NED(vel_NED)) {
            return;
        }
        _vel_desired = vel_NED.xy() + _pid_vel.get_error();

        _ekf_xy_reset_count = reset_count;
    }
}
