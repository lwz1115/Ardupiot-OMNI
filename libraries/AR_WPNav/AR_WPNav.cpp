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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AR_WPNav.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

#define AR_WPNAV_TIMEOUT_MS             100
#define AR_WPNAV_SPEED_DEFAULT          2.0f
#define AR_WPNAV_SPEED_MIN              0.05f   // 航点间最小速度（m/s）
#define AR_WPNAV_SPEED_UPDATE_MIN_MS    500     // 最大速度在此毫秒数内不能更新超过一次
#define AR_WPNAV_RADIUS_DEFAULT         2.0f
#define AR_WPNAV_OVERSPEED_RATIO_MAX    5.0f    // 若 _overspeed_enabled，车辆最高可达到 WP_SPEED 的 5 倍速度
#define AR_WPNAV_SNAP_MAX               15.0f   // S 曲线 snap（加加速度变化率）m/s/s/s/s
#define AR_WPNAV_ACCEL_MAX              20.0    // 用户未指定加速度限制时使用的加速度

const AP_Param::GroupInfo AR_WPNav::var_info[] = {

    // @Param: SPEED
    // @DisplayName: 航点默认速度
    // @Description: 航点默认速度
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: 标准
    AP_GROUPINFO("SPEED", 1, AR_WPNav, _speed_max, AR_WPNAV_SPEED_DEFAULT),

    // @Param: RADIUS
    // @DisplayName: 航点半径
    // @Description: 判定到达航点的距离（米）。决定车辆何时转向下一航点。
    // @Units: m
    // @Range: 0 100
    // @Increment: 0.1
    // @User: 标准
    AP_GROUPINFO("RADIUS", 2, AR_WPNav, _radius, AR_WPNAV_RADIUS_DEFAULT),

    // 3 曾是 OVERSHOOT（越过航点）

    // 4 曾是 PIVOT_ANGLE（原地转向角）
    // 5 曾是 PIVOT_RATE（原地转向率）
    // 6 曾是 SPEED_MIN（最小速度）
    // 7 曾是 PIVOT_DELAY（原地转向延迟）

    // @Group: PIVOT_
    // @Path: AR_PivotTurn.cpp
    AP_SUBGROUPINFO(_pivot, "PIVOT_", 8, AR_WPNav, AR_PivotTurn),

    // @Param: ACCEL
    // @DisplayName: 航点加速度
    // @Description: 航点加速度。若为零则使用 ATC_ACCEL_MAX
    // @Units: m/s/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: 标准
    AP_GROUPINFO("ACCEL", 9, AR_WPNav, _accel_max, 0),

    // @Param: JERK
    // @DisplayName: 航点加加速度
    // @Description: 航点加加速度（加速度变化率）。若为零则加加速度等于加速度
    // @Units: m/s/s/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: 标准
    AP_GROUPINFO("JERK", 10, AR_WPNav, _jerk_max, 0),

    AP_GROUPEND
};

AR_WPNav::AR_WPNav(AR_AttitudeControl& atc, AR_PosControl &pos_control) :
    _pivot(atc),
    _atc(atc),
    _pos_control(pos_control)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// 初始化航点控制器。speed_max 应设置为最大速度（m/s）（留零则使用默认速度）
void AR_WPNav::init(float speed_max)
{
    // 确定最大速度、加速度和加加速度
    _check_speed_param_change = !is_positive(speed_max);
    if (is_positive(speed_max)) {
        _base_speed_max = speed_max;
    } else {
        _base_speed_max = _speed_max;
    }
    _base_speed_max = MAX(AR_WPNAV_SPEED_MIN, _base_speed_max);
    const float accel_max = get_accel_max();
    const float jerk_max = get_jerk_max();
    _last_speed_param_ms = _speed_max;

    // 初始化位置控制器
    _pos_control.set_limits(_base_speed_max, accel_max, _atc.get_turn_lat_accel_max(), jerk_max);

    _scurve_prev_leg.init();
    _scurve_this_leg.init();
    _scurve_next_leg.init();
    _track_scalar_dt = 1.0f;

    // 初始化一些标志位
    _reached_destination = false;
    _fast_waypoint = false;

    // 确保原地转向被停用
    _pivot.deactivate();
    _pivot_at_next_wp = false;

    // 将起点和终点初始化为停车点
    _orig_and_dest_valid = false;
    set_origin_and_destination_to_stopping_point();

    // 将微调速度初始化为零
    set_nudge_speed_max(0);
}

// 更新导航
void AR_WPNav::update(float dt)
{
    // 若无当前位置、起点或终点则立即退出
    Location current_loc;
    float speed;
    if (!hal.util->get_soft_armed() || !_orig_and_dest_valid || !AP::ahrs().get_location(current_loc) || !_atc.get_forward_speed(speed)) {
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _cross_track_error = 0;
        return;
    }

    // 若近期无调用，将 desired_speed_limited 初始化为当前速度
    if (!is_active()) {
        _desired_speed_limited = speed;
    }
    _last_update_ms = AP_HAL::millis();

    update_distance_and_bearing_to_destination();

    // 处理参数变化
    update_limits();

    // 除非车辆正在原地转向，否则沿路径推进目标
    if (!_pivot.active()) {
        switch (_nav_control_type) {
        case NavControllerType::NAV_SCURVE:
            advance_wp_target_along_track(current_loc, dt);
            break;
        case NavControllerType::NAV_PSC_INPUT_SHAPING:
            update_psc_input_shaping(dt);
            break;
        }
    }

    // 更新转向和速度
    update_steering_and_speed(current_loc, dt);
}

// 设置最大速度（m/s）。成功返回 true
// 调用频率不应超过 3Hz，否则 S 曲线路径规划可能无法正确推进
bool AR_WPNav::set_speed_max(float speed_max)
{
    // 对目标速度做范围检查
    if (speed_max < AR_WPNAV_SPEED_MIN) {
        return false;
    }

    _base_speed_max = speed_max;
    // 显式覆盖优先；禁用 WP_SPEED 参数刷新，使其无法覆盖此覆盖值
    _check_speed_param_change = false;
    return true;
}

// 设置速度微调量（m/s）。除非 nudge_speed_max > speed_max，否则不生效
// nudge_speed_max 应始终为正，无论车辆前进还是倒车
void AR_WPNav::set_nudge_speed_max(float nudge_speed_max)
{
    _nudge_speed_max = nudge_speed_max;
}

// 设置目标位置，以及（可选的）下一目标点
// 若已知下一目标点应提供，以实现平滑转弯
bool AR_WPNav::set_desired_location(const Location& destination, Location next_destination)
{
    // 若未激活、之前的终点被打断或使用了不同的控制器，则重新初始化
    if (!is_active() || !_reached_destination || (_nav_control_type != NavControllerType::NAV_SCURVE)) {
        if (!set_origin_and_destination_to_stopping_point()) {
            return false;
        }
        // 清空 S 曲线
        _scurve_prev_leg.init();
        _scurve_this_leg.init();
        _scurve_next_leg.init();
    }

    // 将本航段前移为上一航段
    _scurve_prev_leg = _scurve_this_leg;

    // 初始化一些变量
    _origin = _destination;
    _destination = destination;
    _orig_and_dest_valid = true;
    _reached_destination = false;

    update_distance_and_bearing_to_destination();

    // 若车辆在上一航点停下、或到上一航点的行程被打断、或导航刚开始，
    // 则检查车辆是否应原地转向
    if (!_fast_waypoint) {
        _pivot.deactivate();
        _pivot.check_activation((_reversed ? wrap_360_cd(oa_wp_bearing_cd() + 18000) : oa_wp_bearing_cd()) * 0.01, _pivot_at_next_wp);
    }

    // 将起点和终点转换为相对 EKF 原点的偏移
    Vector2f origin_NE;
    Vector2f destination_NE;
    if (!_origin.get_vector_xy_from_origin_NE_cm(origin_NE) ||
        !_destination.get_vector_xy_from_origin_NE_cm(destination_NE)) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    origin_NE *= 0.01f;
    destination_NE *= 0.01f;

    // 计算到终点的航迹
    if (_fast_waypoint && !_scurve_next_leg.finished()) {
        // 直接前移下一航段，跳过重新计算本航段
        _scurve_this_leg = _scurve_next_leg;
    } else {
        _scurve_this_leg.calculate_track(Vector3p{origin_NE.x, origin_NE.y, 0.0f},              // 起点
                                         Vector3p{destination_NE.x, destination_NE.y, 0.0f},    // 终点
                                         0.0, // 直线航段弧长为零
                                         _pos_control.get_speed_max(),
                                         _pos_control.get_speed_max(),  // 上升速度（未使用）
                                         _pos_control.get_speed_max(),  // 下降速度（未使用）
                                         _pos_control.get_accel_max(),  // 前后向加速度
                                         _pos_control.get_accel_max(),  // 垂直加速度（未使用）
                                         _pos_control.get_accel_max(),  // 转弯加速度
                                         AR_WPNAV_SNAP_MAX,             // snap（加加速度变化率）
                                         _pos_control.get_jerk_max());
    }

    // 处理下一终点
    _scurve_next_leg.init();
    _fast_waypoint = false;
    _pivot_at_next_wp = false;
    if (next_destination.initialised()) {
        // 检查车辆是否应在下一航点原地转向
        const float next_wp_yaw_change = get_corner_angle(_origin, destination, next_destination);
        _pivot_at_next_wp = _pivot.would_activate(next_wp_yaw_change);
        if (!_pivot_at_next_wp) {
            // 将 next_destination 转换为相对 EKF 原点的偏移
            Vector2f next_destination_NE;
            if (!next_destination.get_vector_xy_from_origin_NE_cm(next_destination_NE)) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return false;
            }
            next_destination_NE *= 0.01f;
            _scurve_next_leg.calculate_track(Vector3p{destination_NE.x, destination_NE.y, 0.0f},
                                             Vector3p{next_destination_NE.x, next_destination_NE.y, 0.0f},
                                             0.0, // 直线航段弧长为零
                                             _pos_control.get_speed_max(),
                                             _pos_control.get_speed_max(),  // 上升速度（未使用）
                                             _pos_control.get_speed_max(),  // 下降速度（未使用）
                                             _pos_control.get_accel_max(),  // 前后向加速度
                                             _pos_control.get_accel_max(),  // 垂直加速度（未使用）
                                             _pos_control.get_accel_max(),  // 转弯加速度
                                             AR_WPNAV_SNAP_MAX,             // snap（加加速度变化率）
                                             _pos_control.get_jerk_max());

            // 提供了下一终点，因此为快航点
            _fast_waypoint = true;
        }
    }

    // 导航到终点使用 S 曲线
    _nav_control_type = NavControllerType::NAV_SCURVE;

    update_distance_and_bearing_to_destination();

    return true;
}

// 将目标位置设置为合理的停车点，成功返回 true
bool AR_WPNav::set_desired_location_to_stopping_location()
{
    Location stopping_loc;
    if (!get_stopping_location(stopping_loc)) {
        return false;
    }
    return set_desired_location(stopping_loc);
}

// 以相对 EKF 原点的偏移设置目标位置，成功返回 true
bool AR_WPNav::set_desired_location_NED(const Vector3f& destination)
{
    // 将终点初始化为 EKF 原点
    Location destination_ned;
    if (!AP::ahrs().get_origin(destination_ned)) {
        return false;
    }

    // 应用偏移
    destination_ned.offset(destination.x, destination.y);
    return set_desired_location(destination_ned);
}

bool AR_WPNav::set_desired_location_NED(const Vector3f &destination, const Vector3f &next_destination)
{
    // 将终点初始化为 EKF 原点
    Location dest_loc, next_dest_loc;
    if (!AP::ahrs().get_origin(dest_loc)) {
        return false;
    }
    next_dest_loc = dest_loc;

    // 应用偏移
    dest_loc.offset(destination.x, destination.y);
    next_dest_loc.offset(next_destination.x, next_destination.y);
    return set_desired_location(dest_loc, next_dest_loc);
}

// 设置目标位置，但预期目标在不久的将来会被再次更新
// 导航将使用位置控制器输入整形，而非 S 曲线
// 注意：使用此方法时不支持避障
bool AR_WPNav::set_desired_location_expect_fast_update(const Location &destination)
{
    // 若未激活则初始化
    if (!is_active() || (_nav_control_type != NavControllerType::NAV_PSC_INPUT_SHAPING)) {
        if (!set_origin_and_destination_to_stopping_point()) {
            return false;
        }
    }

    // 初始化一些变量
    _origin = _destination;
    _destination = destination;
    _orig_and_dest_valid = true;
    _reached_destination = false;

    update_distance_and_bearing_to_destination();

    // 检查车辆是否应原地转向
    _pivot.check_activation((_reversed ? wrap_360_cd(oa_wp_bearing_cd() + 18000) : oa_wp_bearing_cd()) * 0.01);

    // 导航到终点使用位置控制器输入整形
    _nav_control_type = NavControllerType::NAV_PSC_INPUT_SHAPING;
    return true;
}

// 使用当前位置、速度和最大加速度计算车辆停车点
bool AR_WPNav::get_stopping_location(Location& stopping_loc)
{
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return false;
    }

    // 获取当前速度向量和速度
    const Vector2f velocity = AP::ahrs().groundspeed_vector();
    const float speed = velocity.length();

    // 避免除以零
    if (!is_positive(speed)) {
        stopping_loc = current_loc;
        return true;
    }

    // 获取停车距离（米）
    const float stopping_dist = _atc.get_stopping_distance(speed);

    // 以米为单位计算距当前位置的停车位置
    const Vector2f stopping_offset = velocity.normalized() * stopping_dist;
    stopping_loc = current_loc;
    stopping_loc.offset(stopping_offset.x, stopping_offset.y);

    return true;
}

// 最近是否调用过 update
bool AR_WPNav::is_active() const
{
    return ((AP_HAL::millis() - _last_update_ms) < AR_WPNAV_TIMEOUT_MS);
}

// 使用 S 曲线导航沿航迹从起点向终点推进目标位置
void AR_WPNav::advance_wp_target_along_track(const Location &current_loc, float dt)
{
    // 若无当前位置、终点或未解锁则立即退出
    Vector2f curr_pos_NE;
    Vector3f curr_vel_NED;
    if (!AP::ahrs().get_relative_position_NE_origin_float(curr_pos_NE) || !AP::ahrs().get_velocity_NED(curr_vel_NED)) {
        return;
    }

    // 若无法将航点起点转换为相对 EKF 原点的偏移则立即退出
    Vector2f origin_NE;
    if (!_origin.get_vector_xy_from_origin_NE_cm(origin_NE)) {
        return;
    }
    // 从厘米转换为米
    origin_NE *= 0.01f;

    // 使用 _track_scalar_dt 减慢 S 曲线时间，防止目标移动到车辆前方太远
    Vector2f curr_target_vel = _pos_control.get_desired_velocity();
    float track_scaler_dt = 1.0f;
    if (is_positive(curr_target_vel.length())) {
        Vector2f track_direction = curr_target_vel.normalized();
        const float track_error = _pos_control.get_pos_error().tofloat().dot(track_direction);
        float track_velocity = curr_vel_NED.xy().dot(track_direction);
        // 设置时间缩放系数以匹配车辆可达到的速度，并留 5% 缓冲应对短期变化
        const float time_scaler_dt_max = _overspeed_enabled ? AR_WPNAV_OVERSPEED_RATIO_MAX : 1.0f;
        track_scaler_dt = constrain_float(0.05f + (track_velocity - _pos_control.get_pos_p().kP() * track_error) / curr_target_vel.length(), 0.0f, time_scaler_dt_max);
    }
    // 以最大加速度/最大加加速度为时间常数改变 S 曲线时间速度
    float track_scaler_tc = 1.0f;
    if (is_positive(_pos_control.get_jerk_max())) {
        track_scaler_tc = _pos_control.get_accel_max() / _pos_control.get_jerk_max();
    }
    _track_scalar_dt += (track_scaler_dt - _track_scalar_dt) * (dt / track_scaler_tc);

    // 从直线或样条计算器获取目标位置、速度和加速度
    Vector3p target_pos_3d{origin_NE.x, origin_NE.y, 0.0f};
    Vector3f target_vel, target_accel;

    // 更新目标位置、速度和加速度
    const float wp_radius = MAX(_radius, _turn_radius);
    bool s_finished = _scurve_this_leg.advance_target_along_track(_scurve_prev_leg, _scurve_next_leg, wp_radius, _pos_control.get_lat_accel_max(), _fast_waypoint, _track_scalar_dt * dt, target_pos_3d, target_vel, target_accel);

    // 将新目标传递给位置控制器
    init_pos_control_if_necessary();
    Vector2p target_pos_ptype{target_pos_3d.x, target_pos_3d.y};
    _pos_control.set_pos_vel_accel_target(target_pos_ptype, target_vel.xy(), target_accel.xy());

    // 检查是否已到达航点
    if (!_reached_destination && s_finished) {
        // "快"航点在中间点到达终点时即完成
        if (_fast_waypoint) {
            _reached_destination = true;
        } else {
            // 普通航点还要求车辆位于航点半径内或越过"终点线"
            const bool near_wp = current_loc.get_distance(_destination) <= _radius;
            const bool past_wp = current_loc.past_interval_finish_line(_origin, _destination);
            _reached_destination = near_wp || past_wp;
        }
    }
}

// update psc input shaping navigation controller
void AR_WPNav::update_psc_input_shaping(float dt)
{
    // 将终点位置转换为相对 EKF 原点的偏移（米）
    Vector2f pos_target_cm;
    if (!_destination.get_vector_xy_from_origin_NE_cm(pos_target_cm)) {
        return;
    }

    // 若位置控制器近期未被调用则初始化它
    init_pos_control_if_necessary();

    // 转换为米并更新目标
    const Vector2p pos_target = pos_target_cm.topostype() * 0.01;
    _pos_control.input_pos_target(pos_target, dt);

    // 更新 reached_destination
    if (!_reached_destination) {
        // 计算终点与位置控制器整形目标之间的位置差
        Vector2p pos_target_diff = pos_target - _pos_control.get_pos_target();
        // 当目标距离终点小于 1cm 且车辆位于航点半径内时，视为已到达终点
        _reached_destination = (pos_target_diff.length_squared() < sq(0.01)) && (_pos_control.get_pos_error().length_squared() < sq(_radius));
    }
}

// 更新从车辆当前位置到终点的直线距离和方位
void AR_WPNav::update_distance_and_bearing_to_destination()
{
    // 若无当前位置则保持距离不变
    Location current_loc;
    if (!_orig_and_dest_valid || !AP::ahrs().get_location(current_loc)) {
        _distance_to_destination = 0.0f;
        _wp_bearing_cd = 0.0f;
        return;
    }
    _distance_to_destination = current_loc.get_distance(_destination);
    _wp_bearing_cd = current_loc.get_bearing_to(_destination);
}

// 计算沿起点到终点航点连线行驶所需的转向和速度
void AR_WPNav::update_steering_and_speed(const Location &current_loc, float dt)
{
    _cross_track_error = calc_crosstrack_error(current_loc);

    // 更新位置控制器
    _pos_control.set_reversed(_reversed);
    _pos_control.update(dt);

    // 处理原地转向
    if (_pivot.active()) {
        // 减速到零
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_heading_cd = _reversed ? wrap_360_cd(oa_wp_bearing_cd() + 18000) : oa_wp_bearing_cd();
        _desired_turn_rate_rads = is_zero(_desired_speed_limited) ? _pivot.get_turn_rate_rads(_desired_heading_cd * 0.01, dt) : 0;
        _desired_lat_accel = 0.0f;
    } else {
        _desired_speed_limited = _pos_control.get_desired_speed();
        _desired_turn_rate_rads = _pos_control.get_desired_turn_rate_rads();
        _desired_lat_accel = _pos_control.get_desired_lat_accel();
    }
}

// 设置器：允许载具代码向本库提供转向相关参数值（应定期更新）
void AR_WPNav::set_turn_params(float turn_radius, bool pivot_possible)
{
    _turn_radius = pivot_possible ? 0.0 : turn_radius;
    _pivot.enable(pivot_possible);
}

// 计算侧向偏航误差（横迹误差）
// 车辆位于航迹左侧时该值为负
float AR_WPNav::calc_crosstrack_error(const Location& current_loc) const
{
    if (!_orig_and_dest_valid) {
        return 0.0f;
    }

    // 获取经避障调整后的起点和终点
    const Location &orig = get_oa_origin();
    const Location &dest = get_oa_destination();

    // 计算终点相对起点的东北坐标位置
    Vector2f dest_from_origin = orig.get_distance_NE(dest);

    // 若航迹长度非常小，则返回到终点的距离
    if (dest_from_origin.length() < 1.0e-6f) {
        return current_loc.get_distance_NE(dest).length();
    }

    // 转换为仅表示方向的向量
    dest_from_origin.normalize();

    // 计算车辆相对起点的东北坐标位置
    const Vector2f veh_from_origin = orig.get_distance_NE(current_loc);

    // 计算到目标航迹的距离，用于上报
    return veh_from_origin % dest_from_origin;
}

// 计算下一航点处的偏航角变化（度）
// 若部分点重合而无法计算角度，返回零
float AR_WPNav::get_corner_angle(const Location& loc1, const Location& loc2, const Location& loc3) const
{
    // 合理性检查
    if (!loc1.initialised() || !loc2.initialised() || !loc3.initialised()) {
        return 0;
    }
    const float loc1_to_loc2_deg = loc1.get_bearing_to(loc2) * 0.01;
    const float loc2_to_loc3_deg = loc2.get_bearing_to(loc3) * 0.01;
    const float diff_yaw_deg = wrap_180(loc2_to_loc3_deg - loc1_to_loc2_deg);
    return diff_yaw_deg;
}

// 辅助函数：若位置控制器近期未被调用则初始化它
// 应在更新位置控制器新目标之前、且 EKF 已有良好位置估计之后调用
void AR_WPNav::init_pos_control_if_necessary()
{
    // 若位置控制器近期未被调用则初始化它
    if (!_pos_control.is_active()) {
        if (!_pos_control.init()) {
            // 此时应始终有有效的位置估计，因此此调用不应失败
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return;
        }
    }
}

// 将起点和终点设置为停车点
bool AR_WPNav::set_origin_and_destination_to_stopping_point()
{
    // 将起点和终点初始化为停车点
    Location stopping_loc;
    if (!get_stopping_location(stopping_loc)) {
        return false;
    }
    _origin = _destination = stopping_loc;
    _orig_and_dest_valid = true;
    return true;
}

// 检查 _nudge_speed_max、_base_speed_max、_accel_max、_jerk_max 或
// _atc.get_turn_lat_accel_max() 是否变化，必要时更新位置控制器限制
void AR_WPNav::update_limits()
{
    // 若自初始化以来 WP_SPEED 参数发生变化，则刷新 _base_speed_max
    if (_check_speed_param_change && !is_equal(_speed_max.get(), _last_speed_param_ms)) {
        _base_speed_max = MAX(AR_WPNAV_SPEED_MIN, _speed_max.get());
        _last_speed_param_ms = _speed_max;
    }

    // 更新限制
    // 注意：这些限制要到下一个航点（或在快航点情况下
    // 再到下一个航点）才会应用到 S 曲线航段
    const float accel_max = get_accel_max();
    const float jerk_max = get_jerk_max();
    const float speed_max = MAX(_base_speed_max, _nudge_speed_max);
    const float lat_accel_max = _atc.get_turn_lat_accel_max();

    // 忽略不改变速度、加速度、加加速度或横向加速度限制的调用
    if (is_equal(speed_max, _pos_control.get_speed_max()) && is_equal(accel_max, _pos_control.get_accel_max()) &&
        is_equal(jerk_max, _pos_control.get_jerk_max()) &&
        is_equal(lat_accel_max, _pos_control.get_lat_accel_max())) {
        return;
    }

    // 防止过快更新
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_speed_update_ms < AR_WPNAV_SPEED_UPDATE_MIN_MS) {
        return;
    }
    _last_speed_update_ms = now_ms;

    // 更新位置控制器
    _pos_control.set_limits(speed_max, accel_max, lat_accel_max, jerk_max);

    // 改变航迹速度
    _scurve_this_leg.set_speed_max(_pos_control.get_speed_max(), _pos_control.get_speed_max(), _pos_control.get_speed_max());
    _scurve_next_leg.set_speed_max(_pos_control.get_speed_max(), _pos_control.get_speed_max(), _pos_control.get_speed_max());
}

float AR_WPNav::get_accel_max() const
{
    float atc_accel_max = MIN(_atc.get_accel_max(), _atc.get_decel_max());
    if (!is_positive(atc_accel_max)) {
        // accel_max 为零表示无限制，因此使用最大加速度
        atc_accel_max = AR_WPNAV_ACCEL_MAX;
    }
    return is_positive(_accel_max) ? MIN(_accel_max, atc_accel_max) : atc_accel_max;
}

float AR_WPNav::get_jerk_max() const
{
    return is_positive(_jerk_max) ? _jerk_max : get_accel_max();
}
