#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/SCurve.h>
#include <APM_Control/AR_AttitudeControl.h>
#include <APM_Control/AR_PosControl.h>
#include <AC_Avoidance/AP_OAPathPlanner.h>
#include "AR_PivotTurn.h"

class AR_WPNav {
public:

    // 构造函数
    AR_WPNav(AR_AttitudeControl& atc, AR_PosControl &pos_control);

    // 初始化航点控制器。speed_max 应设置为最大速度（m/s），留零则使用默认速度
    void init(float speed_max = 0);

    // 更新导航
    virtual void update(float dt);

    // 获取或设置最大速度（m/s）
    // 若在短时间内连续调用 set_speed_max，速度变化可能延迟最多 0.5 秒
    float get_speed_max() const { return _base_speed_max; }
    bool set_speed_max(float speed_max);

    // 设置速度微调量（m/s）。除非 nudge_speed_max > speed_max，否则不生效
    // nudge_speed_max 应始终为正，无论车辆是前进还是倒车
    void set_nudge_speed_max(float nudge_speed_max);

    // 反向执行任务（即倒车驶向终点）
    bool get_reversed() const { return _reversed; }
    void set_reversed(bool reversed) { _reversed = reversed; }

    // 获取导航输出：速度（m/s）和转向率（rad/s）
    float get_speed() const { return _desired_speed_limited; }
    float get_turn_rate_rads() const { return _desired_turn_rate_rads; }

    // 获取期望横向加速度（仅供上报，原地转向时为零）
    float get_lat_accel() const { return _desired_lat_accel; }

    // 设置目标位置，以及（可选的）下一目标点
    // 若已知下一目标点应提供，以实现平滑转弯
    virtual bool set_desired_location(const Location &destination, Location next_destination = Location()) WARN_IF_UNUSED;

    // 将目标位置设置为合理的停车点，成功返回 true
    bool set_desired_location_to_stopping_location()  WARN_IF_UNUSED;

    // 以相对 EKF 原点的偏移设置目标位置，成功返回 true
    bool set_desired_location_NED(const Vector3f& destination) WARN_IF_UNUSED;
    bool set_desired_location_NED(const Vector3f &destination, const Vector3f &next_destination) WARN_IF_UNUSED;

    // 设置目标位置，但预期目标在不久的将来会被再次更新
    // 导航将使用位置控制器输入整形，而非 S 曲线
    // 注意：使用此方法时不支持避障
    bool set_desired_location_expect_fast_update(const Location &destination) WARN_IF_UNUSED;

    // 车辆是否已到达目标位置。默认返回 true，因为此接口通常用于任务系统，不希望任务卡死
    virtual bool reached_destination() const { return _reached_destination; }

    // 返回到终点的直线距离（米）
    float get_distance_to_destination() const { return _distance_to_destination; }

    // 目标位置是否有效
    bool is_destination_valid() const { return _orig_and_dest_valid; }

    // 获取当前目标位置。注意：不保证有效（即未检查 _orig_and_dest_valid）
    const Location &get_destination() const { return _destination; }

    // 供上报地面站使用（NAV_CONTROLLER_OUTPUT 消息）
    float wp_bearing_cd() const { return _wp_bearing_cd; }
    float nav_bearing_cd() const { return _desired_heading_cd; }
    float crosstrack_error_m() const { return _cross_track_error; }

    // 获取经避障调整后的起点。注意：不保证有效（即未检查 _orig_and_dest_valid）
    virtual const Location &get_oa_origin() const { return _origin; }

    // 获取经避障调整后的终点。注意：不保证有效（即未检查 _orig_and_dest_valid）
    virtual const Location &get_oa_destination() const { return get_destination(); }

    // 返回考虑避障后的到下一航点方位（百分度），（供帆船使用）
    virtual float oa_wp_bearing_cd() const { return wp_bearing_cd(); }

    // 设置器：允许载具代码向本库提供转向相关参数值（应定期更新）
    void set_turn_params(float turn_radius, bool pivot_possible);

    // 启用位置目标加速，以追赶速度高于 WP_SPEED 的载具
    // 用于支持无精确速度控制的帆船
    // 仅在使用 S 曲线时支持，位置控制器输入整形不支持
    void enable_overspeed(bool enable) { _overspeed_enabled = enable; }

    // 参数值访问器
    float get_default_speed() const { return _speed_max; }
    float get_default_accel() const { return _accel_max; }
    float get_default_jerk() const { return _jerk_max; }
    float get_radius() const { return _radius; }
    float get_pivot_rate() const { return _pivot.get_rate_max(); }

    // 使用当前位置和姿态控制器提供的最大减速度计算停车位置
    // 成功返回 true，失败返回 false
    bool get_stopping_location(Location& stopping_loc) WARN_IF_UNUSED;

    // 若车辆在终点不停车（例如 set_desired_location 提供了下一目标点），is_fast_waypoint 返回 true
    bool is_fast_waypoint() const { return _fast_waypoint; }

    // 参数变量表
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // 最近是否调用过 update
    bool is_active() const;

    // 使用 S 曲线导航沿航迹从起点向终点推进目标位置
    void advance_wp_target_along_track(const Location &current_loc, float dt);

    // 更新位置控制器输入整形导航控制器
    void update_psc_input_shaping(float dt);

    // 更新从车辆当前位置到终点的直线距离和方位
    void update_distance_and_bearing_to_destination();

    // 计算沿起点到终点航点连线行驶所需的转向和速度
    void update_steering_and_speed(const Location &current_loc, float dt);

    // 计算侧向偏航误差（横迹误差）
    // 车辆位于航迹左侧时该值为负
    float calc_crosstrack_error(const Location& current_loc) const;

    // 计算下一航点处的偏航角变化（度）
    // 若部分点重合而无法计算角度，返回零
    float get_corner_angle(const Location& loc1, const Location& loc2, const Location& loc3) const;

    // 辅助函数：若位置控制器近期未被调用则初始化它
    // 应在更新位置控制器新目标之前、且 EKF 已有良好位置估计之后调用
    void init_pos_control_if_necessary();

    // 将起点和终点设置为停车点
    bool set_origin_and_destination_to_stopping_point();

    // 检查 _nudge_speed_max、_base_speed_max、_accel_max、_jerk_max 或
    // _atc.get_turn_lat_accel_max() 是否变化，必要时更新位置控制器限制
    void update_limits();

    // 获取位置控制器使用的最大加速度
    float get_accel_max() const;

    // 获取位置控制器使用的最大加加速度（jerk）
    float get_jerk_max() const;

    // 参数
    AP_Float _speed_max;            // 航点间目标速度（m/s）
    AP_Float _radius;               // 判定到达航点的距离（米）
    AR_PivotTurn _pivot;            // 原地转向控制器
    AP_Float _accel_max;            // 最大加速度。若为零则使用姿态控制器指定的最大加速度
    AP_Float _jerk_max;             // 最大加加速度（加速度变化率）。若为零则与 accel_max 相同

    // 引用
    AR_AttitudeControl& _atc;       // 地面车姿态控制库
    AR_PosControl &_pos_control;    // 地面车位置控制库

    // S 曲线
    SCurve _scurve_prev_leg;        // 上一段 S 曲线轨迹，用于与当前轨迹平滑衔接
    SCurve _scurve_this_leg;        // 当前段 S 曲线轨迹
    SCurve _scurve_next_leg;        // 下一段 S 曲线轨迹，用于与当前轨迹平滑衔接
    bool _fast_waypoint;            // 在下一航点不停车则为 true
    bool _pivot_at_next_wp;         // 应在下一航点原地转向则为 true
    bool _overspeed_enabled;        // 若为 true，S 曲线位置目标将加速以追赶速度高于 WP_SPEED 的载具
    float _track_scalar_dt;         // 时间缩放系数，确保 S 曲线目标不会超前车辆太远

    // 暂存在载具代码中的变量
    float _turn_radius;             // 车辆转弯半径（米）

    // 导航用变量
    uint32_t _last_update_ms;       // 上次调用 update 的系统时间
    Location _origin;               // 起点位置（车辆将从起点驶向终点）
    Location _destination;          // Guided_WP 模式下的终点位置
    Location _next_destination;     // Guided_WP 模式下的下一终点位置
    bool _orig_and_dest_valid;      // 起点和终点已设置则为 true
    bool _reversed;                 // 以倒车方式执行任务
    enum class NavControllerType {
        NAV_SCURVE = 0,             // 导航使用 S 曲线
        NAV_PSC_INPUT_SHAPING       // 导航使用位置控制器输入整形
    } _nav_control_type;            // 从 _origin 驶向 _destination 应使用的导航控制器

    // speed_max 处理
    float _base_speed_max;          // 由参数或 init 传入导出的最大速度（m/s）
    float _nudge_speed_max;         // 通常来自飞行员的"微调"最大速度（m/s）。小于 _base_speed_max 时不生效。始终为正
    uint32_t _last_speed_update_ms; // speed_max 上次更新的系统时间。用于防止 speed_max 更新过快
    bool _check_speed_param_change; // 导航期间是否监控 WP_SPEED 参数变化
    float _last_speed_param_ms;     // 上次记录的 WP_SPEED 值（m/s），用于变化检测

    // 导航库主要输出
    float _desired_speed_limited;   // 期望速度（上）但受加速/减速限制
    float _desired_turn_rate_rads;  // 期望转向率（rad/s）（逆时针为负，顺时针为正）
    float _desired_lat_accel;       // 期望横向加速度（仅供上报）
    float _desired_heading_cd;      // 期望航向（朝向起点与终点之间的连线）
    float _wp_bearing_cd;           // 到航点的方位（百分度）
    float _cross_track_error;       // 侧向偏航误差（米），车辆在航迹左侧时为负

    // 上报用变量
    float _distance_to_destination; // 车辆到最终终点的直线距离（米）
    bool _reached_destination;      // 车辆到达终点后为 true
};
