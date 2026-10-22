#include "AP_Shuttle.h"
#include "Rover.h"

const AP_Param::GroupInfo AP_Shuttle::var_info[] = {
    // @Param: EN
    // @DisplayName: 穿梭模式使能
    // @Description: 使能两点穿梭模式（基于 MAV_CMD_DO_REPOSITION 的点到点导航，到达后悬停）。
    // @Values: 0:禁用,1:使能
    // @User: Advanced
    AP_GROUPINFO("EN", 0, AP_Shuttle, _enabled, 1),

    // @Param: ARRIVE_RAD
    // @DisplayName: 穿梭到达半径
    // @Description: 距目标小于该距离（米）时，穿梭可判定为已到达。可通过 MAV_CMD_DO_REPOSITION 的 param5 字段按指令覆盖。
    // @Units: m
    // @Range: 0.1 100
    // @User: Advanced
    AP_GROUPINFO("RAD", 1, AP_Shuttle, _arrive_rad, 1.0f),

    // @Param: ARRIVE_SPD
    // @DisplayName: 穿梭到达速度阈值
    // @Description: 地速低于该值（m/s）且同时满足到达半径条件时，穿梭判定为已到达。
    // @Units: m/s
    // @Range: 0.05 5
    // @User: Advanced
    AP_GROUPINFO("SPD", 2, AP_Shuttle, _arrive_spd, 0.2f),

    // @Param: MIN_DIST
    // @DisplayName: 穿梭最小目标距离
    // @Description: 车辆到目标的最小水平距离，低于该距离的航程将被拒绝（上报 SHUTTLE_ERR_DIST）。
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("MIN", 3, AP_Shuttle, _min_dist, 0.5f),

    // @Param: MAX_DIST
    // @DisplayName: 穿梭最大目标距离
    // @Description: 车辆到目标的最大水平距离，高于该距离的航程将被拒绝（上报 SHUTTLE_ERR_DIST）。
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("MAX", 4, AP_Shuttle, _max_dist, 5000.0f),

    // @Param: SPD
    // @DisplayName: 穿梭默认速度
    // @Description: 当 MAV_CMD_DO_REPOSITION 的 param1 字段非正时使用的默认航程速度。
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("CRUISE", 5, AP_Shuttle, _speed, 1.0f),

    AP_GROUPEND
};

// 校验一条航程但不改变任何状态。当该指令必须被拒绝时返回 false。
bool AP_Shuttle::validate_leg(const Location &target)
{
    if (!enabled()) {
        // 穿梭已禁用：让原生的 DO_REPOSITION 行为继续
        return true;
    }

    const float dist = rover.current_loc.get_distance(target);
    // 钳制配置的最大值，避免配置不当的 SHUTTLE_MAX <= SHUTTLE_MIN
    // 导致所有航程被静默拒绝
    const float min_dist = MAX(_min_dist.get(), 0.0f);
    const float max_dist = MAX(_max_dist.get(), min_dist + 1.0f);
    if (dist < min_dist || dist > max_dist) {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "SHUTTLE_ERR_DIST %.1f", (double)dist);
        return false;
    }
    return true;
}

// 启动一条已通过校验的航程。必须在车辆已进入 GUIDED 且目标已被接受后
// 才可调用，以保证穿梭状态始终与车辆状态一致。
void AP_Shuttle::begin_leg(const Location &target)
{
    _target = target;
    _leg_speed_ms = MAX(_speed.get(), 0.1f);

    // Mode::navigate_to_waypoint() 中的全向平移分支按 wp_nav 的最大速度
    // 航行，因此在此处把航程速度应用到那里
    rover.g2.wp_nav.set_speed_max(_leg_speed_ms);

    set_state(State::TASK);
    rover.gcs().send_text(MAV_SEVERITY_INFO, "SHUTTLE_TASK_START");
}

// 终止当前航程
void AP_Shuttle::stop(StopReason reason)
{
    if (_state == State::IDLE || _state == State::MANUAL_MODE) {
        return;
    }
    (void)reason;
    _task_type = TaskType::NONE;
    set_state(State::IDLE);
    rover.gcs().send_text(MAV_SEVERITY_INFO, "SHUTTLE_TASK_END");
}

// 由 Rover::set_mode() 在切入人工驾驶模式时调用
void AP_Shuttle::on_manual_override()
{
    if (_state == State::MANUAL_MODE) {
        return;
    }
    // 锁定：保持该状态直到 QGC 发送新的航程指令
    _task_type = TaskType::NONE;
    set_state(State::MANUAL_MODE);
    rover.gcs().send_text(MAV_SEVERITY_WARNING, "MANUAL_MODE_OVERRIDE");
}

// 由主循环调用
void AP_Shuttle::update()
{
    if (!enabled()) {
        return;
    }

    // 持续维护"当前所在点"（A/B）
    update_current_wp();

    switch (_state) {
    case State::IDLE:
    case State::MANUAL_MODE:
    case State::HOVER:
        return;
    case State::TASKING:
        update_task();
        return;
    case State::TASK:
    case State::MOVING:
        update_active_leg();
        return;
    }
}

// 判断当前所在点：与 A/B 的距离取近者
void AP_Shuttle::update_current_wp()
{
    if (!_wp_a_valid || !_wp_b_valid) {
        // 两点未全部定义，保持/清零
        if (!_wp_a_valid && !_wp_b_valid) {
            _current_wp = 0;
        }
        return;
    }
    const float d_a = rover.current_loc.get_distance(_wp_a);
    const float d_b = rover.current_loc.get_distance(_wp_b);
    _current_wp = (d_a < d_b) ? 1 : 2;
}

// 设 A 点：校验与 B 不重复（水平距离过近视为重复）
bool AP_Shuttle::set_wp_a(const Location &loc)
{
    if (!loc.check_latlng()) {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "SHUTTLE_ERR_LOC");
        return false;
    }
    if (_wp_b_valid && loc.get_distance(_wp_b) < 2.0f) {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "SHUTTLE_ERR_DUP_AB");
        return false;
    }
    _wp_a = loc;
    _wp_a_valid = true;
    update_current_wp();
    rover.gcs().send_text(MAV_SEVERITY_INFO, "SHUTTLE_WP_A_OK");
    return true;
}

// 设 B 点：校验与 A 不重复
bool AP_Shuttle::set_wp_b(const Location &loc)
{
    if (!loc.check_latlng()) {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "SHUTTLE_ERR_LOC");
        return false;
    }
    if (_wp_a_valid && loc.get_distance(_wp_a) < 2.0f) {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "SHUTTLE_ERR_DUP_AB");
        return false;
    }
    _wp_b = loc;
    _wp_b_valid = true;
    update_current_wp();
    rover.gcs().send_text(MAV_SEVERITY_INFO, "SHUTTLE_WP_B_OK");
    return true;
}

// 请求切到另一个点。返回目标点位置；两点未定义、当前位置未知、
// 或正处于移动中（应等到达悬停后再切）时返回 false。
bool AP_Shuttle::get_switch_target(Location &target) const
{
    if (!_wp_a_valid || !_wp_b_valid) {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "SHUTTLE_ERR_POINTS");
        return false;
    }
    // 仅允许在悬停（已到点）或空闲时切换；移动/任务中切换会打乱航程
    if (_state == State::MOVING || _state == State::TASK || _state == State::TASKING) {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "SHUTTLE_ERR_BUSY");
        return false;
    }
    if (_current_wp == 1) {
        target = _wp_b;
    } else if (_current_wp == 2) {
        target = _wp_a;
    } else {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "SHUTTLE_ERR_NOWP");
        return false;
    }
    return true;
}

// 启动模拟任务：仅 HOVER（已到达悬停）可启动；任务期间保持悬停并按本地计时
bool AP_Shuttle::start_task(TaskType type)
{
    if (_state != State::HOVER) {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "SHUTTLE_ERR_NOT_HOVER");
        return false;
    }
    switch (type) {
    case TaskType::SAMPLE:
        _task_duration_ms = 10000;   // 采样：模拟 10s
        break;
    case TaskType::MEASURE:
        _task_duration_ms = 5000;    // 测量：模拟 5s
        break;
    default:
        return false;
    }
    _task_type = type;
    _task_start_ms = AP_HAL::millis();
    set_state(State::TASKING);
    rover.gcs().send_text(MAV_SEVERITY_INFO, "SHUTTLE_TASK_START %u", (unsigned)_task_duration_ms / 1000);
    return true;
}

// 任务执行期间：保持悬停，本地计时到点后回到 HOVER
void AP_Shuttle::update_task()
{
    // 任务应保持在悬停（loiter）状态执行；若期间被切走，视为任务中断。
    // 此时船已在航点悬停，因此直接退回 HOVER 而不是清空为 IDLE，
    // 避免丢掉当前航点上下文（current_wp / A、B 点仍保留）。
    if (rover.control_mode != &rover.mode_loiter) {
        rover.gcs().send_text(MAV_SEVERITY_WARNING, "SHUTTLE_TASK_INTERRUPTED");
        _task_type = TaskType::NONE;
        set_state(State::HOVER);
        return;
    }
    if (AP_HAL::millis() - _task_start_ms >= _task_duration_ms) {
        _task_type = TaskType::NONE;
        set_state(State::HOVER);
        rover.gcs().send_text(MAV_SEVERITY_INFO, "SHUTTLE_TASK_DONE");
    }
}

// 航程运行期间的到达检测与故障保护检查
void AP_Shuttle::update_active_leg()
{
    // 航程被中断：车辆已不在 GUIDED（例如故障保护切到 HOLD/RTL）。
    // 人工模式切换由 on_manual_override() 单独处理。
    if (rover.control_mode != &rover.mode_guided) {
        stop(StopReason::INTERRUPTED);
        return;
    }

    // 安全：无可用的位置估计 -> 放弃航程，让原生的故障保护（hold/RTH）
    // 来接管车辆
    if (!rover.ekf_position_ok() || rover.failsafe.ekf) {
        rover.gcs().send_text(MAV_SEVERITY_CRITICAL, "SHUTTLE_ERR_FAILSAFE");
        stop(StopReason::FAILSAFE);
        return;
    }

    const float dist = rover.current_loc.get_distance(_target);
    const float spd  = rover.ahrs.groundspeed();
    const float radius = arrive_radius();   // 实时读取 SHUTTLE_RAD

    // TASK 与 MOVING 仅在航行中才有意义；一旦到达，下面会将状态锁定为
    // HOVER，避免每一帧都在状态间闪烁。
    if (_state != State::HOVER) {
        set_state((dist > radius) ? State::MOVING : State::TASK);
    }

    // 到达判定：优先采用 GUIDED 的 SCurve 原生判定（reached_destination）。
    // GUIDED 到达后会切到 loiter 拉回/保持，船的本地 dist/spd 会在目标附近
    // 反复振荡，永远凑不齐 "dist<=R && spd<V" 两个条件；而原生判定在
    // SCurve 停稳时一次性置位，与本库的 HOVER 状态切换天然同步。
    // 兜底条件保留（dist+spd 同时满足），用于无 SCurve 的路径。
    if (rover.g2.wp_nav.reached_destination() ||
        (dist <= radius && spd < arrive_speed_ms())) {
        rover.gcs().send_text(MAV_SEVERITY_INFO, "REACHED_WP");
        rover.gcs().send_mission_item_reached_message(0);
        // 切换到 loiter 以保持定点（mode_loiter 保持航向，并在无偏航
        // 机架上遵循 OMNI_YAW_HOLD）
        rover.set_mode(rover.mode_loiter, ModeReason::MISSION_END);
        set_state(State::HOVER);
        // 到达后刷新"当前所在点"（用目标点位置直接判定，比距离近更准）
        if (_wp_a_valid && _target.get_distance(_wp_a) < 1.0f) {
            _current_wp = 1;
        } else if (_wp_b_valid && _target.get_distance(_wp_b) < 1.0f) {
            _current_wp = 2;
        } else {
            update_current_wp();
        }
    }
}
