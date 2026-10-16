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
    set_state(State::MANUAL_MODE);
    rover.gcs().send_text(MAV_SEVERITY_WARNING, "MANUAL_MODE_OVERRIDE");
}

// 由主循环调用
void AP_Shuttle::update()
{
    if (!enabled() || _state == State::IDLE || _state == State::MANUAL_MODE ||
        _state == State::HOVER) {
        return;
    }
    update_active_leg();
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

    // 到达需要同时满足距离与速度两个条件
    if (dist <= radius && spd < arrive_speed_ms()) {
        rover.gcs().send_text(MAV_SEVERITY_INFO, "REACHED_WP");
        rover.gcs().send_mission_item_reached_message(0);
        // 切换到 loiter 以保持定点（mode_loiter 保持航向，并在无偏航
        // 机架上遵循 OMNI_YAW_HOLD）
        rover.set_mode(rover.mode_loiter, ModeReason::MISSION_END);
        set_state(State::HOVER);
    }
}
