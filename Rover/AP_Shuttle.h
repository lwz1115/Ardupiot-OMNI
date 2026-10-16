/*
   全向X水面船的点对点穿梭模式。

   飞控只负责单条航程：接收一条"前往该目标点"的指令，驶向目标（在
   全向机架上横向平移），到达后定点悬停。A->B->A 的往返编排、停泊延时
   以及所有界面逻辑都放在 QGC 端。

   QGC 接口：
     - MAV_CMD_DO_REPOSITION  启动一条航程
     - MAV_CMD_USER_1         param1=1 停止穿梭，param1=0 查询状态
   事件通过 STATUSTEXT 上报（见设计文档）。
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>

class AP_Shuttle
{
public:
    AP_Shuttle() {}

    // 状态机状态
    enum class State : uint8_t {
        IDLE        = 0,   // 无进行中的航程
        TASK        = 1,   // 航程已接受并通过校验
        MOVING      = 2,   // 正在驶向目标
        HOVER       = 3,   // 已到达，保持定点悬停
        MANUAL_MODE = 4,   // 驾驶员已接管，锁定直到收到新指令
    };

    enum class StopReason : uint8_t {
        GCS_REQUEST = 0,   // 地面站请求停止
        INTERRUPTED = 1,   // 因其他原因退出了 GUIDED（如故障保护……）
        FAILSAFE    = 2,   // 故障保护触发退出
    };

    // 参数组
    static const struct AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enabled.get() != 0; }
    State get_state() const { return _state; }
    bool is_active() const {
        return _state == State::TASK || _state == State::MOVING || _state == State::HOVER;
    }

    // 校验航程但不改变任何状态。当该指令必须被拒绝时返回 false（并上报
    // 原因），例如目标超出允许的距离范围。
    bool validate_leg(const Location &target);

    // 启动一条已通过校验的航程。只有在车辆确实已进入 GUIDED 且目标
    // 已被接受后才可调用，以保证穿梭状态始终与车辆状态一致。
    void begin_leg(const Location &target);

    // 终止当前航程
    void stop(StopReason reason);

    // 由主循环调用，用于到达检测与故障保护检查
    void update();

    // 由 Rover::set_mode() 在切入人工驾驶模式时调用
    void on_manual_override();

    float arrive_radius_default() const { return MAX(_arrive_rad.get(), 0.1f); }
    float arrive_radius() const { return arrive_radius_default(); }
    float arrive_speed_ms() const { return MAX(_arrive_spd.get(), 0.05f); }

private:
    void set_state(State s) { _state = s; }
    void update_active_leg();

    // 穿梭功能使能/禁用
    AP_Int8  _enabled;              // SHUTTLE_EN
    AP_Float _arrive_rad;           // SHUTTLE_RAD
    AP_Float _arrive_spd;           // SHUTTLE_SPD
    AP_Float _min_dist;             // SHUTTLE_MIN
    AP_Float _max_dist;             // SHUTTLE_MAX
    AP_Float _speed;                // SHUTTLE_CRUISE

    // 运行时状态（不持久化）
    State  _state = State::IDLE;
    Location _target {};
    float  _leg_speed_ms = 1.0f;
};
