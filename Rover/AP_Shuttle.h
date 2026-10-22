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
        TASKING     = 5,   // 任务执行中（本地计时模拟，保持悬停）
    };

    // 任务类型（QGC 通过 USER_1 param1=5/6 触发）
    enum class TaskType : uint8_t {
        NONE     = 0,
        SAMPLE   = 1,   // 采样：模拟延迟 10s
        MEASURE  = 2,   // 测量：模拟延迟 5s
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
    TaskType get_task_type() const { return _task_type; }
    bool is_active() const {
        return _state == State::TASK || _state == State::MOVING || _state == State::HOVER ||
               _state == State::TASKING;
    }

    // A/B 点管理（两点穿梭：当前在 A 则切到 B，在 B 则切回 A）
    bool set_wp_a(const Location &loc);   // 设 A 点，校验与 B 不重复
    bool set_wp_b(const Location &loc);   // 设 B 点，校验与 A 不重复
    bool has_wp_a() const { return _wp_a_valid; }
    bool has_wp_b() const { return _wp_b_valid; }
    // 当前所在点（1=A, 2=B, 0=未知）
    uint8_t current_wp() const { return _current_wp; }
    // 判断当前所在点（距离 A/B 哪个近）；都没设则保持 0
    void update_current_wp();
    // 请求切到另一个点。返回目标点位置；若尚未定义两点或当前位置未知返回 false。
    bool get_switch_target(Location &target) const;
    // 启动一个模拟任务（保持悬停，本地计时）。仅 HOVER 状态可启动。
    bool start_task(TaskType type);

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
    void update_task();

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

    // 两点穿梭：A/B 点与当前所在点
    Location _wp_a {};
    Location _wp_b {};
    bool _wp_a_valid = false;
    bool _wp_b_valid = false;
    uint8_t _current_wp = 0;      // 1=A, 2=B, 0=未知

    // 模拟任务状态
    TaskType _task_type = TaskType::NONE;
    uint32_t _task_start_ms = 0;
    uint32_t _task_duration_ms = 0;   // 采样 10s / 测量 5s
};
