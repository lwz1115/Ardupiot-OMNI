#pragma once

#include <AP_Common/AP_Common.h>
#include <APM_Control/AR_AttitudeControl.h>

/*
 * 滑移转向地面车和船的原地转向（Pivot Turn）控制器
 *
 * 使用方法：
 *    1. 对滑移转向车辆调用一次 "enable(true)" 以启用此控制器
 *    2. 在车辆开始驶向航点前调用 "check_activation"，并提供到航点的大地坐标系航向
 *       这可能会将控制器的内部状态改为 "active"（激活）
 *    3. 在每个主循环迭代中调用 "active()" 以查看此控制器是否认为它在控制车辆
 *    4. 调用 "get_turn_rate_rads()" 获取朝向下一个航点的期望转向率
 *    5. 将上述转向率传入速率控制器，并将速度控制器的速度设为零
 *    6. 此控制器的 "active" 状态将在完成原地转向后变为 false
 */

class AR_PivotTurn {
public:

    // 构造函数
    AR_PivotTurn(AR_AttitudeControl& atc);

    // 启用或禁用原地转向控制器
    void enable(bool enable_pivot);

    // 此控制器是否正在控制车辆
    bool active() const;

    // 检查原地转向是否应被激活或停用
    // 若调用者希望无论航向误差如何都触发原地转向开始，force_active 应为 true
    void check_activation(float desired_heading_deg, bool force_active = false);

    // 检查给定预期偏航变化（度）时原地转向是否会被激活
    // 注意：此函数不会真正激活原地转向。要激活请使用 check_activation 方法
    bool would_activate(float yaw_change_deg) const WARN_IF_UNUSED;

    // 强制停用此控制器
    void deactivate() { _active = false; };

    // 获取转向率（rad/s）
    // desired heading 应为朝向下一个航点的航向（度）
    // dt 应为距上次调用的时间（秒）
    float get_turn_rate_rads(float desired_heading_deg, float dt);

    // 参数值访问器
    float get_rate_max() const { return _rate_max; }

    // 参数变量表
    static const struct AP_Param::GroupInfo var_info[];

private:

    // 返回转向后延迟时长（毫秒）
    uint32_t get_delay_duration_ms() const;

    // 参数
    AP_Int16 _angle;                // 触发原地转向的最小角度误差（度）
    AP_Int16 _rate_max;             // 原地转向期间的最大转向率（度）
    AP_Float _delay;                // 原地转向完成后的等待时间（秒）

    // 引用
    AR_AttitudeControl& _atc;       // 地面车姿态控制库

    // 局部变量
    bool _enabled;                  // 车辆可以原地转向则为 true
    bool _active;                   // 车辆当前正在原地转向则为 true
    uint32_t _delay_start_ms;       // 转向后延迟开始的系统时间
};
