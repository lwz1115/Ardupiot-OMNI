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
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AR_PivotTurn.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

#define AR_PIVOT_TIMEOUT_MS     100 // 原地转向控制器在此毫秒数内未被调用则超时并重置目标
#define AR_PIVOT_ANGLE_DEFAULT  0   // PIVOT_ANGLE 参数默认值（默认禁用）
#define AR_PIVOT_ANGLE_ACCURACY 5   // 车辆将原地转向到距目标此度数以内
#define AR_PIVOT_RATE_DEFAULT   60  // PIVOT_RATE 参数默认值
#define AR_PIVOT_DELAY_DEFAULT  0   // PIVOT_DELAY 参数默认值

const AP_Param::GroupInfo AR_PivotTurn::var_info[] = {

    // @Param: ANGLE
    // @DisplayName: 原地转向角
    // @Description: 当车辆航向与目标航向之差大于此度数时进行原地转向。设为零禁用原地转向。此参数应大于 5 度原地转向才能工作。
    // @Units: deg
    // @Range: 0 360
    // @Increment: 1
    // @User: 标准
    AP_GROUPINFO("ANGLE", 1, AR_PivotTurn, _angle, AR_PIVOT_ANGLE_DEFAULT),

    // @Param: RATE
    // @DisplayName: 原地转向率
    // @Description: 原地转向期间的转向率
    // @Units: deg/s
    // @Range: 0 360
    // @Increment: 1
    // @User: 标准
    AP_GROUPINFO("RATE", 2, AR_PivotTurn, _rate_max, AR_PIVOT_RATE_DEFAULT),

    // @Param: DELAY
    // @DisplayName: 原地转向延迟
    // @Description: 车辆完成原地转向后等待此秒数再继续
    // @Units: s
    // @Range: 0 60
    // @Increment: 0.1
    // @User: 标准
    AP_GROUPINFO("DELAY", 3, AR_PivotTurn, _delay, AR_PIVOT_DELAY_DEFAULT),

    AP_GROUPEND
};

AR_PivotTurn::AR_PivotTurn(AR_AttitudeControl& atc) :
    _atc(atc)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// 启用或禁用原地转向控制器
void AR_PivotTurn::enable(bool enable_pivot)
{
    _enabled = enable_pivot;
    if (!_enabled) {
        _active = false;
    }
}

// 最近是否调用过 update
bool AR_PivotTurn::active() const
{
    return _enabled && _active;
}

// 检查原地转向是否应被激活或停用
// 若调用者希望无论航向误差如何都触发原地转向开始，force_active 应为 true
void AR_PivotTurn::check_activation(float desired_heading_deg, bool force_active)
{
    // 检查明确无法使用原地转向的情况
    if (!_enabled || (_angle <= AR_PIVOT_ANGLE_ACCURACY)) {
        _active = false;
        return;
    }

    // 计算偏航误差（度）
    const float yaw_error = fabsf(wrap_180(desired_heading_deg - (AP::ahrs().get_yaw_deg())));

    // 若误差大于 _pivot_angle 则开始原地转向
    if (yaw_error > _angle || force_active) {
        _active = true;
        _delay_start_ms = 0;
        return;
    }

    uint32_t now_ms = AP_HAL::millis();

    // 若距目标航向 5 度以内，设置原地转向开始时间
    if (_active && (yaw_error < AR_PIVOT_ANGLE_ACCURACY) && (_delay_start_ms == 0)) {
        _delay_start_ms = now_ms;
    }

    // pivot_delay 设定的时间过去后退出原地转向
    if ((_delay_start_ms > 0) &&
        (now_ms - _delay_start_ms) >= get_delay_duration_ms()) {
        _active = false;
        _delay_start_ms = 0;
    }
}

// 检查给定预期偏航变化（度）时原地转向是否会被激活
// 注意：此函数不会真正激活原地转向。要激活请使用 check_activation 方法
bool AR_PivotTurn::would_activate(float yaw_change_deg) const
{
    // 检查明确无法使用原地转向的情况
    if (!_enabled || (_angle <= AR_PIVOT_ANGLE_ACCURACY)) {
        return false;
    }

    // 若偏航变化大于 _pivot_angle 则返回 true
    return fabsf(wrap_180(yaw_change_deg)) > _angle;
}

// 获取转向率（rad/s）
// desired heading 应为朝向下一个航点的航向（度）
// dt 应为距上次调用的时间（秒）
float AR_PivotTurn::get_turn_rate_rads(float desired_heading_deg, float dt)
{
    // 处理原地转向
    const float desired_turn_rate_rads = _atc.get_turn_rate_from_heading(radians(desired_heading_deg), radians(_rate_max));

    // 更新标志以便被清除
    check_activation(desired_heading_deg);

    return desired_turn_rate_rads;
}

// 返回转向后延迟时长（毫秒）
uint32_t AR_PivotTurn::get_delay_duration_ms() const
{
    return constrain_float(_delay.get(), 0.0f, 60.0f) * 1000;
}
