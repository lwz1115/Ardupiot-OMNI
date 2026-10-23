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

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsUGV.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Scheduler/AP_Scheduler.h>

#define SERVO_MAX 4500  // 该值代表 45 度，仅为舵机最大行程的任意表示。

extern const AP_HAL::HAL& hal;

// 单例实例
AP_MotorsUGV *AP_MotorsUGV::_singleton;

// 电机类参数
const AP_Param::GroupInfo AP_MotorsUGV::var_info[] = {
    // @Param: PWM_TYPE
    // @DisplayName: 电机输出 PWM 类型
    // @Description: 选择输出 PWM 类型，包括常规 PWM、OneShot、使用 PWM（占空比）和分离方向信号的有刷电机支持、带独立油门和方向 PWM（占空比）的有刷电机支持
    // @Values: 0:正常,1:OneShot,2:OneShot125,3:带继电器有刷,4:双极有刷,5:DShot150,6:DShot300,7:DShot600,8:DShot1200
    // @User: 高级
    // @RebootRequired: True
    AP_GROUPINFO("PWM_TYPE", 1, AP_MotorsUGV, _pwm_type, PWMType::NORMAL),

    // @Param: PWM_FREQ
    // @DisplayName: 有刷电机输出 PWM 频率
    // @Description: 有刷电机输出 PWM 频率
    // @Units: kHz
    // @Range: 1 20
    // @Increment: 1
    // @User: 高级
    // @RebootRequired: True
    AP_GROUPINFO("PWM_FREQ", 2, AP_MotorsUGV, _pwm_freq, 16),

    // @Param: SAFE_DISARM
    // @DisplayName: 解锁（disarmed）时禁用电机 PWM 输出
    // @Description: 解锁时禁用电机 PWM 输出
    // @Values: 0:解锁时启用 PWM, 1:解锁时禁用 PWM
    // @User: 高级
    AP_GROUPINFO("SAFE_DISARM", 3, AP_MotorsUGV, _disarm_disable_pwm, 0),

    // @Param: THR_MIN
    // @DisplayName: 油门最小值
    // @Description: 自动驾驶仪将应用的最小油门百分比。用于处理低油门附近的死区，以及防止内燃机在任务中熄火。必须小于 MOT_THR_MAX。
    // @Units: %
    // @Range: 0 20
    // @Increment: 1
    // @User: 标准
    AP_GROUPINFO("THR_MIN", 5, AP_MotorsUGV, _throttle_min, 0),

    // @Param: THR_MAX
    // @DisplayName: 油门最大值
    // @Description: 自动驾驶仪将应用的最大油门百分比。可用于防止电动地面车上的电调或电机过热
    // @Units: %
    // @Range: 5 100
    // @Increment: 1
    // @User: 标准
    AP_GROUPINFO("THR_MAX", 6, AP_MotorsUGV, _throttle_max, 100),

    // @Param: SLEWRATE
    // @DisplayName: 油门转换速率
    // @Description: 油门转换速率，以每秒占总范围的百分比表示。值 100 允许电机在一秒内完成全范围变化。值为零禁用限制。注意某些镍氢电池供电的地面车需要较低设置 40 以减少电流需求，避免电压跌落。
    // @Units: %/s
    // @Range: 0 1000
    // @Increment: 1
    // @User: 标准
    AP_GROUPINFO("SLEWRATE", 8, AP_MotorsUGV, _slew_rate, 100),

    // @Param: THST_EXPO
    // @DisplayName: 推力曲线指数
    // @Description: 推力曲线指数（-1 到 +1，0 为线性）
    // @Range: -1.0 1.0
    // @User: 高级
    AP_GROUPINFO("THST_EXPO", 9, AP_MotorsUGV, _thrust_curve_expo, 0.0f),

    // 10 曾是 VEC_THR_BASE

    // @Param: SPD_SCA_BASE
    // @DisplayName: 电机速度缩放基础速度
    // @Description: 常规转向/油门车辆中，超过此速度时转向会被缩放降低。零则禁用速度缩放
    // @Units: m/s
    // @Range: 0 10
    // @User: 高级
    AP_GROUPINFO("SPD_SCA_BASE", 11, AP_MotorsUGV, _speed_scale_base, 1.0f),

    // @Param: STR_THR_MIX
    // @DisplayName: 电机转向与油门优先级
    // @Description: 转向与油门优先级。数值越高越优先转向，越低越优先油门。仅对滑移转向车辆有效
    // @Range: 0.2 1.0
    // @User: 高级
    AP_GROUPINFO("STR_THR_MIX", 12, AP_MotorsUGV, _steering_throttle_mix, 0.5f),

    // @Param: VEC_ANGLEMAX
    // @DisplayName: 矢量推力最大角度
    // @Description: 使用矢量推力时，转向中间位置与最大位置之间的角度（仅限船）
    // @Units: deg
    // @Range: 0 90
    // @User: 标准
    AP_GROUPINFO("VEC_ANGLEMAX", 13, AP_MotorsUGV, _vector_angle_max, 0.0f),

    // @Param: THST_ASYM
    // @DisplayName: 电机推力非对称
    // @Description: 推力非对称。用于滑移转向。2.0 表示电机前进速度是后退速度的两倍。
    // @Range: 1.0 10.0
    // @User: 高级
    AP_GROUPINFO("THST_ASYM", 14, AP_MotorsUGV, _thrust_asymmetry, 1.0f),

    // @Param: REV_DELAY
    // @DisplayName: 电机倒转延迟
    // @Description: 对于需要延迟才能改变方向的可逆电机。当大于零时，在电机方向改变后，油门将先归零该时长，再输出新的油门。
    // @Units: s
    // @Range: 0.1 1.0
    // @Increment: 0.1
    // @User: 标准
    AP_GROUPINFO("REV_DELAY", 15, AP_MotorsUGV, _reverse_delay, 0),

#if AP_BATTERY_WATT_MAX_ENABLED
    // @Param: BAT_WATT_TC
    // @DisplayName: 电池功率限制时间常数
    // @Description: 用于限制平稳功率节流的时间常数。
    // @Range: 0 10
    // @Units: s
    // @User: 高级
    AP_GROUPINFO("BAT_WATT_TC", 16, AP_MotorsUGV, _batt_power_time_constant, 2.0f),
#endif

    AP_GROUPEND
};

AP_MotorsUGV::AP_MotorsUGV(AP_WheelRateControl& rate_controller) :
    _rate_controller(rate_controller)
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void AP_MotorsUGV::init(uint8_t frtype)
{
    _frame_type = frame_type(frtype);

    // 为全向车辆进行设置
    if (_frame_type != FRAME_TYPE_UNDEFINED) {
        setup_omni();
    }
    
    // 设置舵机输出
    setup_servo_output();

    // 设置 PWM 类型
    setup_pwm_type();

    // 设置安全输出
    setup_safety_output();

}

bool AP_MotorsUGV::get_legacy_relay_index(int8_t &index1, int8_t &index2, int8_t &index3, int8_t &index4) const
{
    if (_pwm_type != PWMType::BRUSHED_WITH_RELAY) {
        // 仅当 PWM 类型设置为带继电器的有刷时使用继电器
        return false;
    }

    // 第一个继电器始终使用，用于油门、左油门或电机 1
    index1 = 0;

    // 第二个继电器用于滑移转向的右油门和全向的电机 2
    if (have_skid_steering()) {
        index2 = 1;
    }

    // 全向可以有可变数量的电机
    if (is_omni()) {
        // 全向至少有 3 个电机
        index2 = 2;
        if (_motors_num >= 4) {
            index2 = 3;
        }
    }

    return true;
}

// 主 CPU 故障时设置安全输出
void AP_MotorsUGV::setup_safety_output()
{
    if (_pwm_type == PWMType::BRUSHED_WITH_RELAY) {
        // 将 trim 设为 min，以将占空比范围（0 - 100%）映射到舵机范围
        // 忽略舵机反向标志，它由继电器使用
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttle, true);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleLeft, true);
        SRV_Channels::set_trim_to_min_for(SRV_Channel::k_throttleRight, true);
    }

    // 主 CPU 故障时停止发送 PWM
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::ZERO_PWM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::ZERO_PWM);
}

// 设置舵机输出范围
void AP_MotorsUGV::setup_servo_output()
{
    // k_steering 限制在 -45 到 45 度
    SRV_Channels::set_angle(SRV_Channel::k_steering, SERVO_MAX);

    // k_throttle 以功率百分比表示，即 -100 ... 100
    SRV_Channels::set_angle(SRV_Channel::k_throttle, 100);

    // 滑移转向左/右油门，值为 -1000 到 1000
    SRV_Channels::set_angle(SRV_Channel::k_throttleLeft,  1000);
    SRV_Channels::set_angle(SRV_Channel::k_throttleRight, 1000);

    // 全向电机以功率百分比表示，即 -100 ... 100
    for (uint8_t i=0; i<AP_MOTORS_NUM_MOTORS_MAX; i++) {
        SRV_Channel::Function function = SRV_Channels::get_motor_function(i);
        SRV_Channels::set_angle(function, 100);
    }

    // 主帆范围 0 到 100
    SRV_Channels::set_range(SRV_Channel::k_mainsail_sheet, 100);
    // 翼帆 -100 到 100
    SRV_Channels::set_angle(SRV_Channel::k_wingsail_elevator, 100);
    // 桅杆旋转 -100 到 100
    SRV_Channels::set_angle(SRV_Channel::k_mast_rotation, 100);

}

// 设置转向值（范围 -4500 到 +4500）
//   手动模式下 apply_scaling 应设置为 false，
//   此时不进行速度或角度缩放
void AP_MotorsUGV::set_steering(float steering, bool apply_scaling)
{
    _steering = steering;
    _scale_steering = apply_scaling;
}

// 设置油门值（范围 -100 到 100）
void AP_MotorsUGV::set_throttle(float throttle)
{
    // 仅在已解锁（armed）时允许设置油门
    if (!hal.util->get_soft_armed()) {
        return;
    }

    // 检查油门在 -_throttle_max 和 +_throttle_max 之间
    _throttle = constrain_float(throttle, -_throttle_max, _throttle_max);
}

// 设置横向输入值（范围 -100 到 +100）
void AP_MotorsUGV::set_lateral(float lateral)
{
    _lateral = constrain_float(lateral, -100.0f, 100.0f);
}

// 设置滚转输入值（范围 -1 到 +1）
void AP_MotorsUGV::set_roll(float roll)
{
    _roll = constrain_float(roll, -1.0f, 1.0f);
}

// 设置俯仰输入值（范围 -1 到 +1）
void AP_MotorsUGV::set_pitch(float pitch)
{
    _pitch = constrain_float(pitch, -1.0f, 1.0f);
}

// 设置行走高度输入值（范围 -1 到 +1）
void AP_MotorsUGV::set_walking_height(float walking_height)
{
    _walking_height = constrain_float(walking_height, -1.0f, 1.0f);
}

// 设置主帆输入值（范围 0 到 100）
void AP_MotorsUGV::set_mainsail(float mainsail)
{
    _mainsail = constrain_float(mainsail, 0.0f, 100.0f);
}

// 设置翼帆输入值（范围 -100 到 100）
void AP_MotorsUGV::set_wingsail(float wingsail)
{
    _wingsail = constrain_float(wingsail, -100.0f, 100.0f);
}

// 设置桅杆旋转输入值（范围 -100 到 100）
void AP_MotorsUGV::set_mast_rotation(float mast_rotation)
{
    _mast_rotation = constrain_float(mast_rotation, -100.0f, 100.0f);
}

// 获取受转换速率限制的油门
// 手动模式使用，以避免从前进到倒车过渡期间转向性能不佳
// 与私有 slew_limit_throttle 方法相同（见下），但不更新油门状态
float AP_MotorsUGV::get_slew_limited_throttle(float throttle, float dt) const
{
    if (_slew_rate <= 0) {
        return throttle;
    }

    const float throttle_change_max = static_cast<float>(_slew_rate) * dt;
    return constrain_float(throttle, _throttle_prev - throttle_change_max, _throttle_prev + throttle_change_max);
}

/*
  判断滑移转向是否可用
 */
bool AP_MotorsUGV::have_skid_steering() const
{
    return (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) && SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) || is_omni();
}

// 车辆是否有主帆
bool AP_MotorsUGV::has_sail() const
{
    return SRV_Channels::function_assigned(SRV_Channel::k_mainsail_sheet) || SRV_Channels::function_assigned(SRV_Channel::k_wingsail_elevator) || SRV_Channels::function_assigned(SRV_Channel::k_mast_rotation);
}

void AP_MotorsUGV::output(bool armed, float ground_speed, float dt)
{
    // 软解锁状态覆盖传入的 armed 状态
    if (!hal.util->get_soft_armed()) {
        armed = false;
        _throttle = 0.0f;
    }

    // 清除限制标志
    // output_ 方法负责在每次迭代中按需将它们设置为 true
    limit.steer_left = limit.steer_right = limit.throttle_lower = limit.throttle_upper = false;

    // 参数合理性检查
    sanity_check_parameters();

    // 油门转换速率限制
    slew_limit_throttle(dt);

    // 横向转换速率限制（复用 MOT_SLEWRATE，使全向船
    // 不会在高频率下猛击横向通道）
    slew_limit_lateral(dt);

    // 对油门应用功率限制
#if AP_BATTERY_WATT_MAX_ENABLED
    power_limit_throttle(dt);
#endif

    // 常规转向/油门机架输出
    output_regular(armed, ground_speed, _steering, _throttle);

    // 滑移转向机架输出
    output_skid_steering(armed, _steering, _throttle, dt);

    // 全向机架输出
    output_omni(armed, _steering, _throttle, _lateral);

    // 帆输出
    output_sail();

    // 将值发送到 PWM 定时器进行输出
    auto &srv = AP::srv();
    SRV_Channels::calc_pwm();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();
}

// 以总量的百分比（范围 -100 到 +100）测试转向或油门输出
// 用于响应 DO_MOTOR_TEST MAVLink 命令
bool AP_MotorsUGV::output_test_pct(motor_test_order motor_seq, float pct)
{
    // 检查 motor_seq 是否有效
    if (motor_seq >= MOTOR_TEST_LAST) {
        return false;
    }
    pct = constrain_float(pct, -100.0f, 100.0f);

    switch (motor_seq) {
        case MOTOR_TEST_THROTTLE: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor1)) {
                output_throttle(SRV_Channel::k_motor1, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
                output_throttle(SRV_Channel::k_throttle, pct);
            }
            break;
        }
        case MOTOR_TEST_STEERING: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor2)) {
                output_throttle(SRV_Channel::k_motor2, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_steering, pct * 45.0f);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor3)) {
                output_throttle(SRV_Channel::k_motor3, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft)) {
                output_throttle(SRV_Channel::k_throttleLeft, pct);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor4)) {
                output_throttle(SRV_Channel::k_motor4, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
                output_throttle(SRV_Channel::k_throttleRight, pct);
            }
            break;
        }
        case MOTOR_TEST_MAINSAIL: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_mainsail_sheet)) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_mainsail_sheet, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_wingsail_elevator)) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_wingsail_elevator, pct);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_mast_rotation)) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_mast_rotation, pct);
            }
            break;
        }
        case MOTOR_TEST_LAST:
            return false;
    }
    auto &srv = AP::srv();
    SRV_Channels::calc_pwm();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();
    return true;
}

// 使用 PWM 值测试转向或油门输出
bool AP_MotorsUGV::output_test_pwm(motor_test_order motor_seq, float pwm)
{
    // 检查 motor_seq 是否有效
    if (motor_seq > MOTOR_TEST_THROTTLE_RIGHT) {
        return false;
    }
    switch (motor_seq) {
        case MOTOR_TEST_THROTTLE: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor1)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttle)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, pwm);
            }
            break;
        }
        case MOTOR_TEST_STEERING: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor2)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_steering, pwm);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_LEFT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor3)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            }
            break;
        }
        case MOTOR_TEST_THROTTLE_RIGHT: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_motor4)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_throttleRight)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            }
            break;
        }
        case MOTOR_TEST_MAINSAIL: {
            if (SRV_Channels::function_assigned(SRV_Channel::k_mainsail_sheet)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_mainsail_sheet, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_wingsail_elevator)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_wingsail_elevator, pwm);
            }
            if (SRV_Channels::function_assigned(SRV_Channel::k_mast_rotation)) {
                SRV_Channels::set_output_pwm(SRV_Channel::k_mast_rotation, pwm);
            }
            break;
        }
        default:
            return false;
    }
    auto &srv = AP::srv();
    SRV_Channels::calc_pwm();
    srv.cork();
    SRV_Channels::output_ch_all();
    srv.push();
    return true;
}

// 检查通过返回 true，失败返回 false。report 为 true 时应向地面站发送文本消息
bool AP_MotorsUGV::pre_arm_check(bool report) const
{
    const bool have_throttle = SRV_Channels::function_assigned(SRV_Channel::k_throttle);
    const bool have_throttle_left = SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft);
    const bool have_throttle_right = SRV_Channels::function_assigned(SRV_Channel::k_throttleRight);

    // 检查是否定义了输出，包括脚本和帆
    if(!have_throttle_left &&
       !have_throttle_right &&
       !have_throttle &&
       !SRV_Channels::function_assigned(SRV_Channel::k_steering) &&
       !SRV_Channels::function_assigned(SRV_Channel::k_scripting1) &&
       !has_sail() &&
       !is_omni()) {
        if (report) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: no motor, sail or scripting outputs defined");
        }
        return false;
    }
    // 检查是否仅配置了滑移转向输出的一个
    if (have_throttle_left != have_throttle_right) {
        if (report) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: check skid steering config");
        }
        return false;
    }
    // 检查是否仅配置了油门或转向输出中的一个；若有帆则允许无油门
    if ((has_sail() || have_throttle) != SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
        if (report) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: check steering and throttle config");
        }
        return false;
    }
    // 检查所有全向电机输出是否已配置
    for (uint8_t i=0; i<_motors_num; i++) {
        SRV_Channel::Function function = SRV_Channels::get_motor_function(i);
        if (!SRV_Channels::function_assigned(function)) {
            if (report) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: servo function %u unassigned", function);
            }
            return false;
        }
    }

    // 检查带继电器的有刷输出是否配置了继电器
#if AP_RELAY_ENABLED
    AP_Relay*relay = AP::relay();
    if ((_pwm_type == PWMType::BRUSHED_WITH_RELAY) && (relay != nullptr)) {
        // 若配置了输出，其继电器必须启用
        struct RelayTable {
            bool output_assigned;
            AP_Relay_Params::FUNCTION fun;
        };

        const RelayTable relay_table[] = {
            { have_throttle || have_throttle_left || (SRV_Channels::function_assigned(SRV_Channel::k_motor1) && (_motors_num >= 1)), AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_1 },
            { have_throttle_right || (SRV_Channels::function_assigned(SRV_Channel::k_motor2) && (_motors_num >= 2)),                 AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_2 },
            { SRV_Channels::function_assigned(SRV_Channel::k_motor3) && (_motors_num >= 3),                                          AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_3 },
            { SRV_Channels::function_assigned(SRV_Channel::k_motor4) && (_motors_num >= 4),                                          AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_4 },
        };

        for (uint8_t i=0; i<ARRAY_SIZE(relay_table); i++) {
            if (relay_table[i].output_assigned && !relay->enabled(relay_table[i].fun)) {
                if (report) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "PreArm: relay function %u unassigned", uint8_t(relay_table[i].fun));
                }
                return false;
            }
        }
    }
#endif

    return true;
}

// 参数合理性检查
void AP_MotorsUGV::sanity_check_parameters()
{
    _throttle_max.set(constrain_int16(_throttle_max, 5, 100));
    _throttle_min.set(constrain_int16(_throttle_min, 0, MIN(20, _throttle_max)));
    _vector_angle_max.set(constrain_float(_vector_angle_max, 0.0f, 90.0f));
}

// 设置 PWM 输出类型
void AP_MotorsUGV::setup_pwm_type()
{
    _motor_mask = 0;

    hal.rcout->set_dshot_esc_type(SRV_Channels::get_dshot_esc_type());

    // 计算分配给电机的通道掩码
    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_throttle);
    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_throttleLeft);
    _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channel::k_throttleRight);
    for (uint8_t i=0; i<_motors_num; i++) {
        _motor_mask |= SRV_Channels::get_output_channel_mask(SRV_Channels::get_motor_function(i));
    }

    switch (_pwm_type) {
    case PWMType::ONESHOT:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        break;
    case PWMType::ONESHOT125:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT125);
        break;
    case PWMType::BRUSHED_WITH_RELAY:
    case PWMType::BRUSHED_BIPOLAR:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        hal.rcout->set_freq(_motor_mask, uint16_t(_pwm_freq * 1000));
        break;
    case PWMType::DSHOT150:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT150);
        break;
    case PWMType::DSHOT300:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT300);
        break;
    case PWMType::DSHOT600:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT600);
        break;
    case PWMType::DSHOT1200:
        hal.rcout->set_output_mode(_motor_mask, AP_HAL::RCOutput::MODE_PWM_DSHOT1200);
        break;
    default:
        // 不做任何操作
        break;
    }
}

// 为带全向电机的机架进行设置
void AP_MotorsUGV::setup_omni()
{
    // 移除现有电机
    for (int8_t i=0; i<AP_MOTORS_NUM_MOTORS_MAX; i++) {
        clear_omni_motors(i);
    }

    // 硬编码的因子配置
    switch (_frame_type) {

    //   机架类型名称
    case FRAME_TYPE_UNDEFINED:
        break;

    case FRAME_TYPE_OMNI3:
        _motors_num = 3;
        add_omni_motor(0, 1.0f, -1.0f, -1.0f);
        add_omni_motor(1, 0.0f, -1.0f, 1.0f);
        add_omni_motor(2, 1.0f, 1.0f, 1.0f);
        break;

    case FRAME_TYPE_OMNIX:
        // 物理接线（用户实测）：SERVO1=右前, SERVO2=右后, SERVO3=左后, SERVO4=左前
        // 电机编号 = 通道号（get_motor_function: 0->k_motor1=SERVO1 ... 3->k_motor4=SERVO4）
        // 系数与 SIM_Rover.cpp 的 OmniX 逆混控保持一致：正油门=前进, 正转向=右转, 正横向=右移
        _motors_num = 4,
        add_omni_motor(0, 1.0f, -1.0f, -1.0f);  // 右前 (SERVO1)
        add_omni_motor(1, 1.0f,  1.0f, -1.0f);  // 右后 (SERVO2)
        add_omni_motor(2, 1.0f,  1.0f,  1.0f);  // 左后 (SERVO3)
        add_omni_motor(3, 1.0f, -1.0f,  1.0f);  // 左前 (SERVO4)
        break;

    case FRAME_TYPE_OMNIPLUS:
        _motors_num = 4;
        add_omni_motor(0, 0.0f, 1.0f, 1.0f);
        add_omni_motor(1, 1.0f, 0.0f, 0.0f);
        add_omni_motor(2, 0.0f, -1.0f, 1.0f);
        add_omni_motor(3, 1.0f, 0.0f, 0.0f);
        break;

    case FRAME_TYPE_OMNI3MECANUM:
        _motors_num = 3;
        add_omni_motor(0,  -1.0f,    1.0f,  -0.26795f);
        add_omni_motor(1,  0.73205f, 1.0f,  -0.73205f);
        add_omni_motor(2,  0.26795f, 1.0f,   1.0f);
        break;
    }
}

// 使用独立的油门、转向和横向因子添加全向电机
void AP_MotorsUGV::add_omni_motor(int8_t motor_num, float throttle_factor, float steering_factor, float lateral_factor)
{
    // 确保提供有效的电机编号
    if (motor_num >= 0 && motor_num < AP_MOTORS_NUM_MOTORS_MAX) {

        // 设置油门、转向和横向因子
        _throttle_factor[motor_num] = throttle_factor;
        _steering_factor[motor_num] = steering_factor;
        _lateral_factor[motor_num] = lateral_factor;

        add_omni_motor_num(motor_num);
    }
}

// 添加全向电机并设置默认输出功能
void AP_MotorsUGV::add_omni_motor_num(int8_t motor_num)
{
    // 确保提供有效的电机编号
    if (motor_num >= 0 && motor_num < AP_MOTORS_NUM_MOTORS_MAX) {
        uint8_t chan;
        SRV_Channel::Function function = SRV_Channels::get_motor_function(motor_num);
        SRV_Channels::set_aux_channel_default(function, motor_num);
        if (!SRV_Channels::find_channel(function, chan)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Motors: unable to setup motor %u", motor_num);
        }
    }
}

// 禁用全向电机并移除该电机的所有油门、转向和横向因子
void AP_MotorsUGV::clear_omni_motors(int8_t motor_num)
{
    // 确保提供有效的电机编号
    if (motor_num >= 0 && motor_num < AP_MOTORS_NUM_MOTORS_MAX) {
        // 禁用电机并将因子设为零
        _throttle_factor[motor_num] = 0;
        _steering_factor[motor_num] = 0;
        _lateral_factor[motor_num] = 0;
    }
}

// 输出到常规转向和油门通道
void AP_MotorsUGV::output_regular(bool armed, float ground_speed, float steering, float throttle)
{
    // 输出到油门通道
    if (armed) {
        if (_scale_steering) {
            // 矢量推力处理
            if (have_vectored_thrust()) {

                // 归一化期望转向和油门以简化计算
                float steering_norm = steering / 4500.0f;
                const float throttle_norm = throttle * 0.01f;

                // 转向永不能超过 throttle * tan(_vector_angle_max)
                const float vector_angle_max_rad = radians(constrain_float(_vector_angle_max, 0.0f, 90.0f));
                const float steering_norm_lim = fabsf(throttle_norm * tanf(vector_angle_max_rad));
                if (fabsf(steering_norm) > steering_norm_lim) {
                    if (is_positive(steering_norm)) {
                        steering_norm = steering_norm_lim;
                    }
                    if (is_negative(steering_norm)) {
                        steering_norm = -steering_norm_lim;
                    }
                    limit.steer_right = true;
                    limit.steer_left = true;
                }

                if (!is_zero(throttle_norm)) {
                    // 计算转向角
                    float steering_angle_rad = atanf(steering_norm / throttle_norm);
                    // 将转向角限制到 vector_angle_max
                    if (fabsf(steering_angle_rad) > vector_angle_max_rad) {
                        steering_angle_rad = constrain_float(steering_angle_rad, -vector_angle_max_rad, vector_angle_max_rad);
                        limit.steer_right = true;
                        limit.steer_left = true;
                     }

                    // 将转向角转换为转向输出
                    steering = steering_angle_rad / vector_angle_max_rad * 4500.0f;

                    // 放大油门以补偿转向角
                    const float throttle_scaler_inv = cosf(steering_angle_rad);
                    if (!is_zero(throttle_scaler_inv)) {
                        throttle /= throttle_scaler_inv;
                    }
                }
            } else {
                // 速度超过 MOT_SPD_SCA_BASE（默认 1 m/s）时按比例缩小转向
                if (is_positive(_speed_scale_base) && (fabsf(ground_speed) > _speed_scale_base)) {
                    steering *= (_speed_scale_base / fabsf(ground_speed));
                } else {
                    // 低速常规转向地面车设置限制，以阻止控制器中 I 项累积
                    if (!have_skid_steering()) {
                        limit.steer_left = true;
                        limit.steer_right = true;
                    }
                }
                // 倒车时反转转向方向
                if (is_negative(ground_speed)) {
                    steering *= -1.0f;
                }
            }
        } else {
            // 倒车时反转转向方向
            if (is_negative(throttle)) {
                steering *= -1.0f;
            }
        }
        output_throttle(SRV_Channel::k_throttle, throttle);
    } else {
        // 处理解锁（disarmed）情况
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
        }
    }

    // 基于输入清除并设置限制
    // 在此处执行是因为矢量推力或速度缩放可能已减小转向请求
    set_limits_from_input(armed, steering, throttle);

    // 限制转向
    steering = constrain_float(steering, -4500.0f, 4500.0f);

    // 始终允许转向移动
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering);
}

// 输出到滑移转向通道
void AP_MotorsUGV::output_skid_steering(bool armed, float steering, float throttle, float dt)
{
    if (!have_skid_steering()) {
        return;
    }

    // 基于输入清除并设置限制
    set_limits_from_input(armed, steering, throttle);

    // 限制转向
    steering = constrain_float(steering, -4500.0f, 4500.0f);

    // 处理较简单的解锁（disarmed）情况
    if (!armed) {
        if (_disarm_disable_pwm) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::ZERO_PWM);
        } else {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
        }
        return;
    }

    // 滑移转向混合器
    float steering_scaled = steering / 4500.0f; // 转向缩放 -1 到 +1
    float throttle_scaled = throttle * 0.01f;  // 油门缩放 -1 到 +1

    // 对推力非对称值进行整理，混合器假设前进推力始终大于倒车推力
    const float thrust_asymmetry = MAX(_thrust_asymmetry, 1.0);
    const float lower_throttle_limit = -1.0 / thrust_asymmetry;

    // 最大转向为上下限之间的中点
    const float best_steering_throttle = (1.0 + lower_throttle_limit) * 0.5;
    float steering_range;
    if (throttle_scaled < best_steering_throttle) {
        // 转向范围减小，因为混合器永远不会增大油门
        steering_range = MAX(throttle_scaled,0.0) - lower_throttle_limit;
    } else {
        // 可用全范围，油门始终可以降低到 best_steering_throttle
        steering_range = 1 - best_steering_throttle;
    }

    // 应用约束
    if (steering_scaled > steering_range) {
        limit.steer_right = true;
        steering_scaled = steering_range;
    } else if (steering_scaled < -steering_range) {
        limit.steer_left = true;
        steering_scaled = -steering_range;
    }
    if (throttle_scaled > 1.0) {
        limit.throttle_upper = true;
        throttle_scaled = 1.0;
    } else if (throttle_scaled < lower_throttle_limit) {
        limit.throttle_lower = true;
        throttle_scaled = lower_throttle_limit;
    }

    // 全油门或全转向现在都可容纳，检查两者是否能同时容纳
    const float max_output = throttle_scaled + fabsf(steering_scaled);
    const float min_output = throttle_scaled - fabsf(steering_scaled);

    // 检查饱和并按比例回缩油门和转向
    const float saturation_value = MAX(max_output, min_output / lower_throttle_limit);
    if (saturation_value > 1.0f) {
        // 保存缩放前的值，以便之后设置限制标志
        const float steering_scaled_orig = steering_scaled;
        const float throttle_scaled_orig = throttle_scaled;

        const float str_thr_mix = constrain_float(_steering_throttle_mix, 0.0f, 1.0f);
        const float fair_scaler = 1.0f / saturation_value;
        if (str_thr_mix >= 0.5f) {
            // 转向优先于油门
            steering_scaled *= linear_interpolate(fair_scaler, 1.0f, str_thr_mix, 0.5f, 1.0f);
            if (throttle_scaled >= best_steering_throttle) {
                // 受上限约束
                throttle_scaled = 1.0 - fabsf(steering_scaled);
            } else {
                // 受下限约束
                throttle_scaled = fabsf(steering_scaled) + lower_throttle_limit;
            }

        } else {
            // 油门优先于转向
            throttle_scaled *= linear_interpolate(fair_scaler, 1.0f, 0.5f - str_thr_mix, 0.0f, 0.5f);
            const float steering_sign = is_positive(steering_scaled) ? 1.0 : -1.0;
            if (throttle_scaled >= best_steering_throttle) {
                // 受上限约束
                steering_scaled = (1.0 - throttle_scaled) * steering_sign;
            } else {
                // 受下限约束
                steering_scaled = (throttle_scaled - lower_throttle_limit) * steering_sign;
            }
        }

        // 若转向或油门任一被减小，更新限制
        if (fabsf(steering_scaled) < fabsf(steering_scaled_orig)) {
            limit.steer_left |= is_negative(steering_scaled_orig);
            limit.steer_right |= is_positive(steering_scaled_orig);
        }
        if (fabsf(throttle_scaled) < fabsf(throttle_scaled_orig)) {
            limit.throttle_lower |= is_negative(throttle_scaled_orig);
            limit.throttle_upper |= is_positive(throttle_scaled_orig);
        }
    }

    // 加入油门和转向
    float motor_left = throttle_scaled + steering_scaled;
    float motor_right = throttle_scaled - steering_scaled;

    // 应用非对称修正
    if (is_negative(motor_right)) {
        motor_right *= thrust_asymmetry;
    }
    if (is_negative(motor_left)) {
        motor_left *= thrust_asymmetry;
    }

    // 向每个电机发送 PWM 值
    output_throttle(SRV_Channel::k_throttleLeft, 100.0f * motor_left, dt);
    output_throttle(SRV_Channel::k_throttleRight, 100.0f * motor_right, dt);
}

// 全向机架输出
void AP_MotorsUGV::output_omni(bool armed, float steering, float throttle, float lateral)
{
    // 车辆不是全向则立即退出
    if (!is_omni()) {
        return;
    }

    if (armed) {
        // 基于输入清除并设置限制
        set_limits_from_input(armed, steering, throttle);

        // 限制转向
        steering = constrain_float(steering, -4500.0f, 4500.0f);

        // 差速模式（REAR/FRONT）：只激活一对电机，用原生差速公式
        //   motor_left  = throttle + steering
        //   motor_right = throttle - steering
        // 其余电机输出 TRIM(1500) 保持停转，避免搅动 ADCP 附近水流。
        if (_motor_pair != MotorPair::ALL) {
            const float scaled_throttle = constrain_float(throttle * 0.01f, -1.0f, 1.0f);
            const float scaled_steering = constrain_float(steering / 4500.0f, -1.0f, 1.0f);
            const float motor_left  = constrain_float(scaled_throttle + scaled_steering, -1.0f, 1.0f);
            const float motor_right = constrain_float(scaled_throttle - scaled_steering, -1.0f, 1.0f);

            const uint8_t left_motor  = (_motor_pair == MotorPair::REAR) ? 2 : 3;   // 后左/前左
            const uint8_t right_motor = (_motor_pair == MotorPair::REAR) ? 1 : 0;   // 后右/前右

            for (uint8_t i=0; i<_motors_num; i++) {
                if (i == left_motor) {
                    output_throttle(SRV_Channels::get_motor_function(i), motor_left * 100.0f);
                } else if (i == right_motor) {
                    output_throttle(SRV_Channels::get_motor_function(i), motor_right * 100.0f);
                } else {
                    // 未激活电机停转（TRIM = 中位 PWM）
                    SRV_Channels::set_output_limit(SRV_Channels::get_motor_function(i), SRV_Channel::Limit::TRIM);
                }
            }
            return;
        }

        // 将油门、转向和横向输入缩放到 -1 到 1
        const float scaled_throttle = throttle * 0.01f;
        const float scaled_steering = steering / 4500.0f;
        const float scaled_lateral = lateral * 0.01f;

        // 按最大电机数而非运行时的电机数分配数组：
        // 变长数组不是标准 C++，且 setup_omni() 仅
        // 将 _motors_num 设置为 3 或 4
        float thr_str_ltr_out[AP_MOTORS_NUM_MOTORS_MAX] {};
        float thr_str_ltr_max = 1;
        for (uint8_t i=0; i<_motors_num; i++) {
            // 每个电机输出 油门 + 转向 + 横向
            thr_str_ltr_out[i] = (scaled_throttle * _throttle_factor[i]) +
                              (scaled_steering * _steering_factor[i]) +
                              (scaled_lateral * _lateral_factor[i]);
            // 记录高于 1 的最大输出
            if (fabsf(thr_str_ltr_out[i]) > thr_str_ltr_max) {
                thr_str_ltr_max = fabsf(thr_str_ltr_out[i]);
            }
        }
        // 将所有输出均匀缩放，使最大值适配
        const float output_scale = 1 / thr_str_ltr_max;
        for (uint8_t i=0; i<_motors_num; i++) {
            // 为每个电机发送输出
            output_throttle(SRV_Channels::get_motor_function(i), thr_str_ltr_out[i] * 100.0f * output_scale);
        }
        if (output_scale < 1.0) {
            // 无法确定是哪个指令导致了缩放回退，因此限制所有
            limit.steer_left = true;
            limit.steer_right = true;
            limit.throttle_lower = true;
            limit.throttle_upper = true;
        }
    } else {
        // 处理解锁（disarmed）情况
        if (_disarm_disable_pwm) {
            for (uint8_t i=0; i<_motors_num; i++) {
                SRV_Channels::set_output_limit(SRV_Channels::get_motor_function(i), SRV_Channel::Limit::ZERO_PWM);
            }
        } else {
            for (uint8_t i=0; i<_motors_num; i++) {
                SRV_Channels::set_output_limit(SRV_Channels::get_motor_function(i), SRV_Channel::Limit::TRIM);
            }
        }
    }
}

// return power_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
#if AP_BATTERY_WATT_MAX_ENABLED
float AP_MotorsUGV::get_power_limit_max_throttle(float dt)
{
    AP_BattMonitor &battery = AP::battery();

    float _batt_current;
    const float watt_max = battery.get_watt_max();
    const float _batt_voltage = battery.voltage();
    
    if (watt_max <= 0 ||  // return maximum if power limiting is disabled
        !hal.util->get_soft_armed() ||   // remove throttle limit if disarmed
        !battery.current_amps(_batt_current) || // no current monitoring is available
        is_zero(_batt_voltage)) {  // no voltage monitoring is available
        _throttle_limit = 1.0f;
        return 1.0f;
    }

    const float power = _batt_voltage * _batt_current;
    const float power_ratio = power / watt_max;

    _throttle_limit += (dt / (dt + _batt_power_time_constant)) * (1.0f - power_ratio);

    // ensure throttle limit is within min and max throttle limits
    _throttle_limit = constrain_float(_throttle_limit, _throttle_min * 0.01f, _throttle_max * 0.01f);

    return _throttle_limit;
}
#endif

// 向主油门通道、左油门或右油门输出油门值。throttle 应缩放为 -100 到 100
void AP_MotorsUGV::output_throttle(SRV_Channel::Function function, float throttle, float dt)
{
    // 对舵机功能做合理性检查
    if (function != SRV_Channel::k_throttle && function != SRV_Channel::k_throttleLeft && function != SRV_Channel::k_throttleRight && function != SRV_Channel::k_motor1 && function != SRV_Channel::k_motor2 && function != SRV_Channel::k_motor3 && function!= SRV_Channel::k_motor4) {
        return;
    }

    // 限制并缩放输出
    throttle = get_scaled_throttle(throttle);

    // 应用速率控制
    throttle = get_rate_controlled_throttle(function, throttle, dt);

    // 必要时设置继电器
#if AP_RELAY_ENABLED
    AP_Relay*relay = AP::relay();
    if ((_pwm_type == PWMType::BRUSHED_WITH_RELAY) && (relay != nullptr)) {

        // 查找输出通道，若未找到则返回
        const SRV_Channel *out_chan = SRV_Channels::get_channel_for(function);
        if (out_chan == nullptr) {
            return;
        }
        const int8_t reverse_multiplier = out_chan->get_reversed() ? -1 : 1;
        bool relay_high = is_negative(reverse_multiplier * throttle);

        AP_Relay_Params::FUNCTION relay_function;
        switch (function) {
            case SRV_Channel::k_throttle:
            case SRV_Channel::k_throttleLeft:
            case SRV_Channel::k_motor1:
            default:
                relay_function = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_1;
                break;
            case SRV_Channel::k_throttleRight:
            case SRV_Channel::k_motor2:
                relay_function = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_2;
                break;
            case SRV_Channel::k_motor3:
                relay_function = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_3;
                break;
            case SRV_Channel::k_motor4:
                relay_function = AP_Relay_Params::FUNCTION::BRUSHED_REVERSE_4;
                break;
        }
        relay->set(relay_function, relay_high);

        // 反转输出，使 calc_pwm 计算的始终为正值
        throttle = reverse_multiplier * fabsf(throttle);
    }
#endif  // AP_RELAY_ENABLED

    if (_reverse_delay > 0) {
        switch (function) {
        case SRV_Channel::k_throttle:
            rev_delay_throttle.output(function, throttle, _reverse_delay);
            return;
        case SRV_Channel::k_throttleLeft:
            rev_delay_throttleLeft.output(function, throttle * 10, _reverse_delay);
            return;
        case SRV_Channel::k_throttleRight:
            rev_delay_throttleRight.output(function, throttle * 10, _reverse_delay);
            return;
        default:
            // 落入其他无延迟输出
            break;
        }
    }

    // 输出到舵机通道
    switch (function) {
        case SRV_Channel::k_throttle:
        case SRV_Channel::k_motor1:
        case SRV_Channel::k_motor2:
        case SRV_Channel::k_motor3:
        case SRV_Channel::k_motor4:
            SRV_Channels::set_output_scaled(function,  throttle);
            break;
        case SRV_Channel::k_throttleLeft:
        case SRV_Channel::k_throttleRight:
            SRV_Channels::set_output_scaled(function,  throttle * 10.0f);
            break;
        default:
            // 不做任何操作
            break;
    }
}

// 输出帆船的帆
void AP_MotorsUGV::output_sail()
{
    if (!has_sail()) {
        return;
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_mainsail_sheet, _mainsail);
    SRV_Channels::set_output_scaled(SRV_Channel::k_wingsail_elevator, _wingsail);
    SRV_Channels::set_output_scaled(SRV_Channel::k_mast_rotation, _mast_rotation);
}

// 对油门进行一次迭代的转换速率限制
void AP_MotorsUGV::slew_limit_throttle(float dt)
{
    const float throttle_orig = _throttle;
    _throttle = get_slew_limited_throttle(_throttle, dt);
    if (throttle_orig > _throttle) {
        limit.throttle_upper = true;
    } else if (throttle_orig < _throttle) {
        limit.throttle_lower = true;
    }
    _throttle_prev = _throttle;
}

// 对横向进行一次迭代的转换速率限制。复用 MOT_SLEWRATE，使全向船
// 在横向通道上获得与油门相同的高频指令平滑。这防止推进器
// 来回猛击（并搅动水流），当船在导航中因水流/潮流而
// 航向漂移时尤为重要。
void AP_MotorsUGV::slew_limit_lateral(float dt)
{
    const float lateral_orig = _lateral;
    if (_slew_rate > 0) {
        const float lateral_change_max = static_cast<float>(_slew_rate) * dt;
        _lateral = constrain_float(_lateral,
                                   _lateral_prev - lateral_change_max,
                                   _lateral_prev + lateral_change_max);
    }
    if (lateral_orig > _lateral) {
        limit.throttle_upper = true;
    } else if (lateral_orig < _lateral) {
        limit.throttle_lower = true;
    }
    _lateral_prev = _lateral;
}

// 对油门进行一次迭代的功率限制
#if AP_BATTERY_WATT_MAX_ENABLED
void AP_MotorsUGV::power_limit_throttle(float dt)
{
    const float power_limit_max = get_power_limit_max_throttle(dt);
    
    const float throttle_max = 100.0f * power_limit_max;
    const float throttle_min = -100.0f * power_limit_max;
    
    if (_throttle > throttle_max) {
        _throttle = throttle_max;
        limit.throttle_upper = true;
    } else if (_throttle < throttle_min) {
        _throttle = throttle_min;
        limit.throttle_lower = true;
    }
}
#endif

// 基于转向和油门输入设置限制
void AP_MotorsUGV::set_limits_from_input(bool armed, float steering, float throttle)
{
    // 基于输入设置限制
    limit.steer_left |= !armed || (steering <= -4500.0f);
    limit.steer_right |= !armed || (steering >= 4500.0f);
    limit.throttle_lower |= !armed || (throttle <= -_throttle_max);
    limit.throttle_upper |= !armed || (throttle >= _throttle_max);
}

// 使用 _throttle_min 和 _thrust_curve_expo 参数缩放油门。throttle 应在 -100 到 +100 范围内
float AP_MotorsUGV::get_scaled_throttle(float throttle) const
{
    // 油门为零则立即退出
    if (is_zero(throttle)) {
        return throttle;
    }

    // 使用 throttle_min 缩放
    if (_throttle_min > 0) {
        if (is_negative(throttle)) {
            throttle = -_throttle_min + (throttle * ((100.0f - _throttle_min) * 0.01f));
        } else {
            throttle = _throttle_min + (throttle * ((100.0f - _throttle_min) * 0.01f));
        }
    }

    // 若推力曲线禁用或无效则跳过进一步缩放
    if (is_zero(_thrust_curve_expo) || (_thrust_curve_expo > 1.0f) || (_thrust_curve_expo < -1.0f)) {
        return throttle;
    }

    // 计算缩放器
    const float sign = (throttle < 0.0f) ? -1.0f : 1.0f;
    const float throttle_pct = constrain_float(throttle, -100.0f, 100.0f) * 0.01f;
    return 100.0f * sign * ((_thrust_curve_expo - 1.0f) + safe_sqrt((1.0f - _thrust_curve_expo) * (1.0f - _thrust_curve_expo) + 4.0f * _thrust_curve_expo * fabsf(throttle_pct))) / (2.0f * _thrust_curve_expo);
}

// 使用速率控制器实现期望油门
float AP_MotorsUGV::get_rate_controlled_throttle(SRV_Channel::Function function, float throttle, float dt)
{
    // 要求非零 dt
    if (!is_positive(dt)) {
        return throttle;
    }

    // 尝试对左油门进行速率控制
    if ((function == SRV_Channel::k_throttleLeft) && _rate_controller.enabled(0)) {
        return _rate_controller.get_rate_controlled_throttle(0, throttle, dt);
    }

    // 对右油门进行速率控制
    if ((function == SRV_Channel::k_throttleRight) && _rate_controller.enabled(1)) {
        return _rate_controller.get_rate_controlled_throttle(1, throttle, dt);
    }

    // 保持油门不变返回
    return throttle;
}

// 电机是否在运动，返回 true
bool AP_MotorsUGV::active() const
{
    // 若软解锁（soft disarmed），电机不运动
    if (!hal.util->get_soft_armed()) {
        return false;
    }

    // 检查油门是否激活
    if (!is_zero(get_throttle())) {
        return true;
    }

    // 滑移转向车辆转向时激活
    if (have_skid_steering() && !is_zero(get_steering())) {
        return true;
    }

    return false;
}

// 配置的 PWM 类型为数字且应有固定端点时返回 true
bool AP_MotorsUGV::is_digital_pwm_type() const
{
    switch (_pwm_type) {
    case PWMType::DSHOT150:
    case PWMType::DSHOT300:
    case PWMType::DSHOT600:
    case PWMType::DSHOT1200:
        return true;
    case PWMType::NORMAL:
    case PWMType::ONESHOT:
    case PWMType::ONESHOT125:
    case PWMType::BRUSHED_WITH_RELAY:
    case PWMType::BRUSHED_BIPOLAR:
        break;
    }
    return false;
}

/*
  处理油门倒转延迟
 */
void AP_MotorsUGV::ReverseThrottle::output(SRV_Channel::Function function, float throttle, float delay)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (is_zero(throttle)) {
        // 直接通过，不改变，不更新上次油门
    } else if (throttle * last_throttle < 0 &&
        now_ms - last_output_ms < delay*1000) {
        // 符号变化，添加暂停
        throttle = 0;
    } else {
        last_output_ms = now_ms;
        last_throttle = throttle;
    }
    SRV_Channels::set_output_scaled(function, throttle);
}

namespace AP {
    AP_MotorsUGV *motors_ugv()
    {
        return AP_MotorsUGV::get_singleton();
    }
}

