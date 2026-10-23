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
#include "AR_AttitudeControl.h"
#include <AP_GPS/AP_GPS.h>

// 姿态控制默认定义
#define AR_ATTCONTROL_STEER_ANG_P       2.00f
#define AR_ATTCONTROL_STEER_RATE_FF     0.20f
#define AR_ATTCONTROL_STEER_RATE_P      0.20f
#define AR_ATTCONTROL_STEER_RATE_I      0.20f
#define AR_ATTCONTROL_STEER_RATE_IMAX   1.00f
#define AR_ATTCONTROL_STEER_RATE_D      0.00f
#define AR_ATTCONTROL_STEER_RATE_FILT   10.00f
#define AR_ATTCONTROL_STEER_RATE_MAX    120.0f
#define AR_ATTCONTROL_STEER_ACCEL_MAX   120.0f
#define AR_ATTCONTROL_STEER_DECEL_MAX   0.0f
#define AR_ATTCONTROL_THR_SPEED_P       0.20f
#define AR_ATTCONTROL_THR_SPEED_I       0.20f
#define AR_ATTCONTROL_THR_SPEED_IMAX    1.00f
#define AR_ATTCONTROL_THR_SPEED_D       0.00f
#define AR_ATTCONTROL_THR_SPEED_FILT    10.00f
#define AR_ATTCONTROL_PITCH_THR_P       1.80f
#define AR_ATTCONTROL_PITCH_THR_I       1.50f
#define AR_ATTCONTROL_PITCH_THR_D       0.03f
#define AR_ATTCONTROL_PITCH_THR_IMAX    1.0f
#define AR_ATTCONTROL_PITCH_THR_FILT    10.0f
#define AR_ATTCONTROL_BAL_PITCH_FF      0.4f
#define AR_ATTCONTROL_PITCH_LIM_TC      0.5f        // 俯仰限制默认时间常数
#define AR_ATTCONTROL_PITCH_RELAX_RATIO 0.5f        // 俯仰限制放松速度比限制慢 2 倍
#define AR_ATTCONTROL_PITCH_LIM_THR_THRESH  0.60    // 油门超过 60% 时开始俯仰限制
#define AR_ATTCONTROL_DT                0.02f
#define AR_ATTCONTROL_TIMEOUT_MS        200
#define AR_ATTCONTROL_HEEL_SAIL_P       1.0f
#define AR_ATTCONTROL_HEEL_SAIL_I       0.1f
#define AR_ATTCONTROL_HEEL_SAIL_D       0.0f
#define AR_ATTCONTROL_HEEL_SAIL_IMAX    1.0f
#define AR_ATTCONTROL_HEEL_SAIL_FILT    10.0f
#define AR_ATTCONTROL_DT                0.02f

// 油门/速度控制最大加速度/减速度（米/秒）（_ACCEL_MAX 参数默认值）
#define AR_ATTCONTROL_THR_ACCEL_MAX     1.00f

// 最小速度（m/s）
#define AR_ATTCONTROL_STEER_SPEED_MIN   1.0f

// 车辆被认为已停止的速度（m/s）（_STOP_SPEED 参数默认值）
#define AR_ATTCONTROL_STOP_SPEED_DEFAULT    0.1f

extern const AP_HAL::HAL& hal;

AR_AttitudeControl *AR_AttitudeControl::_singleton;

const AP_Param::GroupInfo AR_AttitudeControl::var_info[] = {

    // @Param: _STR_RAT_P
    // @DisplayName: 转向控制速率 P 增益
    // @Description: 转向控制速率 P 增益。将转向率误差（弧度/秒）转换为转向控制输出（范围 -1 到 +1）
    // @Range: 0.000 2.000
    // @Increment: 0.001
    // @User: 标准

    // @Param: _STR_RAT_I
    // @DisplayName: 转向控制 I 增益
    // @Description: 转向控制 I 增益。修正期望转向率（rad/s）与实际转向率之间的长期误差
    // @Range: 0.000 2.000
    // @Increment: 0.001
    // @User: 标准

    // @Param: _STR_RAT_IMAX
    // @DisplayName: 转向控制 I 增益最大值
    // @Description: 转向控制 I 增益最大值。限制 I 项产生的转向输出（范围 -1 到 +1）
    // @Range: 0.000 1.000
    // @Increment: 0.01
    // @User: 标准

    // @Param: _STR_RAT_D
    // @DisplayName: 转向控制 D 增益
    // @Description: 转向控制 D 增益。补偿期望转向率与实际转向率之间的短期变化
    // @Range: 0.000 0.400
    // @Increment: 0.001
    // @User: 标准

    // @Param: _STR_RAT_FF
    // @DisplayName: 转向控制前馈
    // @Description: 转向控制前馈
    // @Range: 0.000 3.000
    // @Increment: 0.001
    // @User: 标准

    // @Param: _STR_RAT_FILT
    // @DisplayName: 转向控制滤波器频率
    // @Description: 转向控制输入滤波器。较低的值降低噪声但增加延迟。
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _STR_RAT_FLTT
    // @DisplayName: 转向控制目标滤波器频率（Hz）
    // @Description: 目标滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _STR_RAT_FLTE
    // @DisplayName: 转向控制误差滤波器频率（Hz）
    // @Description: 误差滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _STR_RAT_FLTD
    // @DisplayName: 转向控制微分项滤波器频率（Hz）
    // @Description: 微分滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _STR_RAT_SMAX
    // @DisplayName: 转向转换速率限制
    // @Description: 对 P 和 D 增益组合产生的转换速率设置上限。若速率反馈产生的控制作用幅值超过此值，则减小 D+P 增益以遵守限制。这限制了由过大增益引起的高频振荡幅值。考虑到负载效应，限制应设置为不超过执行器最大转换速率的 25%。注意：增益不会降低到标称值的 10% 以下。值为零将禁用此功能。
    // @Range: 0 200
    // @Increment: 0.5
    // @User: 高级

    // @Param: _STR_RAT_PDMX
    // @DisplayName: 转向控制 PD 和最大值
    // @Description: 转向控制 PD 和最大值。P 项和 D 项之和可以输出的最大值/最小值
    // @Range: 0.000 1.000
    // @Increment: 0.01

    // @Param: _STR_RAT_D_FF
    // @DisplayName: 转向控制微分前馈增益
    // @Description: FF D 增益，产生与目标变化率成比例的输出
    // @Range: 0 0.03
    // @Increment: 0.001
    // @User: 高级

    // @Param: _STR_RAT_NTF
    // @DisplayName: 转向控制目标陷波滤波器索引
    // @Description: 转向控制目标陷波滤波器索引，零禁用
    // @Range: 0 8
    // @User: 高级

    // @Param: _STR_RAT_NEF
    // @DisplayName: 转向控制误差陷波滤波器索引
    // @Description: 转向控制误差陷波滤波器索引，零禁用
    // @Range: 0 8
    // @User: 高级

    AP_SUBGROUPINFO(_steer_rate_pid, "_STR_RAT_", 1, AR_AttitudeControl, AC_PID),

    // @Param: _SPEED_P
    // @DisplayName: 速度控制 P 增益
    // @Description: 速度控制 P 增益。将期望速度（m/s）与实际速度之间的误差转换为电机输出（范围 -1 到 +1）
    // @Range: 0.010 2.000
    // @Increment: 0.01
    // @User: 标准

    // @Param: _SPEED_I
    // @DisplayName: 速度控制 I 增益
    // @Description: 速度控制 I 增益。修正期望速度（m/s）与实际速度之间的长期误差
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: 标准

    // @Param: _SPEED_IMAX
    // @DisplayName: 速度控制 I 增益最大值
    // @Description: 速度控制 I 增益最大值。限制 I 项产生的最大电机输出（范围 -1 到 +1）
    // @Range: 0.000 1.000
    // @Increment: 0.01
    // @User: 标准

    // @Param: _SPEED_D
    // @DisplayName: 速度控制 D 增益
    // @Description: 速度控制 D 增益。补偿期望速度与实际速度之间的短期变化
    // @Range: 0.000 0.400
    // @Increment: 0.001
    // @User: 标准

    // @Param: _SPEED_FF
    // @DisplayName: 速度控制前馈
    // @Description: 速度控制前馈
    // @Range: 0.000 0.500
    // @Increment: 0.001
    // @User: 标准

    // @Param: _SPEED_FILT
    // @DisplayName: 速度控制滤波器频率
    // @Description: 速度控制输入滤波器。较低的值降低噪声但增加延迟。
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _SPEED_FLTT
    // @DisplayName: 速度控制目标滤波器频率（Hz）
    // @Description: 目标滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _SPEED_FLTE
    // @DisplayName: 速度控制误差滤波器频率（Hz）
    // @Description: 误差滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _SPEED_FLTD
    // @DisplayName: 速度控制微分项滤波器频率（Hz）
    // @Description: 微分滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _SPEED_SMAX
    // @DisplayName: 速度控制转换速率限制
    // @Description: 对 P 和 D 增益组合产生的转换速率设置上限。若速率反馈产生的控制作用幅值超过此值，则减小 D+P 增益以遵守限制。这限制了由过大增益引起的高频振荡幅值。考虑到负载效应，限制应设置为不超过执行器最大转换速率的 25%。注意：增益不会降低到标称值的 10% 以下。值为零将禁用此功能。
    // @Range: 0 200
    // @Increment: 0.5
    // @User: 高级

    // @Param: _SPEED_PDMX
    // @DisplayName: 速度控制 PD 和最大值
    // @Description: 速度控制 PD 和最大值。P 项和 D 项之和可以输出的最大值/最小值
    // @Range: 0.000 1.000
    // @Increment: 0.01

    // @Param: _SPEED_D_FF
    // @DisplayName: 速度控制微分前馈增益
    // @Description: FF D 增益，产生与目标变化率成比例的输出
    // @Range: 0 0.03
    // @Increment: 0.001
    // @User: 高级

    // @Param: _SPEED_NTF
    // @DisplayName: 速度控制目标陷波滤波器索引
    // @Description: 速度控制目标陷波滤波器索引，零禁用
    // @Range: 0 8
    // @User: 高级

    // @Param: _SPEED_NEF
    // @DisplayName: 速度控制误差陷波滤波器索引
    // @Description: 速度控制误差陷波滤波器索引，零禁用
    // @Range: 0 8
    // @User: 高级

    AP_SUBGROUPINFO(_throttle_speed_pid, "_SPEED_", 2, AR_AttitudeControl, AC_PID),

    // @Param: _ACCEL_MAX
    // @DisplayName: 速度控制最大加速度（和减速度）（m/s/s）
    // @Description: 速度控制最大加速度（和减速度）（m/s/s）。0 表示禁用加速度限制
    // @Range: 0.0 10.0
    // @Increment: 0.1
    // @Units: m/s/s
    // @User: 标准
    AP_GROUPINFO("_ACCEL_MAX", 3, AR_AttitudeControl, _throttle_accel_max, AR_ATTCONTROL_THR_ACCEL_MAX),

    // @Param: _BRAKE
    // @DisplayName: 速度控制刹车使能/禁用
    // @Description: 速度控制刹车使能/禁用。允许向电机发送反向输出以减慢车辆。
    // @Values: 0:禁用,1:启用
    // @User: 标准
    AP_GROUPINFO("_BRAKE", 4, AR_AttitudeControl, _brake_enable, 1),

    // @Param: _STOP_SPEED
    // @DisplayName: 速度控制停车速度
    // @Description: 速度控制停车速度。车辆速度降至该值以下时电机输出归零
    // @Range: 0.00 0.50
    // @Increment: 0.01
    // @Units: m/s
    // @User: 标准
    AP_GROUPINFO("_STOP_SPEED", 5, AR_AttitudeControl, _stop_speed, AR_ATTCONTROL_STOP_SPEED_DEFAULT),

    // @Param: _STR_ANG_P
    // @DisplayName: 转向控制角度 P 增益
    // @Description: 转向控制角度 P 增益。将期望航向/偏航（弧度）与实际航向/偏航之间的误差转换为期望转向率（rad/s）
    // @Range: 1.000 10.000
    // @Increment: 0.1
    // @User: 标准
    AP_SUBGROUPINFO(_steer_angle_p, "_STR_ANG_", 6, AR_AttitudeControl, AC_P),

    // @Param: _STR_ACC_MAX
    // @DisplayName: 转向控制角加速度最大值
    // @Description: 转向控制角加速度最大值（deg/s/s）。0 表示禁用加速度限制
    // @Range: 0 1000
    // @Increment: 0.1
    // @Units: deg/s/s
    // @User: 标准
    AP_GROUPINFO("_STR_ACC_MAX", 7, AR_AttitudeControl, _steer_accel_max, AR_ATTCONTROL_STEER_ACCEL_MAX),

    // @Param: _STR_RAT_MAX
    // @DisplayName: 转向控制转速最大值
    // @Description: 转向控制转速最大值（deg/s）。0 表示移除速率限制
    // @Range: 0 1000
    // @Increment: 0.1
    // @Units: deg/s
    // @User: 标准
    AP_GROUPINFO("_STR_RAT_MAX", 8, AR_AttitudeControl, _steer_rate_max, AR_ATTCONTROL_STEER_RATE_MAX),

    // @Param: _DECEL_MAX
    // @DisplayName: 速度控制最大减速度（m/s/s）
    // @Description: 速度控制最大减速度（m/s/s）。0 表示减速使用 ATC_ACCEL_MAX
    // @Range: 0.0 10.0
    // @Increment: 0.1
    // @Units: m/s/s
    // @User: 标准
    AP_GROUPINFO("_DECEL_MAX", 9, AR_AttitudeControl, _throttle_decel_max, 0.00f),

    // @Param: _BAL_P
    // @DisplayName: 俯仰控制 P 增益
    // @Description: BalanceBots 的俯仰控制 P 增益。将期望俯仰（弧度）与实际俯仰之间的误差转换为电机输出（范围 -1 到 +1）
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: 标准

    // @Param: _BAL_I
    // @DisplayName: 俯仰控制 I 增益
    // @Description: BalanceBots 的俯仰控制 I 增益。修正期望俯仰（弧度）与实际俯仰之间的长期误差
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: 标准

    // @Param: _BAL_IMAX
    // @DisplayName: 俯仰控制 I 增益最大值
    // @Description: 俯仰控制 I 增益最大值。限制 I 项产生的最大电机输出（范围 -1 到 +1）
    // @Range: 0.000 1.000
    // @Increment: 0.01
    // @User: 标准

    // @Param: _BAL_D
    // @DisplayName: 俯仰控制 D 增益
    // @Description: 俯仰控制 D 增益。补偿期望俯仰与实际俯仰之间的短期变化
    // @Range: 0.000 0.100
    // @Increment: 0.001
    // @User: 标准

    // @Param: _BAL_FF
    // @DisplayName: 俯仰控制前馈
    // @Description: 俯仰控制前馈
    // @Range: 0.000 0.500
    // @Increment: 0.001
    // @User: 标准

    // @Param: _BAL_FILT
    // @DisplayName: 俯仰控制滤波器频率
    // @Description: 俯仰控制输入滤波器。较低的值降低噪声但增加延迟。
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _BAL_FLTT
    // @DisplayName: 俯仰控制目标滤波器频率（Hz）
    // @Description: 俯仰控制目标滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _BAL_FLTE
    // @DisplayName: 俯仰控制误差滤波器频率（Hz）
    // @Description: 俯仰控制误差滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _BAL_FLTD
    // @DisplayName: 俯仰控制微分项滤波器频率（Hz）
    // @Description: 俯仰控制微分滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _BAL_SMAX
    // @DisplayName: 俯仰控制转换速率限制
    // @Description: 对 P 和 D 增益组合产生的转换速率设置上限。若速率反馈产生的控制作用幅值超过此值，则减小 D+P 增益以遵守限制。这限制了由过大增益引起的高频振荡幅值。考虑到负载效应，限制应设置为不超过执行器最大转换速率的 25%。注意：增益不会降低到标称值的 10% 以下。值为零将禁用此功能。
    // @Range: 0 200
    // @Increment: 0.5
    // @User: 高级

    // @Param: _BAL_PDMX
    // @DisplayName: 俯仰控制 PD 和最大值
    // @Description: 俯仰控制 PD 和最大值。P 项和 D 项之和可以输出的最大值/最小值
    // @Range: 0.000 1.000
    // @Increment: 0.01

    // @Param: _BAL_D_FF
    // @DisplayName: 俯仰控制微分前馈增益
    // @Description: FF D 增益，产生与目标变化率成比例的输出
    // @Range: 0 0.03
    // @Increment: 0.001
    // @User: 高级

    // @Param: _BAL_NTF
    // @DisplayName: 俯仰控制目标陷波滤波器索引
    // @Description: 俯仰控制目标陷波滤波器索引，零禁用
    // @Range: 0 8
    // @User: 高级

    // @Param: _BAL_NEF
    // @DisplayName: 俯仰控制误差陷波滤波器索引
    // @Description: 俯仰控制误差陷波滤波器索引，零禁用
    // @Range: 0 8
    // @User: 高级

    AP_SUBGROUPINFO(_pitch_to_throttle_pid, "_BAL_", 10, AR_AttitudeControl, AC_PID),

    // @Param: _BAL_PIT_FF
    // @DisplayName: 当前俯仰角俯仰控制前馈
    // @Description: 当前俯仰角俯仰控制前馈
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: 标准
    AP_GROUPINFO("_BAL_PIT_FF", 11, AR_AttitudeControl, _pitch_to_throttle_ff, AR_ATTCONTROL_BAL_PITCH_FF),

    // @Param: _SAIL_P
    // @DisplayName: 帆横倾控制 P 增益
    // @Description: 帆船的帆横倾控制 P 增益。将期望横倾角（弧度）与实际横倾之间的误差转换为主帆输出（范围 -1 到 +1）
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: 标准

    // @Param: _SAIL_I
    // @DisplayName: 帆横倾控制 I 增益
    // @Description: 帆船的帆横倾控制 I 增益。修正期望横倾角（弧度）与实际横倾之间的长期误差
    // @Range: 0.000 2.000
    // @Increment: 0.01
    // @User: 标准

    // @Param: _SAIL_IMAX
    // @DisplayName: 帆横倾控制 I 增益最大值
    // @Description: 帆横倾控制 I 增益最大值。限制 I 项对主帆输出的最大贡献（范围 -1 到 +1）
    // @Range: 0.000 1.000
    // @Increment: 0.01
    // @User: 标准

    // @Param: _SAIL_D
    // @DisplayName: 帆横倾控制 D 增益
    // @Description: 帆横倾控制 D 增益。补偿期望横倾角与实际横倾之间的短期变化
    // @Range: 0.000 0.100
    // @Increment: 0.001
    // @User: 标准

    // @Param: _SAIL_FF
    // @DisplayName: 帆横倾控制前馈
    // @Description: 帆横倾控制前馈
    // @Range: 0.000 0.500
    // @Increment: 0.001
    // @User: 标准

    // @Param: _SAIL_FILT
    // @DisplayName: 帆横倾控制滤波器频率
    // @Description: 帆横倾控制输入滤波器。较低的值降低噪声但增加延迟。
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _SAIL_FLTT
    // @DisplayName: 帆横倾目标滤波器频率（Hz）
    // @Description: 目标滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _SAIL_FLTE
    // @DisplayName: 帆横倾误差滤波器频率（Hz）
    // @Description: 误差滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _SAIL_FLTD
    // @DisplayName: 帆横倾微分项滤波器频率（Hz）
    // @Description: 微分滤波器频率（Hz）
    // @Range: 0.000 100.000
    // @Increment: 0.1
    // @Units: Hz
    // @User: 标准

    // @Param: _SAIL_SMAX
    // @DisplayName: 帆横倾转换速率限制
    // @Description: 对 P 和 D 增益组合产生的转换速率设置上限。若速率反馈产生的控制作用幅值超过此值，则减小 D+P 增益以遵守限制。这限制了由过大增益引起的高频振荡幅值。考虑到负载效应，限制应设置为不超过执行器最大转换速率的 25%。注意：增益不会降低到标称值的 10% 以下。值为零将禁用此功能。
    // @Range: 0 200
    // @Increment: 0.5
    // @User: 高级

    // @Param: _SAIL_PDMX
    // @DisplayName: 帆横倾控制 PD 和最大值
    // @Description: 帆横倾控制 PD 和最大值。P 项和 D 项之和可以输出的最大值/最小值
    // @Range: 0.000 1.000
    // @Increment: 0.01

    // @Param: _SAIL_D_FF
    // @DisplayName: 帆横倾微分前馈增益
    // @Description: FF D 增益，产生与目标变化率成比例的输出
    // @Range: 0 0.03
    // @Increment: 0.001
    // @User: 高级

    // @Param: _SAIL_NTF
    // @DisplayName: 帆横倾目标陷波滤波器索引
    // @Description: 帆横倾目标陷波滤波器索引，零禁用
    // @Range: 0 8
    // @User: 高级

    // @Param: _SAIL_NEF
    // @DisplayName: 帆横倾误差陷波滤波器索引
    // @Description: 帆横倾误差陷波滤波器索引，零禁用
    // @Range: 0 8
    // @User: 高级

    AP_SUBGROUPINFO(_sailboat_heel_pid, "_SAIL_", 12, AR_AttitudeControl, AC_PID),

    // @Param: _TURN_MAX_G
    // @DisplayName: 转弯最大 G 力
    // @Description: 地面车在保持稳定时可承受的最大转弯加速度（重力单位）。导航代码将让横向加速度保持在该水平以下，以避免转弯时侧翻或车轮打滑
    // @Units: 重力单位
    // @Range: 0.1 10
    // @Increment: 0.01
    // @User: 标准
    AP_GROUPINFO("_TURN_MAX_G", 13, AR_AttitudeControl, _turn_lateral_G_max, 0.6f),

    // @Param: _BAL_LIM_TC
    // @DisplayName: 俯仰控制限制时间常数
    // @Description: 俯仰控制限制时间常数，用于防止翻倒。较低的值更快限制俯仰，较高的值更慢限制。设为 0 以禁用
    // @Range: 0.0 5.0
    // @Increment: 0.01
    // @User: 标准
    AP_GROUPINFO("_BAL_LIM_TC", 14, AR_AttitudeControl, _pitch_limit_tc, AR_ATTCONTROL_PITCH_LIM_TC),

    // @Param: _BAL_LIM_THR
    // @DisplayName: 俯仰控制限制油门阈值
    // @Description: 俯仰控制限制油门阈值。若油门越过此阈值（从 0 到 1），俯仰角将被限制
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: 标准
    AP_GROUPINFO("_BAL_LIM_THR", 15, AR_AttitudeControl, _pitch_limit_throttle_thresh, AR_ATTCONTROL_PITCH_LIM_THR_THRESH),

    // @Param: _STR_DEC_MAX
    // @DisplayName: 转向控制角减速度最大值
    // @Description: 转向控制角减速度最大值（deg/s/s）。0 表示禁用减速度限制
    // @Range: 0 1000
    // @Increment: 0.1
    // @Units: deg/s/s
    // @User: 标准
    AP_GROUPINFO("_STR_DEC_MAX", 16, AR_AttitudeControl, _steer_decel_max, AR_ATTCONTROL_STEER_DECEL_MAX),

    AP_GROUPEND
};

AR_AttitudeControl::AR_AttitudeControl() :
    _steer_angle_p(AR_ATTCONTROL_STEER_ANG_P),
    _steer_rate_pid(AR_ATTCONTROL_STEER_RATE_P, AR_ATTCONTROL_STEER_RATE_I, AR_ATTCONTROL_STEER_RATE_D, AR_ATTCONTROL_STEER_RATE_FF, AR_ATTCONTROL_STEER_RATE_IMAX, 0.0f, AR_ATTCONTROL_STEER_RATE_FILT, 0.0f),
    _throttle_speed_pid(AR_ATTCONTROL_THR_SPEED_P, AR_ATTCONTROL_THR_SPEED_I, AR_ATTCONTROL_THR_SPEED_D, 0.0f, AR_ATTCONTROL_THR_SPEED_IMAX, 0.0f, AR_ATTCONTROL_THR_SPEED_FILT, 0.0f),
    _pitch_to_throttle_pid(AR_ATTCONTROL_PITCH_THR_P, AR_ATTCONTROL_PITCH_THR_I, AR_ATTCONTROL_PITCH_THR_D, 0.0f, AR_ATTCONTROL_PITCH_THR_IMAX, 0.0f, AR_ATTCONTROL_PITCH_THR_FILT, 0.0f),
    _sailboat_heel_pid(AR_ATTCONTROL_HEEL_SAIL_P, AR_ATTCONTROL_HEEL_SAIL_I, AR_ATTCONTROL_HEEL_SAIL_D, 0.0f, AR_ATTCONTROL_HEEL_SAIL_IMAX, 0.0f, AR_ATTCONTROL_HEEL_SAIL_FILT, 0.0f)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// 根据期望横向加速度（m/s/s）返回 -1.0 到 +1.0 的转向舵机输出。
// 正的横向加速度为向右。
float AR_AttitudeControl::get_steering_out_lat_accel(float desired_accel, bool motor_limit_left, bool motor_limit_right, float dt)
{
    // 记录期望加速度，用于上报
    _steer_lat_accel_last_ms = AP_HAL::millis();
    _desired_lat_accel = desired_accel;

    // 获取前进速度
    float speed;
    if (!get_forward_speed(speed)) {
        // 我们期望调用者不会在没有有效速度估计时尝试用速率控制控制航向
        // 获取速度失败时不尝试转向
        return 0.0f;
    }

    const float desired_rate = get_turn_rate_from_lat_accel(desired_accel, speed);

    return get_steering_out_rate(desired_rate, motor_limit_left, motor_limit_right, dt);
}

// 根据航向（弧度）返回 -1 到 +1 的转向舵机输出
// 将 rate_max_rads 设置为非零值以限制期望转向率
// 返回值通常在 -1.0 到 +1.0 之间，但可能更高或更低
float AR_AttitudeControl::get_steering_out_heading(float heading_rad, float rate_max_rads, bool motor_limit_left, bool motor_limit_right, float dt)
{
    // 从角度误差（弧度）计算期望转向率（弧度）
    float desired_rate = get_turn_rate_from_heading(heading_rad, rate_max_rads);

    return get_steering_out_rate(desired_rate, motor_limit_left, motor_limit_right, dt);
}

// 根据期望航向（弧度）返回期望转向率
float AR_AttitudeControl::get_turn_rate_from_heading(float heading_rad, float rate_max_rads) const
{
    const float yaw_error = wrap_PI(heading_rad - AP::ahrs().get_yaw_rad());

    // 从角度误差（弧度）计算期望转向率（弧度）
    float desired_rate = _steer_angle_p.get_p(yaw_error);

    // 若选择了自定义原地转向率则限制 desired_rate，否则使用 ATC_STR_RAT_MAX
    if (is_positive(rate_max_rads)) {
        desired_rate = constrain_float(desired_rate, -rate_max_rads, rate_max_rads);
    }

    // 若提供了减速度限制，确保速率能及时减速到零以停在 heading_rad（即避免过冲）
    if (is_positive(_steer_decel_max)) {
        const float steer_decel_rate_max_rads = safe_sqrt(2.0 * fabsf(yaw_error) * radians(_steer_decel_max));
        desired_rate = constrain_float(desired_rate, -steer_decel_rate_max_rads, steer_decel_rate_max_rads);
    } else if (is_positive(_steer_accel_max)) {
        // 若无减速度限制，使用加速度限制
        const float steer_accel_rate_max_rads = safe_sqrt(2.0 * fabsf(yaw_error) * radians(_steer_accel_max));
        desired_rate = constrain_float(desired_rate, -steer_accel_rate_max_rads, steer_accel_rate_max_rads);
    }

    return desired_rate;
}

// 根据期望偏航角速度（弧度/秒）返回转向舵机输出。
// 正向偏航为向右
// 返回值通常在 -1.0 到 +1.0 之间，但可能更高或更低
// 同时设置 steering_limit_left 和 steering_limit_right 标志
float AR_AttitudeControl::get_steering_out_rate(float desired_rate, bool motor_limit_left, bool motor_limit_right, float dt)
{
    // 对 dt 做合理性检查
    dt = constrain_float(dt, 0.0f, 1.0f);

    // 更新高层控制器（如位置控制器）使用的转向限位标志
    _steering_limit_left = motor_limit_left;
    _steering_limit_right = motor_limit_right;

    // 若近期未被调用，将输入滤波器和期望转向率重置为实际转向率（用于加速度限制）
    const uint32_t now = AP_HAL::millis();
    if ((_steer_turn_last_ms == 0) || ((now - _steer_turn_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        _steer_rate_pid.reset_filter();
        _steer_rate_pid.reset_I();
        _desired_turn_rate = AP::ahrs().get_yaw_rate_earth();
    }
    _steer_turn_last_ms = now;

    // 对期望转向率做加速度限制
    if (is_positive(_steer_accel_max)) {
        const float change_max = radians(_steer_accel_max) * dt;
        if (desired_rate <= _desired_turn_rate - change_max) {
            _steering_limit_left = true;
        }
        if (desired_rate >= _desired_turn_rate + change_max) {
            _steering_limit_right = true;
        }
        desired_rate = constrain_float(desired_rate, _desired_turn_rate - change_max, _desired_turn_rate + change_max);
    }
    _desired_turn_rate = desired_rate;

    // 对期望转向率做速率限制
    if (is_positive(_steer_rate_max)) {
        const float steer_rate_max_rad = radians(_steer_rate_max);
        if (_desired_turn_rate <= -steer_rate_max_rad) {
            _steering_limit_left = true;
        }
        if (_desired_turn_rate >= steer_rate_max_rad) {
            _steering_limit_right = true;
        }
        _desired_turn_rate = constrain_float(_desired_turn_rate, -steer_rate_max_rad, steer_rate_max_rad);
    }

    // 基于速度的 G 力限制
    float speed;
    if (get_forward_speed(speed)) {
        // 不要限制到小于 1 deg/s
        const float turn_rate_max = MAX(get_turn_rate_from_lat_accel(get_turn_lat_accel_max(), fabsf(speed)), radians(1.0f));
        if (_desired_turn_rate <= -turn_rate_max) {
            _steering_limit_left = true;
        }
        if (_desired_turn_rate >= turn_rate_max) {
            _steering_limit_right = true;
        }
        _desired_turn_rate = constrain_float(_desired_turn_rate, -turn_rate_max, turn_rate_max);
    }

    // 更新 PID 以计算电机输出
    float output = _steer_rate_pid.update_all(_desired_turn_rate, AP::ahrs().get_yaw_rate_earth(), dt, (motor_limit_left || motor_limit_right));
    output += _steer_rate_pid.get_ff();
    // 限制并返回最终输出
    return output;
}

// 获取最近调用 get_steering_out_rate 期间记录的最新期望转向率（rad/s）
float AR_AttitudeControl::get_desired_turn_rate() const
{
    // 若近期无调用转向率控制器则返回零
    if ((_steer_turn_last_ms == 0) || ((AP_HAL::millis() - _steer_turn_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        return 0.0f;
    }
    return _desired_turn_rate;
}

// 获取最近调用 get_steering_out_lat_accel 期间记录的最新期望横向加速度（m/s/s）
float AR_AttitudeControl::get_desired_lat_accel() const
{
    // 若近期无调用横向加速度控制器则返回零
    if ((_steer_lat_accel_last_ms == 0) || ((AP_HAL::millis() - _steer_lat_accel_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        return 0.0f;
    }
    return _desired_lat_accel;
}

// 获取实际横向加速度（m/s/s）。成功返回 true
bool AR_AttitudeControl::get_lat_accel(float &lat_accel) const
{
    float speed;
    if (!get_forward_speed(speed)) {
        return false;
    }
    lat_accel = speed * AP::ahrs().get_yaw_rate_earth();
    return true;
}

// 根据横向加速度（m/s/s）和速度（m/s）计算转向率（rad/s）
float AR_AttitudeControl::get_turn_rate_from_lat_accel(float lat_accel, float speed) const
{
    // 强制最小速度，避免刚开始移动时产生振荡
    if (fabsf(speed) < AR_ATTCONTROL_STEER_SPEED_MIN) {
        if (is_negative(speed)) {
            speed = -AR_ATTCONTROL_STEER_SPEED_MIN;
        } else {
            speed = AR_ATTCONTROL_STEER_SPEED_MIN;
        }
    }

    return lat_accel / speed;
}

// 根据期望速度（m/s）返回 -1 到 +1 的油门输出（负速度表示倒车）
//   motor_limit 表示电机是否已达上/下限
//   cruise speed 单位为 m/s，cruise throttle 为 -1 到 +1 的数字
float AR_AttitudeControl::get_throttle_out_speed(float desired_speed, bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt)
{
    // 对 dt 做合理性检查
    dt = constrain_float(dt, 0.0f, 1.0f);

    // 获取前进速度
    float speed;
    if (!get_forward_speed(speed)) {
        // 我们期望调用者不会在没有有效速度估计时尝试用速率控制控制航向
        // 获取速度失败时不尝试转向
        return 0.0f;
    }

    // 若近期未被调用，将输入滤波器和期望速度重置为实际速度（用于加速度限制）
    if (!speed_control_active()) {
        _throttle_speed_pid.reset_filter();
        _throttle_speed_pid.reset_I();
        _desired_speed = speed;
    }
    _speed_last_ms = AP_HAL::millis();

    // 对期望速度做加速度限制
    _desired_speed = get_desired_speed_accel_limited(desired_speed, dt);

    // 计算基础油门（防止除以零）
    float throttle_base = 0.0f;
    if (is_positive(cruise_speed) && is_positive(cruise_throttle)) {
        throttle_base = _desired_speed * (cruise_throttle / cruise_speed);
    }

    // 计算最终输出
    float throttle_out = _throttle_speed_pid.update_all(_desired_speed, speed, dt, (motor_limit_low || motor_limit_high || _throttle_limit_low || _throttle_limit_high));
    throttle_out += _throttle_speed_pid.get_ff();
    throttle_out += throttle_base;

    // 更新 PID 信息用于上报
    _throttle_speed_pid_info = _throttle_speed_pid.get_pid_info();
    _throttle_speed_pid_info.FF += throttle_base;

    // 清除本地限制标志，以停止 I 项累积，因为不再向电机发送反向输出
    _throttle_limit_low = false;
    _throttle_limit_high = false;

    // 防止向电机发送反向输出，除非已启用刹车
    if (!_brake_enable) {
        // 若期望速度和实际速度均为正，不允许负值
        if ((_desired_speed >= 0.0f) && (throttle_out <= 0.0f)) {
            throttle_out = 0.0f;
            _throttle_limit_low = true;
        } else if ((_desired_speed <= 0.0f) && (throttle_out >= 0.0f)) {
            throttle_out = 0.0f;
            _throttle_limit_high = true;
        }
    }

    // 最终油门输出范围 -1 到 1
    return throttle_out;
}

// 返回 -1 到 +1 的油门输出以执行受控停车。车辆停止后返回 true
float AR_AttitudeControl::get_throttle_out_stop(bool motor_limit_low, bool motor_limit_high, float cruise_speed, float cruise_throttle, float dt, bool &stopped)
{
    // 获取当前系统时间
    const uint32_t now = AP_HAL::millis();

    // 若在过去 300ms 内已停止，认为仍然停止
    bool _stopped = (_stop_last_ms != 0) && (now - _stop_last_ms) < 300;

    // 获取受减速度限制的速度
    float desired_speed_limited = get_desired_speed_accel_limited(0.0f, dt);

    // 获取前进速度
    float speed;
    if (!get_forward_speed(speed)) {
        // 无法获取速度，认为已停止
        _stopped = true;
    } else {
        // 若期望速度为零且车辆降至 _stop_speed 以下，认为已停止
        if (is_zero(desired_speed_limited) && fabsf(speed) <= fabsf(_stop_speed)) {
            _stopped = true;
        }
    }

    // 为调用者设置停止状态
    stopped = _stopped;

    // 若已停止返回零
    if (stopped) {
        // 更新认为已停止的时间
        _stop_last_ms = now;
        // 设置速度控制器最后运行时间，使加速度被限制
        _speed_last_ms = now;
        // 重置滤波器和 I 项
        _throttle_speed_pid.reset_filter();
        _throttle_speed_pid.reset_I();
        // 确保期望速度为零
        _desired_speed = 0.0f;
        return 0.0f;
    }

    // 清除停止系统时间
    _stop_last_ms = 0;
    // 运行速度控制器使车辆停止
    return get_throttle_out_speed(desired_speed_limited, motor_limit_low, motor_limit_high, cruise_speed, cruise_throttle, dt);
}

// 平衡机器人俯仰到油门控制器
// 根据期望俯仰角（弧度）返回 -1 到 +1 的油门输出
// pitch_max 应为用户定义的最大俯仰角（弧度）
// 若电机已达上/下限，motor_limit 应为 true
float AR_AttitudeControl::get_throttle_out_from_pitch(float desired_pitch, float pitch_max, bool motor_limit, float dt)
{
    // 对 dt 做合理性检查
    dt = constrain_float(dt, 0.0f, 1.0f);

    // 若近期未被调用，重置输入滤波器
    const uint32_t now = AP_HAL::millis();
    if ((_balance_last_ms == 0) || ((now - _balance_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        _pitch_to_throttle_pid.reset_filter();
        _pitch_to_throttle_pid.reset_I();
        _pitch_limit_low = -pitch_max;
        _pitch_limit_high = pitch_max;
    }
    _balance_last_ms = now;

    // 限制期望俯仰以防止翻倒
    const bool pitch_limit_active = (_pitch_limit_tc >= 0.01) && (_pitch_limit_throttle_thresh > 0);
    if (pitch_limit_active) {
        desired_pitch = constrain_float(desired_pitch, _pitch_limit_low, _pitch_limit_high);
        _pitch_limited = (desired_pitch <= _pitch_limit_low || desired_pitch >= _pitch_limit_high);
    } else {
        _pitch_limited = false;
    }

    // 将输出初始化为当前俯仰角的前馈
    const float pitch_rad = AP::ahrs().get_pitch_rad();
    float output = sinf(pitch_rad) * _pitch_to_throttle_ff;

    // 添加常规 PID 控制
    output += _pitch_to_throttle_pid.update_all(desired_pitch, pitch_rad, dt, motor_limit);
    output += _pitch_to_throttle_pid.get_ff();

    // 为下一次迭代更新俯仰限制
    // 注意：向后倾斜时俯仰为正，向前倾斜时为负
    if (pitch_limit_active) {
        const float pitch_limit_incr = 1.0/_pitch_limit_tc * dt * pitch_max;
        const float pitch_relax_incr = pitch_limit_incr * AR_ATTCONTROL_PITCH_RELAX_RATIO;
        if (output <= -_pitch_limit_throttle_thresh) {
            // 非常低的负油门输出意味着必须降低 pitch_high（例如减少向后倾斜）
            _pitch_limit_high = MAX(_pitch_limit_high - pitch_limit_incr, 0);
        } else {
            _pitch_limit_high = MIN(_pitch_limit_high + pitch_relax_incr, pitch_max);
        }
        if (output >= _pitch_limit_throttle_thresh) {
            // 非常高的正油门输出意味着必须提高 pitch_low（减少向前倾斜）
            _pitch_limit_low = MIN(_pitch_limit_low + pitch_limit_incr, 0);
        } else {
            _pitch_limit_low = MAX(_pitch_limit_low - pitch_relax_incr, -pitch_max);
        }
    }

    // 限制并返回最终输出
    return output;
}

// 获取最新期望俯仰角（弧度），仅供上报
float AR_AttitudeControl::get_desired_pitch() const
{
    // 若近期未被调用，返回 0
    if ((_balance_last_ms == 0) || ((AP_HAL::millis() - _balance_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        return 0.0f;
    }

    return _pitch_to_throttle_pid.get_pid_info().target;
}

// 帆船横倾（滚转）角控制器释放帆以保持最大横倾角
// 但不尝试达到最大横倾角，即只放帆，不拉帆
float AR_AttitudeControl::get_sail_out_from_heel(float desired_heel, float dt)
{
    // 对 dt 做合理性检查
    dt = constrain_float(dt, 0.0f, 1.0f);

    // 若近期未被调用，重置输入滤波器
    const uint32_t now = AP_HAL::millis();
    if ((_heel_controller_last_ms == 0) || ((now - _heel_controller_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        _sailboat_heel_pid.reset_filter();
        _sailboat_heel_pid.reset_I();
    }
    _heel_controller_last_ms = now;

    _sailboat_heel_pid.update_all(desired_heel, fabsf(AP::ahrs().get_roll_rad()), dt);

    // 获取前馈
    const float ff = _sailboat_heel_pid.get_ff();

    // 获取 p，限制为零或负
    float p = _sailboat_heel_pid.get_p();
    if (is_positive(p)) {
        p = 0.0f;
    }

    // 获取 i，限制为零或负
    float i = _sailboat_heel_pid.get_i();
    if (is_positive(i)) {
        i = 0.0f;
        _sailboat_heel_pid.reset_I();
    }

    // 获取 d
    const float d = _sailboat_heel_pid.get_d();

    // 限制并返回最终输出
    return (ff + p + i + d) * -1.0f;
}

// 获取速度和转向的转换速率值，供 lua 脚本进行振荡检测
void AR_AttitudeControl::get_srate(float &steering_srate, float &speed_srate)
{
    steering_srate = get_steering_rate_pid().get_pid_info().slew_rate;
    speed_srate = _throttle_speed_pid_info.slew_rate;
}

// 获取前进速度（m/s）（大地坐标系水平速度，但仅沿车辆 x 轴）。成功返回 true
bool AR_AttitudeControl::get_forward_speed(float &speed) const
{
    Vector3f velocity;
    const AP_AHRS &_ahrs = AP::ahrs();
    if (!_ahrs.get_velocity_NED(velocity)) {
        // 使用精度较低的 GPS，假定整个长度沿车辆前后轴
        if (AP::gps().status() >= AP_GPS_FixType::FIX_3D) {
            if (abs(wrap_180_cd(_ahrs.yaw_sensor - AP::gps().ground_course_cd())) <= 9000) {
                speed = AP::gps().ground_speed();
            } else {
                speed = -AP::gps().ground_speed();
            }
            return true;
        } else {
            return false;
        }
    }
    // 将前进速度速度向量旋转到机体坐标系
    speed = velocity.x*_ahrs.cos_yaw() + velocity.y*_ahrs.sin_yaw();
    return true;
}

float AR_AttitudeControl::get_decel_max() const
{
    if (is_positive(_throttle_decel_max)) {
        return _throttle_decel_max;
    } else {
        return _throttle_accel_max;
    }
}

// 检查速度控制器是否激活
bool AR_AttitudeControl::speed_control_active() const
{
    // 若近期有调用速度控制器则激活
    if ((_speed_last_ms == 0) || ((AP_HAL::millis() - _speed_last_ms) > AR_ATTCONTROL_TIMEOUT_MS)) {
        return false;
    }
    return true;
}

// 获取调用 get_throttle_out_speed 期间记录的最新期望速度。仅供上报
float AR_AttitudeControl::get_desired_speed() const
{
    // 若近期无调用速度控制器则返回零
    if (!speed_control_active()) {
        return 0.0f;
    }
    return _desired_speed;
}

// 获取受加速度限制的期望速度
float AR_AttitudeControl::get_desired_speed_accel_limited(float desired_speed, float dt) const
{
    // 若近期无调用速度控制器则返回输入值
    // ATC_ACCEL_MAX 设置为零时不应用限制
    if (!speed_control_active() || !is_positive(_throttle_accel_max)) {
        return desired_speed;
    }

    // 对 dt 做合理性检查
    dt = constrain_float(dt, 0.0f, 1.0f);

    // 使用先前的期望速度作为加速度限制的基础
    float speed_prev = _desired_speed;

    // 若近期无调用速度控制器，则基于当前速度进行限制
    if (!speed_control_active()) {
        get_forward_speed(speed_prev);
    }

    // 对期望速度做加速度限制
    float speed_change_max;
    if (fabsf(desired_speed) < fabsf(_desired_speed) && is_positive(_throttle_decel_max)) {
        speed_change_max = _throttle_decel_max * dt;
    } else {
        speed_change_max = _throttle_accel_max * dt;
    }
    return constrain_float(desired_speed, speed_prev - speed_change_max, speed_prev + speed_change_max);
}

// 根据速度（m/s）获取最小停车距离（米）
float AR_AttitudeControl::get_stopping_distance(float speed) const
{
    // 获取最大车辆减速度
    const float decel_max = get_decel_max();

    if ((decel_max <= 0.0f) || is_zero(speed)) {
        return 0.0f;
    }

    // 假设线性减速度
    return 0.5f * sq(speed) / decel_max;
}

// 放松油门和转向控制器的 I 项
void AR_AttitudeControl::relax_I()
{
    _steer_rate_pid.reset_I();
    _throttle_speed_pid.reset_I();
    _pitch_to_throttle_pid.reset_I();
}

void AR_AttitudeControl::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    _steer_rate_pid.set_notch_sample_rate(sample_rate);
    _throttle_speed_pid.set_notch_sample_rate(sample_rate);
    _pitch_to_throttle_pid.set_notch_sample_rate(sample_rate);
#endif
}
