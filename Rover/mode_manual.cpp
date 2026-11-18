#include "Rover.h"

// 将 MANUAL_PAIR 参数值映射为电机对（0=14前,1=23后,2=12右舷,3=34左舷）
static AP_MotorsUGV::MotorPair manual_motor_pair_from(uint8_t pair)
{
    switch (pair) {
    case 0:
        return AP_MotorsUGV::MotorPair::FRONT;   // 14：前电机对 M1/M4
    case 2:
        return AP_MotorsUGV::MotorPair::RIGHT;   // 12：右舷对 M1/M2
    case 3:
        return AP_MotorsUGV::MotorPair::LEFT;    // 34：左舷对 M3/M4
    case 1:
    default:
        return AP_MotorsUGV::MotorPair::REAR;    // 23：后电机对 M2/M3（默认）
    }
}

bool ModeManual::_enter()
{
    // 手动模式按差速船驱动：默认只用后电机对 M2/M3，前电机停转
    // （减少对船中 ADCP 水流的干扰）；单电机故障时可用 MANUAL_PAIR
    // 参数（QGC 可设置）切换到其他相邻电机对应急航行。
    // 手动模式启用“按对方向映射”（船头参考翻转/转向系数按对定义），
    // 自动模式不启用，保持原有固定船头与固定电机对解算。
    g2.motors.set_manual_pair_mapping(true);
    _last_pair = g2.manual_pair.get();
    g2.motors.set_motor_pair(manual_motor_pair_from(_last_pair));
    return true;
}

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);

    // 关闭手动映射，恢复全电机对：自动/悬停回到固定船头与全向解算，
    // 不把手动模式的电机对/方向系数带进其他模式。
    g2.motors.set_manual_pair_mapping(false);
    g2.motors.set_motor_pair(AP_MotorsUGV::MotorPair::ALL);
}

void ModeManual::update()
{
    // 电机对切换：检测到 MANUAL_PAIR 变化时，先停一帧输出再换对，
    // 避免正反突变冲击电机；下一周期再按新对的方向映射解算。
    const uint8_t pair = g2.manual_pair.get();
    if (pair != _last_pair) {
        _last_pair = pair;
        g2.motors.set_throttle(0);
        g2.motors.set_steering(0);
        g2.motors.set_lateral(0);
        g2.motors.set_motor_pair(manual_motor_pair_from(pair));
        return;
    }

    float desired_steering, desired_throttle;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

    // apply manual steering expo
    desired_steering = 4500.0 * input_expo(desired_steering / 4500, g2.manual_steering_expo);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);
    }

    // walking robots support roll, pitch and walking_height
    float desired_roll, desired_pitch, desired_walking_height;
    get_pilot_desired_roll_and_pitch(desired_roll, desired_pitch);
    get_pilot_desired_walking_height(desired_walking_height);
    g2.motors.set_roll(desired_roll);
    g2.motors.set_pitch(desired_pitch);
    g2.motors.set_walking_height(desired_walking_height);

    // set sailboat sails
    g2.sailboat.set_pilot_desired_mainsail();

    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering, (g2.manual_options & ManualOptions::SPEED_SCALING));
    // 差速船无横移能力：手动模式禁用横向通道（差速模式下输出_omni 也
    // 不会让未激活电机响应 lateral，这里显式清零保证语义一致）
    g2.motors.set_lateral(0);
}
