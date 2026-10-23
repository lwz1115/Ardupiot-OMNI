#include "Rover.h"

bool ModeManual::_enter()
{
    // 手动模式按差速船驱动：只用后电机对 M2/M3，前电机停转
    // （减少对船中 ADCP 水流的干扰），退场时恢复全部电机。
    g2.motors.set_motor_pair(AP_MotorsUGV::MotorPair::REAR);
    return true;
}

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);

    // 恢复全电机对，避免影响其他模式（悬停需要四电机全向）
    g2.motors.set_motor_pair(AP_MotorsUGV::MotorPair::ALL);
}

void ModeManual::update()
{
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
