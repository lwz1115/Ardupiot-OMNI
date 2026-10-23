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
/*
  rover simulator class
*/

#include "SIM_Rover.h"
 
#include <string.h>
#include <stdio.h>

#include <AP_Math/AP_Math.h>

namespace SITL {

SimRover::SimRover(const char *frame_str) :
    Aircraft(frame_str)
{
    constexpr float default_battery_resistance_ohm = 0.01f;
    battery.setup(sitl->batt_capacity_ah,
                  default_battery_resistance_ohm,
                  sitl->batt_voltage,
                  ambient_outside_temperature_degC());
    battery_voltage = battery.get_voltage();

    skid_steering = strstr(frame_str, "skid") != nullptr;

    if (skid_steering) {
        printf("SKID Steering Rover Simulation Started\n");
        // these are taken from a 6V wild thumper with skid steering,
        // with a sabertooth controller
        max_accel = 14;
        max_speed = 4;
        return;
    }

    vectored_thrust = strstr(frame_str, "vector") != nullptr;
    if (vectored_thrust) {
        printf("Vectored Thrust Rover Simulation Started\n");
    }

    omni3 = strstr(frame_str, "omni3mecanum") != nullptr;
    if (omni3) {
        printf("Omni3 Mecanum Rover Simulation Started\n");
    }

    omnix = strstr(frame_str, "omnix") != nullptr;
    if (omnix) {
        printf("OmniX 4-thruster Rover/Boat Simulation Started\n");
        // 可选的偏航漂移（度/秒），例如 --model rover-omnix:yawdrift=5
        // 注意：在模型字符串中添加 ":yawdrift=N" 会让 SITL 跳过该载具的
        // ROMFS 默认参数加载，因此你依赖的任何参数都必须显式设置
        //（例如通过脚本）。
        const char *p = strstr(frame_str, "yawdrift=");
        if (p != nullptr) {
            const float drift = atof(p + strlen("yawdrift="));
            if (isfinite(drift)) {
                omnix_yaw_drift = drift;
            } else {
                // atof() 对非数字输入返回 0，会静默禁用漂移；改为上报
                printf("OmniX: invalid yawdrift value, ignored\n");
            }
            printf("OmniX yaw drift %.1f deg/s\n", (double)omnix_yaw_drift);
        }
    }

    lock_step_scheduled = true;
}

/*
  return turning circle (diameter) in meters for steering angle proportion in degrees
*/
float SimRover::turn_circle(float steering) const
{
    if (fabsf(steering) < 1.0e-6) {
        return 0;
    }
    return turning_circle * sinf(radians(max_wheel_turn)) / sinf(radians(steering*max_wheel_turn));
}

/*
   return yaw rate in degrees/second given steering_angle and speed
*/
float SimRover::calc_yaw_rate(float steering, float speed)
{
    if (skid_steering) {
        return constrain_float(steering * skid_turn_rate, -MAX_YAW_RATE, MAX_YAW_RATE);
    }
    if (vectored_thrust) {
        return constrain_float(steering * vectored_turn_rate_max, -MAX_YAW_RATE, MAX_YAW_RATE);
    }
    if (fabsf(steering) < 1.0e-6 or fabsf(speed) < 1.0e-6) {
        return 0;
    }
    float d = turn_circle(steering);
    float c = M_PI * d;
    float t = c / speed;
    float rate = constrain_float(360.0f / t, -MAX_YAW_RATE, MAX_YAW_RATE);
    return rate;
}

/*
  return lateral acceleration in m/s/s
*/
float SimRover::calc_lat_accel(float steering_angle, float speed)
{
    float yaw_rate = calc_yaw_rate(steering_angle, speed);
    float accel = radians(yaw_rate) * speed;
    return accel;
}

/*
  update the rover simulation by one time step
 */
void SimRover::update(const struct sitl_input &input)
{
    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    // update gyro and accel_body according to frame type
    if (omni3) {
        update_omni3(input, delta_time);
    } else if (omnix) {
        update_omnix(input, delta_time);
    } else {
        update_ackermann_or_skid(input, delta_time);
    }

    // common to all rovers

    // now in earth frame
    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    position += (velocity_ef * delta_time).todouble();

    update_external_payload(input);

    // allow for changes in physics step
    adjust_frame_time(constrain_float(sitl->loop_rate_hz, rate_hz-1, rate_hz+1));

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();

    update_battery();
}

/*
  update the ackermann or skid rover simulation by one time step
 */
void SimRover::update_ackermann_or_skid(const struct sitl_input &input, float delta_time)
{
    float steering = 0.0f;
    float throttle = 0.0f;

    if (!battery_is_empty()) {
        // if in skid steering mode the steering and throttle values are used for motor1 and motor2
        if (skid_steering) {
            const float motor1 = input.servos[0] ? normalise_servo_input(input.servos[0]) : 0;
            const float motor2 = input.servos[2] ? normalise_servo_input(input.servos[2]) : 0;
            steering = motor1 - motor2;
            throttle = 0.5*(motor1 + motor2);
            battery_current = 10.0f * (fabsf(motor1) + fabsf(motor2));
        } else {
            // steering here should really be "old steering" as no-pulses
            // should yield no servo movement
            steering = input.servos[0] ? normalise_servo_input(input.servos[0]) : 0;
            throttle = input.servos[2] ? normalise_servo_input(input.servos[2]) : 0;
            battery_current = 20.0f * fabsf(throttle) + 1.0f * fabsf(steering);

            // vectored thrust conversion
            if (vectored_thrust) {
                const float thrust = throttle;
                const float steering_angle_rad = radians(steering * vectored_angle_max);
                steering = sinf(steering_angle_rad) * thrust;
                throttle = cosf(steering_angle_rad) * thrust;
                battery_current = 20.0f * fabsf(thrust);
            }
        }
    } else {
        battery_current = 0.0f;
    }

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // speed along x axis, +ve is forward
    float speed = velocity_body.x;

    // yaw rate in degrees/s
    float yaw_rate = calc_yaw_rate(steering, speed);

    // target speed with current throttle
    float target_speed = throttle * max_speed;

    // linear acceleration in m/s/s - very crude model
    float accel = max_accel * (target_speed - speed) / max_speed;

    gyro = Vector3f(0,0,radians(yaw_rate));

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due to motor (excluding gravity)
    accel_body = Vector3f(accel, 0, 0);

    // add in accel due to direction change
    accel_body.y += radians(yaw_rate) * speed;
}

/*
  update the omni3 rover simulation by one time step
 */
void SimRover::update_omni3(const struct sitl_input &input, float delta_time)
{
    // omni3 模式下，前三个伺服信号为电机速度

// 使用正运动学计算机体坐标系速度
    // 注意：必须做值初始化——当 battery_is_empty() 为真时下面的循环被
    // 完全跳过，而该向量会在下方的 Minv * wheel_ang_vel 运动学中被原样使用。
    Vector3f wheel_ang_vel{};
    battery_current = 0.0f;
    if (!battery_is_empty()) {
        for (uint8_t i=0; i<3; i++) {
            wheel_ang_vel[i] = input.servos[i] ? normalise_servo_input(input.servos[i]) : 0;
            battery_current += 10.0f * fabsf(wheel_ang_vel[i]);
        };
        wheel_ang_vel *= omni3_wheel_max_ang_vel;
    }

    // derivation of forward kinematics for an Omni3Mecanum rover
    // A. Gfrerrer. "Geometry and kinematics of the Mecanum wheel",
    // Computer Aided Geometric Design 25 (2008) 784–791.
    // Retrieved from https://www.geometrie.tugraz.at/gfrerrer/publications/MecanumWheel.pdf.
    //
    // the frame is equilateral triangle
    //
    // d[i] = 0.18 m is distance from frame centre to each wheel 
    // r_w = 0.04725 m is the wheel radius.
    // delta = radians(-45) is angle of the roller to the direction of forward rotation
    // alpha[i] is the angle the wheel axis is rotated about the body z-axis
    // c[i] = cos(alpha[i] + delta)
    // s[i] = sin(alpha[i] + delta)
    //
    // wheel  d[i]  alpha[i]  a_x[i]   a_y[i]      c[i]      s[i]
    //     1  0.18   1.04719    0.09  0.15588  0.965925  0.258819
    //     2  0.18   3.14159   -0.18  0.0     -0.707106  0.707106
    //     3  0.18   5.23598    0.09 -0.15588 -0.258819 -0.965925
    //
    //  k = 1/(r_w * sin(delta)) = -29.930445 is a scale factor
    //
    // inverse kinematic matrix
    // M[i, 0] = k * c[i]
    // M[i, 1] = k * s[i]
    // M[i, 2] = k * (a_x[i] s[i] - a_y[i] c[i])
    //
    // forward kinematics matrix: Minv = M^-1
    constexpr Matrix3f Minv(
      -0.0215149, 0.01575, 0.0057649,
      -0.0057649, -0.01575, 0.0215149,
      0.0875, 0.0875, 0.0875);

    // twist - this is the target linear and angular velocity
    Vector3f twist = Minv * wheel_ang_vel;

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // linear acceleration in m/s/s - very crude model
    float accel_x = omni3_max_accel * (twist.x - velocity_body.x) / omni3_max_speed;
    float accel_y = omni3_max_accel * (twist.y - velocity_body.y) / omni3_max_speed;

    gyro = Vector3f(0, 0, twist.z);

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due to motors (excluding gravity)
    accel_body = Vector3f(accel_x, accel_y, 0);
}

/*
  按一个时间步长更新 omniX 车/船仿真

  omniX 是 X 形机架，装有 4 个以固定 45 度角安装的推进器（每个角落一个，
  朝外推进）。这与 AP_MotorsUGV FRAME_TYPE_OMNIX 的几何结构相同：

    电机 0 : 右前推进器，朝外右方推进 (SERVO1)
    电机 1 : 右后推进器，朝外右方推进 (SERVO2)
    电机 2 : 左后推进器，朝外左方推进 (SERVO3)
    电机 3 : 左前推进器，朝外左方推进 (SERVO4)

  每个推进器贡献一个机体坐标系力：

    f_i = T_i * [ cos(45度) * sign_x_i
                , sin(45度) * sign_y_i ]

  其中 sign_x_i/sign_y_i 编码推进器的固定朝向。
  四个推进器合力提供前后（surge）、左右（sway）与偏航（yaw）自由度，
  因此车辆无需转向即可向任意方向移动。

  下面的符号约定与 AP_MotorsUGV::setup_omni() 的混合系数一致：正的
  油门指令产生前进，正的转向产生顺时针（右转）偏航，正的横向产生向右
  运动。
 */
void SimRover::update_omnix(const struct sitl_input &input, float delta_time)
{
    // 默认的 Rover/Boat OmniX 配置将四个推进器输出映射到伺服通道 1..4，
    // 对应电机功能 33..36。除非用户在其他地方重映射这些功能，本仿真器
    // 假定这一映射。任何重映射都应以相同方式反映到模型中。
    float thr[4] {};
    battery_current = 0.0f;
    if (!battery_is_empty()) {
        for (uint8_t i=0; i<4; i++) {
            const uint16_t servo_pwm = input.servos[i];
            thr[i] = (servo_pwm > 0) ? normalise_servo_input(servo_pwm) : 0.0f;
            battery_current += 10.0f * fabsf(thr[i]);
        }
    }

    // 后两电机差速模式（REAR：前电机停转，手动/自动/穿梭去程用）：
    // 真机后两推进器(1右后、2左后)以 V 形斜向后推水，差速产生偏航力矩，
    // 行为与原生差速船(skid)一致。标准 X 形逆矩阵在此模式下会串扰
    // （把差速误判成横移、把油门误判成偏航），因此单独按差速船物理模拟。
    const bool rear_pair_active = (fabsf(thr[0]) < 0.01f) &&
                                  (fabsf(thr[3]) < 0.01f) &&
                                  (fabsf(thr[1]) > 0.01f || fabsf(thr[2]) > 0.01f);

    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    if (rear_pair_active) {
        // 差速船物理（与原生 SIM_Rover::update_ackermann_or_skid 一致）：
        //   motor_left  = thr[2] (左后电机 SERVO3, 固件 REAR 输出 t+s)
        //   motor_right = thr[1] (右后电机 SERVO2, 固件 REAR 输出 t-s)
        //   steering = left - right, throttle = 0.5*(left + right)
        const float motor_left  = thr[2];
        const float motor_right = thr[1];
        const float steering = motor_left - motor_right;
        const float throttle = 0.5f * (motor_left + motor_right);

        // 船体 x 向速度
        const float speed = velocity_body.x;

        // 目标速度与加速度（一阶油门模型，与原生差速船相同）
        const float target_speed = throttle * max_speed;
        const float accel = max_accel * (target_speed - speed) / max_speed;

        // 偏航率（与原生 skid 模型一致，低速也可原地转向）
        const float yaw_rate = calc_yaw_rate(steering, speed);

        gyro = Vector3f(0, 0, radians(yaw_rate) + radians(omnix_yaw_drift));
        dcm.rotate(gyro * delta_time);
        dcm.normalize();

        // 纵向加速度 + 转弯向心加速度
        accel_body = Vector3f(accel, 0, 0);
        accel_body.y += radians(yaw_rate) * speed;
        return;
    }

    // AP_MotorsUGV::output_omni() 使用以下混合器（电机编号按实际物理接线）：
    //  m0(右前) = t - s - l
    //  m1(右后) = t + s - l
    //  m2(左后) = t + s + l
    //  m3(左前) = t - s + l
    // 求逆以获得仿真使用的机体坐标系指令。
    // t、s、l 各自在混合矩阵的每一列中出现一次 +1 和一次 -1，
    // 因此逆矩阵为转置矩阵乘以 1/4（= 1/电机数）。
    const float inv_mix_scale = 0.25f;
    const float throttle_cmd  = inv_mix_scale * ( thr[0] + thr[1] + thr[2] + thr[3]);
    const float steering_cmd  = inv_mix_scale * (-thr[0] + thr[1] + thr[2] - thr[3]);
    const float lateral_cmd   = inv_mix_scale * (-thr[0] - thr[1] + thr[2] + thr[3]);

    const float surge_target = throttle_cmd * omnix_max_speed;
    // 注意：此处符号必须与 AP_MotorsUGV::setup_omni()（FRAME_TYPE_OMNIX）
    // 一致：正的横向指令产生向船头左侧的运动（机体坐标系 -y），因此取反
    // 以模拟真实车辆。未经与真实船交叉验证，不要移除这个取反——移除它会
    // 使所有侧向穿梭平移反向。
    const float sway_target  = -lateral_cmd * omnix_max_lateral_speed;
    const float yaw_target   = steering_cmd * radians(omnix_max_yaw_rate);

    // 非常粗略的一阶油门模型。纯比例 accel = gain * (target - velocity)
    // 使稳态速度等于指令的目标速度。若在此处额外加入线性阻尼项，会把稳态
    // 拉到目标速度的一小部分（例如旧阻尼增益下仅约 45%），使船远慢于预期。
    // 当 target = 0 时，同一模型仍给出自然的指数滑行停车，如同真实带阻力的船。
    const float surge_scale = MAX(omnix_max_speed, 0.1f);
    const float sway_scale  = MAX(omnix_max_lateral_speed, 0.1f);
    float accel_x = omnix_max_accel * (surge_target - velocity_body.x) / surge_scale;
    float accel_y = omnix_max_accel * (sway_target - velocity_body.y) / sway_scale;

    // 使用角加速度并对其积分，而不是直接施加角速率滞后。
    // 这给车辆旋转惯性，并对转向指令产生更物理、更真实的响应。
    const float yaw_rate = gyro.z;
    const float yaw_accel = (yaw_target - yaw_rate) * radians(omnix_max_yaw_accel)
                          / MAX(radians(omnix_max_yaw_rate), 0.1f)
                          - omnix_yaw_damping * yaw_rate;
    const float new_yaw_rate = yaw_rate + yaw_accel * delta_time;

    // 设置陀螺并更新姿态
    // 可选的偏航漂移是一个恒定的旋转速率（像船被水流推转），叠加在指令
    // 驱动的偏航速率之上
    gyro = Vector3f(0, 0, new_yaw_rate + radians(omnix_yaw_drift));
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // accel in body frame due to motors (excluding gravity)
    accel_body = Vector3f(accel_x, accel_y, 0);
}

void SimRover::update_battery()
{
    battery.maybe_reset(sitl->batt_voltage, sitl->batt_capacity_ah);
    const uint64_t now_us = AP_HAL::micros64();
    battery.consume_energy(battery_current, now_us);
    battery_voltage = battery.get_voltage();
    battery_temperature_degC = battery.get_temperature_degC();
}

} // namespace SITL
