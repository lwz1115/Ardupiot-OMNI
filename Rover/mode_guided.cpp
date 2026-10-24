#include "Rover.h"

bool ModeGuided::_enter()
{
    // initialise submode to stop or loiter
    if (rover.is_boat()) {
        if (!start_loiter()) {
            start_stop();
        }
    } else {
        start_stop();
    }

    // initialise waypoint navigation library
    g2.wp_nav.init();

    send_notification = false;

    return true;
}

void ModeGuided::update()
{
    switch (_guided_mode) {
        case SubMode::WP:
        {
            // check if we've reached the destination
            if (!g2.wp_nav.reached_destination()) {
                // update navigation controller
                navigate_to_waypoint();
            } else {
                // send notification
                if (send_notification) {
                    send_notification = false;
                    rover.gcs().send_mission_item_reached_message(0);
                }

                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
                // update distance to destination
                _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
            }
            break;
        }

        case SubMode::HeadingAndSpeed:
        {
            // stop vehicle if target not updated within GUID_TIMEOUT seconds
            if (have_attitude_target && (millis() - _des_att_time_ms) > get_timeout_ms()) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "target not received last %.1f secs, stopping", get_timeout_ms()/1000.0f);
                have_attitude_target = false;
            }
            if (have_attitude_target) {
                // run steering and throttle controllers
                calc_steering_to_heading(_desired_yaw_cd);
                calc_throttle(calc_speed_nudge(_desired_speed, is_negative(_desired_speed)), true);
            } else {
                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case SubMode::TurnRateAndSpeed:
        {
            // stop vehicle if target not updated within GUID_TIMEOUT seconds
            if (have_attitude_target && (millis() - _des_att_time_ms) > get_timeout_ms()) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "target not received last %.1f secs, stopping", get_timeout_ms()/1000.0f);
                have_attitude_target = false;
            }
            if (have_attitude_target) {
                // run steering and throttle controllers
                float steering_out = attitude_control.get_steering_out_rate(radians(_desired_yaw_rate_cds * 0.01f),
                                                                            g2.motors.limit.steer_left,
                                                                            g2.motors.limit.steer_right,
                                                                            rover.G_Dt);
                set_steering(steering_out * 4500.0f);
                calc_throttle(calc_speed_nudge(_desired_speed, is_negative(_desired_speed)), true);
            } else {
                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case SubMode::Loiter:
        {
            rover.mode_loiter.update();
            break;
        }

        case SubMode::SteeringAndThrottle:
        {
            // handle timeout
            if (_have_strthr && (AP_HAL::millis() - _strthr_time_ms) > get_timeout_ms()) {
                _have_strthr = false;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "target not received last %.1f secs, stopping", get_timeout_ms()/1000.0f);
            }
            if (_have_strthr) {
                // pass latest steering and throttle directly to motors library
                g2.motors.set_steering(_strthr_steering * 4500.0f, false);
                g2.motors.set_throttle(_strthr_throttle * 100.0f);
            } else {
                // loiter or stop vehicle
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case SubMode::Stop:
            stop_vehicle();
            break;

        default:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Unknown GUIDED mode");
            break;
    }
}

// return heading (in degrees) and cross track error (in meters) for reporting to ground station (NAV_CONTROLLER_OUTPUT message)
float ModeGuided::wp_bearing() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.wp_bearing_cd() * 0.01f;
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.wp_bearing();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeGuided::nav_bearing() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.nav_bearing_cd() * 0.01f;
    case SubMode::HeadingAndSpeed:
        return have_attitude_target ? wrap_180(_desired_yaw_cd * 0.01f) : 0.0f;
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.nav_bearing();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeGuided::crosstrack_error_m() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.crosstrack_error_m();
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.crosstrack_error_m();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

float ModeGuided::get_desired_lat_accel() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.get_lat_accel();
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.get_desired_lat_accel();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

// return straight-line distance (in meters) to destination
float ModeGuided::get_distance_to_destination() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return _distance_to_destination;
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        return 0.0f;
    case SubMode::Loiter:
        return rover.mode_loiter.get_distance_to_destination();
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return 0.0f;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

// return true if vehicle has reached or even passed destination
bool ModeGuided::reached_destination() const
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.reached_destination();
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
    case SubMode::Loiter:
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        return true;
    }

    // we should never reach here but just in case, return true is the safer option
    return true;
}

// set desired speed in m/s
bool ModeGuided::set_desired_speed(float speed_ms)
{
    switch (_guided_mode) {
    case SubMode::WP:
        return g2.wp_nav.set_speed_max(speed_ms);
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        // speed is set from mavlink message
        return false;
    case SubMode::Loiter:
        return rover.mode_loiter.set_desired_speed(speed_ms);
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        // no speed control
        return false;
    }
    return false;
}

// get desired location
bool ModeGuided::get_desired_location(Location& destination) const
{
    switch (_guided_mode) {
    case SubMode::WP:
        if (g2.wp_nav.is_destination_valid()) {
            destination = g2.wp_nav.get_oa_destination();
            return true;
        }
        return false;
    case SubMode::HeadingAndSpeed:
    case SubMode::TurnRateAndSpeed:
        // not supported in these submodes
        return false;
    case SubMode::Loiter:
        // get destination from loiter
        return rover.mode_loiter.get_desired_location(destination);
    case SubMode::SteeringAndThrottle:
    case SubMode::Stop:
        // no desired location in this submode
        break;
    }

    // should never get here but just in case
    return false;
}

// set desired location
bool ModeGuided::set_desired_location(const Location &destination, Location next_destination)
{
#if AP_FENCE_ENABLED
    // reject destination outside the fence
    if (!rover.fence.check_location_within_fence(destination)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        return false;
    }
#endif

    // 摆渡（Ferry）电机对自动选择：目标在前方用后电机对（REAR 差速前进），
    // 目标在后方用前电机对（FRONT 差速倒车前进），船体无需掉头即可直线往返。
    // AR_WPNav 原生支持 reversed 倒车导航，位置控制器已自动处理倒车转向符号。
    if (g2.motors.is_omni()) {
        const float target_bearing_cd = rover.current_loc.get_bearing_to(destination);
        const float yaw_error = wrap_180_cd(target_bearing_cd - ahrs.yaw_sensor);
        const bool reverse = fabsf(yaw_error) > 9000;   // 目标在船尾方向
        g2.motors.set_motor_pair(reverse ? AP_MotorsUGV::MotorPair::FRONT : AP_MotorsUGV::MotorPair::REAR);
        set_reversed(reverse);
    } else {
        g2.motors.set_motor_pair(AP_MotorsUGV::MotorPair::ALL);
        set_reversed(false);
    }

    const bool ret = use_scurves_for_navigation()
                         ? g2.wp_nav.set_desired_location(destination, next_destination)
                         : g2.wp_nav.set_desired_location_expect_fast_update(destination);
    if (!ret) {
        // 目标设置失败（例如位置无效）：恢复全向电机对，避免悬挂在
        // 已切换的 REAR/FRONT 上导致后续 loiter 用错误电机对悬停。
        g2.motors.set_motor_pair(AP_MotorsUGV::MotorPair::ALL);
        set_reversed(false);
        return false;
    }

    // handle guided specific initialisation and logging
    _guided_mode = SubMode::WP;
    send_notification = true;
#if HAL_LOGGING_ENABLED
    rover.Log_Write_GuidedTarget((uint8_t)_guided_mode, Vector3f(destination.lat, destination.lng, 0), Vector3f(g2.wp_nav.get_speed_max(), 0.0f, 0.0f));
#endif
    return true;
}

// set desired attitude
void ModeGuided::set_desired_heading_and_speed(float yaw_angle_cd, float target_speed)
{
    // initialisation and logging
    _guided_mode = SubMode::HeadingAndSpeed;
    _des_att_time_ms = AP_HAL::millis();

    // record targets
    _desired_yaw_cd = yaw_angle_cd;
    _desired_speed = target_speed;
    have_attitude_target = true;

#if HAL_LOGGING_ENABLED
    // log new target
    rover.Log_Write_GuidedTarget((uint8_t)_guided_mode, Vector3f(_desired_yaw_cd, 0.0f, 0.0f), Vector3f(_desired_speed, 0.0f, 0.0f));
#endif
}

void ModeGuided::set_desired_heading_delta_and_speed(float yaw_delta_cd, float target_speed)
{
    // handle initialisation
    if (_guided_mode != SubMode::HeadingAndSpeed) {
        _guided_mode = SubMode::HeadingAndSpeed;
        _desired_yaw_cd = ahrs.yaw_sensor;
    }
    set_desired_heading_and_speed(wrap_180_cd(_desired_yaw_cd + yaw_delta_cd), target_speed);
}

// set desired velocity
void ModeGuided::set_desired_turn_rate_and_speed(float turn_rate_cds, float target_speed)
{
    // handle initialisation
    _guided_mode = SubMode::TurnRateAndSpeed;
    _des_att_time_ms = AP_HAL::millis();

    // record targets
    _desired_yaw_rate_cds = turn_rate_cds;
    _desired_speed = target_speed;
    have_attitude_target = true;

#if HAL_LOGGING_ENABLED
    // log new target
    rover.Log_Write_GuidedTarget((uint8_t)_guided_mode, Vector3f(_desired_yaw_rate_cds, 0.0f, 0.0f), Vector3f(_desired_speed, 0.0f, 0.0f));
#endif
}

// set steering and throttle (both in the range -1 to +1)
void ModeGuided::set_steering_and_throttle(float steering, float throttle)
{
    _guided_mode = SubMode::SteeringAndThrottle;
    _strthr_time_ms = AP_HAL::millis();
    _strthr_steering = constrain_float(steering, -1.0f, 1.0f);
    _strthr_throttle = constrain_float(throttle, -1.0f, 1.0f);
    _have_strthr = true;
}

bool ModeGuided::start_loiter()
{
    if (rover.mode_loiter.enter()) {
        _guided_mode = SubMode::Loiter;
        return true;
    }
    return false;
}


// start stopping vehicle as quickly as possible
void ModeGuided::start_stop()
{
    _guided_mode = SubMode::Stop;
}

// set guided timeout and movement limits
void ModeGuided::limit_set(uint32_t timeout_ms, float horiz_max)
{
    limit.timeout_ms = timeout_ms;
    limit.horiz_max = horiz_max;
}

// clear/turn off guided limits
void ModeGuided::limit_clear()
{
    limit.timeout_ms = 0;
    limit.horiz_max = 0.0f;
}

// initialise guided start time and location as reference for limit checking
//  only called from AUTO mode's start_guided method
void ModeGuided::limit_init_time_and_location()
{
    limit.start_time_ms = AP_HAL::millis();
    limit.start_loc = rover.current_loc;
}

// returns true if guided mode has breached a limit
bool ModeGuided::limit_breached() const
{
    // check if we have passed the timeout
    if ((limit.timeout_ms > 0) && (millis() - limit.start_time_ms >= limit.timeout_ms)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (is_positive(limit.horiz_max)) {
        return (rover.current_loc.get_distance(limit.start_loc) > limit.horiz_max);
    }

    // if we got this far we must be within limits
    return false;
}

// returns true if GUID_OPTIONS bit set to use scurve navigation instead of position controller input shaping
// scurves provide path planning and object avoidance but cannot handle fast updates to the destination (for fast updates use position controller input shaping)
bool ModeGuided::use_scurves_for_navigation() const
{
    return ((g2.guided_options.get() & uint32_t(Options::SCurvesUsedForNavigation)) != 0);
}

// return guided mode timeout in milliseconds. Only used for velocity, throttle, heading or turn rate control
uint32_t ModeGuided::get_timeout_ms() const
{
    return MAX(g2.guided_timeout, 0.1) * 1000;
}
