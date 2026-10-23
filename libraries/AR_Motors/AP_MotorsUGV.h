#pragma once

#include <AP_Arming/AP_Arming.h>
#include <AP_WheelEncoder/AP_WheelRateControl.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_BattMonitor/AP_BattMonitor_config.h>

class AP_MotorsUGV {
public:
    // 构造函数
    AP_MotorsUGV(AP_WheelRateControl& rate_controller);

    // 单例支持
    static AP_MotorsUGV    *get_singleton(void) { return _singleton; }

    enum motor_test_order {
        MOTOR_TEST_THROTTLE = 1,
        MOTOR_TEST_STEERING = 2,
        MOTOR_TEST_THROTTLE_LEFT = 3,
        MOTOR_TEST_THROTTLE_RIGHT = 4,
        MOTOR_TEST_MAINSAIL = 5,
        MOTOR_TEST_LAST
    };

    // 支持的全向电机配置
    enum frame_type {
        FRAME_TYPE_UNDEFINED = 0,
        FRAME_TYPE_OMNI3 = 1,
        FRAME_TYPE_OMNIX = 2,
        FRAME_TYPE_OMNIPLUS = 3,
        FRAME_TYPE_OMNI3MECANUM = 4,
    };

    // 初始化电机
    void init(uint8_t ftype);

    // 电机是否激活，返回 true
    bool active() const;

    // 主 CPU 故障时设置安全输出
    void setup_safety_output();

    // 设置舵机输出范围
    void setup_servo_output();

    // 获取或设置转向值（范围 -4500 到 +4500）
    //   手动模式下 apply_scaling 应设置为 false，
    //   此时不进行速度或角度缩放
    float get_steering() const { return _steering; }
    void set_steering(float steering, bool apply_scaling = true);

    // 获取或设置油门值（范围 -100 到 100）
    float get_throttle() const { return _throttle; }
    void set_throttle(float throttle);

    // 获取或设置滚转值（范围 -1 到 1）
    float get_roll() const { return _roll; }
    void set_roll(float roll);

    // 获取或设置俯仰值（范围 -1 到 1）
    float get_pitch() const { return _pitch; }
    void set_pitch(float pitch);

    // 获取或设置行走高度值（范围 -1 到 1）
    float get_walking_height() const { return _walking_height; }
    void set_walking_height(float walking_height);

    // 获取或设置横向输入值（范围 -100 到 +100）
    float get_lateral() const { return _lateral; }
    void set_lateral(float lateral);

    // 设置或获取主帆输入值（范围 0 到 100）
    void set_mainsail(float mainsail);
    float get_mainsail() const { return _mainsail; }

    // 设置或获取翼帆输入值（范围 -100 到 100）
    void set_wingsail(float wingsail);
    float get_wingsail() const { return _wingsail; }

    // 设置或获取桅杆旋转输入值（范围 -100 到 100）
    void set_mast_rotation(float mast_rotation);
    float get_mast_rotation() const { return _mast_rotation; }

    // 获取受转换速率限制的油门
    // 手动模式使用，以避免从前进到倒车过渡期间转向性能不佳
    // 与私有 slew_limit_throttle 方法相同（见下），但不更新油门状态
    float get_slew_limited_throttle(float throttle, float dt) const;

    // 车辆是否具备滑移转向能力
    bool have_skid_steering() const;

    // 车辆是否具有矢量推力（即电机在转向舵机上的船）
    bool have_vectored_thrust() const { return is_positive(_vector_angle_max); }

    // 输出到电机和转向舵机
    // ground_speed 应为车辆在地面上的速度（m/s）
    // dt 应为两次调用此函数的预期间隔时间
    void output(bool armed, float ground_speed, float dt);

    // 以总量的百分比（范围 -100 到 +100）测试转向或油门输出
    // 用于响应 DO_MOTOR_TEST MAVLink 命令
    bool output_test_pct(motor_test_order motor_seq, float pct);

    // 使用 PWM 值测试转向或油门输出
    bool output_test_pwm(motor_test_order motor_seq, float pwm);

    // 检查通过返回 true，失败返回 false。display_failure 参数为 true 时应向地面站发送文本消息
    bool pre_arm_check(bool report) const;

    // 返回电机掩码
    uint32_t get_motor_mask() const { return _motor_mask; }

    // 配置的 PWM 类型为数字且应有固定端点时返回 true
    bool is_digital_pwm_type() const;

    // 车辆是否为全向时返回 true
    bool is_omni() const { return _frame_type != FRAME_TYPE_UNDEFINED && _motors_num > 0; }

    // 电机对：全向机架在差速模式下只激活部分电机（其余输出 TRIM 停转）
    enum class MotorPair : uint8_t {
        ALL   = 0,  // 全部电机参与（悬停/全向平移）
        REAR  = 1,  // 只激活后电机对 M2/M3（手动/自动/穿梭去程差速）
        FRONT = 2,  // 只激活前电机对 M0/M1（穿梭回程倒车差速）
    };

    // 设置激活的电机对
    void set_motor_pair(MotorPair pair) { _motor_pair = pair; }

    // 当前激活的电机对
    MotorPair get_motor_pair() const { return _motor_pair; }

    // 返回用于参数转换到继电器功能的继电器索引
    bool get_legacy_relay_index(int8_t &index1, int8_t &index2, int8_t &index3, int8_t &index4) const;

    // 用于保存电机限制标志的结构
    struct AP_MotorsUGV_limit {
        uint8_t steer_left      : 1; // 已到达转向控制器最左极限
        uint8_t steer_right     : 1; // 已到达转向控制器最右极限
        uint8_t throttle_lower  : 1; // 已到达油门下限
        uint8_t throttle_upper  : 1; // 已到达油门上限
    } limit;

    // 用于保存参数信息的 var_info
    static const struct AP_Param::GroupInfo var_info[];

private:

    enum PWMType {
        NORMAL = 0,
        ONESHOT = 1,
        ONESHOT125 = 2,
        BRUSHED_WITH_RELAY = 3,
        BRUSHED_BIPOLAR = 4,
        DSHOT150 = 5,
        DSHOT300 = 6,
        DSHOT600 = 7,
        DSHOT1200 = 8
    };

    // 参数合理性检查
    void sanity_check_parameters();

    // 设置 PWM 输出类型
    void setup_pwm_type();

    // 为带全向电机的机架进行设置
    void setup_omni();

    // 使用独立的油门、转向和横向因子添加全向电机
    void add_omni_motor(int8_t motor_num, float throttle_factor, float steering_factor, float lateral_factor);

    // 添加电机并设置输出功能
    void add_omni_motor_num(int8_t motor_num);

    // 禁用全向电机并移除该电机的所有油门、转向和横向因子
    void clear_omni_motors(int8_t motor_num);

    // 输出到常规转向和油门通道
    void output_regular(bool armed, float ground_speed, float steering, float throttle);

    // 输出到滑移转向通道
    void output_skid_steering(bool armed, float steering, float throttle, float dt);

    // 全向电机输出
    void output_omni(bool armed, float steering, float throttle, float lateral);

    // 向油门通道输出油门（-100 ~ +100）。需要时设置继电器
    // dt 为主循环时间间隔，需要速率控制时必须提供
    void output_throttle(SRV_Channel::Function function, float throttle, float dt = 0.0f);

    // 输出帆船主帆（范围 0 到 100）和翼帆（范围 +-100）
    void output_sail();

    // 车辆是否有主帆或翼帆
    bool has_sail() const;

    // 对油门进行一次迭代的转换速率限制
    void slew_limit_throttle(float dt);

    // 对横向进行一次迭代的转换速率限制（复用 MOT_SLEWRATE）
    void slew_limit_lateral(float dt);

    // 对油门进行一次迭代的功率限制
#if AP_BATTERY_WATT_MAX_ENABLED
    void power_limit_throttle(float dt);
#endif

    // 基于转向和油门输入设置限制
    void set_limits_from_input(bool armed, float steering, float throttle);

    // 使用 _thrust_curve_expo 参数缩放油门。油门应在 -100 到 +100 范围内
    float get_scaled_throttle(float throttle) const;

    // 使用速率控制器实现期望油门
    float get_rate_controlled_throttle(SRV_Channel::Function function, float throttle, float dt);

    // 返回功率限制，范围为 throttle_min 到 throttle_max 之间的 0~1 数字
    float get_power_limit_max_throttle(float dt);

    // 外部引用
    AP_WheelRateControl &_rate_controller;

    static const int8_t AP_MOTORS_NUM_MOTORS_MAX = 4;

    // 参数
    AP_Int8 _pwm_type;  // PWM 输出类型
    AP_UInt8 _pwm_freq;  // 有刷电机的 PWM 输出频率
    AP_Int8 _disarm_disable_pwm;    // 解锁状态下禁用 PWM 输出
    AP_Int16 _slew_rate; // 转换速率，以百分比/秒表示
    AP_Int8 _throttle_min; // 油门最小百分比
    AP_Int8 _throttle_max; // 油门最大百分比
    AP_Float _thrust_curve_expo; // 推力曲线指数，从 -1 到 +1，0 为线性
    AP_Float _thrust_asymmetry; // 非对称因子，滑移转向电机前进比后退好多少（前进/后退推力比）
    AP_Float _vector_angle_max;  // 使用矢量推力时，转向中间位置与最大位置之间的角度。零则禁用矢量推力
    AP_Float _speed_scale_base;  // 常规转向/油门车辆中，超过此速度时转向会被缩放降低。零则禁用速度缩放
    AP_Float _steering_throttle_mix; // 转向与油门优先级。数值越高越优先转向，越低越优先油门。仅对滑移转向车辆有效
    AP_Float _reverse_delay; // 电机倒车时的延迟（秒）
    AP_Float _batt_power_time_constant;    // 用于限制电池功率的时间常数

    // 内部变量
    float   _steering;  // 请求的转向值（范围 -4500 到 +4500）
    float   _throttle;  // 请求的油门值（范围 -100 到 100）
    float   _throttle_prev; // 上一次迭代的油门输入
    float   _throttle_limit = 1.0f;  // 用于限流
    bool    _scale_steering = true; // 是否应按速度或角度缩放转向
    float   _lateral;  // 请求的横向输入值（范围 -100 到 +100）
    float   _lateral_prev = 0.0f; // 上一次迭代的横向输入（用于转换速率限制）
    float   _roll;      // 请求的滚转值（范围 -1 到 +1）
    float   _pitch;     // 请求的俯仰值（范围 -1 到 +1）
    float   _walking_height; // 请求的高度值（范围 -1 到 +1）   
    float   _mainsail;  // 请求的主帆输入值（范围 0 到 100）
    float   _wingsail;  // 请求的翼帆输入值（范围 +- 100）
    float   _mast_rotation;  // 请求的桅杆旋转输入值（范围 +- 100）
    uint32_t _motor_mask;   // 配置了 pwm_type 的电机掩码
    frame_type _frame_type; // 初始化时请求的机架类型

    // 全向变量
    float   _throttle_factor[AP_MOTORS_NUM_MOTORS_MAX];
    float   _steering_factor[AP_MOTORS_NUM_MOTORS_MAX];
    float   _lateral_factor[AP_MOTORS_NUM_MOTORS_MAX];
    uint8_t   _motors_num;

    // 差速模式激活的电机对（默认全部，保持原有全向行为）
    MotorPair _motor_pair = MotorPair::ALL;

    /*
      3 个倒转处理结构体，分别用于 k_throttle、k_throttleLeft 和 k_throttleRight
     */
    struct ReverseThrottle {
        float last_throttle;
        uint32_t last_output_ms;

        // 带倒转延迟的输出
        void output(SRV_Channel::Function function, float throttle, float delay);
    } rev_delay_throttle, rev_delay_throttleLeft, rev_delay_throttleRight;

    static AP_MotorsUGV *_singleton;
};

namespace AP {
    AP_MotorsUGV *motors_ugv();
};
