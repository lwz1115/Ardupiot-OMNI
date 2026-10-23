#pragma once

#include "AR_WPNav.h"

class AR_WPNav_OA : public AR_WPNav {
public:

    // 复用父类构造函数
    using AR_WPNav::AR_WPNav;

    // 更新导航
    void update(float dt) override;

    // 设置目标位置，以及（可选的）下一目标点
    // 若已知下一目标点应提供，以实现平滑转弯
    bool set_desired_location(const Location &destination, Location next_destination = Location()) override WARN_IF_UNUSED;

    // 车辆是否已到达目标位置。默认返回 true，因为此接口通常用于任务系统，不希望任务卡死
    bool reached_destination() const override;

    // 获取经避障调整后的起点。注意：不保证有效（即未检查 _orig_and_dest_valid）
    const Location &get_oa_origin() const override;

    // 获取经避障调整后的终点。注意：不保证有效（即未检查 _orig_and_dest_valid）
    const Location &get_oa_destination() const override;

    // 返回考虑避障后的到下一航点方位（百分度），（供帆船使用）
    float oa_wp_bearing_cd() const override;

private:

    // 更新从车辆当前位置到目标的距离和方位
    void update_oa_distance_and_bearing_to_destination();

    // 避障变量
    bool _oa_active;                // 需要使用替代目标以避开障碍物时为 true
    Location _origin_oabak;         // _origin 的备份，以便避障完成后恢复
    Location _destination_oabak;    // _desitnation 的备份，以便避障完成后恢复
    Location _next_destination_oabak; // _next_destination 的备份，以便避障完成后恢复
    Location _oa_origin;            // 避障期间的中间起点
    Location _oa_destination;       // 避障期间的中间目标
    Location _oa_next_destination;  // 避障期间的中间下一目标
    float _oa_distance_to_destination; // 避障时车辆到 _oa_destination 的距离（米）
    float _oa_wp_bearing_cd;        // 经避障调整后到 _oa_destination 的方位（百分度）
};
