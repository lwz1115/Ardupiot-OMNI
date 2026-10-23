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
#include "AR_WPNav_OA.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_InternalError/AP_InternalError.h>

extern const AP_HAL::HAL& hal;

// 更新导航
void AR_WPNav_OA::update(float dt)
{
#if AP_OAPATHPLANNER_ENABLED
    // 若无当前位置、起点或目标则立即退出
    Location current_loc;
    float speed;
    if (!hal.util->get_soft_armed() || !is_destination_valid() || !AP::ahrs().get_location(current_loc) || !_atc.get_forward_speed(speed)) {
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        _oa_active = false;
        return;
    }

    // 运行绕障路径规划
    bool stop_vehicle = false;

    // 未进行避障时备份 _origin、_destination 和 _next_destination
    if (!_oa_active) {
        _origin_oabak = _origin;
        _destination_oabak = _destination;
        _next_destination_oabak = _next_destination;
    }

    AP_OAPathPlanner *oa = AP_OAPathPlanner::get_singleton();
    if (oa != nullptr) {
        Location oa_origin_new, oa_destination_new, oa_next_destination_new;
        AP_OAPathPlanner::OAPathPlannerUsed path_planner_used;
        bool dest_to_next_dest_clear;
        const AP_OAPathPlanner::OA_RetState oa_retstate = oa->mission_avoidance(current_loc,
                                                                                _origin_oabak,
                                                                                _destination_oabak,
                                                                                _next_destination_oabak,
                                                                                oa_origin_new,
                                                                                oa_destination_new,
                                                                                oa_next_destination_new,
                                                                                dest_to_next_dest_clear,
                                                                                path_planner_used);

        switch (oa_retstate) {

        case AP_OAPathPlanner::OA_NOT_REQUIRED:
            if (_oa_active) {
                // 避障已失效，因此将目标重置为原始目标
                if (!AR_WPNav::set_desired_location(_destination_oabak)) {
                    // 不应发生，因为应有 EKF 原点且目标必须有效
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    stop_vehicle = true;
                }
                _oa_active = false;
                // ToDo: handle "if (oa->get_options() & AP_OAPathPlanner::OA_OPTION_WP_RESET)"
            }
            break;

        case AP_OAPathPlanner::OA_PROCESSING:
        case AP_OAPathPlanner::OA_ERROR:
            // 处理中或出错时，让车辆减速停车
            stop_vehicle = true;
            _oa_active = false;
            break;

        case AP_OAPathPlanner::OA_SUCCESS:
            // 返回目标的处理方式取决于使用的路径规划器
            switch (path_planner_used) {

            case AP_OAPathPlanner::OAPathPlannerUsed::None:
            case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerVertical:
                // 不应发生。这表示路径规划器返回成功但返回了无效的规划器
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                _oa_active = false;
                stop_vehicle = true;
                return;

            case AP_OAPathPlanner::OAPathPlannerUsed::Dijkstras:
                // Dijkstra 算法。仅当路径规划器刚激活或目标位置的纬度/经度发生变化时才需要动作
                if (!_oa_active || !oa_destination_new.same_latlon_as(_oa_destination)) {
                    if (AR_WPNav::set_desired_location(oa_destination_new)) {
                        // 若新目标设置成功，更新避障状态和目标
                        _oa_active = true;
                        _oa_origin = oa_origin_new;
                        _oa_destination = oa_destination_new;
                    } else {
                        // 不应发生
                        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                        stop_vehicle = true;
                    }
                }
                break;

            case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerHorizontal: {
                // BendyRuler。仅当路径规划器刚激活或目标位置的纬度/经度发生变化时才需要动作
                if (!_oa_active || !oa_destination_new.same_latlon_as(_oa_destination)) {
                    if (AR_WPNav::set_desired_location_expect_fast_update(oa_destination_new)) {
                        // 若新目标设置成功，更新避障状态和目标
                        _oa_active = true;
                        _oa_origin = oa_origin_new;
                        _oa_destination = oa_destination_new;
                    } else {
                        // 不应发生
                        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                        stop_vehicle = true;
                    }
                }
            }
            break;

            } // switch (path_planner_used) {
        } // switch (oa_retstate) {
    } // if (oa != nullptr) {

    update_oa_distance_and_bearing_to_destination();

    // 避障失败时处理停车
    if (stop_vehicle) {
        // 减速到零并将转向率设为零
        _desired_speed_limited = _atc.get_desired_speed_accel_limited(0.0f, dt);
        _desired_lat_accel = 0.0f;
        _desired_turn_rate_rads = 0.0f;
        return;
    }
#endif  // AP_OAPATHPLANNER_ENABLED

    // 调用父类 update
    AR_WPNav::update(dt);
}

// 设置目标位置，以及（可选的）下一目标点
// 若已知下一目标点应提供，以实现平滑转弯
bool AR_WPNav_OA::set_desired_location(const Location& destination, Location next_destination)
{
    const bool ret = AR_WPNav::set_desired_location(destination, next_destination);

    if (ret) {
        // 禁用避障，下次更新时将按需重新启用
        _oa_active = false;
    }

    return ret;
}

// 车辆是否已到达目标位置。默认返回 true，因为此接口通常用于任务系统，不希望任务卡死
bool AR_WPNav_OA::reached_destination() const
{
    // 到达最终目标前避障应始终停用
    if (_oa_active) {
        return false;
    }

    return AR_WPNav::reached_destination();
}

// 获取经避障调整后的起点。注意：不保证有效（即未检查 _orig_and_dest_valid）
const Location &AR_WPNav_OA::get_oa_origin() const
{
    if (_oa_active) {
        return _oa_origin;
    }

    return _origin;
}

// 获取经避障调整后的终点。注意：不保证有效（即未检查 _orig_and_dest_valid）
const Location &AR_WPNav_OA::get_oa_destination() const
{
    if (_oa_active) {
        return _oa_destination;
    }

    return AR_WPNav::get_oa_destination();
}

// 返回考虑避障后的到下一航点方位（百分度），（供帆船使用）
float AR_WPNav_OA::oa_wp_bearing_cd() const
{
    if (_oa_active) {
        return _oa_wp_bearing_cd;
    }

    return AR_WPNav::oa_wp_bearing_cd();
}

// 更新从车辆当前位置到目标的距离
void AR_WPNav_OA::update_oa_distance_and_bearing_to_destination()
{
    // 更新经避障调整后的值
    Location current_loc;
    if (_oa_active && AP::ahrs().get_location(current_loc)) {
        _oa_distance_to_destination = current_loc.get_distance(_oa_destination);
        _oa_wp_bearing_cd = current_loc.get_bearing_to(_oa_destination);
    } else {
        _oa_distance_to_destination = AR_WPNav::get_distance_to_destination();
        _oa_wp_bearing_cd = AR_WPNav::wp_bearing_cd();
    }
}
