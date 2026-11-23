#ifndef LI_INIT_RUNTIME_SAFETY_H
#define LI_INIT_RUNTIME_SAFETY_H

#include <Eigen/Core>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <sensor_msgs/msg/imu.hpp>

inline bool li_init_check_due(int frame_num, int orig_odom_freq, int cut_frame_num)
{
    if (frame_num <= 0 || orig_odom_freq <= 0 || cut_frame_num <= 0)
        return false;
    const auto frames_per_second = static_cast<std::int64_t>(orig_odom_freq) * cut_frame_num;
    return frame_num % frames_per_second == 0;
}

inline void li_init_ensure_jacobian_rows(Eigen::MatrixXd &jacobian, int frame_num)
{
    if (frame_num < 0 || frame_num > (std::numeric_limits<int>::max() / 3) - 1)
        throw std::out_of_range("LI-Init Jacobian frame index is invalid");
    const Eigen::Index needed = 3LL * (static_cast<Eigen::Index>(frame_num) + 1);
    if (jacobian.rows() >= needed)
        return;
    const Eigen::Index old_rows = jacobian.rows();
    const Eigen::Index new_rows = std::max(needed, old_rows * 2);
    jacobian.conservativeResize(new_rows, 3);
    jacobian.bottomRows(new_rows - old_rows).setZero();
}

inline bool li_init_valid_imu(const sensor_msgs::msg::Imu &msg)
{
    const auto &a = msg.linear_acceleration;
    const auto &w = msg.angular_velocity;
    return msg.header.stamp.nanosec < 1000000000u &&
        std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z) &&
        std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z);
}

#endif
