#include "position_monitor.hpp"
#include <cmath>

PositionMonitor::PositionMonitor()
: Node("position_monitor")
{
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PositionMonitor::odom_callback, this, std::placeholders::_1));
    
    last_position_ = std::nullopt;
    start_time_ = this->now();
}

void PositionMonitor::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    auto current_position = msg->pose.pose.position;
    auto current_time = this->now();

    if (!last_position_) {
        // 初期位置を保存
        last_position_ = current_position;
        start_time_ = current_time;
        return;
    }

    // 移動距離を計算
    double distance = calculate_distance(*last_position_, current_position);

    // 経過時間を計算
    double elapsed_time = (current_time - start_time_).seconds();

    if (distance >= distance_threshold_ && elapsed_time >= time_threshold_) {
        // 条件を満たした場合、ポイントを保存
        save_point(current_position);
        // 位置と時間をリセット
        last_position_ = current_position;
        start_time_ = current_time;
    }
}

double PositionMonitor::calculate_distance(const geometry_msgs::msg::Point &pos1, const geometry_msgs::msg::Point &pos2)
{
    // 2点間のユークリッド距離を計算
    return std::sqrt(std::pow(pos2.x - pos1.x, 2) + std::pow(pos2.y - pos1.y, 2));
}

void PositionMonitor::save_point(const geometry_msgs::msg::Point &position)
{
    saved_points_.push_back(position);
    RCLCPP_INFO(this->get_logger(), "ポイント保存: x=%.2f, y=%.2f, z=%.2f", position.x, position.y, position.z);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionMonitor>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
