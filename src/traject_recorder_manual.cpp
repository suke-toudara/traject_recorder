#include "traject_recorder/traject_recorder_manual.hpp"
#include <cmath>

namespace traject_recorder
{
TrajectRecorderManual::TrajectRecorderAuto(const rclcpp::NodeOptions & options)
: Node("traject_recorder_manual", options),
{
}

double TrajectRecorderAuto::calculate_distance(const geometry_msgs::msg::Point &pos1, const geometry_msgs::msg::Point &pos2)
{
    return std::sqrt(std::pow(pos2.x - pos1.x, 2) + std::pow(pos2.y - pos1.y, 2));
}

void TrajectRecorderManual::save_point(const geometry_msgs::msg::Point &position)
{
    saved_points_.push_back(position);
    RCLCPP_INFO(this->get_logger(), "ポイント保存: x=%.2f, y=%.2f, z=%.2f", position.x, position.y, position.z);
}

void TrajectRecorderManual::publish_marker()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
    marker.color.a = 1.0;  
    marker.color.r = 1.0;  
    marker.color.g = 0.0;  
    marker.color.b = 0.0;  
    marker.lifetime = rclcpp::Duration(0.0);
    for (const auto &point : saved_points_) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
    }
    marker_pub_->publish(marker);
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traject_recorder::TrajectRecorderManual)
