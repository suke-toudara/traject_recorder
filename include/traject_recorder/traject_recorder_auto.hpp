#ifndef TRAJECT_RECORDER_AUTO_HPP
#define TRAJECT_RECORDER_AUTO_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <optional>
#include <vector>
#include <fstream>

class TrajectRecorderAuto : public rclcpp::Node
{
public:
    TrajectRecorderAuto();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    double calculate_distance(const geometry_msgs::msg::Point &pos1, const geometry_msgs::msg::Point &pos2);
    void save_point(const geometry_msgs::msg::Point &position);
    void publish_marker();
    void write_point_to_csv(const geometry_msgs::msg::Point &point);


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::optional<geometry_msgs::msg::Point> last_position_;
    rclcpp::Time start_time_;
    std::vector<geometry_msgs::msg::Point> saved_points_;
    const double distance_interval_;
    const double sampling_time_;
    
    std::ofstream csv_file_;
}
#endif  // TRAJECT_RECORDER_AUTO_HPP
