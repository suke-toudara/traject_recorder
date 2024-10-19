#ifndef TRAJECT_RECORDER_AUTO_HPP
#define TRAJECT_RECORDER_AUTO_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <optional>
#include <vector>
#include <fstream>
#include "visualization_msgs/msg/marker_array.hpp"

namespace traject_recorder
{
class TrajectRecorderAuto : public rclcpp::Node
{
public:
    explicit TrajectRecorderAuto(const rclcpp::NodeOptions & options);

private:
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void init_recoder(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    double calculate_distance(const geometry_msgs::msg::Point &pos1, const geometry_msgs::msg::Point &pos2);
    void save_point(const geometry_msgs::msg::Point &position);
    void publish_marker();
    void write_point_to_csv(const geometry_msgs::msg::Point &point);

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traject_pub_; 
    std::optional<geometry_msgs::msg::Point> last_position_;
    rclcpp::Time start_time_;
    std::vector<geometry_msgs::msg::Point> saved_points_;
    double distance_interval_;
    double sampling_time_;
    bool inti_pose = false;
    std::ofstream csv_file_;
};
} // namespace traject_recorde
#endif  // TRAJECT_RECORDER_AUTO_HPP
