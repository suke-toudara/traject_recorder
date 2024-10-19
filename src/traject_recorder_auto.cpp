#include <traject_recorder/traject_recorder_auto.hpp>
#include <cmath>

namespace traject_recorder
{
TrajectRecorderAuto::TrajectRecorderAuto(const rclcpp::NodeOptions & options)
: Node("traject_recorder_auto", options)
{
    declare_parameter<double>("sampling_time",10.0);               //[s]
    declare_parameter<double>("distance_interval",10.0);           //[m]
    get_parameter("sampling_time", sampling_time_);
    get_parameter("distance_interval", distance_interval_);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/estimated_pose", 10, std::bind(&TrajectRecorderAuto::pose_callback, this, std::placeholders::_1));
    init_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&TrajectRecorderAuto::init_recoder, this, std::placeholders::_1));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_marker", 10);
    traject_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("traject_marker", 10);
    
    last_position_ = std::nullopt;
    start_time_ = this->now();

    std::string csv_file_path = "saved_points.csv" ;
    csv_file_.open(csv_file_path);
    if (!csv_file_) {
        RCLCPP_ERROR(this->get_logger(), "can't open csv file");
    } else {
        csv_file_ << "x,y,z\n";
    }
}

void TrajectRecorderAuto::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    auto current_position = msg->pose.pose.position;
    auto current_time = this->now();
    if (!inti_pose & !last_position_) {
        return;
    }

    double distance = calculate_distance(*last_position_, current_position);
    RCLCPP_INFO(this->get_logger(), "ポイント: %.2f", distance);
    
    
    double elapsed_time = (current_time - start_time_).seconds();
    if (distance >= distance_interval_ && elapsed_time >= sampling_time_) {
        save_point(current_position);
        write_point_to_csv(current_position);
        last_position_ = current_position;
        start_time_ = current_time;
        publish_marker();
         
    }
}

void TrajectRecorderAuto::init_recoder(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    auto inti_position = msg->pose.pose.position;
    auto current_time = this->now();
    if (inti_pose) {
        return;
    }
    save_point(inti_position);
    write_point_to_csv(inti_position);
    last_position_ = inti_position;
    start_time_ = current_time;
    publish_marker();
    inti_pose = true;
    RCLCPP_INFO(this->get_logger(), "初期位置設定");
}

double TrajectRecorderAuto::calculate_distance(const geometry_msgs::msg::Point &pos1, const geometry_msgs::msg::Point &pos2)
{
    return std::sqrt(std::pow(pos2.x - pos1.x, 2) + std::pow(pos2.y - pos1.y, 2));
}

void TrajectRecorderAuto::save_point(const geometry_msgs::msg::Point &position)
{
    saved_points_.push_back(position);
}

void TrajectRecorderAuto::publish_marker()
{
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::MarkerArray traject_markers;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "sphere";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
    marker.color.a = 1.0;  
    marker.color.r = 0.0;  
    marker.color.g = 1.0;  
    marker.color.b = 0.0;  
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    for (const auto &point : saved_points_) {
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = point.z;
        markers.markers.push_back(marker);
        marker.id++;  // 各マーカーに異なるIDを設定
    }
    marker_pub_->publish(markers);
    
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->now();
    line_marker.ns = "lines";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.2;  
    line_marker.color.a = 1.0;  
    line_marker.color.r = 0.0;  
    line_marker.color.g = 0.0;  
    line_marker.color.b = 1.0;  
    line_marker.lifetime = rclcpp::Duration::from_seconds(0);
    for (const auto &point : saved_points_) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        line_marker.points.push_back(p);
    }
    traject_markers.markers.push_back(line_marker); 
    traject_pub_ ->publish(traject_markers);
}

void TrajectRecorderAuto::write_point_to_csv(const geometry_msgs::msg::Point &point)
{
    if (csv_file_.is_open()) {
        csv_file_ << point.x << "," << point.y << "," << point.z << "\n";
        RCLCPP_INFO(this->get_logger(), "waypoint 保存: x=%.2f, y=%.2f, z=%.2f", point.x, point.y, point.z);
    }
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traject_recorder::TrajectRecorderAuto)
