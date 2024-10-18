#include <traject_recorder/traject_recorder_auto.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<traject_recorder::TrajectRecorderAuto>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
