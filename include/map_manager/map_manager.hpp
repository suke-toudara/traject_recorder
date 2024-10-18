#ifndef MAP_SERVER_HPP_
#define MAP_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace map_server
{
struct Pgm
{
  std::string header, image;
  int rows, cols, max_val, negate;
  double resolution, occupied_thresh, free_thresh;
  std::vector<unsigned char> pixels;
  std::vector<double> origin;
};

class MapServer : public rclcpp::Node
{
public:
  explicit MapServer(const rclcpp::NodeOptions & options);

private:    
  Pgm pgm;
  std::string map_yaml_path_;
  std::string mode_;

  void initMAP();
  bool loadParamsFromYaml();
  void createMAP();

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;  
};
}  // namespace map_server
#endif  