#include "map_manager/map_manager.hpp"

namespace map_server
{
MapServer::MapServer(const rclcpp::NodeOptions & options) 
: Node("map_server", options)
{
  declare_parameter<std::string>("map_yaml_path","map_yaml_path");
  declare_parameter<std::string>("mode","image");
  get_parameter("map_yaml_path",map_yaml_path_);
  get_parameter("mode",mode_);
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  initMAP();
}



void MapServer::initMAP(){
  loadParamsFromYaml();
  createMAP();
}

bool MapServer::loadParamsFromYaml()
{
  YAML::Node config = YAML::LoadFile(map_yaml_path_);
  try {
    pgm.resolution = config["resolution"].as<double>();
  } catch (YAML::InvalidScalar &) {
    RCLCPP_ERROR(this->get_logger(),"The map does not contain a resolution tag or it is invalid.");
    return false;
  }
  try {
    pgm.image = config["image"].as<std::string>();
  } catch (YAML::InvalidScalar &) {
    RCLCPP_ERROR(this->get_logger(),"The map does not contain a image tag or it is invalid.");
    return false;
  }
  try {
    pgm.negate = config["negate"].as<int>();
  } catch (YAML::InvalidScalar &) {
    RCLCPP_ERROR(this->get_logger(),"The map does not contain a negate tag or it is invalid.");
    return false;
  }
  try {
    pgm.occupied_thresh = config["occupied_thresh"].as<double>();
  } catch (YAML::InvalidScalar &) {
    RCLCPP_ERROR(this->get_logger(),"The map does not contain a occupied_thresh tag or it is invalid.");
    return false;
  }
  try {
    pgm.free_thresh = config["free_thresh"].as<double>();
  } catch (YAML::InvalidScalar &) {
    RCLCPP_ERROR(this->get_logger(),"The map does not contain a free_thresh tag or it is invalid.");
    return false;
  }
  return true;

  // try {
  //   pgm.origin[0] = doc["origin"][0];
  //   pgm.origin[1] = doc["origin"][1];
  //   pgm.origin[2] = doc["origin"][2];
  // } catch (YAML::InvalidScalar &) {
  //   RCLCPP_ERROR("The map does not contain a origin tag or it is invalid.");
  //   return false;
  // }
  return true;
}

void MapServer::createMAP()
{
}

}

