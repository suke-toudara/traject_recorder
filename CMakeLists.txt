cmake_minimum_required(VERSION 3.8)
project(traject_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(traject_recorder_auto SHARED
  src/traject_recorder_auto.cpp
)

rclcpp_components_register_nodes(traject_recorder_auto
  "traject_recorder::TrajectRecorderAuto"
)

ament_auto_add_executable(traject_recorder_node
    NO_TARGET_LINK_LIBRARIES
    src/traject_recorder_node.cpp
)

target_link_libraries(traject_recorder_node
  traject_recorder_auto
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES DESTINATION share/${PROJECT_NAME})

ament_auto_package()
