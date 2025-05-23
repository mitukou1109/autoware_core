cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_planner_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scene_module_interface.cpp
  src/planner_data.cpp
  src/utilization/path_utilization.cpp
  src/utilization/trajectory_utils.cpp
  src/utilization/arc_lane_util.cpp
  src/utilization/boost_geometry_helper.cpp
  src/utilization/util.cpp
  src/utilization/debug.cpp
)

if(BUILD_TESTING)
  file(GLOB TEST_SOURCES test/src/*.cpp)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME} ${TEST_SOURCES})
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
)
