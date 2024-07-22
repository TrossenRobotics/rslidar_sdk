cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(rslidar_sdk)

set(POINT_TYPE XYZI)
set(COMPILE_METHOD COLCON)

option(ENABLE_TRANSFORM "Enable transform functions" OFF)
if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")

  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(${ENABLE_TRANSFORM})

option(ENABLE_EPOLL_RECEIVE "Receive packets with epoll() instead of select()" OFF)
if(${ENABLE_EPOLL_RECEIVE})
  add_definitions("-DENABLE_EPOLL_RECEIVE")
endif(${ENABLE_EPOLL_RECEIVE})

option(ENABLE_DOUBLE_RCVBUF "Enable double size of RCVBUF" OFF)
if(${ENABLE_DOUBLE_RCVBUF})
  add_definitions("-DENABLE_DOUBLE_RCVBUF")
endif(${ENABLE_DOUBLE_RCVBUF})

option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
if(${ENABLE_WAIT_IF_QUEUE_EMPTY})
  add_definitions("-DENABLE_WAIT_IF_QUEUE_EMPTY")
endif(${ENABLE_WAIT_IF_QUEUE_EMPTY})

option(ENABLE_STAMP_WITH_LOCAL "Enable stamp point cloud with local time" OFF)
if(${ENABLE_STAMP_WITH_LOCAL})
  add_definitions("-DENABLE_STAMP_WITH_LOCAL")
endif(${ENABLE_STAMP_WITH_LOCAL})

option(ENABLE_SOURCE_PACKET_LEGACY "Enable ROS Source of MSOP/DIFOP Packet v1.3.x" OFF)
if(${ENABLE_SOURCE_PACKET_LEGACY})
  add_definitions("-DENABLE_SOURCE_PACKET_LEGACY")
endif(${ENABLE_SOURCE_PACKET_LEGACY})

set(PROJECT_NAME rslidar_sdk)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")  # the ros2 humble requires c++17
add_definitions(-std=c++17)
else()
add_definitions(-std=c++14)
endif()

add_compile_options(-Wall)

#========================
# Point Type Definition
#========================
if(${POINT_TYPE} STREQUAL "XYZI")
  add_definitions(-DPOINT_TYPE_XYZI)
elseif(${POINT_TYPE} STREQUAL "XYZIRT")
  add_definitions(-DPOINT_TYPE_XYZIRT)
endif()

#========================
# Dependencies Setup
#========================

find_package(ament_cmake REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp QUIET)
find_package(rclcpp_components REQUIRED)
find_package(rslidar_msg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_definitions(-DROS2_FOUND)
include_directories(${rclcpp_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 14)

# Include directory #
include_directories(
  ${PROJECT_SOURCE_DIR}/src
  include
)

# Driver core #
add_subdirectory(include/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})

set(DEPENDENCIES
  nav2_util
  rclcpp
  rclcpp_components
  rslidar_msg
  sensor_msgs
  std_msgs
)

add_library(trossen_rslidar_node SHARED
  src/trossen_rslidar_node.cpp
  src/rslidar_helper.cpp
)

add_library(trossen_rslidar_node_managed SHARED
  src/trossen_rslidar_node_managed.cpp
  src/rslidar_helper.cpp
)

rclcpp_components_register_nodes(trossen_rslidar_node "robosense::lidar::PointCloudLFNode")
rclcpp_components_register_nodes(trossen_rslidar_node_managed "robosense::lidar::PointCloudLFNodeManaged")

ament_target_dependencies(trossen_rslidar_node ${DEPENDENCIES})
ament_target_dependencies(trossen_rslidar_node_managed ${DEPENDENCIES})

add_executable(trossen_rslidar_main
  src/trossen_rslidar_main.cpp
)

ament_target_dependencies(trossen_rslidar_main ${DEPENDENCIES})

target_link_libraries(trossen_rslidar_main
  trossen_rslidar_node
  ${YAML_CPP_LIBRARIES}
  ${rs_driver_LIBRARIES}
)

install(
  TARGETS
    trossen_rslidar_node
    trossen_rslidar_node_managed
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS trossen_rslidar_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/
  DESTINATION
    include/
)

install(
  DIRECTORY
    config
    launch
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
