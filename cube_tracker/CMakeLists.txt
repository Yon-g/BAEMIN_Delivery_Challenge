cmake_minimum_required(VERSION 3.0.2)
project(cube_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# add_compile_options(-std=c++17)

find_package(PCL 1.3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  morai_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
)

generate_dynamic_reconfigure_options(
  cfg/SetClusterParam.cfg
)

add_message_files(
  FILES
  Tracker.msg
  TrackerArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions pcl_msgs pcl_ros roscpp sensor_msgs std_msgs visualization_msgs tf cv_bridge image_transport
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(cube_tracker src/cube_tracker.cpp)
add_executable(cluster src/cluster.cpp)
add_executable(dbscan src/dbscan.cpp)

add_dependencies(cube_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cluster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(dbscan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cube_tracker
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBS}
)
target_link_libraries(cluster
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBS}
)

target_link_libraries(dbscan
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBS}
)