option(BUILD_JSK_PKGS "Enable building of required components of jsk_recognition_msgs and jsk_rviz_plugins" ON)

if(BUILD_JSK_PKGS)
    add_subdirectory(include/jsk_recognition_msgs)
#     add_subdirectory(include/jsk_rviz_plugins) #TODO: allow building of rviz plugins as well
endif()

cmake_minimum_required(VERSION 3.0.2)
project(patchworkpp)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        roslaunch
        cv_bridge
        pcl_conversions
        pcl_ros
        geometry_msgs
        laser_geometry
        sensor_msgs
        message_generation
        jsk_recognition_msgs
)

find_package(OpenCV REQUIRED)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
)

add_executable(patchworkpp src/patchworkpp.cpp)
target_link_libraries(patchworkpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(patchworkpp patchworkpp_generate_messages_cpp)

#add_executable(cluster src/point_cluster.cpp)
#target_link_libraries(cluster ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(crop src/scan_crop.cpp)
target_link_libraries(crop ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# ==== Install ====
install(TARGETS patchworkpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
