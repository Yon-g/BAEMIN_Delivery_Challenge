#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <vector>
#include <list>
#include <stdint.h>
#include <time.h>
//#include "types.h"
#include <iomanip> // to format image names using setw() and setfill()
#include <unistd.h>
#include <set>
#include <sstream>

#include <Eigen/Dense>

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ros/ros.h"

#include <ros/ros.h>

//MESSAGES
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/publisher.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/gicp.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

//Tracker Library

#include "Hungarian.cpp"
#include "KalmanTracker.cpp"

//TF
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>    
//VISUALIZATION
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/core/types.hpp>

#include "cube_tracker/Tracker.h"
#include "cube_tracker/TrackerArray.h"

#include <dynamic_reconfigure/server.h>
#include <cube_tracker/SetClusterParamConfig.h>

#include "DBSCAN_simple.h"
#include "DBSCAN_precomp.h"
#include "DBSCAN_kdtree.h"