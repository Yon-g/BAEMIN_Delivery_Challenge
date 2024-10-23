#pragma once

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32MultiArray.h> 

class Crop
{
public:
    Crop();
    ~Crop();

private:
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher point_pub_;
    ros::Publisher xy_pub_;
};