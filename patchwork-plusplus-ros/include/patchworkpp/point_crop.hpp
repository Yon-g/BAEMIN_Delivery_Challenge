#pragma once

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

using PointT = pcl::PointXYZI;

class Crop
{
public:
    Crop();
    ~Crop();

private:
    void point_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber point_sub_;
    ros::Publisher point_pub_;

};