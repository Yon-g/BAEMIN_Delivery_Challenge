#include "patchworkpp/point_crop.hpp"

Crop::Crop()
{
    point_sub_ = nh_.subscribe("/velodyne_points", 1, &Crop::point_callback, this);
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cropped_cloud", 1);

    ROS_INFO("crop node start");
}

Crop::~Crop()
{
    ROS_INFO("crop node finished");
}

void Crop::point_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // 클라우드 객체 생성 및 ROS 메시지 변환
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);

    // CropBox 필터 설정
    pcl::CropBox<PointT> cropFilter;
    cropFilter.setInputCloud(cloud);
    cropFilter.setMin(Eigen::Vector4f(-15.0, -15.0, -1.0, 1.0));
    cropFilter.setMax(Eigen::Vector4f(15.0, 15.0, 2.0, 1.0));

    // 필터 적용
    pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>);
    cropFilter.filter(*cropped_cloud);

    // 필터링된 클라우드가 비어있지 않을 경우에만 퍼블리시
    // if (!cropped_cloud->points.empty())
    // {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cropped_cloud, output);
    output.header = msg->header;
    point_pub_.publish(output);
    // }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "crop_node");
    Crop crop;
    ros::spin();
    return 0;
}
