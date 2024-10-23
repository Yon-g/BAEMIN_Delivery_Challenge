#include "patchworkpp/point_crop.hpp"


Crop::Crop()
{
    // LaserScan 데이터를 구독
    scan_sub_ = nh_.subscribe("/scan", 1, &Crop::scan_callback, this);
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cropped_downsampled_scan", 1);
    xy_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/xy_points", 1);  // x, y 배열 퍼블리셔

    ROS_INFO("LaserScan crop and downsampling node with XY array publishing start");
}

Crop::~Crop()
{
    ROS_INFO("LaserScan crop and downsampling node finished");
}

void Crop::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // LaserScan 데이터를 PointCloud로 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std_msgs::Float32MultiArray xy_array;  // x, y 좌표를 저장할 배열

    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        float range = msg->ranges[i];
        if (range >= msg->range_min && range <= msg->range_max)
        {
            // 각도와 범위를 사용하여 X, Y 좌표 계산
            float x = range * cos(angle);
            float y = range * sin(angle);

            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = 0;  // LaserScan은 2D 데이터이므로 Z는 0

            cloud->points.push_back(point);
        }
        angle += msg->angle_increment;
    }

    // CropBox 필터 적용
    pcl::CropBox<pcl::PointXYZ> cropFilter;
    cropFilter.setMin(Eigen::Vector4f(-7.0, -15.0, -1.0, 1.0));  // 크롭 박스 최소값
    cropFilter.setMax(Eigen::Vector4f(15.0, 15.0, 2.0, 1.0));     // 크롭 박스 최대값
    cropFilter.setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cropFilter.filter(*cropped_cloud);

    // VoxelGrid 필터 적용 (다운샘플링)
    pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
    voxelFilter.setInputCloud(cropped_cloud);
    voxelFilter.setLeafSize(0.1f, 0.1f, 0.1f);  // 그리드 셀 크기 설정 (단위: meters)

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxelFilter.filter(*downsampled_cloud);

    // 다운샘플링된 포인트 클라우드를 퍼블리시
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*downsampled_cloud, output);
    output.header = msg->header;
    point_pub_.publish(output);

    // 다운샘플링된 클라우드에서 x, y 좌표를 배열에 추가
    for (const auto& point : downsampled_cloud->points)
    {
        xy_array.data.push_back(point.x);
        xy_array.data.push_back(point.y);
    }

    // x, y 좌표 배열 퍼블리시
    xy_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    xy_array.layout.dim[0].label = "points";
    xy_array.layout.dim[0].size = xy_array.data.size() / 2;  // 각 포인트마다 x, y 두 개의 좌표값
    xy_array.layout.dim[0].stride = 2;
    xy_pub_.publish(xy_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "crop_node");
    Crop crop;
    ros::spin();
    return 0;
}