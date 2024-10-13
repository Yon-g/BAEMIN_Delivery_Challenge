#include "patchworkpp/point_cluster.hpp"

Cluster::Cluster()
{
    point_sub_ = nh_.subscribe("/ground_segmentation/nonground", 1, &Cluster::point_callback, this);
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/clustered_cloud", 1);

    ROS_INFO("cluster node start");
}

Cluster::~Cluster()
{
    ROS_INFO("cluster node finished");
}


void Cluster::point_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.2); // 클러스터링 거리 허용 오차 (m)
    ec.setMinClusterSize(200); // 최소 클러스터 크기
    ec.setMaxClusterSize(2000); // 최대 클러스터 크기
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<PointT> clustered_point;

    int color = 0;
    clusters.clear();
    for (const auto& it : cluster_indices)
    {
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for (const auto& index : it.indices)
        {
            cluster->points.push_back(cloud->points[index]);

            const PointT& pt = cloud->points[index];
            PointT pt2;
            pt2.x = pt.x;
            pt2.y = pt.y;
            pt2.z = pt.z;
            pt2.intensity = static_cast<float>(color++);

            clustered_point.push_back(pt2);
        }
        
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
        
    }

    pcl::PCLPointCloud2 cloud_out;
    pcl::toPCLPointCloud2(clustered_point, cloud_out);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_out, output);
    output.header = msg->header;

    point_pub_.publish(output);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "cluster_node");
    Cluster cluster;
    ros::spin();
    return 0;
}