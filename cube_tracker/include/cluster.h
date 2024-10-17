#pragma once 
#include "utils.h"
#include <dynamic_reconfigure/server.h>
#include <cube_tracker/SetClusterParamConfig.h>

Eigen::Matrix4f GeometryPoseToEigen(geometry_msgs::Pose);

namespace obstacle_cluster
{
    class  ObstacleCluster
    {
    public:
        ObstacleCluster();
        ~ObstacleCluster();
    public: 
        void dynamicReconfigureCallback(cube_tracker::SetClusterParamConfig &, uint32_t);
        void nonGroundCb(const sensor_msgs::PointCloud2::ConstPtr&);
        std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, pcl::PointCloud<pcl::PointXYZI>::Ptr> getClusters(pcl::PointCloud<pcl::PointXYZI>::Ptr, double, double, double, double,double, double,double,int, double, int, int, double);
        Eigen::VectorXf computeBbox(pcl::PointCloud<pcl::PointXYZI>::Ptr); 
        visualization_msgs::Marker setVisMarker(Eigen::VectorXf, int);
        visualization_msgs::Marker setCubeMarker(Eigen::VectorXf, int);
        visualization_msgs::Marker setTextMarker(Eigen::VectorXf, int, int);
        bool filterBbox(Eigen::VectorXf);
        void spin();

    private: 
        ros::NodeHandle nh_;

        // subscriber
        ros::Subscriber sub_nonground_;

        // publisher
        ros::Publisher pub_clusters_;
        ros::Publisher pub_bounding_boxes_cube_;
        ros::Publisher pub_bounding_boxes_vis_;
        ros::Publisher pub_text_array_;
        ros::Publisher pub_process_time_;

        dynamic_reconfigure::Server<cube_tracker::SetClusterParamConfig> server_;
        dynamic_reconfigure::Server<cube_tracker::SetClusterParamConfig>::CallbackType f_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        sensor_msgs::PointCloud2 pc_msg_;

        // topic name
        std::string nonground_sub_name_;
        std::string obstacle_array_pub_name_;
        std::string cluster_pub_name_;
        std::string cluster_sorted_pub_name_;
        std::string bbox_cube_pub_name_;
        std::string bbox_line_pub_name_;
        std::string bbox_text_pub_name_;
        std::string frame_id_name_;
        std::string lookup_frame_;

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_vector_;

        double cluster_tolerance_, eps_;
        double filter_min_x_, filter_max_x_, filter_min_y_, filter_max_y_, filter_min_z_, filter_max_z_;
        double bbox_filter_max_size_x_, bbox_filter_min_size_x_, bbox_filter_max_size_y_, bbox_filter_min_size_y_, bbox_filter_max_size_z_, bbox_filter_min_size_z_;
        double bbox_filter_max_pose_z_, bbox_filter_min_pose_z_, volume_;
        double voxel_size_;

        int core_point_min_, min_cluster_size_, max_cluster_size_;

        bool time_check_, cloud_sub_, visualize_;

    }; 
} // namespace obstacle_cluster