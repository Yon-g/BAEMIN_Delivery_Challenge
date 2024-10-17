#include "cluster.h"

Eigen::Matrix4f GeometryPoseToEigen(geometry_msgs::Pose pose_msg) {

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity(4, 4);
    Eigen::Quaternionf q(pose_msg.orientation.w,
                         pose_msg.orientation.x,
                         pose_msg.orientation.y,
                         pose_msg.orientation.z);
    T.block(0, 0, 3, 3) = q.matrix();
    T.block(0, 3, 3, 1) = Eigen::Vector3f(pose_msg.position.x,
                                          pose_msg.position.y,
                                          pose_msg.position.z);
    return T;
}

namespace obstacle_cluster 
{
    ObstacleCluster::ObstacleCluster() : nh_("~"), tf_listener_(tf_buffer_)
    {
        nh_.param("nonground_points_sub", nonground_sub_name_, std::string("nonground_pc_sub"));
        nh_.param("cluster_pub", cluster_pub_name_, std::string("clustered_points"));
        nh_.param("bbox_cube_pub", bbox_cube_pub_name_, std::string("bounding_boxes_cube"));
        nh_.param("bbox_vis_pub", bbox_line_pub_name_, std::string("bounding_boxes_vis"));
        nh_.param("bbox_text_pub", bbox_text_pub_name_, std::string("bounding_boxes_text"));
        nh_.param("frame_id", frame_id_name_, std::string("os_sensor"));
        nh_.param("lookup_frame", lookup_frame_, std::string("os_sensor"));
        nh_.param("core_point_min", core_point_min_, 10);
        nh_.param("cluster_tolerance", cluster_tolerance_, 0.4);
        nh_.param("min_cluster_size", min_cluster_size_, 10);
        nh_.param("max_cluster_size", max_cluster_size_, 1000);
        nh_.param("eps", eps_, 0.5);
        nh_.param("visualize", visualize_, true);
        nh_.param("time_check", time_check_, false);

        sub_nonground_ = nh_.subscribe<sensor_msgs::PointCloud2>(nonground_sub_name_, 1, &ObstacleCluster::nonGroundCb, this);

        pub_clusters_ = nh_.advertise<sensor_msgs::PointCloud2>(cluster_pub_name_, 1);
        pub_bounding_boxes_cube_ = nh_.advertise<visualization_msgs::MarkerArray>(bbox_cube_pub_name_, 1);
        pub_bounding_boxes_vis_  = nh_.advertise<visualization_msgs::MarkerArray>(bbox_line_pub_name_, 1);
        pub_text_array_ = nh_.advertise<visualization_msgs::MarkerArray>(bbox_text_pub_name_, 1);

        // dynamic reconfiguring
        f_ = boost::bind(&ObstacleCluster::dynamicReconfigureCallback, this, _1, _2);
        server_.setCallback(f_);        

        cloud_sub_ = false;

        ROS_INFO("ObstacleCluster node initialized");
    }

    ObstacleCluster::~ObstacleCluster()
    {
    }

    void ObstacleCluster::dynamicReconfigureCallback(cube_tracker::SetClusterParamConfig &config, uint32_t level) {
        core_point_min_ = config.core_point_min_;
        cluster_tolerance_ = config.cluster_tolerance_;
        min_cluster_size_ = config.min_cluster_size_;
        max_cluster_size_ = config.max_cluster_size_;
        voxel_size_ = config.voxel_size_;
        filter_min_x_ = config.filter_min_x;
        filter_max_x_ = config.filter_max_x;
        filter_min_y_ = config.filter_min_y;
        filter_max_y_ = config.filter_max_y;
        filter_min_z_ = config.filter_min_z;
        filter_max_z_ = config.filter_max_z;
        volume_ = config.volume;

        std::cout << "\033[1;33m" << "Reconfigure parameters : " 
                                    << "core_point_min : "       << core_point_min_ 
                                    << ", cluster_tolerance : "  << cluster_tolerance_ 
                                    << ", min_cluster_size : "   << min_cluster_size_ 
                                    << ", max_cluster_size : "   << max_cluster_size_ << std::endl
                                    << "\033[0m" << std::endl;       
    }

    void ObstacleCluster::nonGroundCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        cloud_sub_ = true;
        pc_msg_ = *msg;
    }    

    // get clusters
    std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, pcl::PointCloud<pcl::PointXYZI>::Ptr> ObstacleCluster::getClusters(
                                                                                                        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                                                                                                        double filter_min_x, double filter_max_x, 
                                                                                                        double filter_min_y, double filter_max_y,
                                                                                                        double filter_min_z, double filter_max_z,
                                                                                                        double voxel_size,
                                                                                                        int core_point_min, double cluster_tolerance, 
                                                                                                        int min_cluster_size, int max_cluster_size, double eps) {
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxelized_cloud(new pcl::PointCloud<pcl::PointXYZI>),
                                                clustered_clouds (new pcl::PointCloud<pcl::PointXYZI>),
                                                x_filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>),
                                                xy_filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>),
                                                xyz_filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_vector;
 
        clustered_clouds->header.frame_id = input_cloud->header.frame_id;
        clustered_clouds->header.stamp = input_cloud->header.stamp;

        pcl::VoxelGrid<pcl::PointXYZI> VoxelGridFilter;
        VoxelGridFilter.setInputCloud(input_cloud);
        VoxelGridFilter.setLeafSize(voxel_size, voxel_size, voxel_size);
        VoxelGridFilter.filter(*voxelized_cloud);

        pcl::PassThrough<pcl::PointXYZI> x_filter;
        x_filter.setInputCloud(voxelized_cloud);
        x_filter.setFilterFieldName("x");
        x_filter.setFilterLimits(filter_min_x, filter_max_x);
        x_filter.filter(*x_filtered_cloud);

        pcl::PassThrough<pcl::PointXYZI> y_filter;
        y_filter.setInputCloud(x_filtered_cloud);
        y_filter.setFilterFieldName("y");
        y_filter.setFilterLimits(filter_min_y, filter_max_y);
        y_filter.filter(*xy_filtered_cloud);

        pcl::PassThrough<pcl::PointXYZI> z_filter;
        z_filter.setInputCloud(xy_filtered_cloud);
        z_filter.setFilterFieldName("z");
        z_filter.setFilterLimits(filter_min_z, filter_max_z);
        z_filter.filter(*xyz_filtered_cloud);

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(xyz_filtered_cloud);
        std::vector<pcl::PointIndices> cluster_indices;

        DBSCANKdtreeCluster<pcl::PointXYZI> ec;
        ec.setCorePointMinPts(core_point_min);
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(xyz_filtered_cloud);
        ec.extract(cluster_indices);

        int num_clusters = cluster_indices.size();
        int cluster_id = 0;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, cluster_id++) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                pcl::PointXYZI temp_point;
                temp_point.x = xyz_filtered_cloud->points[*pit].x;
                temp_point.y = xyz_filtered_cloud->points[*pit].y;
                temp_point.z = xyz_filtered_cloud->points[*pit].z;
                temp_point.intensity = (float) cluster_id / (float) num_clusters * 255.f;

                temp_cloud->points.push_back(temp_point);
                clustered_clouds->points.push_back(temp_point);
            }
            clusters_vector.push_back(temp_cloud);
        }
        return std::make_pair(clusters_vector, clustered_clouds);
    }

    Eigen::VectorXf ObstacleCluster::computeBbox(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        for (auto& point : input_cloud->points) {
            pcl::PointXYZI projected_point;
            pcl::PointXYZI projected_point_upper;
            projected_point.x = point.x;
            projected_point.y = point.y;
            projected_point.z = 0.0f;
            projected_point_upper.x = point.x;
            projected_point_upper.y = point.y;
            projected_point_upper.z = 1.5f;
            projected_point.intensity = 0.0f; 
            projected_cloud->points.push_back(projected_point);
            projected_cloud->points.push_back(projected_point_upper);
        }        

        Eigen::Vector4f pca_centroid;
        pcl::compute3DCentroid(*projected_cloud, pca_centroid);
        Eigen::Matrix3f covariance; 
        pcl::computeCovarianceMatrixNormalized(*projected_cloud, pca_centroid, covariance);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigen_vectors_pca = eigen_solver.eigenvectors(); // 공분산행렬의 고유벡터
        //eigen_vectors_pca.col(2) = eigen_vectors_pca.col(0).cross(eigen_vectors_pca.col(1)); // 고유벡터를 이용해 pca의 방향 결정

        double roll, pitch, yaw;
        tf::Matrix3x3 m = tf::Matrix3x3(eigen_vectors_pca(0,0), eigen_vectors_pca(0,1), eigen_vectors_pca(0,2),
                                        eigen_vectors_pca(1,0), eigen_vectors_pca(1,1), eigen_vectors_pca(1,2),
                                        eigen_vectors_pca(2,0), eigen_vectors_pca(2,1), eigen_vectors_pca(2,2));
        m.getRPY(roll, pitch, yaw); 

        tf2::Quaternion bbox_yaw; 
        bbox_yaw.setRPY(roll, pitch, yaw); // bbox의 방향 결정

        // transform to pca
        Eigen::Matrix4f transform_matrix(Eigen::Matrix4f::Identity());
        transform_matrix.block<3,3>(0,0) = eigen_vectors_pca.transpose();
        transform_matrix.block<3,1>(0,3) = -1.f * (transform_matrix.block<3,3>(0,0) * pca_centroid.head<3>());
        pcl::transformPointCloud(*input_cloud, *transformed_cloud, transform_matrix); // 센서 좌표계의 포인트클라우드를 pca 좌표계로 변환

        // get min max
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
        const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap()); // pcl::XYZI를 Eigen::Vector3f로 mapping(pcl타입을 벡터로 매핑)
    
        const Eigen::Quaternionf bbox_quaternion(eigen_vectors_pca); // 공분산행렬을 사용해 회전을 나타내는 쿼터니언 생성
        const Eigen::Vector3f bbox_transform = eigen_vectors_pca * mean_diag + pca_centroid.head<3>(); // 클러스터의 중심점을 pca좌표로 바꾸고, 거기에 로봇기준 좌표계로 변환

        float x_len = abs(max_pt.x - min_pt.x);
        float y_len = abs(max_pt.y - min_pt.y);
        float z_len = abs(max_pt.z - min_pt.z);
        float x_center = bbox_transform[0];
        float y_center = bbox_transform[1];
        float z_center = bbox_transform[2];
        float x_orient = bbox_yaw.x();
        float y_orient = bbox_yaw.y();
        float z_orient = bbox_yaw.z();
        float w_orient = bbox_yaw.w();

        Eigen::VectorXf bbox_3d(10);
        bbox_3d << x_len, y_len, z_len, x_center, y_center, z_center, x_orient, y_orient, z_orient, w_orient;        

        return bbox_3d;    
    }

    visualization_msgs::Marker ObstacleCluster::setCubeMarker(Eigen::VectorXf input_bbox, int cluster_id) {
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.header.frame_id = frame_id_name_;
        bbox_marker.id = cluster_id;
        bbox_marker.ns = "unknown";
        bbox_marker.type = visualization_msgs::Marker::CUBE;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        bbox_marker.pose.position.x = input_bbox[3];
        bbox_marker.pose.position.y = input_bbox[4];
        bbox_marker.pose.position.z = input_bbox[5];
        bbox_marker.pose.orientation.x = input_bbox[6];
        bbox_marker.pose.orientation.y = input_bbox[7];
        bbox_marker.pose.orientation.z = input_bbox[8];
        bbox_marker.pose.orientation.w = input_bbox[9];
        bbox_marker.scale.x = input_bbox[0];
        bbox_marker.scale.y = input_bbox[1];
        bbox_marker.scale.z = input_bbox[2];
        bbox_marker.color.a = 0.5;
        bbox_marker.color.r = 0;
        bbox_marker.color.g = 0;
        bbox_marker.color.b = 1;
        bbox_marker.lifetime = ros::Duration(0.1);

        return bbox_marker;
    }

    visualization_msgs::Marker ObstacleCluster::setVisMarker(Eigen::VectorXf input_pose, int cluster_id) {
        Eigen::Vector4f min, max;

        min[0] = -input_pose[0] / 2;
        min[1] = -input_pose[1] / 2;
        min[2] = -input_pose[2] / 2;
        max[0] = input_pose[0] / 2;
        max[1] = input_pose[1] / 2;
        max[2] = input_pose[2] / 2;

        visualization_msgs::Marker vis_marker;
        vis_marker.header.frame_id = frame_id_name_;
        vis_marker.header.stamp = ros::Time::now();
        vis_marker.ns = "clustered_bbox";
        vis_marker.id = cluster_id;
        vis_marker.type = visualization_msgs::Marker::LINE_LIST;

        geometry_msgs::Point p[24];
        p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
        p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
        p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
        p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
        p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
        p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
        p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
        p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
        p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
        p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
        p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
        p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
        p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
        p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
        p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
        p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
        p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
        p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
        p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
        p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
        p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
        p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
        p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
        p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2]; 
        for (int i = 0; i < 24; i++) {
            vis_marker.points.push_back(p[i]);
        }
        vis_marker.scale.x = 0.05;
        vis_marker.color.a = 1.0;
        vis_marker.color.r = 0.0;
        vis_marker.color.g = 0.0;
        vis_marker.color.b = 1.0;
        vis_marker.pose.position.x = input_pose[3];
        vis_marker.pose.position.y = input_pose[4];
        vis_marker.pose.position.z = input_pose[5];
        vis_marker.pose.orientation.x = input_pose[6];
        vis_marker.pose.orientation.y = input_pose[7];
        vis_marker.pose.orientation.z = input_pose[8];
        vis_marker.pose.orientation.w = input_pose[9];
        vis_marker.lifetime = ros::Duration(0.1);

        return vis_marker;
    }    

    visualization_msgs::Marker ObstacleCluster::setTextMarker(Eigen::VectorXf input_bbox, int cluster_size, int cluster_id) {
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = frame_id_name_;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "clustered_bbox";
        text_marker.id = cluster_id;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = input_bbox[3];
        text_marker.pose.position.y = input_bbox[4];
        text_marker.pose.position.z = input_bbox[5] + input_bbox[2];
        text_marker.pose.orientation.x = 0.0;
        text_marker.pose.orientation.y = 0.0;
        text_marker.pose.orientation.z = 0.0;
        text_marker.pose.orientation.w = 1.0;
        text_marker.text = std::to_string(cluster_size);
        text_marker.scale.z = 0.7;
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.lifetime = ros::Duration(0.2);

        return text_marker;
    }

    bool ObstacleCluster::filterBbox(Eigen::VectorXf input_bbox) {
        if (bbox_filter_max_size_x_ < input_bbox[0] || bbox_filter_min_size_x_ > input_bbox[0])
            return false;
        if (bbox_filter_max_size_y_ < input_bbox[1] || bbox_filter_min_size_y_ > input_bbox[1])
            return false;
        if (bbox_filter_max_size_z_ < input_bbox[2] || bbox_filter_min_size_z_ > input_bbox[2])
            return false;
        if (bbox_filter_max_pose_z_ < input_bbox[5] || bbox_filter_min_pose_z_ > input_bbox[5])  
            return false;
        if (input_bbox[0] * input_bbox[1] * input_bbox[2] > volume_)
            return false;
        return true;      
    }

    void ObstacleCluster::spin() {
        if (!cloud_sub_) {
            ROS_INFO_THROTTLE(1, "clustering node : nonground Lidar data is not subscribed yet");
            return;
        }
        double start = ros::Time::now().toSec();

        sensor_msgs::PointCloud2 clustered_msg;
        visualization_msgs::MarkerArray bbox_cube_msg;
        visualization_msgs::MarkerArray bbox_line_msg;
        visualization_msgs::MarkerArray bbox_text_msg;

        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_.lookupTransform(lookup_frame_, pc_msg_.header.frame_id, ros::Time(0), ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("clustering node : %s", ex.what());
            return;
        }

        geometry_msgs::Pose sensor_pose;
        sensor_pose.position.x = transformStamped.transform.translation.x;
        sensor_pose.position.y = transformStamped.transform.translation.y;
        sensor_pose.position.z = transformStamped.transform.translation.z;
        sensor_pose.orientation = transformStamped.transform.rotation;
        Eigen::Matrix4f T = GeometryPoseToEigen(sensor_pose);

        pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_pc(new pcl::PointCloud<pcl::PointXYZI>);
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_vector;

        pcl::fromROSMsg(pc_msg_, *pc);
        pcl::transformPointCloud(*pc, *transformed_pc, T);

        auto results= getClusters(transformed_pc,
                                    filter_min_x_, filter_max_x_, 
                                    filter_min_y_, filter_max_y_,
                                    filter_min_z_, filter_max_z_,
                                    voxel_size_,
                                    core_point_min_, cluster_tolerance_, min_cluster_size_, max_cluster_size_, eps_);
        clusters_vector = results.first;
        clustered_pc = results.second;

        pcl::toROSMsg(*clustered_pc, clustered_msg);
        // visualization
        if (visualize_) {
            std::vector<Eigen::VectorXf> bbox_vector;
            for (int i = 0; i < clusters_vector.size(); i++) {
                Eigen::VectorXf bbox = computeBbox(clusters_vector[i]);
                // if (filterBbox(bbox))
                //     bbox_vector.push_back(bbox);
                bbox_vector.push_back(bbox);
            }
            for (int i = 0; i < bbox_vector.size(); i++) {
                if (bbox_vector[i][5] - bbox_vector[i][2] > eps_) continue;
                int size = clusters_vector[i]->points.size();

                visualization_msgs::Marker bbox_cube_marker = setCubeMarker(bbox_vector[i], i);
                bbox_cube_msg.markers.push_back(bbox_cube_marker);
                visualization_msgs::Marker bbox_line_marker = setVisMarker(bbox_vector[i], i);
                bbox_line_msg.markers.push_back(bbox_line_marker);
                visualization_msgs::Marker bbox_text_marker = setTextMarker(bbox_vector[i], size, i);
                bbox_text_msg.markers.push_back(bbox_text_marker);
            }
            pub_bounding_boxes_cube_.publish(bbox_cube_msg);
            pub_bounding_boxes_vis_.publish(bbox_line_msg);
            pub_text_array_.publish(bbox_text_msg);
        }
        pub_clusters_.publish(clustered_msg);
        if (time_check_) {
            double end = ros::Time::now().toSec();
            // std::cout << "\033[1;32m clustering node : process time : " << end - start << "ms \033[0m" << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster_node");
    obstacle_cluster::ObstacleCluster cluster;

    ros::Rate r(100);
    while (ros::ok()){
        cluster.spin();
        ros::spinOnce();
        r.sleep();                   
    }
    return 0;
}