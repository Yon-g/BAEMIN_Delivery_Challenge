#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float32MultiArray.h>  // XY 배열 데이터
#include <sensor_msgs/PointCloud2.h>     // PointCloud 데이터
#include <morai_msgs/EgoDdVehicleStatus.h> // Local ODOM 데이터를 구독하기 위한 메시지
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <deque>

class ObstacleDetection {
public:
    ObstacleDetection() {
        // 전처리된 XY 배열 구독
        xy_sub_ = nh_.subscribe("/xy_points", 1, &ObstacleDetection::xyCallback, this);
        // Local ODOM 데이터 구독 (UTM 좌표 포함)
        odom_sub_ = nh_.subscribe("/Local/odom", 1, &ObstacleDetection::odomCallback, this);

        // 고정 및 동적 장애물 PointCloud 퍼블리셔
        static_obs_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/static_obstacles", 1);
        dynamic_obs_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_obstacles", 1);

        // 고정 및 동적 장애물 Float32MultiArray 퍼블리셔 (UTM 좌표계 기준)
        static_obs_array_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/static_obstacle_array", 1);
        dynamic_obs_array_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/dynamic_obstacle_array", 1);

        ROS_INFO("Obstacle detection node started. Node name: %s", ros::this_node::getName().c_str());


        // LiDAR와 GPS 센서의 상대적 위치 차이 (로봇 중심 기준, 단위: meters)
        lidar_offset_x = 0.2;
        lidar_offset_y = 0.0;
        lidar_offset_z = 0.7;

        gps_utm_x = 0.0;
        gps_utm_y = 0.0;

        prev_utm_x = 0.0;
        prev_utm_y = 0.0;
    }

    // Local ODOM 콜백 함수에서 UTM 좌표를 업데이트
    void odomCallback(const morai_msgs::EgoDdVehicleStatus::ConstPtr& msg) {
        if (prev_utm_x == 0.0 && prev_utm_y == 0.0) {
            prev_utm_x = msg->position.x;
            prev_utm_y = msg->position.y;
        } else {
            prev_utm_x = gps_utm_x;
            prev_utm_y = gps_utm_y;
        }

        gps_utm_x = msg->position.x;  // UTM X 좌표
        gps_utm_y = msg->position.y;  // UTM Y 좌표
    }

    void xyCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        std::vector<std::vector<double>> points;

        ROS_INFO("Received XY points with size: %zu", msg->data.size());

        std_msgs::Float32MultiArray static_array;
        std_msgs::Float32MultiArray dynamic_array;

        // Float32MultiArray로부터 x, y 좌표 추출
        for (size_t i = 0; i < msg->data.size(); i += 2) {
            double x = msg->data[i];
            double y = msg->data[i + 1];

            // LiDAR 기준 상대좌표로 변환 (PointCloud)
            double relative_x = x;  // LiDAR 기준 상대좌표
            double relative_y = y;

            // UTM 좌표계 변환 (Float32MultiArray)
            double utm_x = gps_utm_x + x + lidar_offset_x;
            double utm_y = gps_utm_y + y + lidar_offset_y;

            // 10m 이내인 경우만 저장 (PointCloud 및 UTM)
            if (distanceFromRobot(utm_x, utm_y) <= 4.2) {
                points.push_back({relative_x, relative_y});

                static_array.data.push_back(utm_x);  // UTM 좌표
                static_array.data.push_back(utm_y);
            }
        }

        if (!points.empty()) {
            applyDBSCAN(points, static_array, dynamic_array);
        }
    }

    void applyDBSCAN(const std::vector<std::vector<double>>& points, std_msgs::Float32MultiArray& static_array, std_msgs::Float32MultiArray& dynamic_array) {
        // DBSCAN 파라미터 설정
        double epsilon = 0.5; // 장애물 간 거리 기준
        int minPts = 1;       // 군집을 형성할 최소 포인트 개수

        std::vector<int> labels(points.size(), -1); // 군집 레이블
        int cluster_id = 0;

        std::vector<std::vector<double>> cluster_centroids;

        for (size_t i = 0; i < points.size(); ++i) {
            if (labels[i] != -1) continue; // 이미 처리된 포인트는 건너뛰기

            std::vector<int> neighbors = regionQuery(points, i, epsilon);

            if (neighbors.size() < minPts) {
                // 노이즈로 처리
                labels[i] = -1;
            } else {
                // 새로운 클러스터 생성
                expandCluster(points, labels, i, neighbors, cluster_id, epsilon, minPts);
                cluster_centroids.push_back(calculateCentroid(points, labels, cluster_id));
                ++cluster_id;
            }
        }
        if (cluster_centroids.empty()) {
            ROS_WARN("No clusters detected.");
        }

        // 고정 및 동적 장애물 분리 및 퍼블리시
        processObstacles(points, labels, cluster_centroids, static_array, dynamic_array);
        // 클러스터 중심 좌표 업데이트
        prev_cluster_centroids = cluster_centroids;  // 이번에 구한 클러스터 중심을 저장
    }

    std::vector<int> regionQuery(const std::vector<std::vector<double>>& points, int index, double epsilon) {
        std::vector<int> neighbors;
        for (size_t i = 0; i < points.size(); ++i) {
            double dist = sqrt(pow(points[i][0] - points[index][0], 2) + pow(points[i][1] - points[index][1], 2));
            if (dist <= epsilon) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    }

    void expandCluster(const std::vector<std::vector<double>>& points, std::vector<int>& labels, int index,
                       const std::vector<int>& neighbors, int cluster_id, double epsilon, int minPts) {
        labels[index] = cluster_id;

        std::vector<int> search_queue = neighbors;
        size_t i = 0;
        while (i < search_queue.size()) {
            int pt_index = search_queue[i];
            if (labels[pt_index] == -1) {
                // 이전에 노이즈로 처리된 포인트가 클러스터에 포함될 수 있음
                labels[pt_index] = cluster_id;
            }
            if (labels[pt_index] == -1) {
                labels[pt_index] = cluster_id;
                std::vector<int> pt_neighbors = regionQuery(points, pt_index, epsilon);
                if (pt_neighbors.size() >= minPts) {
                    search_queue.insert(search_queue.end(), pt_neighbors.begin(), pt_neighbors.end());
                }
            }
            ++i;
        }
    }

    std::vector<double> calculateCentroid(const std::vector<std::vector<double>>& points, const std::vector<int>& labels, int cluster_id) {
        double sum_x = 0.0;
        double sum_y = 0.0;
        int count = 0;

        for (size_t i = 0; i < points.size(); ++i) {
            if (labels[i] == cluster_id) {
                sum_x += points[i][0];
                sum_y += points[i][1];
                count++;
            }
        }

        return {sum_x / count, sum_y / count};
    }

    void publishObstacles(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, ros::Publisher& cloud_pub, std_msgs::Float32MultiArray& array, ros::Publisher& array_pub, const std::string& type, int count) {
        if (cloud->points.size() > 0) {
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);
            output.header.frame_id = "velodyne";  // LiDAR 기준 좌표
            cloud_pub.publish(output);
            array_pub.publish(array); // UTM 좌표
            ROS_INFO("Published %d %s obstacles", count, type.c_str());
        } else {
            ROS_WARN("No %s obstacles to publish", type.c_str());
        }
    }

    void processObstacles(const std::vector<std::vector<double>>& points, const std::vector<int>& labels, const std::vector<std::vector<double>>& centroids, std_msgs::Float32MultiArray& static_array, std_msgs::Float32MultiArray& dynamic_array) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        static const double movement_threshold = 1;  // 클러스터가 고정 장애물로 간주될 최소 이동 거리
        static const int static_cluster_size_threshold = 1;  // 클러스터 크기 임계값

        int static_count = 0;
        int dynamic_count = 0;

        // 클러스터 크기 확인하여 정적/동적 장애물 분리
        std::vector<int> cluster_sizes(centroids.size(), 0);
        for (size_t i = 0; i < labels.size(); ++i) {
            if (labels[i] >= 0) {
                cluster_sizes[labels[i]]++;
            }
        }

        for (size_t i = 0; i < points.size(); ++i) {
            pcl::PointXYZ point;
            point.x = points[i][0];  // LiDAR 기준 상대좌표
            point.y = points[i][1];
            point.z = 0;

            if (labels[i] < 0 || labels[i] >= centroids.size()) {
                continue;
            }

            // 클러스터 크기가 임계값 이상일 경우 정적 장애물로 간주
            if (cluster_sizes[labels[i]] >= static_cluster_size_threshold) {
                static_cloud->points.push_back(point);  // 정적 장애물
                static_array.data.push_back(gps_utm_x + points[i][0]);
                static_array.data.push_back(gps_utm_y + points[i][1]);
                static_count++;
            } else if (clusterMoved(centroids[labels[i]], movement_threshold)) {
                // 그렇지 않은 경우 중심점 이동 여부 확인
                dynamic_cloud->points.push_back(point);  // 동적 장애물
                dynamic_array.data.push_back(gps_utm_x + points[i][0]);
                dynamic_array.data.push_back(gps_utm_y + points[i][1]);
                dynamic_count++;
            } else {
                static_cloud->points.push_back(point);  // 정적 장애물
                static_array.data.push_back(gps_utm_x + points[i][0]);
                static_array.data.push_back(gps_utm_y + points[i][1]);
                static_count++;
            }
        }

        // 최근 고정 장애물 좌표를 기록
        if (static_obstacle_history.size() >= history_size) {
            static_obstacle_history.pop_front();
        }
        // static_array.data (std::vector<float>)를 std::vector<std::vector<double>>로 변환하여 저장
        std::vector<std::vector<double>> static_obstacles;
        for (size_t i = 0; i < static_array.data.size(); i += 2) {
            static_obstacles.push_back({static_array.data[i], static_array.data[i + 1]});
        }
        static_obstacle_history.push_back(static_obstacles);
        
        publishObstacles(static_cloud, static_obs_pub_, static_array, static_obs_array_pub_, "static", static_count);
        publishObstacles(dynamic_cloud, dynamic_obs_pub_, dynamic_array, dynamic_obs_array_pub_, "dynamic", dynamic_count);

        // 클러스터 중심 좌표 업데이트
        prev_cluster_centroids = centroids;
    }

    double distanceFromRobot(double utm_x, double utm_y) {
        // 로봇과 장애물 사이의 거리를 계산 (UTM 기준)
        double dx = utm_x - gps_utm_x;
        double dy = utm_y - gps_utm_y;
        double distance = sqrt(dx * dx + dy * dy);
        ROS_INFO("Calculated distance: %f", distance);
        return distance;
    }

    bool clusterMoved(const std::vector<double>& current_centroid, double threshold) {
        double min_distance = std::numeric_limits<double>::max();
        int matched_cluster_idx = -1;

        // 이전 클러스터 중심 중에서 가장 가까운 클러스터를 찾음
        for (size_t i = 0; i < prev_cluster_centroids.size(); ++i) {
            double dx = (current_centroid[0] + gps_utm_x - prev_utm_x) - prev_cluster_centroids[i][0];
            double dy = (current_centroid[1] + gps_utm_y - prev_utm_y) - prev_cluster_centroids[i][1];
            double dist = sqrt(dx * dx + dy * dy);

            if (dist < min_distance) {
                min_distance = dist;
                matched_cluster_idx = i;
            }
        }

        // 이동 거리와 threshold 비교
        if (min_distance <= threshold) {
            return false;  // 이동하지 않음 (static)
        }
        return true;  // 일정 거리 이상 이동 (dynamic)
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber xy_sub_;              // XY 배열 데이터 구독
    ros::Subscriber odom_sub_;            // Local ODOM 데이터 구독 (UTM 좌표 포함)
    ros::Publisher static_obs_pub_;       // 고정 장애물 퍼블리시
    ros::Publisher dynamic_obs_pub_;      // 동적 장애물 퍼블리시
    ros::Publisher static_obs_array_pub_; // 고정 장애물 퍼블리시 (Float32MultiArray)
    ros::Publisher dynamic_obs_array_pub_;// 동적 장애물 퍼블리시 (Float32MultiArray)

    // GPS 및 LiDAR 위치 오프셋 관련 변수
    double lidar_offset_x, lidar_offset_y, lidar_offset_z;
    double gps_utm_x, gps_utm_y;
    double prev_utm_x, prev_utm_y;

    // 이전 클러스터 중심 좌표
    std::vector<std::vector<double>> prev_cluster_centroids;

    std::deque<std::vector<std::vector<double>>> static_obstacle_history;
    const int history_size = 100; // 10 프레임 동안의 장애물 기록 유지
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detection");
    ObstacleDetection detector;
    ros::spin();
    return 0;
}

