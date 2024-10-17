#pragma once

#include "utils.h"


double GetVecNorm2(vector<float>);
double GetCenterDist(pCube, pCube);

namespace cube_tracker {
	class CubeTracker  
	{
	public:
		CubeTracker();
		~CubeTracker();

		void targetMarkerCb(const visualization_msgs::MarkerArray::ConstPtr&);
		std::vector<TrackingCube> setDetections(visualization_msgs::MarkerArray);
		std::vector<KalmanTracker> initializeTrackers(std::vector<TrackingCube>);
		// std::vector<TrackingCube> getPredictedCubes(std::vector<KalmanTracker>);
		std::pair<std::vector<cv::Point>, std::set<int>> Sort(std::vector<TrackingCube>, std::vector<TrackingCube>, double, double);
		double GetScore(pCube, pCube, double);
		visualization_msgs::MarkerArray setTextMsg(vector<TrackingCube>, int);
		visualization_msgs::MarkerArray setArrowMsg(vector<TrackingCube>);
		cube_tracker::TrackerArray setTrackedObjects(vector<TrackingCube>);
		void spin();
	private:
		ros::NodeHandle nh_;
		ros::Subscriber target_marker_sub_;
		ros::Publisher tracked_objects_pub_, bbox_tracked_pub_, text_tracked_pub_, arrow_tracked_pub_, class_tracked_pub_;

        tf::TransformListener tf_listener_;
		
		std::string target_marker_sub_topic_;
		std::string tracked_objects_pub_topic_, bbox_tracked_pub_topic_, text_tracked_pub_topic_, arrow_tracked_pub_topic_, class_tracked_pub_topic_;
		std::string frame_id_, target_frame_;

		visualization_msgs::MarkerArray target_marker_msg_;

		bool is_data_sub_, is_init_, is_transform_;
		int frame_count_, max_age_, min_hits_;
		double dist_threshold_, score_threshold_, dt_;
		double dist_thres_from_init_pos_;
		ros::Time last_time_;
		std::vector<KalmanTracker> trackers_;

	}; 

}