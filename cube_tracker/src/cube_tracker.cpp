#include "cube_tracker.h"


namespace cube_tracker {

	CubeTracker::CubeTracker() : nh_("~")
	{
		nh_.param("target_marker_sub_topic", target_marker_sub_topic_, std::string("/target_marker"));

		nh_.param("tracked_objects_pub_topic", tracked_objects_pub_topic_, std::string("/obstacle_cloud"));
		nh_.param("bbox_tracked_pub_topic", bbox_tracked_pub_topic_, std::string("/bbox_tracked"));
		nh_.param("text_tracked_pub_topic", text_tracked_pub_topic_, std::string("/text_tracked"));
		nh_.param("arrow_tracked_pub_topic", arrow_tracked_pub_topic_, std::string("/arrow_tracked"));
		nh_.param("class_tracked_pub_topic", class_tracked_pub_topic_, std::string("/class_tracked"));

		//params
		nh_.param("frame_id", frame_id_, std::string("os_sensor"));
		nh_.param("max_age", max_age_, 10);
		nh_.param("min_hits", min_hits_, 3);
		nh_.param("score_threshold", score_threshold_, 0.3);
		nh_.param("dist_threshold", dist_threshold_, 1.0);
		nh_.param("dist_thres_from_init_pos", dist_thres_from_init_pos_, 2.0);
		nh_.param("transform", is_transform_, false);
		nh_.param("transform_target_frame", target_frame_, std::string("map"));

		target_marker_sub_ = nh_.subscribe<visualization_msgs::MarkerArray>(target_marker_sub_topic_, 1, &CubeTracker::targetMarkerCb, this);

		tracked_objects_pub_ = nh_.advertise<cube_tracker::TrackerArray>(tracked_objects_pub_topic_, 1);
		bbox_tracked_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(bbox_tracked_pub_topic_, 1);
		text_tracked_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(text_tracked_pub_topic_, 1);
		arrow_tracked_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(arrow_tracked_pub_topic_, 1);
		class_tracked_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(class_tracked_pub_topic_, 1);

		is_data_sub_ = false;
		is_init_ = true;
		frame_count_ = 0;
		dt_ = 1.;
		last_time_ = ros::Time::now();
		trackers_.clear();
	}

	CubeTracker::~CubeTracker() 
	{
	}

	void CubeTracker::targetMarkerCb(const visualization_msgs::MarkerArray::ConstPtr& msg) {
		ros::Time current_time = ros::Time::now();
		ros::Duration time_difference = current_time - last_time_;
		dt_ = time_difference.toSec();
		ROS_DEBUG_STREAM("tracker_node : dt_ = " << dt_);
		last_time_ = current_time;
		target_marker_msg_ = *msg;
		is_data_sub_ = true;
	}

	std::vector<TrackingCube> CubeTracker::setDetections(visualization_msgs::MarkerArray msg) {
		std::vector<TrackingCube> detected_objects;
		for (int i = 0; i < msg.markers.size(); i++) {
			TrackingCube detected_object;
			detected_object.cube.stateVec[0] = msg.markers[i].pose.position.x;
			detected_object.cube.stateVec[1] = msg.markers[i].pose.position.y;
			detected_object.cube.stateVec[2] = msg.markers[i].pose.position.z;
			detected_object.cube.stateVec[3] = 0.0;
			detected_object.cube.stateVec[4] = 0.0;
			detected_object.cube.stateVec[5] = 0.0;
			detected_object.cube.sizeVec[0] = msg.markers[i].scale.x;
			detected_object.cube.sizeVec[1] = msg.markers[i].scale.y;
			detected_object.cube.sizeVec[2] = msg.markers[i].scale.z;
			detected_object.cube.quatVec[0] = msg.markers[i].pose.orientation.x;
			detected_object.cube.quatVec[1] = msg.markers[i].pose.orientation.y;
			detected_object.cube.quatVec[2] = msg.markers[i].pose.orientation.z;
			detected_object.cube.quatVec[3] = msg.markers[i].pose.orientation.w;
			detected_object.id = msg.markers[i].id;
			detected_object.dt = dt_;
			detected_object.class_name = msg.markers[i].ns;
			detected_objects.push_back(detected_object);
		}
		return detected_objects;
	}

	std::vector<KalmanTracker> CubeTracker::initializeTrackers(std::vector<TrackingCube> inputs) {
		std::vector<KalmanTracker> trackers;
		for (int i = 0; i < inputs.size(); i++) {
			KalmanTracker tracker = KalmanTracker(inputs[i]); // initKF
			trackers.push_back(tracker);
		}
		return trackers;
	}

	std::pair<std::vector<cv::Point>, std::set<int>> CubeTracker::Sort(std::vector<TrackingCube> trk, std::vector<TrackingCube> det, double score_thres, double dist_thres) {
		unsigned int trk_num = trk.size();
		unsigned int det_num = det.size();
		std::vector<cv::Point> matched_pairs;
		std::vector<std::vector<double>> score_matrix;
		std::vector<int> assignment;

		std::set<int> unmatched_trk, unmatched_det, all_items, matched_items;

		if (trk_num == 0 || det_num == 0) {
			return std::make_pair(matched_pairs, unmatched_det);
		}

		score_matrix.clear();
		score_matrix.resize(trk_num, std::vector<double>(det_num, 0.0)); // row : trk, col : det

        for (unsigned int i = 0; i < trk_num; i++) // compute iou matrix as a distance matrix
		{
			for (unsigned int j = 0; j < det_num; j++)
			{
				// use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
				score_matrix[i][j] = GetScore(trk[i].cube, det[j].cube, dist_thres) / 100; // 낮은 값일수록 점수 높음
			}
		}    

		// solve the assignment problem using hungarian algorithm.
		HungarianAlgorithm HungAlgo;
		assignment.clear();
		HungAlgo.Solve(score_matrix, assignment);

		unmatched_trk.clear();
        unmatched_det.clear();
        all_items.clear();
        matched_items.clear();

		if (det_num > trk_num) {
			for (unsigned int n = 0; n < det_num; n++)
				all_items.insert(n);

			for (unsigned int n = 0; n < trk_num; n++)
				matched_items.insert(assignment[n]);

			std::set_difference(all_items.begin(), all_items.end(), 
								matched_items.begin(), matched_items.end(), 
								std::insert_iterator<std::set<int>>(unmatched_det, unmatched_det.begin()));
		}
		else if (det_num < trk_num) {
            for (unsigned int i = 0; i < trk_num; ++i) 
                if (assignment[i] == -1) 
                    unmatched_trk.insert(i);			
		}

		matched_pairs.clear();
        for (unsigned int i = 0; i < trk_num; i++) {
            if (assignment[i] == -1) 
                continue;
            if (score_matrix[i][assignment[i]] > dist_thres/100) {
                unmatched_trk.insert(i);
                unmatched_det.insert(assignment[i]);
            }
            else {
                matched_pairs.push_back(cv::Point(i, assignment[i])); // 각 인덱스의 의미 : i = trk, assignment[i] = det
            }
        }
        ROS_DEBUG_STREAM("matched pairs : " << matched_pairs); 

		return std::make_pair(matched_pairs, unmatched_det);
	}

    double CubeTracker::GetScore(pCube bbox1, pCube bbox2, double dist_threshold) {	

        Eigen::Vector3f bbox1_center(bbox1.stateVec[0], bbox1.stateVec[1], bbox1.stateVec[2]);
        Eigen::Vector3f bbox2_center(bbox2.stateVec[0], bbox2.stateVec[1], bbox2.stateVec[2]);

        double dist = std::abs((bbox1_center - bbox2_center).norm());
        if (dist > dist_threshold)
            return 100.0;

        return dist;
    }

	visualization_msgs::MarkerArray CubeTracker::setTextMsg(vector<TrackingCube> tracked_results, int type) { // 1:name, 2:vel
		visualization_msgs::MarkerArray marker_array;
		
		for (int i = 0; i < tracked_results.size(); ++i) {
			visualization_msgs::Marker marker;
			marker.header.frame_id = frame_id_;
			marker.header.stamp = ros::Time::now();
			marker.ns = "text_tracked";
			marker.id = i;
			marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = tracked_results[i].cube.stateVec[0];
			marker.pose.position.y = tracked_results[i].cube.stateVec[1];
			marker.pose.position.z = tracked_results[i].cube.stateVec[2] + 1.5;
			marker.color.a = 1.0;
			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 1.0;
			marker.lifetime = ros::Duration(0.13);
			marker.text = "";
			float speed = std::sqrt(std::pow(tracked_results[i].cube.stateVec[3], 2)
									+ std::pow(tracked_results[i].cube.stateVec[4], 2)
									+ std::pow(tracked_results[i].cube.stateVec[5], 2));
			if (type == 1) {
				marker.text = tracked_results[i].class_name;
				if (tracked_results[i].class_name != "unknown")
					marker.scale.z = 2.0;
			}
			else if (type == 2) {
				if (speed > 0.3) {
					// marker.text = "SPEED : " + std::to_string(speed * 3.6) + "km/h";
					std::ostringstream stream;
					stream << std::fixed << std::setprecision(1) << (speed);
					marker.text = "SPEED : " + stream.str() + " m/s" + " id : " + std::to_string(tracked_results[i].id);
					marker.scale.z = 0.7;
				}
			}
			float dist_from_init_pos;
			dist_from_init_pos = std::sqrt(std::pow(tracked_results[i].cube.stateVec[0] - tracked_results[i].x, 2)
										+ std::pow(tracked_results[i].cube.stateVec[1] - tracked_results[i].y, 2)
										+ std::pow(tracked_results[i].cube.stateVec[2] - tracked_results[i].z, 2));

			if (dist_from_init_pos < dist_thres_from_init_pos_ && speed < 0.4) {
				continue;
			}			
			marker.color.b = 0.0;
			marker_array.markers.emplace_back(marker);
		}
		return marker_array;
	}

	visualization_msgs::MarkerArray CubeTracker::setArrowMsg(vector<TrackingCube> tracked_results) {
		visualization_msgs::MarkerArray marker_array;
		for (int i=0; i<tracked_results.size(); ++i) {
			visualization_msgs::Marker marker;
			marker.header.frame_id = frame_id_;
			marker.header.stamp = ros::Time::now();
			marker.ns = "arrow_tracked";
			marker.id = i;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.orientation.x = 0;
			marker.pose.orientation.y = 0;
			marker.pose.orientation.z = 0;
			marker.pose.orientation.w = 1;

			float z_tr = 1.0;
			float vel_scale = 1.0;
			marker.points.resize(2);
			marker.points[0].x = tracked_results[i].cube.stateVec[0];
			marker.points[0].y = tracked_results[i].cube.stateVec[1];
			marker.points[0].z = tracked_results[i].cube.stateVec[2] + z_tr;
			marker.points[1].x = tracked_results[i].cube.stateVec[0] + tracked_results[i].cube.stateVec[3] * vel_scale;
			marker.points[1].y = tracked_results[i].cube.stateVec[1] + tracked_results[i].cube.stateVec[4] * vel_scale;
			marker.points[1].z = tracked_results[i].cube.stateVec[2] + z_tr + tracked_results[i].cube.stateVec[5] * vel_scale;

			marker.scale.x = 0.17;
			marker.scale.y = 0.17;
			marker.scale.z = 0.17;
			marker.color.a = 1.0;
			marker.color.r = 1.0;
			marker.color.g = 0;
			marker.color.b = 0;
			marker.lifetime = ros::Duration(0.13);
			marker_array.markers.emplace_back(marker);
		}
		return marker_array;
	}

	cube_tracker::TrackerArray CubeTracker::setTrackedObjects(vector<TrackingCube> tracked_results) {
		cube_tracker::TrackerArray tracked_object_array;
		for (int i = 0; i < tracked_results.size(); i++) {
			cube_tracker::Tracker tracked_object;
			geometry_msgs::PoseStamped pose_stamped;
			geometry_msgs::TwistStamped twist_stamped;

			pose_stamped.header.frame_id = frame_id_;
			pose_stamped.header.stamp = ros::Time::now();
			pose_stamped.pose.position.x = tracked_results[i].cube.stateVec[0];
			pose_stamped.pose.position.y = tracked_results[i].cube.stateVec[1];
			pose_stamped.pose.position.z = tracked_results[i].cube.stateVec[2];
			pose_stamped.pose.orientation.x = tracked_results[i].cube.quatVec[0];
			pose_stamped.pose.orientation.y = tracked_results[i].cube.quatVec[1];
			pose_stamped.pose.orientation.z = tracked_results[i].cube.quatVec[2];
			pose_stamped.pose.orientation.w = tracked_results[i].cube.quatVec[3];

			twist_stamped.header.frame_id = frame_id_;
			twist_stamped.header.stamp = ros::Time::now();
			twist_stamped.twist.linear.x = tracked_results[i].cube.stateVec[3];
			twist_stamped.twist.linear.y = tracked_results[i].cube.stateVec[4];
			twist_stamped.twist.linear.z = tracked_results[i].cube.stateVec[5];

			tracked_object.pose = pose_stamped;
			tracked_object.twist = twist_stamped;
			tracked_object.track_id = tracked_results[i].id;
			float speed = std::sqrt(std::pow(tracked_results[i].cube.stateVec[3], 2)
									+ std::pow(tracked_results[i].cube.stateVec[4], 2)
									+ std::pow(tracked_results[i].cube.stateVec[5], 2));
			float dist_from_init_pos;
			dist_from_init_pos = std::sqrt(std::pow(tracked_results[i].cube.stateVec[0] - tracked_results[i].x, 2)
										+ std::pow(tracked_results[i].cube.stateVec[1] - tracked_results[i].y, 2)
										+ std::pow(tracked_results[i].cube.stateVec[2] - tracked_results[i].z, 2));
			// if (dist_from_init_pos < dist_thres_from_init_pos_ && speed < 0.4) {
			// 	// std::cout << "\033[1;32m"   << " tracked id : "         << tracked_results[i].id
			// 	// 							<< " dist_from_init_pos : " << dist_from_init_pos 
			// 	// 							<< " speed : "              << speed
			// 	// 							<< " init pose -> x, y, z : "    << tracked_results[i].x << ", " 
			// 	// 															<< tracked_results[i].y << ", " 
			// 	// 															<< tracked_results[i].z << ", " << "\033[0m" << std::endl;
			// 	continue;
			// }
			// // std::cout << "\033[1;33m"   << " tracked id : "         << tracked_results[i].id
			// // 				<< " dist_from_init_pos : " << dist_from_init_pos 
			// // 				<< " speed : "              << speed
			// // 				<< " init pose -> x, y, z : "    << tracked_results[i].x << ", " 
			// // 												<< tracked_results[i].y << ", " 
			// // 												<< tracked_results[i].z << ", " << "\033[0m" << std::endl;
			tracked_object_array.trackers.push_back(tracked_object);
		}
		return tracked_object_array;
	}

	void CubeTracker::spin() {
		if (!is_data_sub_) {
			ROS_INFO_THROTTLE(1, "obstacle_tracker : No data received.");
			return;
		}
		double start = ros::Time::now().toSec();
		frame_count_++;
		std::vector<TrackingCube> detected_objects;
		std::vector<TrackingCube> predicted_cubes;
		if (is_transform_) {
			for (int i = 0; i < target_marker_msg_.markers.size(); i++) {
				tf::Pose source_tf(tf::Quaternion(target_marker_msg_.markers[i].pose.orientation.x,
													target_marker_msg_.markers[i].pose.orientation.y,
													target_marker_msg_.markers[i].pose.orientation.z,
													target_marker_msg_.markers[i].pose.orientation.w),
									tf::Vector3(target_marker_msg_.markers[i].pose.position.x,
												target_marker_msg_.markers[i].pose.position.y,
												target_marker_msg_.markers[i].pose.position.z));
				tf::Stamped<tf::Pose> source_tf_stamped(source_tf, ros::Time(0), target_marker_msg_.markers[i].header.frame_id);
				tf::Stamped<tf::Pose> source_tf_transformed;

				try {
					tf_listener_.transformPose(frame_id_, source_tf_stamped, source_tf_transformed);
				}
				catch (tf::TransformException &ex) {
					ROS_ERROR("%s", ex.what());
					return;
				}

				target_marker_msg_.markers[i].pose.position.x = source_tf_transformed.getOrigin().x();
				target_marker_msg_.markers[i].pose.position.y = source_tf_transformed.getOrigin().y();
				target_marker_msg_.markers[i].pose.position.z = source_tf_transformed.getOrigin().z();
				target_marker_msg_.markers[i].pose.orientation.x = source_tf_transformed.getRotation().x();
				target_marker_msg_.markers[i].pose.orientation.y = source_tf_transformed.getRotation().y();
				target_marker_msg_.markers[i].pose.orientation.z = source_tf_transformed.getRotation().z();
				target_marker_msg_.markers[i].pose.orientation.w = source_tf_transformed.getRotation().w();
				target_marker_msg_.markers[i].header.frame_id = frame_id_;
			}
		}
		detected_objects = setDetections(target_marker_msg_);

		if (trackers_.size() == 0) {
			trackers_ = initializeTrackers(detected_objects);
			ROS_DEBUG_STREAM("\033[1;32m" << "Trackers initialized, tracker size : " << trackers_.size() << "\033[0m");
			return;
		}

		predicted_cubes.clear();
		for (auto it = trackers_.begin(); it != trackers_.end();) {
			StateType predicted_cube = (*it).predict();
			predicted_cubes.push_back(predicted_cube);
			it++;

		}

		// Sort
		std::vector<cv::Point> matched_pairs;
		auto results = Sort(predicted_cubes, detected_objects, score_threshold_, dist_threshold_); // first : matched_pairs, second : unmatched_det
		matched_pairs = results.first;
		std::set<int> unmatched_det = results.second;

		// update trackers both matched and unmatched
		int det_idx, trk_idx;
		for (int i = 0; i < matched_pairs.size(); i++) {
			trk_idx = matched_pairs[i].x;
			det_idx = matched_pairs[i].y;
			trackers_[trk_idx].update(detected_objects[det_idx]);
			trackers_[trk_idx].m_originIdx = det_idx;
			
			if (detected_objects[det_idx].class_name !="unknown")
				trackers_[trk_idx].m_class_name = detected_objects[det_idx].class_name;
		}
		for (auto umd : unmatched_det) {
			KalmanTracker tracker2 = KalmanTracker(detected_objects[umd]);
			trackers_.push_back(tracker2);
		}

		// get output
		vector<TrackingCube> tracked_results;
		tracked_results.clear();
		// std::cout << "\033[1;32m" << "min hits, max age, frame : " << min_hits_ << "," << max_age_ << "," << frame_count_ << "\033[0m" << std::endl;
		for (auto it = trackers_.begin(); it != trackers_.end();) {
			if (((*it).m_time_since_update < 1) &&
				((*it).m_hit_streak >= min_hits_ || frame_count_ <= min_hits_)) 
			{
				StateType res;
				res.cube = (*it).get_state().cube;
				res.originIdx = (*it).m_originIdx;
				res.id = (*it).m_id + 1;
				res.frame = frame_count_;
				res.class_name = (*it).m_class_name;
				res.x = (*it).x;
				res.y = (*it).y;
				res.z = (*it).z;
				tracked_results.push_back(res);
				it++;
			}	
			else 
				it++;
			
			if (it != trackers_.end()) {
				if ((*it).m_time_since_update > max_age_) {
					it = trackers_.erase(it);
					// std::cout << "\033[1;33m" << (*it).m_id << "th tracklet died" << "\033[0m" << std::endl;
				}
			}
		}

		cube_tracker::TrackerArray tracked_object_array;
		visualization_msgs::MarkerArray text_tracked_array;
		visualization_msgs::MarkerArray class_text_tracked_array;
		visualization_msgs::MarkerArray arrow_tracked_array;

		tracked_object_array = setTrackedObjects(tracked_results);
		text_tracked_array = setTextMsg(tracked_results, 2);
		class_text_tracked_array = setTextMsg(tracked_results, 1);
		arrow_tracked_array = setArrowMsg(tracked_results);

		tracked_objects_pub_.publish(tracked_object_array);
		text_tracked_pub_.publish(text_tracked_array);
		class_tracked_pub_.publish(class_text_tracked_array);
		arrow_tracked_pub_.publish(arrow_tracked_array);

		is_data_sub_ = false;
		double end = ros::Time::now().toSec();
		ROS_DEBUG_STREAM("\033[1;32m" << "tracker_node : process Time = " << end - start << "\033[0m");
	}
} 

int main(int argc, char** argv) {
	ros::init(argc, argv, "cube_tracker");
	cube_tracker::CubeTracker tracker;
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		tracker.spin();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}