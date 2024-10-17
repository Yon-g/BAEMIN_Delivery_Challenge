///////////////////////////////////////////////////////////////////////////////
// KalmanTracker.cpp: KalmanTracker Class Implementation Declaration

#include "KalmanTracker.h"

int KalmanTracker::kf_count = 0;

// initialize Kalman filter
void KalmanTracker::init_kf(StateType initCube)
{
	int stateNum = 6;
	int measureNum = 3;
	kf = KalmanFilter(stateNum, measureNum, 0);
	measurement = Mat::zeros(measureNum, 1, CV_32F);

	setIdentity(kf.transitionMatrix, Scalar::all(1));
	kf.transitionMatrix.at<_Float32>(0, 3) = initCube.dt;
	kf.transitionMatrix.at<_Float32>(1, 4) = initCube.dt;
	kf.transitionMatrix.at<_Float32>(2, 5) = initCube.dt;
	setIdentity(kf.measurementMatrix, Scalar::all(1));
	setIdentity(kf.processNoiseCov, Scalar::all(1e-2));
	setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(kf.errorCovPost, Scalar::all(1));
	
	//initalize the state vector
	kf.statePost.at<float>(0) = initCube.cube.stateVec[0]; // x
	kf.statePost.at<float>(1) = initCube.cube.stateVec[1]; // y
	kf.statePost.at<float>(2) = initCube.cube.stateVec[2]; // z
	kf.statePost.at<float>(3) = 0.0; // dx/dt
	kf.statePost.at<float>(4) = 0.0; // dy/dt
	kf.statePost.at<float>(5) = 0.0; // dz/dt 

	x = initCube.cube.stateVec[0];
	y = initCube.cube.stateVec[1];
	z = initCube.cube.stateVec[2]; 

	history_pos.push_back(initCube);
	// kf.statePre = kf.statePost;
	// kf.statePost.at<float>(6, 0) = initCube.cube.stateVec[6]; // yaw
}

// Predict the estimated bounding box.
StateType KalmanTracker::predict()
{
	int stateNum = 6;
	int measureNum = 3;

	// std::cout<<"Corrected State : "<<kf.statePost<<std::endl;
	// std::cout<<"transition Mat : "<<kf.transitionMatrix<<std::endl;
	// std::cout<<"measurement Mat : "<<kf.measurementMatrix<<std::endl;

	// predict
	Mat p = kf.predict(); //statePre
	// std::cout<<"predicted State : "<<kf.statePre<<std::endl;

	m_age += 1;

	StateType post_pos = get_state(); // 마지막에 uodate된 state를 pushback
	x = post_pos.cube.stateVec[0];
	y = post_pos.cube.stateVec[1];
	z = post_pos.cube.stateVec[2];
	history_pos.push_back(post_pos);

	if (m_time_since_update > 0)
		m_hit_streak = 0;
	m_time_since_update += 1;

	vector<float> stateVec;

	for (int i=0; i <= p.type(); ++i) {
		stateVec.push_back(p.at<float>(i, 0));
	}

	StateType predictCube = get_cube(stateVec);

	m_history.push_back(predictCube);
	return m_history.back();
}

// Update the state vector with observed bounding box.
void KalmanTracker::update(StateType stateCube)
{
	m_time_since_update = 0;
	m_history.clear();
	m_hits += 1;
	m_hit_streak += 1;

	// measurement
	measurement.at<float>(0, 0) = stateCube.cube.stateVec[0];
	measurement.at<float>(1, 0) = stateCube.cube.stateVec[1];
	measurement.at<float>(2, 0) = stateCube.cube.stateVec[2];
	kf.transitionMatrix.at<_Float32>(0, 3) = stateCube.dt;
	kf.transitionMatrix.at<_Float32>(1, 4) = stateCube.dt;
	kf.transitionMatrix.at<_Float32>(2, 5) = stateCube.dt;

	// update
	kf.correct(measurement); //returns statePost
}

// Return the current state vector
StateType KalmanTracker::get_state()
{
	Mat s = kf.statePost;

	vector<float> stateVec;
	for (int i=0; i <= s.type(); ++i) {
		stateVec.push_back(s.at<float>(i, 0));
	}

	return get_cube(stateVec);
}

StateType KalmanTracker::get_cube(vector<float> stateVec) {
	TrackingCube cube;

	cube.cube.stateVec[0] = stateVec[0];
	cube.cube.stateVec[1] = stateVec[1];
	cube.cube.stateVec[2] = stateVec[2];
	cube.cube.stateVec[3] = stateVec[3];
	cube.cube.stateVec[4] = stateVec[4];
	cube.cube.stateVec[5] = stateVec[5];
	// cube.cube.stateVec[6] = stateVec[6];

	return StateType(cube);
}

std::vector<float> KalmanTracker::get_history_pos() {
	std::vector<float> pose_pos;
	if (history_pos.size() < 10) {
		StateType pose = get_state();
		pose_pos.push_back(pose.cube.stateVec[0]);
		pose_pos.push_back(pose.cube.stateVec[1]);
		pose_pos.push_back(pose.cube.stateVec[2]);
		return pose_pos;
	}
	for (int i = 0; i < history_pos.size(); i++) {
		pose_pos.push_back(history_pos[i].cube.stateVec[0]);
	}
	int size = history_pos.size();
	pose_pos.push_back(history_pos[size - 10].cube.stateVec[0]);
	pose_pos.push_back(history_pos[size - 10].cube.stateVec[1]);
	pose_pos.push_back(history_pos[size - 10].cube.stateVec[2]);
	return pose_pos;
}