///////////////////////////////////////////////////////////////////////////////
// KalmanTracker.h: KalmanTracker Class Declaration

#ifndef KALMAN_H
#define KALMAN_H 2

#include "opencv2/video/tracking.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

typedef struct pCube
{
	vector<float> stateVec = vector<float>(6,0); // x, y, z, dx/dt, dy/dt, dz/dt, yaw respectively
	vector<float> sizeVec = vector<float>(3,0); // len_x, len_y, len_z
	vector<float> quatVec = vector<float>(4,0); // x, y, z, w
} pCube;

typedef struct TrackingCube
{
	TrackingCube()
	{
		id = 0;
		frame = 0;
		dt = 0.0;
		originIdx = 0;
		class_name = "unknown";
		cube = pCube();
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}
	int id;
	int frame;
	double dt;
	int originIdx;
	std::string class_name;
	pCube cube;

	double x, y, z;

}TrackingCube;

#define StateType TrackingCube

// This class represents the internel state of individual tracked objects observed as bounding box.
class KalmanTracker
{
public:
	KalmanTracker()
	{
		init_kf(StateType());
		m_time_since_update = 0;
		m_hits = 0;
		m_hit_streak = 0;
		m_age = 0;
		m_id = kf_count;
		m_originIdx = 0;
		m_class_name = "unknown";
		m_count_for_filtering = 0;
		// x = 0.0;
		// y = 0.0;
		// z = 0.0;

		//kf_count++;
	}
	KalmanTracker(StateType initCube)
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
		history_pos = std::vector<StateType>();
		init_kf(initCube);
		m_time_since_update = 0;
		m_hits = 0;
		m_hit_streak = 0;
		m_age = 0;
		m_id = kf_count;
		m_originIdx = 0;
		m_class_name = "unknown";
		kf_count++;
		m_count_for_filtering = 0;
	}

	~KalmanTracker()
	{
		m_history.clear();
	}

	StateType predict();
	void update(StateType stateMat);
	
	StateType get_state();
	StateType get_cube(vector<float> stateVec);
	std::vector<float> get_history_pos();

	static int kf_count;

	int m_time_since_update;
	int m_hits;
	int m_hit_streak;
	int m_age;
	int m_id;
	int m_originIdx;
	int m_count_for_filtering;

	std::string m_class_name;

	double x, y, z;
	std::vector<StateType> history_pos;

private:
	void init_kf(StateType stateMat);

	cv::KalmanFilter kf;
	cv::Mat measurement;

	std::vector<StateType> m_history;
};

#endif