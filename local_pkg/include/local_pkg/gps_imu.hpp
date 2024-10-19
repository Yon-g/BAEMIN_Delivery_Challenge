#include "ros/ros.h"
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

// 메세지 종류
#include "morai_msgs/GPSMessage.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

// Proj관련
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include "proj/coordinateoperation.hpp"
#include "proj/crs.hpp"
#include "proj/io.hpp"
#include "proj/util.hpp"

// message_filters관련
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef boost::shared_ptr<const morai_msgs::GPSMessage> GpsMsgConstPtr;
typedef boost::shared_ptr<const sensor_msgs::Imu> ImuConstPtr;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, morai_msgs::GPSMessage> MySyncPolicy;

class LocalSync
{
public:
    LocalSync();
    ~LocalSync();

private:  
    void callback(const ImuConstPtr& msg1, const GpsMsgConstPtr& msg2);
    
    ros::NodeHandle nh_;
    message_filters::Subscriber<morai_msgs::GPSMessage> gps_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;

    ros::Publisher odom_pub_;

    message_filters::Synchronizer<MySyncPolicy> sync_;

    PJ_CONTEXT *C;
    PJ *P;
    PJ_COORD coord;

};