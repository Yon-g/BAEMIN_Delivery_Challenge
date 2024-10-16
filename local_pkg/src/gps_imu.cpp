#include "local_pkg/gps_imu.hpp"

LocalSync::LocalSync() :
    gps_sub_(nh_, "/gps", 1),
    imu_sub_(nh_, "/imu", 1),
    sync_(MySyncPolicy(5), imu_sub_, gps_sub_)
{
    sync_.registerCallback(boost::bind(&LocalSync::callback, this, _1, _2));

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);

    C = proj_context_create();
    P = proj_create_crs_to_crs(C, "EPSG:4326", "+proj=utm +zone=52 +ellps=WGS84", NULL);

    if (P == nullptr)
    {
        std::cerr << "Error creating projection" << std::endl;
    }

    ROS_INFO("LocalSync node starts");
}

LocalSync::~LocalSync()
{
    proj_destroy(P);
    proj_context_destroy(C);
}

void LocalSync::callback(const ImuConstPtr& msg1, const GpsMsgConstPtr& msg2)
{
    nav_msgs::Odometry odom_msg;
    
    coord = proj_coord(msg2->latitude, msg2->longitude, msg2->altitude, 0);
    PJ_COORD result = proj_trans(P, PJ_FWD, coord);

    if (proj_errno(P) != 0)
    {
        const char* error_message = proj_errno_string(proj_errno(P));
        ROS_ERROR("Projection error: %s", error_message);
        return;
    }

    odom_msg.header.stamp = msg2->header.stamp;
    odom_msg.pose.pose.position.x = msg2->latitude;
    odom_msg.pose.pose.position.y = msg2->longitude;
    odom_msg.pose.pose.position.z = msg2->altitude;

    odom_msg.pose.pose.orientation.x = msg1->orientation.x;
    odom_msg.pose.pose.orientation.y = msg1->orientation.y;
    odom_msg.pose.pose.orientation.z = msg1->orientation.z;
    odom_msg.pose.pose.orientation.w = msg1->orientation.w;

    odom_msg.twist.twist.linear.x = result.xy.x;
    odom_msg.twist.twist.linear.y = result.xy.y;

    odom_pub_.publish(odom_msg);

    // ROS_INFO("Received synchronized IMU and GPS data.");
    // ROS_INFO("longitude: %lf, latitude: %lf", msg2->longitude, msg2->latitude);
    // ROS_INFO("x: %f, y: %f, z: %f, w: %f", q1,q2,q3,q4);
    // ROS_INFO("UTM Easting: %lf, UTM Northing: %lf", result.xy.x, result.xy.y);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localsync");
    LocalSync localsync;
    ros::spin();
    return 0;
}


