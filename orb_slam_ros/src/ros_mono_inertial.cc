/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc
*
*/

#include "common.h"
#include <deque>

using namespace std;

// LowPassFilter.h
#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

class LowPassFilter {
public:
    LowPassFilter(double alpha) : alpha(alpha), initialized(false), prev_value(0.0) {}

    double filter(double value) {
        if (!initialized) {
            prev_value = value;
            initialized = true;
            return value;
        }
        double filtered = alpha * value + (1.0 - alpha) * prev_value;
        prev_value = filtered;
        return filtered;
    }

private:
    double alpha;
    bool initialized;
    double prev_value;
};

#endif // LOW_PASS_FILTER_H

class ImuGrabber
{
public:
    ImuGrabber()
        : accFilterX(0.05), accFilterY(0.05), accFilterZ(0.05),
          gyroFilterX(0.02), gyroFilterY(0.02), gyroFilterZ(0.02) {} // alpha 값을 낮춤

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    deque<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
    size_t maxBufSize = 5000; // 버퍼 최대 크기 설정 (예: 2000)

private:
    LowPassFilter accFilterX;
    LowPassFilter accFilterY;
    LowPassFilter accFilterZ;
    LowPassFilter gyroFilterX;
    LowPassFilter gyroFilterY;
    LowPassFilter gyroFilterZ;

    // 클리핑 함수
    double clip(double value, double min_val, double max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }
};

class ImageGrabber
{
public:
    ImageGrabber(ImuGrabber *pImuGb) 
        : mpImuGb(pImuGb), mLastImuTime(-1) {}

    void GrabImage(const sensor_msgs::CompressedImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::CompressedImageConstPtr &img_msg);
    void SyncWithImu();

    deque<sensor_msgs::CompressedImageConstPtr> img0Buf;
    mutex mBufMutex;
    size_t maxBufSize = 500; // 버퍼 최대 크기 설정 (예: 100)
    ImuGrabber *mpImuGb;
    double mLastImuTime;  // 이전 이미지 타임스탬프

private:

    // 클리핑 함수
    double clip(double value, double min_val, double max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");
    node_handler.param<std::string>(node_name + "/imu_frame_id", imu_frame_id, "imu");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
    ros::Subscriber sub_img = node_handler.subscribe("/image_jpeg/compressed", 100, &ImageGrabber::GrabImage, &igb);

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);
    
    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabImage(const sensor_msgs::CompressedImageConstPtr &img_msg)
{
    ROS_INFO("Image message received.");
    mBufMutex.lock();
    // 버퍼 크기가 최대 크기를 초과하면 오래된 데이터 삭제
    if (img0Buf.size() > maxBufSize)
    {
        img0Buf.pop_front();
    }
    img0Buf.push_back(img_msg);
    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::CompressedImageConstPtr &img_msg)
{
    try
    {
        std::vector<unsigned char> compressed_data = img_msg->data;
        cv::Mat image = cv::imdecode(compressed_data, cv::IMREAD_COLOR);
        if(image.empty()) {
            //ROS_ERROR("Image decoding failed.");
        } else {
            //ROS_INFO("Image decoded successfully.");
        }
        return image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

void ImageGrabber::SyncWithImu()
{
    while(1)
    {
        if (!img0Buf.empty())
        {
            cv::Mat im;
            double tIm = 0;
            ros::Time msg_time;

            {
                std::lock_guard<std::mutex> lock(mBufMutex);
                tIm = img0Buf.front()->header.stamp.toSec();
                im = GetImage(img0Buf.front());
                msg_time = img0Buf.front()->header.stamp;
                img0Buf.pop_front();
            }

            // 이미지 타임스탬프 출력
            ROS_INFO("Image timestamp: %f", tIm);

            // 이전 이미지 타임스탬프가 설정되지 않았다면 초기화
            if (mLastImuTime < 0)
                mLastImuTime = tIm;

            vector<ORB_SLAM3::IMU::Point> vImuMeas;

            // IMU 데이터 수집
            {
                std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);
                while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
                {
                    sensor_msgs::ImuConstPtr imu_msg = mpImuGb->imuBuf.front();
                    double t = imu_msg->header.stamp.toSec();

                    // IMU 데이터의 타임스탬프 출력
                    ROS_INFO("IMU timestamp: %f", t);

                    if (t >= mLastImuTime)
                    {
                        // 필터링된 IMU 데이터 사용
                        double filt_ax = imu_msg->linear_acceleration.x;
                        double filt_ay = imu_msg->linear_acceleration.y;
                        double filt_az = imu_msg->linear_acceleration.z;
                        double filt_gx = imu_msg->angular_velocity.x;
                        double filt_gy = imu_msg->angular_velocity.y;
                        double filt_gz = imu_msg->angular_velocity.z;


                        cv::Point3f acc(filt_ax, filt_ay, filt_az);
                        cv::Point3f gyr(filt_gx, filt_gy, filt_gz);

                        vImuMeas.emplace_back(acc, gyr, t);
                    }

                    mpImuGb->imuBuf.pop_front();
                }
            }

            // vImuMeas가 비어있는지 확인
            if (vImuMeas.empty())
            {
                ROS_WARN("Not enough IMU data, skipping frame.");
                mLastImuTime = tIm;
                continue;
            }

            // 시간 간격 검증
            double dt = tIm - mLastImuTime;
            if (dt <= 0)
            {
                ROS_ERROR("Non-positive time difference detected: dt = %f", dt);
                mLastImuTime = tIm;
                continue;
            }

            // ORB-SLAM3는 TrackMonocular()에서 실행됩니다.
            try
            {
                Sophus::SE3f Tcw = pSLAM->TrackMonocular(im, tIm, vImuMeas);
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("Exception caught during TrackMonocular: %s", e.what());
                mLastImuTime = tIm;
                continue;
            }

            publish_topics(msg_time);

            // 현재 이미지 타임스탬프를 저장하여 다음 반복에서 사용할 수 있도록 합니다.
            mLastImuTime = tIm;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    // while(1)
    // {
    //     if (!img0Buf.empty())
    //     {
    //         cv::Mat im;
    //         double tIm = 0;

    //         this->mBufMutex.lock();
    //         tIm = img0Buf.front()->header.stamp.toSec();
    //         im = GetImage(img0Buf.front());
    //         ros::Time msg_time = img0Buf.front()->header.stamp;
    //         img0Buf.pop();
    //         this->mBufMutex.unlock();

    //         // 이전 이미지 타임스탬프가 설정되지 않았다면 초기화
    //         if (mLastImuTime < 0)
    //             mLastImuTime = tIm;

    //         vector<ORB_SLAM3::IMU::Point> vImuMeas;

    //         // IMU 데이터 수집 및 보간
    //         mpImuGb->mBufMutex.lock();
    //         if (!mpImuGb->imuBuf.empty())
    //         {
    //             // mLastImuTime과 tIm 사이의 IMU 데이터를 수집
    //             vector<sensor_msgs::ImuConstPtr> imu_msgs;
    //             while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
    //             {
    //                 imu_msgs.push_back(mpImuGb->imuBuf.front());
    //                 mpImuGb->imuBuf.pop();
    //             }

    //             // IMU 데이터가 2개 이상인 경우에만 보간
    //             if (imu_msgs.size() >= 2)
    //             {
    //                 // 목표 주파수 설정 (예: 200Hz)
    //                 double target_freq = 200.0;
    //                 double target_dt = 1.0 / target_freq;

    //                 // 보간된 타임스탬프 생성
    //                 double t_start = imu_msgs.front()->header.stamp.toSec();
    //                 double t_end = imu_msgs.back()->header.stamp.toSec();

    //                 vector<double> interp_times;
    //                 for (double t = t_start; t <= t_end; t += target_dt)
    //                 {
    //                     interp_times.push_back(t);
    //                 }

    //                 // 각속도와 가속도 보간
    //                 size_t imu_index = 0;
    //                 for (double t : interp_times)
    //                 {
    //                     // 현재 t에 해당하는 두 IMU 메시지 찾기
    //                     while (imu_index < imu_msgs.size() - 1 && imu_msgs[imu_index + 1]->header.stamp.toSec() < t)
    //                     {
    //                         imu_index++;
    //                     }

    //                     sensor_msgs::ImuConstPtr imu1 = imu_msgs[imu_index];
    //                     sensor_msgs::ImuConstPtr imu2 = imu_msgs[imu_index + 1];

    //                     double t1 = imu1->header.stamp.toSec();
    //                     double t2 = imu2->header.stamp.toSec();
    //                     double alpha = (t - t1) / (t2 - t1);

    //                     // 선형 보간 수행
    //                     cv::Point3f acc1(imu1->linear_acceleration.x, imu1->linear_acceleration.y, imu1->linear_acceleration.z);
    //                     cv::Point3f acc2(imu2->linear_acceleration.x, imu2->linear_acceleration.y, imu2->linear_acceleration.z);
    //                     cv::Point3f acc = acc1 * (1.0 - alpha) + acc2 * alpha;

    //                     cv::Point3f gyr1(imu1->angular_velocity.x, imu1->angular_velocity.y, imu1->angular_velocity.z);
    //                     cv::Point3f gyr2(imu2->angular_velocity.x, imu2->angular_velocity.y, imu2->angular_velocity.z);
    //                     cv::Point3f gyr = gyr1 * (1.0 - alpha) + gyr2 * alpha;

    //                     vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
    //                 }
    //             }
    //             else
    //             {
    //                 ROS_WARN("Not enough IMU data for interpolation, skipping frame.");
    //                 mpImuGb->mBufMutex.unlock();
    //                 mLastImuTime = tIm;
    //                 continue;
    //             }
    //         }
    //         else
    //         {
    //             ROS_WARN("IMU buffer is empty, skipping frame.");
    //             mpImuGb->mBufMutex.unlock();
    //             mLastImuTime = tIm;
    //             continue;
    //         }
    //         mpImuGb->mBufMutex.unlock();

    //         // ORB-SLAM3는 TrackMonocular()에서 실행됩니다.
    //         Sophus::SE3f Tcw = pSLAM->TrackMonocular(im, tIm, vImuMeas);

    //         publish_topics(msg_time);

    //         // 현재 이미지 타임스탬프를 저장하여 다음 반복에서 사용할 수 있도록 합니다.
    //         mLastImuTime = tIm;
    //     }
    //     else
    //     {
    //         std::this_thread::sleep_for(std::chrono::milliseconds(5));
    //     }
    // }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // 현재 IMU 데이터의 타임스탬프를 가져옵니다.
    double current_timestamp = imu_msg->header.stamp.toSec();
    
    std::lock_guard<std::mutex> lock(mBufMutex);

    // 이전 IMU 데이터의 타임스탬프와 비교합니다.
    if (!imuBuf.empty())
    {
        double last_timestamp = imuBuf.back()->header.stamp.toSec();

        // 타임스탬프가 이전 값보다 크지 않으면 중복된 데이터이므로 추가하지 않습니다.
        if (current_timestamp < last_timestamp)
        {
            ROS_WARN("Out-of-order IMU data detected. Ignoring current IMU message.");
            return;
        }
    }

    // 원본 데이터
    double raw_ax = imu_msg->linear_acceleration.x;
    double raw_ay = imu_msg->linear_acceleration.y;
    double raw_az = imu_msg->linear_acceleration.z;
    double raw_gx = imu_msg->angular_velocity.x;
    double raw_gy = imu_msg->angular_velocity.y;
    double raw_gz = imu_msg->angular_velocity.z;

    // 저역 통과 필터 적용
    double filt_ax = accFilterX.filter(raw_ax);
    double filt_ay = accFilterY.filter(raw_ay);
    double filt_az = accFilterZ.filter(raw_az);
    double filt_gx = gyroFilterX.filter(raw_gx);
    double filt_gy = gyroFilterY.filter(raw_gy);
    double filt_gz = gyroFilterZ.filter(raw_gz);

    // 클리핑 적용 (예: 가속도는 ±10 m/s², 자이로는 ±10 rad/s)
    filt_ax = clip(filt_ax, -10.0, 10.0);
    filt_ay = clip(filt_ay, -10.0, 10.0);
    filt_az = clip(filt_az, -10.0, 10.0);
    filt_gx = clip(filt_gx, -10.0, 10.0);
    filt_gy = clip(filt_gy, -10.0, 10.0);
    filt_gz = clip(filt_gz, -10.0, 10.0);

    if (std::isnan(filt_ax) || std::isnan(filt_ay) || std::isnan(filt_az) ||
        std::isnan(filt_gx) || std::isnan(filt_gy) || std::isnan(filt_gz))
    {
        ROS_ERROR("Filtered IMU data contains NaN values, skipping this measurement.");
        return;
    }

    imuBuf.push_back(imu_msg);
    // 버퍼 크기가 최대 크기를 초과하면 오래된 데이터 삭제
    if (imuBuf.size() > maxBufSize)
    {
        imuBuf.pop_front();
    }

    // IMU 데이터 출력
    //ROS_INFO("IMU message received at time %f", imu_msg->header.stamp.toSec());
    //ROS_INFO("Angular Velocity: [x: %f, y: %f, z: %f]",
    //         imu_msg->angular_velocity.x,
    //         imu_msg->angular_velocity.y,
    //         imu_msg->angular_velocity.z);
    //ROS_INFO("Linear Acceleration: [x: %f, y: %f, z: %f]",
    //         imu_msg->linear_acceleration.x,
    //         imu_msg->linear_acceleration.y,
    //         imu_msg->linear_acceleration.z);

    return;
}