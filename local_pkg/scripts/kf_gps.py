import rospy
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from pyproj import Proj
from filterpy.kalman import KalmanFilter
from nav_msgs.msg import Odometry
import numpy as np
import time

class InterGPS:
    def __init__(self):
        rospy.init_node('InterGPS', anonymous=True)

        # GPS 및 IMU 데이터 수신
        self.gps_sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.fake_pub = rospy.Publisher('/kf_utm', Odometry, queue_size=1)

        # GPS,IMU 정보
        self.gps_hz = 8
        self.imu_hz = 40

        # UTM 변환 설정
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        # 칼만필터 파라미터 설정
        # self.R_std = 0
        # self.Q_std = 0
        self.init_dt = 1/self.imu_hz

        # 칼만필터 초기화 확인
        self.initialized = False

        # 칼만 필터 및 행렬 설정
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        
        self.kf.F = np.array([[1, 0, self.init_dt, 0],
                              [0, 1, 0, self.init_dt],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        
        self.kf.H = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0]])
        

        self.kf.B = np.array([[0.5 * self.init_dt**2, 0],
                            [0, 0.5 * self.init_dt**2],
                            [self.init_dt, 0],
                            [0, self.init_dt]])

        self.kf.P = np.eye(4) * 1000  # 초기 공분산
        self.kf.R = np.eye(2) * 0.01  # GPS 노이즈 공분산
        self.kf.Q = np.eye(4) * 0.1**2  # 프로세스 노이즈 공분산

        # 이전 시간 기록
        self.prev_time = time.time()

        # ros 메세지
        self.odom_msg=Odometry()

    def gps_callback(self, msg):

        latitude = msg.latitude
        longitude = msg.longitude

        utm_xy = self.proj_UTM(longitude, latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]
        
        if not self.initialized:
            self.kf.x = np.array([utm_x, utm_y, 0, 0])
            self.initialized = True
        else:
            gps_position = np.array([utm_x, utm_y])
            self.kf.update(gps_position) # 칼만 필터 상태 업데이트

    def imu_callback(self, msg):

        if self.initialized:

            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y

            current_time = time.time()
            dt = current_time - self.prev_time
            self.prev_time = current_time

            self.kf.F = np.array([[1, 0, dt, 0],
                                [0, 1, 0, dt],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
            
            self.kf.B = np.array([[0.5 * dt**2, 0],
                            [0, 0.5 * dt**2],
                            [dt, 0],
                            [0, dt]])

            imu_acceleration = np.array([ax, ay])
            self.kf.predict(u=imu_acceleration)  # 업데이트

            # 현재속도 계산
            v_x, v_y = self.kf.x[2], self.kf.x[3]
            velocity_ms = np.sqrt(v_x**2 + v_y**2)
            velocity_kmh = velocity_ms * 3.6
            
            # 딜리 최고 속도 범위
            # if velocity_kmh > 7.8:
            #     velocity_kmh = None
            
            kf_utm_x = self.kf.x[0]  # x 위치
            kf_utm_y = self.kf.x[1]  # y 위치

            self.odom_msg.header = msg.header

            self.odom_msg.pose.pose.position.x = kf_utm_x
            self.odom_msg.pose.pose.position.y = kf_utm_y
            self.odom_msg.pose.pose.position.z = 0.

            self.odom_msg.twist.twist.linear.x = v_x
            self.odom_msg.twist.twist.linear.y = v_y
            self.odom_msg.twist.twist.linear.z = 0.

            self.fake_pub.publish(self.odom_msg)

            print(f"Predicted Position: {self.kf.x[:2]}")
            print(f"Predicted Velocity: {velocity_kmh} km/h")

if __name__ == '__main__':
    try:
        inter_gps = InterGPS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
