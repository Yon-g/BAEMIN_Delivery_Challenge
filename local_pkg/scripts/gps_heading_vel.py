import rospy
import math
from collections import deque
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from morai_msgs.msg import EgoDdVehicleStatus

from pyproj import Proj


class GpsImuHeading:

    def __init__(self):

        rospy.init_node('gps_heading_vel', anonymous=True)
        self.gps_sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher("/Local/odom", EgoDdVehicleStatus, queue_size=1)
        
        # UTM 변환 설정
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.vel_q = deque(maxlen=2)
        self.utm_q = deque(maxlen=2)
        self.yaw_rate_q = deque(maxlen=2)

        self.angular_velocity_z = 0.0
        self.stop_speed = 0.1 # imu로 헤딩 보정할 속도 (m/s)
        self.ready = False
        self.first_gps = True
        self.gps_heading_clear = False
        self.gps_init_speed = 0.5 # imu에서 사용하기 전 gps 헤딩 계산 최소 속도

        self.gps_distance = 0.01 # gps 이전 점과 현재 점 사이 간격을 어느정도 할것인지. (m단위)
        self.gps_array_index = 20 # gps 점을 몇개까지 저장할건지
        self.make_heading_index = 2 # gps heading을 구할 때, 이전 몇 번째의 좌표를 활용할건지

        self.heading_error = 0.0174533 # 라디안
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw_rate = 0
        self.prev_heading = 0.01996
        self.prev_imu = None

        self.x_array = []
        self.y_array = []

        rospy.loginfo("gps_vel node start")

    def gps_callback(self, msg):

        stamp = msg.header.stamp
        utm_xy = self.proj_UTM(msg.longitude, msg.latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]

        distance = math.sqrt(math.pow(utm_y-self.current_y,2)+math.pow(utm_x-self.current_x,2))

        if self.first_gps == True:
            self.current_x = utm_x
            self.current_y = utm_y
            self.first_gps = False

        elif (self.current_x != utm_x and self.current_y != utm_y) and (distance >= self.gps_distance):
            
            if(len(self.x_array) >= self.gps_array_index):
                self.x_array.pop(0)
                self.y_array.pop(0)

            self.x_array.append(utm_x)
            self.y_array.append(utm_y)

        self.current_x = utm_x
        self.current_y = utm_y

        timestamp = stamp.to_sec() # 나노초로 계산하기 추가
        self.utm_q.append((utm_x, utm_y, timestamp))

        if len(self.utm_q) == 2:
            self.ready = True

        self.publish_gps_heading(utm_x, utm_y, stamp)

    def imu_callback(self, msg):

        self.angular_velocity_z = msg.angular_velocity.z
        imu_time = msg.header.stamp.to_sec()

        if self.prev_imu is not None:
            dt = imu_time - self.prev_imu
            if dt > 0:
                self.update_heading_with_imu(dt)

        self.prev_imu = imu_time
    
    def update_heading_with_imu(self, dt):

        if self.prev_heading is not None:
            new_heading = self.prev_heading + self.angular_velocity_z * dt

            # heading을 -pi ~ pi 사이로 normalize
            self.prev_heading = math.atan2(math.sin(new_heading), math.cos(new_heading))

    
    def calculate_speed(self):

        (x1, y1, t1), (x2, y2, t2) = self.utm_q

        distance = math.hypot(x2 - x1, y2 - y1)
        dt = t2 - t1

        if dt < 1e-3:
            return 0
        if distance < 1e-5 or distance > 1:
            return 0
        return min(distance/dt, 2)
    
    def publish_gps_heading(self, utm_x, utm_y, stamp):
        if self.ready and (len(self.x_array) >= self.make_heading_index):
            
            speed = self.calculate_speed()
            self.vel_q.append(speed)

            if speed > self.gps_init_speed:
                self.gps_heading_clear = True
            
            if self.gps_heading_clear:
                heading = math.atan2(self.y_array[-1]-self.y_array[-self.make_heading_index],self.x_array[-1]-self.x_array[-self.make_heading_index])

                if speed < self.stop_speed :
                    heading = self.prev_heading # imu로 부터 계산된 헤딩

                if (abs(heading) - abs(self.prev_heading)) > 10:
                    heading = self.prev_heading

                print(f"heading {heading*180/3.14}")

                gps_heading_msg = EgoDdVehicleStatus()
                gps_heading_msg.header.stamp = stamp
                gps_heading_msg.position.x = utm_x
                gps_heading_msg.position.y = utm_y
                gps_heading_msg.position.z = self.yaw_rate
                gps_heading_msg.heading = heading + self.heading_error
                gps_heading_msg.linear_velocity.x = sum(self.vel_q) / len(self.vel_q)
                self.odom_pub.publish(gps_heading_msg)
                self.prev_heading = heading

if __name__ == '__main__':
    try:
        gps_heading = GpsImuHeading()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass