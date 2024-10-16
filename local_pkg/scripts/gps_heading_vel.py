import rospy
import math
from collections import deque
from morai_msgs.msg import GPSMessage
from morai_msgs.msg import EgoDdVehicleStatus

from pyproj import Proj


class GpsImuHeading:

    def __init__(self):

        rospy.init_node('gps_heading', anonymous=True)
        self.gps_sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        self.odom_pub = rospy.Publisher("/Local/odom", EgoDdVehicleStatus, queue_size=1)
        
        # UTM 변환 설정
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.vel_q = deque(maxlen=2)
        self.utm_q = deque(maxlen=2)

        self.angular_velocity_z = 0.0
        self.rotation_radius = 0.3
        self.ready = False

        rospy.loginfo("gps_vel node start")
          
        self.proj_UTM = Proj(proj='utm', zone=52, ellps = 'WGS84', preserve_units=False)

        self.gps_distance = 0.01 # gps 이전 점과 현재 점 사이 간격을 어느정도 할것인지. (m단위)
        self.gps_array_index = 20 # gps 점을 몇개까지 저장할건지
        self.make_heading_index = 2 # gps heading을 구할 때, 이전 몇 번째의 좌표를 활용할건지
        self.first_gps = True

        self.current_x = 0.0
        self.current_y = 0.0
        self.gps_heading = 0.0
        self.gps_heading_ = 0.0
        self.delta_heading = 0.0

        self.x_array = []
        self.y_array = []

    def gps_callback(self, msg):

        stamp = msg.header.stamp
        utm_xy = self.proj_UTM(msg.longitude, msg.latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]

        easting = utm_x
        northing = utm_y
        distance = math.sqrt(math.pow(northing-self.current_y,2)+math.pow(easting-self.current_x,2))

        if self.first_gps == True:
            self.current_x = easting
            self.current_y = northing
            self.first_gps = False

        elif (self.current_x != easting and self.current_y != northing) and (distance >= self.gps_distance):
            
            if(len(self.x_array) >= self.gps_array_index):
                self.x_array.pop(0)
                self.y_array.pop(0)

            self.x_array.append(easting)
            self.y_array.append(northing)

        self.current_x = easting
        self.current_y = northing

        timestamp = stamp.to_sec() # 나노초로 계산하기 추가
        self.utm_q.append((utm_x, utm_y, timestamp))

        if len(self.utm_q) == 2:
            self.ready = True

        self.publish_gps_heading(utm_x, utm_y, stamp)

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
            heading = math.atan2(self.y_array[-1]-self.y_array[-self.make_heading_index],self.x_array[-1]-self.x_array[-self.make_heading_index])
            speed = self.calculate_speed()
            self.vel_q.append(speed)

            gps_heading_msg = EgoDdVehicleStatus()
            gps_heading_msg.header.stamp = stamp
            gps_heading_msg.point.x = utm_x
            gps_heading_msg.point.y = utm_y
            gps_heading_msg.point.z = heading
            gps_heading_msg.linear_velocity = sum(self.vel_q) / len(self.vel_q)
            self.odom_pub.publish(gps_heading_msg)

if __name__ == '__main__':
    try:
        gps_heading = GpsImuHeading()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
