import rospy
from morai_msgs.msg import GPSMessage
from morai_msgs.msg import EgoDdVehicleStatus
from collections import deque
from pyproj import Proj
import math
class GPSVel:
    def __init__(self):
        rospy.init_node('GPSVel', anonymous=True)
        self.gps_sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        # self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

        self.vel_pub = rospy.Publisher('/Local/vel', PointStamped, queue_size=1)
        
        self.frequency = 30
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.frequency), self.timer_callback)

        # UTM 변환 설정
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.vel_q = deque(maxlen=2)
        self.utm_q = deque(maxlen=2)

        self.angular_velocity_z = 0.0
        self.rotation_radius = 0.3
        self.ready = False

        rospy.loginfo("gps_vel node start")
    
    def gps_callback(self, msg):

        latitude = msg.latitude
        longitude = msg.longitude

        utm_xy = self.proj_UTM(longitude, latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]

        timestamp = msg.header.stamp.to_sec() # 나노초로 계산하기 추가
        self.utm_q.append((utm_x, utm_y, timestamp))

        if len(self.utm_q) == 2:
            self.ready = True

    def timer_callback(self, event):
        if self.ready:
            speed = self.calculate_speed()
            self.vel_q.append(speed)

            avg_speed = PointStamped()
            avg_speed.x = sum(self.vel_q) / len(self.vel_q)
            self.vel_pub.publish(avg_speed)
            # print(f"속도: {avg_speed} km/h")
        
        self.ready = False

    def calculate_speed(self):

        (x1, y1, t1), (x2, y2, t2) = self.utm_q

        distance = math.hypot(x2 - x1, y2 - y1)
        dt = t2 - t1

        if dt < 1e-3:
            return 0
        if distance < 1e-5 or distance > 1:
            return 0
        return min(distance/dt, 2)
        
if __name__ == '__main__':
    try:
        gps_vel = GPSVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("gps_vel node terminated")
