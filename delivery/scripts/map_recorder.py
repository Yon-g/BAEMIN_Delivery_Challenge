import os
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from morai_msgs.msg import EgoDdVehicleStatus
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import time

TARGET_SAVE_PATH = "/root/ws/src/BAEMIN_Delivery_Challenge/delivery/map"
GPS_TOPIC = '/Local/odom'

class MapRecorder:
    def __init__(self):
        rospy.init_node('map_recorder', anonymous=True)
        
        self.utm = np.empty((0, 2), dtype=np.float32)
        self.total_saved = []

        # GPS 데이터를 구독
        self.sub = rospy.Subscriber('/Local/odom', EgoDdVehicleStatus, self.text_callback)

        rospy.loginfo("#### MAP RECORDER STARTED ####")
        rospy.loginfo("Recording UTM coordinates...")

    def text_callback(self, msg):
        # 수신한 GPS 데이터 저장
        target_utm = np.array([msg.position.x, msg.position.y])
        # print(f"{target_utm=}")
        self.utm = np.append(self.utm, [target_utm], axis=0)
        rospy.loginfo(f"Received UTM coordinates: {target_utm}")

    def save_to_txt(self):
        # 데이터를 텍스트 파일로 저장
        if self.utm.shape[0] > 0:
            np.savetxt(os.path.join(TARGET_SAVE_PATH, 'utm_data.txt'), self.utm, delimiter=',', fmt='%.8f')
            rospy.loginfo("UTM coordinates saved to utm_data.txt.")
        else:
            rospy.logwarn("No UTM data to save.")

def main():
    mr = MapRecorder()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 프로그램 종료 시 데이터를 저장하고 맵 시각화
        mr.save_to_txt()

if __name__ == '__main__':
    main()
