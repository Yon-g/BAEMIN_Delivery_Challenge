import rospy

from morai_msgs.srv import WoowaDillyEventCmdSrv # [srv] 배달 요청 및 확인
from morai_msgs.msg import WoowaDillyStatus # 배달 적재 정보
from morai_msgs.msg import DillyCmd # [msg] 배달 요청 및 수신 확인
from morai_msgs.msg import GPSMessage

from pyproj import Proj
import math
import yaml

class Deli:
    def __init__(self):

        rospy.init_node('deli_all', anonymous=True)

        self.gps_sub = rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        self.status_sub = rospy.Subscriber('/WoowaDillyStatus', WoowaDillyStatus, self.status_callback)

        rospy.wait_for_service('/WoowaDillyEventCmd')
        self.service_proxy = rospy.ServiceProxy('/WoowaDillyEventCmd', WoowaDillyEventCmdSrv)

        self.frequency = 1
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.frequency), self.timer_callback)
        
        # UTM 변환 설정
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        # 변수
        self.order = self.load_order()
        self.order_idx = 0
        self.loaded_item = 0
        self.utm_x = None
        self.utm_y = None
        self.distance = None
        # self.check_gps = False # 데이터 수신 확인
        self.can_dis = False # 물건 적재 거리 여부

    def load_order(self):
        path = rospy.get_param('~order')

        # index, utm_x, utm_y, 적재or배달
        with open(path, 'r') as file:
            data = yaml.safe_load(file)
        return data['orders']
    
    # 목표 지점까지의 거리 cb
    def gps_callback(self, msg):

        latitude = msg.latitude
        longitude = msg.longitude

        utm_xy = self.proj_UTM(longitude, latitude)
        self.utm_x = utm_xy[0]
        self.utm_y = utm_xy[1]

        self.distance = math.hypot(self.utm_x - self.order[self.order_idx]['utm_x'], 
                              self.utm_y - self.order[self.order_idx]['utm_y'])

        if self.distance <= 1.35:
            self.can_dis = True
        else:
            self.can_dis = False

    # 딜리에 적재된 물건 수 cb
    def status_callback(self, msg):
        self.loaded_item = len(msg.deliveryItem)

    # 시뮬에 적재 및 배달 요청
    def timer_callback(self, event):
        try:
            if self.can_dis:
                request = DillyCmd()
                request.deliveryItemIndex = self.order[self.order_idx]['index']
                request.isPickup = self.order[self.order_idx]['isPickup'] # True 적재, False 전달
                receive_msg = self.service_proxy(request)

                if receive_msg.response.result:
                    print(f"물건 확인")
                    if len(self.order) > self.order_idx + 1:
                        self.order_idx += 1
            else:
                print(f"목표 물품:{self.order[self.order_idx]['index']} 목표거리:{self.distance} 적재 수:{self.loaded_item}")
                      
                    
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        

if __name__ == '__main__':
    try:
        deli = Deli()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass