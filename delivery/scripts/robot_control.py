import os,sys
sys.path.append(os.path.join(os.path.abspath(__file__),'..','utils'))

import rospy
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd
from morai_msgs.msg import EgoDdVehicleStatus
from morai_msgs.srv import WoowaDillyEventCmdSrv # [srv] 배달 요청 및 확인
from morai_msgs.msg import WoowaDillyStatus # 배달 적재 정보
from morai_msgs.msg import DillyCmd # [msg] 배달 요청 및 수신 확인

from enum import Enum
from utils.dynamic_window_approach import DynamicWindowApproach, RobotType
from utils.get_map import MapList
import math
import numpy as np
import yaml

CURRENT_FILE_PATH = os.path.abspath(__file__)
CURRENT_DIRECTORY = os.path.dirname(CURRENT_FILE_PATH)

# 부모 경로를 가져옴
PARENT_DIRECTORY = os.path.dirname(CURRENT_DIRECTORY)

# 부모 경로에 있는 "map" 디렉토리 경로 설정
GLOBAL_UTM_TXT_ADDRESS = os.path.join(PARENT_DIRECTORY, "map/")

TARGET_DWA_IDX = 15 # 몇 번째 인덱스를 타킷으로 할건지

class Config:
    """
    simulation parameter class
    """
    def __init__(self):
        # robot parameter
        self.max_speed = 7.23  # [m/s]
        self.min_speed = 0.0 # [m/s]
        self.max_yaw_rate = 0.83  # [rad/s]
        self.max_accel = 3  # [m/ss]
        self.max_delta_yaw_rate = 15.0  # [rad/ss]
        self.v_resolution = 0.1  # [m/s] # 0.1, 0.15 사용가능
        self.yaw_rate_resolution = 3 * math.pi / 180.0 # [rad/s]
        self.dt = 1/8  # [s] Time tick for motion prediction
        self.predict_time = 0.5  # [s]
        self.to_goal_cost_gain = 0.3
        self.speed_cost_gain = 0.8
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 1e-2  # constant to prevent robot stucked
        self.robot_type = RobotType.rectangle

        self.robot_radius = 0.7  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 0.7  # [m] for collision check

class MoraiRobotControl:
    def __init__(self):
        rospy.init_node('robot_control_node')

        #gps, heading, velocity sub 
        self.odom_sub = rospy.Subscriber('/Local/odom', EgoDdVehicleStatus, self.odom_callback)

        #control pub
        self.ctrl_pub = rospy.Publisher('/dilly_ctrl', SkidSteer6wUGVCtrlCmd, queue_size=1)
        
        self.status_sub = rospy.Subscriber('/WoowaDillyStatus', WoowaDillyStatus, self.status_callback)

        rospy.wait_for_service('/WoowaDillyEventCmd')
        self.service_proxy = rospy.ServiceProxy('/WoowaDillyEventCmd', WoowaDillyEventCmdSrv)

        self.order = self.load_order()
        self.order_idx = 0
        self.dist_thresh = 1.4
        self.current_state = None
        self.target_point = None
        self.odom_check = False
        self.ob = np.array([[0,0]]) # 임시로 장애물 X 가정
        self.loaded_item = 0 # 현재 딜리 적재 수

        # 초기설정
        self.cfg = Config()
        self.dwa_controller = DynamicWindowApproach(self.cfg)
        self.map_list = MapList(GLOBAL_UTM_TXT_ADDRESS)

        self.frequency = 30
        self.timer = rospy.Timer(rospy.Duration(1.0/self.frequency), self.timer_callback)

        # rospy.on_shutdown(self.shutdown_callback) # 종료 시, 호출

    def load_order(self):
        path = rospy.get_param('~order')

        # index, utm_x, utm_y, 적재or배달
        with open(path, 'r') as file:
            data = yaml.safe_load(file)
        return data['orders']
    
    def odom_callback(self, msg):
        self.current_state = {
            'x': msg.position.x,
            'y': msg.position.y,
            'yaw': msg.heading,  # 라디안 단위의 헤딩
            'v': msg.linear_velocity.x,  # x 축 속도 (앞뒤)
            'yaw_rate' : msg.position.z
        }
        self.odom_check = True
    
    # 딜리에 적재된 물건 수 cb
    def status_callback(self, msg):
        self.loaded_item = len(msg.deliveryItem)

    def cal_target_idx(self, g_path, g_kd_tree):

        target_idx = self.find_target_path_idx(len(g_path),self.current_state['x'],self.current_state['y'], g_kd_tree)
        target_point = g_path[target_idx]
        self.target_point = target_point
        
    def find_target_path_idx(self, path_len,x,y,kd_tree):
        _, idx = kd_tree.query([x,y])

        target_idx = idx+TARGET_DWA_IDX
        if target_idx > path_len-1:
            target_idx = path_len-1
        return target_idx
    
    def send_request(self, distance):
        try:
            if distance <= self.dist_thresh:
                self.send_control(0, 0) # 일단 정지

                request = DillyCmd()
                request.deliveryItemIndex = self.order[self.order_idx]['index']
                request.isPickup = self.order[self.order_idx]['isPickup'] # True 적재, False 전달
                receive_msg = self.service_proxy(request)

                if receive_msg.response.result:
                    print(f"물건 확인")
                    if len(self.order) > self.order_idx + 1:
                        self.order_idx += 1
            else:
                print(f"목표 물품:{self.order[self.order_idx]['index']} 목표거리:{distance} 적재 수:{self.loaded_item}")
                       
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def send_control(self, linear_velocity, angular_velocity):
        cmd = SkidSteer6wUGVCtrlCmd()
        cmd.cmd_type = 3
        cmd.Target_linear_velocity= linear_velocity
        cmd.Target_angular_velocity = angular_velocity
        self.ctrl_pub.publish(cmd)

    def timer_callback(self, event):

        if self.odom_check == True:

            map_info = self.map_list.chose_start_mission_map(self.current_state['x'],self.current_state['y'])
            if map_info is None:
                raise Exception("no map info...")
            
            g_path = map_info.g_path
            # g_yaw = map_info.g_yaw
            g_kd_tree = map_info.g_kd_tree

            x = [self.current_state['x'],
                 self.current_state['y'],
                 self.current_state['yaw'],
                 self.current_state['v'], 
                #  self.current_state['yaw_rate']]
                0]

            self.cal_target_idx(g_path=g_path, g_kd_tree=g_kd_tree)
            u, pred_trajectory = self.dwa_controller.dwa_control(x, self.target_point, self.ob)
            linear_vel, angular_vel = u

            self.send_control(linear_vel, -angular_vel)

            mission_dist = math.hypot(x[0] - self.order[self.order_idx]['utm_x'], 
                                      x[1] - self.order[self.order_idx]['utm_y'])
            
            self.send_request(mission_dist)

            dist_to_goal = math.hypot(x[0] - g_path[-1][0], x[1] - g_path[-1][1])
            if dist_to_goal <= self.cfg.robot_radius:
                rospy.loginfo("전체 미션 종료")
                self.send_control(0, 0)

if __name__ == '__main__':
    try:
        MoraiRobotControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass