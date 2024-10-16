from enum import Enum
from dynamic_window_approach import DynamicWindowApproach
import rospy
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd
from morai_msgs.msg import EgoDbVehicleStatus
import math

import pandas as pd
from scipy.spatial import KDTree
import math
import numpy as np
import os

#맵 가져올 경로
CURRENT_FILE_PATH = os.path.abspath(__file__)
CURRENT_DIRECTORY = os.path.dirname(CURRENT_FILE_PATH)
GLOBAL_UTM_TXT_ADDRESS = CURRENT_DIRECTORY + "/map/"

TARGET_DWA_IDX = 100

class RobotType(Enum):
    circle = 0
    rectangle = 1

class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 2.0  # [m/s]
        self.min_speed = -2.0 # [m/s]
        self.max_yaw_rate = 0.83  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 0.83  # [rad/ss]
        self.v_resolution = 0.1  # [m/s]
        self.yaw_rate_resolution = 0.01 # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.rectangle

        self.robot_radius = 1.0  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check

class MoraiRobotControl:
    def __init__(self):
        rospy.init_node('morai_robot_control')

        #gps, heading, velocity sub 
        self.odom_sub = rospy.Subscriber('/Local/odom', EgoDbVehicleStatus, self.odom_callback)

        #control pub
        self.ctrl_pub = rospy.Publisher('/dilly_ctrl', SkidSteer6wUGVCtrlCmd, queue_size=1)
        
        self.current_state = None

    def odom_callback(self, msg):
        # 차량 상태 (위치, 속도 등)을 받아옴
        self.current_state = {
            'x': msg.position.x,
            'y': msg.position.y,
            'yaw': msg.heading,  # 라디안 단위의 헤딩
            'v': msg.linear_velocity.x  # x 축 속도 (앞뒤)
        }

    def send_control(self, linear_velocity, angular_velocity):
        # DWA 알고리즘 결과로 나온 속도 명령을 Morai에 전달
        cmd = SkidSteer6wUGVCtrlCmd()
        cmd.cmd_type = 3
        cmd.linear= linear_velocity
        cmd.angular = angular_velocity
        self.ctrl_pub.publish(cmd)

class Map:
    def __init__(self,file_name,g_path,g_yaw,g_k):
        self.file_name = file_name
        self.g_path = g_path
        self.g_yaw = g_yaw
        self.g_k = g_k
        self.g_kd_tree = KDTree(self.g_path)

class MapList:
    def __init__(self, map_path):
        self.map_dic = {}
        self.create_map()
        self.map_folder_path = map_path

    def load_g_path_from_txt(self,f_name):
        map_data_txt = pd.read_csv(self.map_folder_path+str(f_name), sep=',', encoding='utf-8') # 파일 내용 저장 -> 2차원 리스트

        UTMmap_arr = map_data_txt.to_numpy() # 리스트를 numpy로 변환
        g_path = UTMmap_arr[:,:2]
        g_yaw = UTMmap_arr[:,2]

        return g_path, g_yaw

    def create_map(self):
        file_list = os.listdir(self.map_folder_path)

        for file_name in file_list:
            g_path,g_yaw,g_k = self.load_g_path_from_txt(file_name)
            self.read_map(file_name,g_path,g_yaw,g_k)

    def read_map(self, file_name, g_path, g_yaw,g_k):
        self.map_dic[file_name] = Map(file_name,g_path,g_yaw,g_k)

    def chose_start_mission_map(self,utm_x, utm_y):
        dist = math.inf
        map_info = None
        for i in self.map_dic:
            kdtree = self.map_dic[i].g_kd_tree
            near_dist, _ = kdtree.query([utm_x,utm_y])
            if dist > near_dist:
                dist = near_dist
                map_info = self.map_dic[i]

        if map_info is not None:
            return map_info
        else:
            return None
        
def find_target_path_idx(path_len,x,y,kd_tree):
    _, idx = kd_tree.query([x,y])

    target_idx = idx+TARGET_DWA_IDX
    if target_idx > path_len-1:
        target_idx = path_len-1
    return target_idx
def main():
    robot_control = MoraiRobotControl()
    cfg = Config()
    dwa_controller = DynamicWindowApproach(cfg)
    map_list = MapList(GLOBAL_UTM_TXT_ADDRESS)
    # goal position 설정
    goal = np.array([5.0, 5.0])  # 목표 지점
    
    ob = []
    
    # DWA 루프 시작
    rate = rospy.Rate(10)  # 10Hz로 루프 실행

    while robot_control.current_state is None: rate.sleep()
    print("odom sub")
    
    map_info = map_list.chose_start_mission_map(robot_control.current_state['x'],robot_control.current_state['y'])
    if map_info is None:
        print("맵 정보 없음")
        return
    else:
        g_path = map_info.g_path
        g_yaw = map_info.g_yaw
        g_kd_tree = map_info.g_kd_tree

    while not rospy.is_shutdown():
        x = [robot_control.current_state['x'],
            robot_control.current_state['y'],
            robot_control.current_state['yaw'],
            robot_control.current_state['v'], 0.0]
        
        target_idx = find_target_path_idx(x[0],x[1],g_kd_tree)

        u, predicted_trajectory = dwa_controller.dwa_control(x, g_path[target_idx], ob)
        linear_velocity, angular_velocity = u

        # Morai에 속도 명령어 보내기
        robot_control.send_control(linear_velocity, angular_velocity)

        # 종료 조건: 목표에 도달했는지 확인
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= cfg.robot_radius:
            rospy.loginfo("Goal Reached")
            break
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
