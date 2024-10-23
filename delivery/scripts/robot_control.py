import os,sys
sys.path.append(os.path.join(os.path.abspath(__file__),'..','utils'))

import rospy
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd
from morai_msgs.msg import EgoDdVehicleStatus
from morai_msgs.srv import WoowaDillyEventCmdSrv # [srv] 배달 요청 및 확인
from morai_msgs.msg import WoowaDillyStatus # 배달 적재 정보
from morai_msgs.msg import DillyCmd # [msg] 배달 요청 및 수신 확인
from cube_tracker.msg import TrackerArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from enum import Enum
from utils.dynamic_window_approach import DynamicWindowApproach, RobotType
from utils.get_map import MapList
from utils.transfrom_utm import to_utm
from sklearn.cluster import DBSCAN
from collections import deque

import math
import numpy as np
import yaml

CURRENT_FILE_PATH = os.path.abspath(__file__)
CURRENT_DIRECTORY = os.path.dirname(CURRENT_FILE_PATH)

# 부모 경로를 가져옴
PARENT_DIRECTORY = os.path.dirname(CURRENT_DIRECTORY)

# 부모 경로에 있는 "map" 디렉토리 경로 설정
GLOBAL_UTM_TXT_ADDRESS = os.path.join(PARENT_DIRECTORY, "map/")

TARGET_DWA_IDX = 5 # 몇 번째 인덱스를 타킷으로 할건지

class Config:

    def __init__(self):
        # robot parameter
        self.max_speed = 7.23  # [m/s]
        self.min_speed = 0.0 # [m/s]
        self.max_yaw_rate = 0.83  # [rad/s]
        self.max_accel = 9  # [m/ss]
        self.max_delta_yaw_rate = 15.0  # [rad/ss]
        self.v_resolution = 0.1  # [m/s] # 0.1, 0.15 사용가능
        self.yaw_rate_resolution = 3 * math.pi / 180.0 # [rad/s]
        self.dt = 1/8  # [s] Time tick for motion prediction
        self.predict_time = 1  # [s]
        self.to_goal_cost_gain = 0.6
        self.speed_cost_gain = 0.4
        self.obstacle_cost_gain = 0.7
        self.robot_stuck_flag_cons = 0.5  # constant to prevent robot stucked
        self.robot_type = RobotType.rectangle

        self.robot_radius = 1.25  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.55  # [m] for collision check
        self.robot_length = 0.8  # [m] for collision check

class MoraiRobotControl:
    def __init__(self):
        rospy.init_node('robot_control_node')

        #gps, heading, velocity sub 
        self.odom_sub = rospy.Subscriber('/Local/odom', EgoDdVehicleStatus, self.odom_callback)
        self.warp_sub = rospy.Subscriber('/Local/warp', Int16, self.warp_callback)
        self.reset_sub = rospy.Subscriber('/Local/reset', Int16, self.reset_callback)

        # self.ob_sub = rospy.Subscriber('/tracked_objects', TrackerArray, self.ob_callback)
        self.ob_sub = rospy.Subscriber('/xy_points', Float32MultiArray, self.ob_callback)


        #control pub
        self.ctrl_pub = rospy.Publisher('/dilly_ctrl', SkidSteer6wUGVCtrlCmd, queue_size=1)
        
        self.status_sub = rospy.Subscriber('/WoowaDillyStatus', WoowaDillyStatus, self.status_callback)

        rospy.wait_for_service('/WoowaDillyEventCmd')
        self.service_proxy = rospy.ServiceProxy('/WoowaDillyEventCmd', WoowaDillyEventCmdSrv)

        self.order = self.load_order()
        self.order_idx = 0
        self.warp_check = 0 # 워프 여부
        self.reset_check = 0 # 초기화 여부
        self.dist_thresh = 1.35
        self.current_state = None
        self.target_point = None
        self.odom_check = False
        self.plot_ready = False
        self.done = True
        self.ob = np.array([[0,0]]) # 임시로 장애물 X 가정
        self.loaded_item = 0 # 현재 딜리 적재 수

        # 초기설정
        self.cfg = Config()
        self.dwa_controller = DynamicWindowApproach(self.cfg)
        self.map_list = MapList(GLOBAL_UTM_TXT_ADDRESS)
        self.map_idx = 0 # 몇 번째 맵 사용할지

        self.frequency = 30
        self.timer = rospy.Timer(rospy.Duration(1.0/self.frequency), self.timer_callback)
        # self.ob_timer = rospy.Timer(rospy.Duration(1.0/self.frequency), self.static_callback)


        # rospy.on_shutdown(self.shutdown_callback) # 종료 시, 호출
        # self.plot_trajectory()


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
    
    def warp_callback(self, msg):
        self.warp_check = msg.data

    def reset_callback(self, msg):
        self.reset_check = msg.data

    def map_change_by_dist(self, threshold_distance=7.0):
        
        current_map_last_point = self.map_list.chose_start_mission_map(self.map_idx).g_path[-1]  # 마지막 좌표 (x, y)
        # print(f"{current_map_last_point=}")


        distance = math.sqrt((self.current_state['x'] - current_map_last_point[0]) ** 2 + 
                         (self.current_state['y'] - current_map_last_point[1]) ** 2)
        
        if distance < 0.2: # 현재 맵 마지막 부분과 나의 현재 거리 임계값

            # 다음 맵이 있는지 확인
            if self.map_idx + 1 < len(self.map_list.map_key):
                # 다음 맵의 첫 좌표
                next_map_first_point = self.map_list.chose_start_mission_map(self.map_idx+1).g_path[0]  # 첫 좌표 (x, y)
                # print(f"{next_map_first_point=}")

                # 두 좌표 간의 유클리드 거리 계산
                distance = math.sqrt((current_map_last_point[0] - next_map_first_point[0]) ** 2 +
                                    (current_map_last_point[1] - next_map_first_point[1]) ** 2)

                # 거리가 임계값보다 작으면 self.map_idx 증가
                if distance <= threshold_distance:
                    self.map_idx += 1
                    print(f"Map index updated to {self.map_idx} as the distance is {distance}m.")
                else:
                    print(f"Distance {distance}m is greater than the threshold. Map index remains {self.map_idx}.")
            else:
                print("No next map to switch to.")


    def ob_callback(self, msg):
        obstacles = []

        if self.current_state is not None:
            for i in range(0, len(msg.data), 2):
                ob_rel_x = msg.data[i]  # x 좌표
                ob_rel_y = msg.data[i+1]  # y 좌표

                # UTM 좌표계로 변환 (로봇 위치와 장애물 상대 좌표 사용)
                ob_utm_x, ob_utm_y = to_utm(self.current_state['x'], 
                                                self.current_state['y'],
                                                ob_rel_x,
                                                ob_rel_y,
                                                self.current_state['yaw'])
                # print(f"Obstacle UTM coordinates: {(ob_utm_x, ob_utm_y)}")

                # 장애물 좌표를 배열에 추가
                obstacles.append([ob_utm_x, ob_utm_y])

            # 장애물이 있으면 numpy 배열로 저장, 없으면 빈 배열로 초기화
            if obstacles:
                self.ob = np.array(obstacles)
            else:
                self.ob = np.empty((0, 2))

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
    
    def map_change_by_warp(self):
        if self.warp_check:
            if len(self.map_list.map_key) > self.map_idx + 1:
                self.map_idx += 1
        
        self.warp_check = 0

    def map_change_by_reset(self):
        pass
        # if self.reset_check : # 또 무슨 조건 필요하지????
        #     if len(self.map_list.map_key) > self.map_idx + 1:
        #         self.map_idx += 1

        # self.reset_check = 0

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
                        print(f"미션 변경")
                        self.order_idx += 1
                    
                        if len(self.map_list.map_key) > self.map_idx + 1:
                            print(f"맵 변경")
                            self.map_idx += 1
            else:
                print(f"목표 물품:{self.order[self.order_idx]['index']} 목표거리:{distance:.2f} 맵idx:{self.map_idx} 적재 수:{self.loaded_item}")
                       
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

            self.map_change_by_warp() # 워프여부 확인
            self.map_change_by_dist() # 다음 맵과의 거리 체크

            map_info = self.map_list.chose_start_mission_map(self.map_idx)
            if map_info is None:
                raise Exception("no map info...")
            
            g_path = map_info.g_path
            g_kd_tree = map_info.g_kd_tree

            x = [self.current_state['x'],
                self.current_state['y'],
                self.current_state['yaw'],
                self.current_state['v'], 
                0]
            
            ob = self.ob
            self.cal_target_idx(g_path=g_path, g_kd_tree=g_kd_tree)
            u, pred_trajectory = self.dwa_controller.dwa_control(x, self.target_point, ob)
            linear_vel, angular_vel = u

            self.send_control(linear_vel, -angular_vel)

            mission_dist = math.hypot(x[0] - self.order[self.order_idx]['utm_x'], 
                                    x[1] - self.order[self.order_idx]['utm_y'])
            
            self.send_request(mission_dist)

            # 마지막 위치가 어디가 될까???
            last_map_key = self.map_list.map_key[-1]
            last_g_path = (self.map_list.map_dic[last_map_key]).g_path
            
            if map_info.file_name == last_map_key:
                final_goal = math.hypot(x[0] - last_g_path[-1][0], x[1] - last_g_path[-1][1])
                if final_goal <= self.cfg.robot_radius:
                    rospy.loginfo("전체 미션 종료")
                    self.send_control(0, 0)

            # self.g_path = g_path
            # self.pred_trajectory = pred_trajectory


    def plot_trajectory(self):
        fig, ax = plt.subplots()

        if not self.odom_check:
            x, y = 0,0
        else:
            x, y= self.current_state['x'], self.current_state['y']
        def update_plot(self):
            ax.clear()
            # 글로벌 경로 시각화
            g_x = [point[0] for point in self.g_path]
            g_y = [point[1] for point in self.g_path]
            ax.plot(g_x, g_y, label='Global Path', color='blue')

            # 예측 궤적 시각화
            pred_x = [point[0] for point in self.pred_trajectory]
            pred_y = [point[1] for point in self.pred_trajectory]
            ax.plot(pred_x, pred_y, label='Predicted Trajectory', color='red', linestyle='--')

            # 장애물 시각화
            if self.ob.size > 0 and self.ob is not None:
                ax.scatter(self.ob[:, 0], self.ob[:, 1], label='Obstacles', color='black', s=1)

            # 현재 위치 표시
            ax.scatter(x,y, label='Current Position', color='green', marker='x')

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_title('Path and Predicted Trajectory')
            ax.legend()
            ax.grid(True)

        ani = animation.FuncAnimation(fig, update_plot, interval=100)
        plt.show()

if __name__ == '__main__':
    try:
        MoraiRobotControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass