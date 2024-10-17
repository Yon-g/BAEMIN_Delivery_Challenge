from enum import Enum
import rospy
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd
from morai_msgs.msg import EgoDdVehicleStatus
import math

import pandas as pd
from scipy.spatial import KDTree
import math
import numpy as np
import os
import threading
import matplotlib.pyplot as plt

CURRENT_FILE_PATH = os.path.abspath(__file__)
CURRENT_DIRECTORY = os.path.dirname(CURRENT_FILE_PATH)

# 부모 경로를 가져옴
PARENT_DIRECTORY = os.path.dirname(CURRENT_DIRECTORY)

# 부모 경로에 있는 "map" 디렉토리 경로 설정
GLOBAL_UTM_TXT_ADDRESS = os.path.join(PARENT_DIRECTORY, "map/")

TARGET_DWA_IDX = 30

class RobotType(Enum):
    circle = 0
    rectangle = 1

class DynamicWindowApproach:
    def __init__(self, config):
        self.config = config

    def dwa_control(self, x, goal, ob):
        """
        Dynamic Window Approach control
        """
        dw = self.calc_dynamic_window(x)
        u, trajectory = self.calc_control_and_trajectory(x, dw, goal, ob)
        return u, trajectory

    def calc_dynamic_window(self, x):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yaw_rate, self.config.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [x[3] - self.config.max_accel * self.config.dt,
              x[3] + self.config.max_accel * self.config.dt,
              x[4] - self.config.max_delta_yaw_rate * self.config.dt,
              x[4] + self.config.max_delta_yaw_rate * self.config.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def calc_control_and_trajectory(self, x, dw, goal, ob):
        """
        calculation final input with dynamic window
        """

        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.config.v_resolution):
            for y in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):
                trajectory = self.predict_trajectory(x_init, v, y)
                # calc cost
                to_goal_cost = self.config.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - trajectory[-1, 3])
                ob_cost = self.config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob)
                print(ob_cost,to_goal_cost,speed_cost)
                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < self.config.robot_stuck_flag_cons \
                            and abs(x[3]) < self.config.robot_stuck_flag_cons:
                        best_u[1] = -self.config.max_delta_yaw_rate

        return best_u, best_trajectory

    def predict_trajectory(self, x_init, v, y):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.config.predict_time:
            x = self.motion(x, [v, y])
            trajectory = np.vstack((trajectory, x))
            time += self.config.dt

        return trajectory

    def motion(self, x, u):
        """
        motion model
        """
        x[2] += u[1] * self.config.dt
        x[0] += u[0] * math.cos(x[2]) * self.config.dt
        x[1] += u[0] * math.sin(x[2]) * self.config.dt
        x[3] = u[0]
        x[4] = u[1]
        return x

    def calc_obstacle_cost(self, trajectory, ob):
        """
        calc obstacle cost inf: collision
        """
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if self.config.robot_type == RobotType.rectangle:
            yaw = trajectory[:, 2]
            rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            rot = np.transpose(rot, [2, 0, 1])
            local_ob = ob[:, None] - trajectory[:, 0:2]
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            local_ob = np.array([local_ob @ x for x in rot])
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            upper_check = local_ob[:, 0] <= self.config.robot_length / 2
            right_check = local_ob[:, 1] <= self.config.robot_width / 2
            bottom_check = local_ob[:, 0] >= -self.config.robot_length / 2
            left_check = local_ob[:, 1] >= -self.config.robot_width / 2
            if (np.logical_and(np.logical_and(upper_check, right_check),
                               np.logical_and(bottom_check, left_check))).any():
                return float("Inf")
        elif self.config.robot_type == RobotType.circle:
            if np.array(r <= self.config.robot_radius).any():
                return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK

    def calc_to_goal_cost(self, trajectory, goal):
        """
        calc to goal cost with angle difference
        """
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        return cost




class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 2.0  # [m/s]
        self.min_speed = 0.0 # [m/s]
        self.max_yaw_rate = 0.83  # [rad/s]
        self.max_accel = 1.0  # [m/ss]
        self.max_delta_yaw_rate = 1.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 # [rad/s]
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
        self.odom_sub = rospy.Subscriber('/Local/odom', EgoDdVehicleStatus, self.odom_callback)

        #control pub
        self.ctrl_pub = rospy.Publisher('/dilly_ctrl', SkidSteer6wUGVCtrlCmd, queue_size=1)
        
        self.current_state = None
        self.target_point = None

    def odom_callback(self, msg):
        # 차량 상태 (위치, 속도 등)을 받아옴
        self.current_state = {
            'x': msg.position.x,
            'y': msg.position.y,
            'yaw': msg.heading,  # 라디안 단위의 헤딩
            'v': msg.linear_velocity.x  # x 축 속도 (앞뒤)
        }
    
    def cal_target_idx(self, g_path, g_kd_tree):

        target_idx = find_target_path_idx(len(g_path),self.current_state['x'],self.current_state['y'],g_kd_tree)
        target_point = g_path[target_idx]

        self.target_point = target_point
    
    def send_control(self, linear_velocity, angular_velocity):
        # DWA 알고리즘 결과로 나온 속도 명령을 Morai에 전달
        cmd = SkidSteer6wUGVCtrlCmd()
        cmd.cmd_type = 3
        cmd.Target_linear_velocity= linear_velocity
        cmd.Target_angular_velocity = angular_velocity
        self.ctrl_pub.publish(cmd)

class Map:
    def __init__(self,file_name,g_path,g_yaw):
        self.file_name = file_name
        self.g_path = g_path
        self.g_yaw = g_yaw
        self.g_kd_tree = KDTree(self.g_path)

class MapList:
    def __init__(self, map_path):
        self.map_dic = {}
        self.map_folder_path = map_path
        self.create_map()

    def load_g_path_from_txt(self,f_name):
        map_data_txt = pd.read_csv(self.map_folder_path+str(f_name), sep=',', encoding='utf-8') # 파일 내용 저장 -> 2차원 리스트

        UTMmap_arr = map_data_txt.to_numpy() # 리스트를 numpy로 변환
        g_path = UTMmap_arr[:,:2]
        g_yaw = UTMmap_arr[:,2]

        return g_path, g_yaw

    def create_map(self):
        file_list = os.listdir(self.map_folder_path)

        for file_name in file_list:
            g_path,g_yaw = self.load_g_path_from_txt(file_name)
            self.read_map(file_name,g_path,g_yaw)

    def read_map(self, file_name, g_path, g_yaw):
        self.map_dic[file_name] = Map(file_name,g_path,g_yaw)

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

def plot_path(g_path, robot_control):
    """
    Function to plot the global path and the vehicle's position in a separate thread.
    Continuously updates the vehicle's position on the plot.
    """
    plt.ion()  # Enable interactive mode for live updating plot
    fig, ax = plt.subplots()
    
    # Plot the global path
    g_path_x = g_path[:, 0]
    g_path_y = g_path[:, 1]
    ax.plot(g_path_x, g_path_y, label="Global Path", color="blue")
    
    # Initialize vehicle's position marker
    vehicle_marker, = ax.plot([], [], 'ro', label="Vehicle Position")
    target_marker, = ax.plot([], [], 'go', label="Vehicle Target")
    
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Global Path and Vehicle Position")
    ax.legend()
    ax.grid(True)
    
    while not rospy.is_shutdown():
        if robot_control.current_state is not None:
            # Update the vehicle's current position on the plot
            vehicle_x = robot_control.current_state['x']
            vehicle_y = robot_control.current_state['y']

            vehicle_marker.set_data(vehicle_x, vehicle_y)
        if robot_control.target_point is not None:
            target_marker.set_data(robot_control.target_point[0],robot_control.target_point)
        plt.draw()
        plt.pause(0.1)

def main():
    print("go")
    robot_control = MoraiRobotControl()
    cfg = Config()
    dwa_controller = DynamicWindowApproach(cfg)
    map_list = MapList(GLOBAL_UTM_TXT_ADDRESS)
    # goal position 설정
    # goal = np.array([5.0, 5.0])  # 목표 지점
    
    ob = np.array([[0,0]])
    
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

    plot_thread = threading.Thread(target=plot_path, args=(g_path, robot_control))
    plot_thread.start()

    while not rospy.is_shutdown():
        x = [robot_control.current_state['x'],
            robot_control.current_state['y'],
            robot_control.current_state['yaw'],
            robot_control.current_state['v'], 
            0.0]
        robot_control.cal_target_idx(g_path,g_kd_tree)
        u, predicted_trajectory = dwa_controller.dwa_control(x, robot_control.target_point, ob)
        linear_velocity, angular_velocity = u

        # Morai에 속도 명령어 보내기
        robot_control.send_control(linear_velocity, angular_velocity)

        # 종료 조건: 목표에 도달했는지 확인
        dist_to_goal = math.hypot(x[0] - robot_control.target_point[0], x[1] - robot_control.target_point[1])
        if dist_to_goal <= cfg.robot_radius:
            rospy.loginfo("Goal Reached")
            break
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
