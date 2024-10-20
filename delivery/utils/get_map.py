import os
import math
import pandas as pd
from scipy.spatial import KDTree
import rospy
import matplotlib.pyplot as plt

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