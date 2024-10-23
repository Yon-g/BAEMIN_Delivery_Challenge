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
        self.map_key = sorted(self.map_dic.keys())

    # 폴더로 부터 맵 로드 (전역경로, 곡률)
    def load_g_path_from_txt(self,f_name):
        map_data_txt = pd.read_csv(self.map_folder_path+str(f_name), sep=',', encoding='utf-8') # 파일 내용 저장 -> 2차원 리스트

        UTMmap_arr = map_data_txt.to_numpy() # 리스트를 numpy로 변환
        g_path = UTMmap_arr[:,:2]
        g_yaw = UTMmap_arr[:,2]

        return g_path, g_yaw

    # 초기에 파일 읽어서 Map 클래스에 데이터 저장
    def create_map(self):
        file_list = os.listdir(self.map_folder_path)

        for file_name in file_list:
            g_path,g_yaw = self.load_g_path_from_txt(file_name)
            self.read_map(file_name,g_path,g_yaw)

    # 파일 읽어서 저장
    def read_map(self, file_name, g_path, g_yaw):
        self.map_dic[file_name] = Map(file_name,g_path,g_yaw)

    # 현재 진행 중인 맵의 인덱스 정보를 넣으면 불러옴
    def chose_start_mission_map(self, map_idx):
        map_info = None
        key = self.map_key[map_idx]
        if key in self.map_dic:
            map_info = self.map_dic[key]

        return map_info



if __name__ == "__main__":
    map = "/root/ws/src/BAEMIN_Delivery_Challenge/delivery/map/"
    map_list = MapList(map)
    map_info = map_list.chose_start_mission_map(0)
    key = map_list.map_key[-1]
    last = map_list.map_dic[key].g_path
    # print(f"{last=}")

    map_idx = 0 # 이거 7미터 거리임

    current_map_last_point = map_list.chose_start_mission_map(map_idx).g_path[-1]  # 마지막 좌표 (x, y)
    print(f"{current_map_last_point[0]=}")

    # 다음 맵이 있는지 확인
    if map_idx + 1 < len(map_list.map_key):
        # 다음 맵의 첫 좌표
        next_map_first_point = map_list.chose_start_mission_map(map_idx+1).g_path[0]  # 첫 좌표 (x, y)
        print(f"{next_map_first_point=}")

        # 두 좌표 간의 유클리드 거리 계산
        distance = math.sqrt((current_map_last_point[0] - next_map_first_point[0]) ** 2 +
                            (current_map_last_point[1] - next_map_first_point[1]) ** 2)
        
        print(f"{distance=}")