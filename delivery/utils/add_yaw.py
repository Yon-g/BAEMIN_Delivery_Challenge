#!/usr/bin/env python3

import os

def process_utm_file(input_file, output_file):
    with open(input_file, 'r') as infile:
        lines = infile.readlines()

    with open(output_file, 'w') as outfile:
        for line in lines:
            # UTM 좌표 불러오기
            coords = line.strip().split(',')
            if len(coords) == 2:
                x, y = coords
                # 세 번째 값으로 yaw 대신 0을 추가
                outfile.write(f'{x},{y},0\n')

    print(f"Processed UTM coordinates with yaw=0 and saved to {output_file}")

if __name__ == '__main__':
    # 입력 파일과 출력 파일 경로 설정
    input_file = '/root/ws/src/BAEMIN_Delivery_Challenge/delivery/map/2.txt'  # UTM 좌표가 있는 파일 경로
    output_file = '/root/ws/src/BAEMIN_Delivery_Challenge/delivery/map/2_1.txt'  # 결과를 저장할 파일 경로

    # UTM 파일 처리
    process_utm_file(input_file, output_file)