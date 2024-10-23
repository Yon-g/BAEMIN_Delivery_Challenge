import math

def to_utm(ego_x, ego_y, rel_x, rel_y, yaw):
    # 라이다 오프셋 고려
    rel_x += 0.2
    rel_y += 0.2

    # 상대 좌표를 차량의 yaw를 기준으로 회전 변환하여 절대 UTM 좌표로 변환
    utm_x = ego_x + (rel_x * math.cos(yaw) - rel_y * math.sin(yaw))
    utm_y = ego_y + (rel_x * math.sin(yaw) + rel_y * math.cos(yaw))

    return utm_x, utm_y

