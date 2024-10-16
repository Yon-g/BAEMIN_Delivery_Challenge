from enum import Enum
import rospy
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd
import math

# int32 cmd_type

# bool Forward_input
# bool Backward_input
# bool Left_Turn_input
# bool Right_Turn_input

# float32 left_front_wheel_rpm
# float32 left_middle_wheel_rpm
# float32 left_rear_wheel_rpm
# float32 right_front_wheel_rpm
# float32 right_middle_wheel_rpm
# float32 right_rear_wheel_rpm

# float32 Target_linear_velocity
# float32 Target_angular_velocity

class MoraiRobotControl:
    def __init__(self):
        rospy.init_node('morai_robot_control')
        
        # 속도 명령어 퍼블리셔
        self.vel_pub = rospy.Publisher('/dilly_ctrl', SkidSteer6wUGVCtrlCmd, queue_size=10)

    def send_control(self):
        # DWA 알고리즘 결과로 나온 속도 명령을 Morai에 전달
        cmd = SkidSteer6wUGVCtrlCmd()
        cmd.cmd_type = 3
        # cmd.Forward_input = False
        # cmd.Left_Turn_input = False
        # cmd.Right_Turn_input = False
        # cmd.Backward_input = False
        # cmd.left_front_wheel_rpm = 200
        # cmd.left_middle_wheel_rpm = 200
        # cmd.left_rear_wheel_rpm = 200
        # cmd.right_front_wheel_rpm = 100
        # cmd.right_middle_wheel_rpm = 100
        # cmd.right_rear_wheel_rpm = 100
        cmd.Target_linear_velocity = 2
        cmd.Target_angular_velocity = -0.83
        self.vel_pub.publish(cmd)

def main():
    robot_control = MoraiRobotControl()

    # DWA 루프 시작
    rate = rospy.Rate(10)  # 10Hz로 루프 실행
    print("pub")
    while not rospy.is_shutdown():
        print("pub")
        robot_control.send_control()

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
