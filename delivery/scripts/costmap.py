#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Imu
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Point, Pose
from tf.transformations import euler_from_quaternion
import tf2_ros
from geometry_msgs.msg import TransformStamped
from morai_msgs.msg import EgoDdVehicleStatus

class CostmapNode:
    def __init__(self):
        rospy.init_node('costmap_node', anonymous=True)

        # Parameters
        self.map_size_x = rospy.get_param('~map_size_x', 10.0)  # meters
        self.map_size_y = rospy.get_param('~map_size_y', 10.0)  # meters
        self.resolution = rospy.get_param('~resolution', 0.1)  # meters per cell
        self.occupancy_threshold = rospy.get_param('~occupancy_threshold', 50)  # Occupancy threshold

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/Local/odom', EgoDdVehicleStatus, self.odom_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)

        # Publishers
        self.costmap_pub = rospy.Publisher('/local_costmap', OccupancyGrid, queue_size=1)
        self.obstacle_marker_pub = rospy.Publisher('/dynamic_obstacles_marker', MarkerArray, queue_size=1)

        # State Variables
        self.current_pose = None  # (x, y, yaw)
        self.imu_orientation = None  # (roll, pitch, yaw)
        self.lidar_points = []  # list of (x, y)
        self.costmap = None  # 2D numpy array

        # Fixed origin
        self.origin_x = -5.0  # meters
        self.origin_y = -5.0  # meters

        # Timer for periodic publishing
        self.publish_rate = rospy.get_param('~publish_rate', 10)  # Hz
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)

        rospy.loginfo("Costmap Node Initialized")

    def odom_callback(self, msg):
        """
        EgoDdVehicleStatus 메시지를 처리하여 현재 로봇 위치 및 방향 업데이트.
        """
        rospy.loginfo("Received /Local/odom message")
        x = msg.position.x
        y = msg.position.y
        yaw = msg.heading  # 라디안 단위의 헤딩
        speed = msg.linear_velocity.x
        yaw_rate = msg.position.z  # position.z을 yaw_rate로 사용

        self.current_pose = (x, y, yaw)

        rospy.logdebug(f"Updated pose: x={x}, y={y}, yaw={yaw}, speed={speed}, yaw_rate={yaw_rate}")

    def imu_callback(self, msg):
        """
        IMU 데이터를 처리하여 로봇의 자세 정보 업데이트.
        """
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.imu_orientation = (roll, pitch, yaw)

        rospy.logdebug(f"Updated IMU orientation: roll={roll}, pitch={pitch}, yaw={yaw}")

    def scan_callback(self, msg):
        """
        LiDAR 데이터를 처리하여 카르테시안 좌표로 변환.
        """
        self.lidar_points = self.lidar_to_cartesian(msg)

    def lidar_to_cartesian(self, scan_msg):
        """
        LiDAR 데이터(LaserScan)를 카르테시안 좌표로 변환.
        """
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges

        cartesian_points = []
        for i, r in enumerate(ranges):
            if np.isinf(r) or np.isnan(r):
                continue
            angle = angle_min + i * angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            cartesian_points.append((x, y))
        return cartesian_points

    def update_costmap(self):
        """
        LiDAR 데이터를 기반으로 로컬 코스트맵을 생성.
        """
        if not self.current_pose:
            rospy.logwarn("Current pose not available. Skipping costmap update.")
            return

        width = int(self.map_size_x / self.resolution)
        height = int(self.map_size_y / self.resolution)

        # Initialize costmap with -1 (unknown)
        costmap = np.full((height, width), -1, dtype=np.int8)

        # Populate costmap with occupied cells (100) based on LiDAR points
        for point in self.lidar_points:
            # Transform LiDAR point to global frame
            x_robot, y_robot, yaw = self.current_pose
            x_lidar, y_lidar = point

            # Rotate LiDAR point based on robot's yaw
            x_global = x_lidar * math.cos(yaw) - y_lidar * math.sin(yaw)
            y_global = x_lidar * math.sin(yaw) + y_lidar * math.cos(yaw)

            rospy.logdebug(f"LiDAR point transformed to global: ({x_global}, {y_global})")

            # Convert global coordinates to map indices
            map_x = int((x_global - self.origin_x) / self.resolution)
            map_y = int((y_global - self.origin_y) / self.resolution)

            if 0 <= map_x < width and 0 <= map_y < height:
                costmap[map_y][map_x] = 100  # Occupied

        self.costmap = costmap


        rospy.logdebug(f"Costmap updated with shape: {self.costmap.shape}")
        rospy.logdebug(f"Costmap origin set to: ({self.origin_x}, {self.origin_y})")
        
    
    
    def visualize_costmap(self):
        """
        로컬 코스트맵을 OccupancyGrid 메시지로 퍼블리시.
        """
        if self.costmap is None:
            rospy.logwarn("Costmap is not available. Skipping visualization.")
            return

        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = rospy.Time.now()
        costmap_msg.header.frame_id = "base_link"

        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.costmap.shape[1]
        costmap_msg.info.height = self.costmap.shape[0]
        costmap_msg.info.origin = Pose()
        costmap_msg.info.origin.position.x = self.origin_x
        costmap_msg.info.origin.position.y = self.origin_y
        costmap_msg.info.origin.position.z = 0.0
        costmap_msg.info.origin.orientation = Quaternion(0, 0, 0, 1)

        # Flatten the costmap
        costmap_flat = self.costmap.flatten(order='C')
        costmap_msg.data = costmap_flat.tolist()

        self.costmap_pub.publish(costmap_msg)
        rospy.loginfo("Published local costmap.")

    def visualize_dynamic_obstacles(self):
        """
        동적 장애물을 MarkerArray 메시지로 퍼블리시.
        """
        if not self.lidar_points:
            rospy.logwarn("No LiDAR points available for dynamic obstacle visualization.")
            return

        marker_array = MarkerArray()
        x_robot, y_robot, yaw = self.current_pose

        for i, point in enumerate(self.lidar_points):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "dynamic_obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Transform LiDAR point to global frame
            x_lidar, y_lidar = point
            x_global = x_lidar * math.cos(yaw) - y_lidar * math.sin(yaw)
            y_global = x_lidar * math.sin(yaw) + y_lidar * math.cos(yaw)

            marker.pose.position = Point(x_global, y_global, 0.0)  # 지면에 위치
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker.scale.x = 0.2  # 구의 반지름
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0  # 불투명
            marker.color.r = 1.0  # 빨간색
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.obstacle_marker_pub.publish(marker_array)
        rospy.loginfo(f"Published {len(marker_array.markers)} dynamic obstacle markers.")

    def timer_callback(self, event):
        """
        주기적으로 로컬 코스트맵을 업데이트하고 퍼블리시.
        """
        self.update_costmap()
        self.visualize_costmap()
        self.visualize_dynamic_obstacles()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        costmap_node = CostmapNode()
        costmap_node.run()
    except rospy.ROSInterruptException:
        pass
