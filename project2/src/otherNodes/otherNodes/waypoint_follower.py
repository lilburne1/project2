import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import String, Bool

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.number_coords = {}
        self.waypoint_list = None

        self.number_coordinates_subscriber = self.create_subscription(
            String,
            "number_coordinates",
            self.new_number,
            10
        )

        self.waypoint_subscriber = self.create_subscription(
            Float32MultiArray,
            "waypoints",
            self.waypoints,
            10
        )

        self.waypoint_start = self.create_subscription(
            Bool,
            "waypoint_start",
            self.waypoint_following,
            10
        )

        # self.start_waypoint = self.create_subscription 
        nav = BasicNavigator()

        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'  
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        init_pose.pose.position.z = 0.0  
        nav.setInitialPose(init_pose)

    def new_number(self, msg):
        message = msg.data.split(',')
        number = message[0]
        x = message[1]
        y = message[2]
        
        self.numbers[number] = (x, y)

        new_point = "New number {number}: ({x},{y})"
        self.get_logger().info(new_point)

    def waypoints(self, msg):
        self.waypoints = msg.data

    def waypoint_following(self, msg):
        PoseList = [] 
        if self.waypoint_list != None and len(self.number_coords) >= 3:
            for waypoint in self.waypoint_list:
                coord = PoseStamped()
                coord.header.frame_id = 'map'  
                coord.header.stamp = self.get_clock().now().to_msg()
                coord.pose.position.x = self.number_coords[waypoint][0]
                coord.pose.position.y = self.number_coords[waypoint][1]
                coord.pose.position.z = 0.0  
                coord.pose.orientation.w = 1.0  
                PoseList.append(coord)
                # Log the waypoint and its coordinates
                self.get_logger().info(f'Waypoint {waypoint}: (x={self.number_coords[waypoint][0]}, y={self.number_coords[waypoint][1]})')

        coord = PoseStamped()
        coord.header.frame_id = 'map'  
        coord.header.stamp = self.get_clock().now().to_msg()
        coord.pose.position.x = 0.0
        coord.pose.position.y = 0.0
        coord.pose.position.z = 0.0  
        coord.pose.orientation.w = 1.0  
        PoseList.append(coord)

        self.nav.goThroughPoses(PoseList)

def main(args = None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)

    waypoint_follower.destroy_node()
    rclpy.shutdown()
