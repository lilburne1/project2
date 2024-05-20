import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
import math

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')

        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            "scan",
            self.laserscan_callback,
            10
        )

        self.robot_position_subscriber = self.create_subscription(
            PoseStamped,
            "robot_position",
            self.position_callback, 
            10
        )

        self.cmd_vel_nav_publisher = self.create_publisher(Twist, "cmd_vel_nav", 10)

    def drive_to_point(self, point):
        x, y = self.position
        orientation = self.orientation

        goal_x, goal_y = point

        distance_to_point = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)

        while distance_to_point < 0.5:
            continue


        

    def position_callback(self, msg):
        self.position = (msg.pose.position.x, msg.pose.position.y)
        self.orientation = msg.pose.orientation
    
    def laserscan_callback(self, msg):
        self.scan = msg.ranges

def main(args = None):
    rclpy.init(args=args)
    explorer = Explorer()
    rclpy.spin(explorer)

    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()