import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, String, Bool
from nav2_simple_commander.robot_navigator import BasicNavigator

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

        self.explore_subscription = self.create_subscription(
            Bool,
            "explore",
            self.go_home,
            10
        )

        self.web_logger_pub = self.create_publisher(String, 'web_logger', 10)

        self.nav = BasicNavigator()

        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'  
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        init_pose.pose.position.z = 0.0  
        self.nav.setInitialPose(init_pose)

    def new_number(self, msg):
        message = msg.data.split(',')
        number = message[0]
        x = float(message[1])
        y = float(message[2])
        
        self.number_coords[number] = (x, y)

    def waypoints(self, msg):
        self.waypoint_list = msg.data

    def waypoint_following(self, msg):
        PoseList = [] 
        if self.waypoint_list is not None and len(self.number_coords) >= 3:
            for waypoint in self.waypoint_list:
                coord = PoseStamped()
                coord.header.frame_id = 'map'  
                coord.header.stamp = self.get_clock().now().to_msg()
                coord.pose.position.x = self.number_coords[waypoint][0]
                coord.pose.position.y = self.number_coords[waypoint][1]
                coord.pose.position.z = 0.0  
                coord.pose.orientation.w = 1.0  
                PoseList.append(coord)
                # Publish to web_logger
                log_msg = String()
                log_msg.data = f"driving to coordinate (x={self.number_coords[waypoint][0]}, y={self.number_coords[waypoint][1]})"
                self.web_logger_pub.publish(log_msg)

        coord = PoseStamped()
        coord.header.frame_id = 'map'  
        coord.header.stamp = self.get_clock().now().to_msg()
        coord.pose.position.x = 0.0
        coord.pose.position.y = 0.0
        coord.pose.position.z = 0.0  
        coord.pose.orientation.w = 1.0  
        PoseList.append(coord)

        self.nav.goThroughPoses(PoseList)

    def go_home(self, msg):
        if not msg.data:
            coord = PoseStamped()
            coord.header.frame_id = 'map'  
            coord.header.stamp = self.get_clock().now().to_msg()
            coord.pose.position.x = 0.0
            coord.pose.position.y = 0.0
            coord.pose.position.z = 0.0  
            coord.pose.orientation.w = 1.0  

            self.nav.goToPose(coord)

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)

    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
