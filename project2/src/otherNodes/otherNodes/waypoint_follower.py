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

        self.waypoints_subscriber = self.create_subscription(
            Float32MultiArray,
            "waypoints",
            self.waypoints,
            10
        )

        nav = BasicNavigator()

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
            

            







#     def send_next_goal(self):
#         goal = self.goals[self.current_goal_index]
#         self.current_goal_index += 1

#         pose = PoseStamped()
#         pose.header.frame_id = 'map'
#         pose.header.stamp = self.get_clock().now().to_msg()
#         pose.pose.position.x = goal[0]
#         pose.pose.position.y = goal[1]
#         pose.pose.orientation.w = 1.0

#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = pose

#         information = "Going to ({goal[0]}, {goal[1]})"
#         self.get_logger().info(information)

#         self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected')
#             self.send_next_goal()
#             return

#         self.get_logger().info('Goal accepted')
#         goal_handle.result().add_done_callback(self.result_callback)

#     def result_callback(self, future):
#         result = future.result().result
#         if result.result == NavigateToPose.Result.SUCCESS:
#             self.get_logger().info('Goal succeeded')
#         else:
#             self.get_logger().info('Goal failed')

#         self.send_next_goal()

# def main(args = None):
#     rclpy.init(args=args)
#     waypoint_follower = WaypointFollower()
#     rclpy.spin(waypoint_follower)

#     waypoint_follower.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
