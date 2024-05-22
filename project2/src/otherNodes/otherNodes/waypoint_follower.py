import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
from std_msgs.msg import String, Bool

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()

        self.number_subscriber = self.create_subscription(
            String,
            "number_coordinates",
            self.new_number,
            10
        )

        self.waypoint_subscriber = self.create_subscription(
            Bool,
            "waypoint_input",
            self.waypoint,
            10
        )

        self.numbers = {}
        self.current_goal_index = 0
        self.goals = []

    def new_number(self, msg):
        number, x, y = msg.data.split(',')
        
        new_point = "New number {number}: ({x},{y})"
        self.numbers[number] = (x, y)

        self.get_logger().info(new_point)

    def waypoint(self, msg):
        user_input = msg.data
        number_goals = user_input.split(",")
        for number in number_goals:
            if number in self.numbers:
                self.goals.append(self.numbers[number])
        self.goals.append((0, 0))
        
        self.current_goal_index = 0
        self.send_next_goal()

    def send_next_goal(self):
        goal = self.goals[self.current_goal_index]
        self.current_goal_index += 1

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal[0]
        pose.pose.position.y = goal[1]
        pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        information = "Going to ({goal[0]}, {goal[1]})"
        self.get_logger().info(information)

        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted')
        goal_handle.result().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.result == NavigateToPose.Result.SUCCESS:
            self.get_logger().info('Goal succeeded')
        else:
            self.get_logger().info('Goal failed')

        self.send_next_goal()

def main(args = None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)

    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
