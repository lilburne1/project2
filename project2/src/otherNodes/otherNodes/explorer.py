import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
from std_msgs.msg import Bool

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()

        self.explore_subscriber = self.create_subscription(
            Bool,
            "explore",
            self.explore, 
            10
        )

    def explore(self, msg):
        self.area_size = 14.0
        self.path_spacing = 1.0
        self.current_goal_index = 0
        self.goals = self.generate_lawn_mower_goals(self.area_size, self.path_spacing)

        self.send_next_goal()

    def generate_lawn_mower_goals(self, area_size, spacing):
        goals = []
        half_area = area_size / 2

        # Adjust the starting point to (0,0) and cover the entire area
        y = 0
        direction = 1

        while y <= half_area:
            # From (0, y) to (half_area, y)
            for x in range(0, int(half_area + 1)):
                goals.append((x * direction, y))
            y += spacing

            if y > half_area:
                break

            # From (half_area, y) to (0, y)
            for x in range(int(half_area), -1, -1):
                goals.append((x * direction, y))
            y += spacing

        # Mirror the goals for the negative y-axis and reverse the path to (0,0)
        mirrored_goals = []
        for x, y in goals:
            mirrored_goals.append((x, -y))

        # Combine and ensure to end at (0,0)
        all_goals = goals + mirrored_goals[::-1]
        all_goals.append((0, 0))

        return all_goals

    def send_next_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('Completed lawn mower pattern')
            return

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
    explorer = Explorer()
    rclpy.spin(explorer)

    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
