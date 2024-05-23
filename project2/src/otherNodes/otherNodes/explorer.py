import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.action import ActionClient
import math
from std_msgs.msg import Bool

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("WHYY")

        # Initialize the navigator
        self.navigator = BasicNavigator()

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
        y = 0.0
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
        pose.pose.position.x = float(goal[0])
        pose.pose.position.y = float(goal[1])
        pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Going to ({goal[0]}, {goal[1]})')

        self.navigator.goToPose(pose)

        # Check the result
        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded')
        elif result == NavigationResult.CANCELED:
            self.get_logger().info('Goal canceled')
        elif result == NavigationResult.FAILED:
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
