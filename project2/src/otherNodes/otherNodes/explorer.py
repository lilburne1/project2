import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.navigator = BasicNavigator()

    def explore(self):
        # Wait for Nav2 to fully activate
        self.navigator.waitUntilNav2Active()

        # Define some waypoints to explore
        waypoints = [
            (1.0, 1.0),
            (1.0, -1.0),
            (-1.0, -1.0),
            (-1.0, 1.0)
        ]

        # Visit each waypoint
        for waypoint in waypoints:
            self.navigator.goToPose(waypoint)
            result = self.navigator.waitUntilNav2GoalAchieved()

def main(args = None):
    rclpy.init(args=args)
    explorer = Explorer()
    rclpy.spin(explorer)

    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
