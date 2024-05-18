import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import Pose

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.create_publisher()
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active(localizer="robot_localization")
        self.navigator.goToPose((1.0, 1.0))
        result = self.navigator.waitUntilNav2GoalAchieved()

        # Define some waypoints to explore
        # waypoints = [
        #     (1.0, 1.0),
        #     (1.0, -1.0),
        #     (-1.0, -1.0),
        #     (-1.0, 1.0)
        # ]

        # # Visit each waypoint
        # for waypoint in waypoints:
        #     self.navigator.goToPose(waypoint)
        #     result = self.navigator.waitUntilNav2GoalAchieved()

def main(args = None):
    rclpy.init(args=args)
    explorer = Explorer()
    rclpy.spin(explorer)

    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
