import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.navigator = BasicNavigator()

        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 0.0
        # initial_pose.pose.position.y = 0.0
        # initial_pose.pose.orientation.z = 0.0
        # initial_pose.pose.orientation.w = 0.0
        # self.navigator.setInitialPose(initial_pose)

        goal_pose = PoseStamped()
        goal_pose.pose.position.x = 0.2
        goal_pose.pose.position.y = 0.2
        self.navigator.goToPose(goal_pose)
 
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
