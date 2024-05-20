import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformedStamped
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException

class RobotPosition(Node):
    def __init__(self):
        super().__init__('robot_position')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Check position every second

        self.robot_position_publisher = self.create_publisher(PoseStamped, "robot_position", 10)

    def timer_callback(self):
        try:
            now = self.get_clock().now()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=Duration(seconds=1.0))

            pose_msg = PoseStamped()
            pose_msg.header.stamp = now.to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = trans.transform.translation.z
            pose_msg.pose.orientation = trans.transform.rotation

            self.robot_position_publisher.publish(pose_msg)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Transform error: {str(e)}")
    
def main(args = None):
    rclpy.init(args=args)
    robot_position = RobotPosition()
    rclpy.spin(robot_position)

    robot_position.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
