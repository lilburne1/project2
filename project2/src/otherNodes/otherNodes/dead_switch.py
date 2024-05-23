#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

class DeadManSwitch(Node):
    def __init__(self):
        super().__init__("dead_man_switch")

        self.drive_state = "STOP"
        self.dead_man_switch = True

        # Creates subscription to joy node to check controller
        self.joy_subscription = self.create_subscription(
            Joy,
            "joy",
            self.button_press_callback,
            10
        )

        # Creates subscription to cmd_vel_joy 
        self.joy_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_joy",
            self.joy_vel_callback,
            10
        )

        # Creates subscription to cmd_vel_nav
        self.nav_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_nav",
            self.nav_vel_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/data_raw",
            self.imu_repub,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.robot_twist_publisher = self.create_publisher(TwistWithCovarianceStamped, "robot_twist", 10)
        self.explore_publisher = self.create_publisher(Bool, "explore", 10)

        self.imu_pub = self.create_publisher(Imu, "transformed_imu", 10)

    def button_press_callback(self, msg):
        # Dead man's switch for robot
        if (msg.buttons[10] == 1 or msg.buttons[9] == 1):
            self.dead_man_switch = True
        else:
            self.dead_man_switch = False

        # Controls state of robots - either AUTO or MANUAL
        if msg.buttons[0] == 1 and self.drive_state != "AUTO":
            self.drive_state = "AUTO"
            self.get_logger().info("In automatic driving mode...")
            
        if msg.buttons[1] == 1 and self.drive_state != "MANUAL":
            self.drive_state = "MANUAL"
            self.get_logger().info("In manual driving mode...")

        if msg.buttons[2] == 1:
            true_msg = Bool()
            true_msg.data = True
            self.explore_publisher.publish(true_msg)
            self.get_logger().info("In automatic exploration mode...")

        if msg.buttons[11] == 1:
            false_msg = Bool()
            false_msg.data = False
            self.explore_publisher.publish(false_msg)
            self.get_logger().info("Stopping mapping, returning home...")


    def nav_vel_callback(self, msg):
        if self.dead_man_switch and self.drive_state == "AUTO":
            self.cmd_vel_publisher.publish(msg)

            twist_cov_msg = self.create_twist_with_covariance(msg)
            self.robot_twist_publisher.publish(twist_cov_msg)
        else:
            self.publish_stop_message()

    def joy_vel_callback(self, msg):
        if self.dead_man_switch and  self.drive_state == "MANUAL":
            self.cmd_vel_publisher.publish(msg)

            twist_cov_msg = self.create_twist_with_covariance(msg)
            self.robot_twist_publisher.publish(twist_cov_msg)
        else:
            self.publish_stop_message()
        
    def publish_stop_message(self):
        stop_msg = Twist() 
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)

        twist_cov_msg = self.create_twist_with_covariance(stop_msg)
        self.robot_twist_publisher.publish(twist_cov_msg)

    def imu_repub(self, msg):
        msg.angular_velocity.z *= -1 
        self.imu_pub.publish(msg)

    def create_twist_with_covariance(self, twist_msg):
        twist_cov_msg = TwistWithCovarianceStamped()
        twist_cov_msg.header.stamp = self.get_clock().now().to_msg()
        twist_cov_msg.header.frame_id = 'base_link'
        twist_cov_msg.twist.twist = twist_msg
        twist_cov_msg.twist.covariance = [0.0] * 36
        return twist_cov_msg

def main(args = None):
    rclpy.init(args=args)
    dead_man_switch = DeadManSwitch()
    rclpy.spin(dead_man_switch)

    dead_man_switch.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
