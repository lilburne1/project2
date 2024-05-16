#!/usr/bin/env python3

import rlcpy
from rlcpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

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

        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        
    def button_press_callback(self, msg):
        # Dead man's switch for robot
        if (msg.button[9] != 1 or msg.button[10] != 1):
             self.dead_man_switch = False

        # Controls state of robots - either AUTO or MANUAL
        if msg.buttons[0] == 1 and self.state != "AUTO":
            self.state = "AUTO"
            
        elif msg.buttons[1] == 1 and self.state != "MANUAL":
            self.state = "MANUAL"

    def nav_vel_callback(self, msg):
        if not self.dead_man_switch and self.state == "AUTO":
            self.cmd_vel_publisher.publish(msg)
        else:
            self.publish_stop_message()

    def joy_vel_callback(self, msg):
        if not self.dead_man_switch and self.state == "MANUAL":
            self.cmd_vel_publisher.publish(msg)
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

def main(args = None):
    rclpy.init(args=args)
    dead_man_switch == DeadManSwitch()
    rlcpy.spin(dead_man_switch)

    dead_man_switch.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
