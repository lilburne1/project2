#!/usr/bin/env python3

import rlcpy
from rlcpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ControllerConnection(Node):
    def __init__(self):
        super().__init__("master_node")

        # Creates subscription to Joy node to check controller
        self.joy_subscription = self.create_subscription(
            Joy,
            "joy",
            10
        )
        self.drive_state = "INITIAL"

        # Creates subscription to cmd_vel_joy 
        self.joy_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_joy",
            10
        )

        # Creates subscription to cmd_vel_nav
        self.nav_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_nav",
            self.velocity_change_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", s10)

        
    def button_press_callback(self, msg):
        # Dead man's switch for robot
        if self.state == "AUTO" and (msg.button[9] != 1 or msg.button[10] != 1):
             self.state == "STOP"

        # Controls state of robots - either AUTO or MANUAL
        if msg.buttons[0] == 1 and self.state != "AUTO":
            self.state = "AUTO"
            
        elif msg.buttons[1] == 1 and self.state != "MANUAL":
            self.state = "MANUAL"


def main(args = None):
    rclpy.init(args=args)
    controller_connection == ControllerConnection()
    rlcpy.spin(controller_connection)

    controller_connection.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
