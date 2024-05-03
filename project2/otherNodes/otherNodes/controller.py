#!/usr/bin/env python3

import rlcpy
from rlcpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String

class ControllerConnection(Node):
    def __init__(self):
        super().__init__("controller_connection")

        # Creates subscription to Joy node to check controller
        self.subscription = self.create_subscription(
            Joy,
            "joy",
            self.button_press_callback,
            10
        )
        self.state = "INITIAL"
        
    
    def button_press_callback(self, msg):
        # Dead man's switch for robot
        if self.state == "AUTO" and (msg.button[9] != 1 or msg.button[10] != 1):
            # STOP 
            pass

        # Controls state of robots - either AUTO or MANUAL
        if msg.buttons[0] == 1 and self.state != "AUTO":
            self.state = "AUTO"
            
        elif msg.buttons[1] == 1 and self.state != "MANUAL":
            self.state = "MANUAL"

def main(args = None):
    rclpy.init(args=args)
    controller_connection == ControllerConnection()

    controller_connection.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

