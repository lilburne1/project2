import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray
import tkinter as tk
 
class WaypointGUI:
    def __init__(self, parent, node):
        self.parent = parent
        self.node = node
        self.parent.title("Number Input")
        
        self.label1 = tk.Label(parent, text="Waypoint 1")
        self.label1.pack()
        self.number1_entry = tk.Entry(parent)
        self.number1_entry.pack()
        
        self.label2 = tk.Label(parent, text="Waypoint 2:")
        self.label2.pack()
        self.number2_entry = tk.Entry(parent)
        self.number2_entry.pack()
        
        self.label3 = tk.Label(parent, text="Waypoint 3:")
        self.label3.pack()
        self.number3_entry = tk.Entry(parent)
        self.number3_entry.pack()
        
        self.submit_button = tk.Button(parent, text="Submit", command=self.submit_numbers)
        self.submit_button.pack()
 
    def submit_numbers(self):
        numbers = [
            int(self.number1_entry.get()),
            int(self.number2_entry.get()),
            int(self.number3_entry.get())
        ]
        # Publish numbers to ROS2 topic
        numbers_msg = Int32MultiArray(data=numbers)
        self.node.selected_numbers_pub.publish(numbers_msg)
        print("Published selected numbers:", numbers)
        self.node.numbers_received = True
        self.parent.destroy()
 
class TaskBridge(Node):
    def __init__(self):
        super().__init__('task_bridge')
        self.explore_finished_sub = self.create_subscription(
            Bool,
            '/explore_finished',
            self.explore_finished_callback,
            10
        )
        self.selected_numbers_pub = self.create_publisher(
            Int32MultiArray,
            '/selected_numbers',
            10
        )
        self.numbers_received_pub = self.create_publisher(
            Bool,
            '/numbers_received',
            10
        )
        self.numbers_received = False
        self.explore_finished = False
 
        self.numbers_sub = self.create_subscription(
            Int32MultiArray,
            '/selected_numbers',
            self.numbers_callback,
            10
        )
 
    def explore_finished_callback(self, msg):
        if not self.explore_finished and msg.data:
            self.get_logger().info('Explore finished state changed to True, opening number input dialog')
            self.explore_finished = True
            # Launch Tkinter GUI
            root = tk.Tk()
            dialog = NumberInputDialog(root, self)
            root.mainloop()
 
    def numbers_callback(self, msg):
        self.get_logger().info(f'Numbers received: {msg.data}')
        self.numbers_received = True
        numbers_received_msg = Bool(data=True)
        self.numbers_received_pub.publish(numbers_received_msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = TaskBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()