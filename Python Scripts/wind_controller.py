# Python Script to control wind settings by subscribing and publishing to a specific topic

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # ROS 2 Node class
from std_msgs.msg import String  # ROS 2 String message type
import time  # Library for time-related functions

class WindControl(Node):

    def __init__(self):
        super().__init__('wind_node')  # Initialize the node with the name 'wind_node'
        
        # Create a publisher to the 'wind_control' topic
        self.publisher_ = self.create_publisher(String, "wind_control", 1)
        
        # Initialize the message to be published
        self.msg = String()
        
        # Create a timer that triggers the timer_callback function every 0.3 seconds
        self.timer = self.create_timer(0.3, self.timer_callback)

    def timer_callback(self):
        # Callback function to handle timer events
        self.msg.data = input("Enter wind details: ")  # Get wind details from user input
        self.publisher_.publish(self.msg)  # Publish the message
        self.get_logger().info(f'Publishing: "{self.msg.data}"')  # Log the published message

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create the WindControl node and start spinning
    node = WindControl()
    rclpy.spin(node)
    
    # Destroy the node and shutdown ROS 2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
