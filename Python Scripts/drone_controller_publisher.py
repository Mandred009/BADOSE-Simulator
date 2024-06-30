# Python Script to show how a drone can be navigated by subscribing and publishing to desired nodes

#!/usr/bin/env python3

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # ROS 2 Node class
from std_msgs.msg import String  # ROS 2 String message type
import time  # Library for time-related functions

class DroneControl(Node):

    def __init__(self):
        super().__init__('my_node_drone')
        
        # Get the drone number from the user
        drone_no = int(input("Enter the Drone No to navigate: "))
        
        # Publisher to control the drone
        self.publisher_ = self.create_publisher(String, f"control_drone{drone_no}", 1)
        
        # Subscriber to get the position of the drone
        self.pos_subscriber = self.create_subscription(String, f"drone_pos_drone{drone_no}", self.drone_pos_callback, 4)
        
        # Initialize message and position data
        self.msg = String()
        self.pos_data = []  # Position data: [x, y, z, rx, ry, rz, vx, vy, vz]
        self.prev_pos = []  # Previous position data
        self.init_alt = 0  # Initial altitude
        self.ht_points = [100, 200, 300, 100]  # Altitude points for navigation
        self.ht_idx = 0  # Index for altitude points
        self.drone_mass = 10  # Mass of the drone in kg
        self.balance_thrust = (self.drone_mass / 4) * 9.81  # Thrust to balance the drone
        self.kp = 0.7  # Proportional gain for altitude control
        self.kd = 0.8  # Derivative gain for altitude control
        self.ht_thresh = 0.2  # Threshold for height comparison
        # Uncomment the following line to use a periodic timer for publishing messages
        # self.timer = self.create_timer(0.3, self.timer_callback)

    def timer_callback(self):
        # Publish the control message periodically
        self.publisher_.publish(self.msg)
        self.get_logger().info(f'Publishing: "{self.msg.data}"')

    def drone_pos_callback(self, msg):
        # Callback function to handle received position data
        self.pos_extractor(msg.data)
        self.start_nav()

    def pos_extractor(self, str_val):
        # Extract position data from the received string
        if not self.pos_data:
            # Initialize position data and initial altitude if pos_data is empty
            self.pos_data = [float(val.strip()) for val in str_val.split(":")[1::2]]
            self.init_alt = self.pos_data[1]
            self.prev_pos = self.pos_data
        self.pos_data = [float(val.strip()) for val in str_val.split(":")[1::2]]
        # Convert angles to a consistent range
        self.pos_data[3], self.pos_data[4], self.pos_data[5] = (
            self.angle_convert(self.pos_data[3]), 
            self.angle_convert(self.pos_data[4]), 
            self.angle_convert(self.pos_data[5])
        )
        self.get_logger().info(f'Position Data: {self.pos_data}')

    def start_nav(self):
        # Start navigation based on the current and target altitudes
        target = self.init_alt + self.ht_points[self.ht_idx]
        print(target)
        if abs(target - self.pos_data[1]) > self.ht_thresh:
            # Calculate the thrust required to reach the target altitude
            alt_thrust = self.altitude_val(target)
            self.prev_pos = self.pos_data
            thrust = [self.balance_thrust + alt_thrust] * 4
            self.msg.data = "*".join(map(str, thrust))
            self.timer_callback()
        else:
            # Move to the next height point if the current target is reached
            self.ht_idx += 1
            if self.ht_idx >= len(self.ht_points):
                self.get_logger().info("Height Navigation Done")
                exit()
            self.get_logger().info(f'Height Point {self.ht_idx} Reached')
            time.sleep(1)

    def altitude_val(self, target):
        # Calculate the altitude control value using PD control
        return (((target - self.pos_data[1]) * self.kp) + ((self.pos_data[1] - self.prev_pos[1]) * self.kd)) / 4

    def angle_convert(self, angle):
        # Convert angles to a range of -180 to 180 degrees
        return -1 * (360 - angle) if angle > 180 else angle

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create the node and start spinning
    node = DroneControl()
    rclpy.spin(node)

    # Destroy the node and shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
