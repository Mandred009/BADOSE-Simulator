# Python Script to show how a ship can be navigated by subscribing and publishing to desired nodes
# A PD control has been used for navigation to the desired position

# !/usr/bin/env python3

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # ROS 2 Node class
from std_msgs.msg import String  # ROS 2 String message type
import re
import time
import math  # Library for mathematical functions

class ShipControl(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Get the ship number from the user
        ship_no = int(input("Enter the Ship No to navigate"))
        
        # Publisher to control the ship
        self.publisher_ = self.create_publisher(String, "control_ship" + str(ship_no), 2)
        
        # Subscriber to get the position of the ship
        self.pos_subscriber = self.create_subscription(String, "ship_pos_ship" + str(ship_no), self.ship_pos_callback, 5)
        
        # Timer to call the timer_callback function periodically
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize message and position data
        self.msg = String()
        self.msg.data = ""
        self.pos_data = []  # Position data: [x, y, z, rx, ry, rz, av, fx, fy, fz]
        
        # Navigation waypoints
        self.ship_square_nav_coords = [[2000, -2000], [0, 0], [2000, 2000]]
        self.pos_mark = 0
        self.pos_thresh = 20  # Position threshold
        
        # Speed and control parameters
        self.left_speed = 0
        self.right_speed = 0
        self.linear_speed = 0
        self.point_loc = 0  # -1: left, +1: right, 0: on the line
        self.turn_kp = 0.5  # Proportional gain for turning
        self.turn_kd = 0.5  # Derivative gain for turning
        self.kp = 0.08  # Proportional gain for linear speed
        self.kd = 0.1  # Derivative gain for linear speed
        self.prev_angle = 0
        self.prev_dist = 0

    def timer_callback(self):
        # Publish the control message periodically
        self.publisher_.publish(self.msg)
        self.get_logger().info('Publishing: "%s"' % self.msg.data)

    def ship_pos_callback(self, msg):
        # Callback function to handle received position data
        self.pos_extractor(msg.data)
        self.nav_start()

    def pos_extractor(self, str_val):
        # Extract position data from the received string
        self.pos_data = []
        str_list = str_val.split(":")
        for i in range(0, len(str_list)):
            if i % 2 != 0:
                self.pos_data.append(float(str_list[i].strip()))
        print(self.pos_data)
    
    def angle_calc(self, ship_x, ship_z, shipf_x, shipf_z, target_x, target_z):
        # Calculate the angle between the ship's direction and the target direction
        v1_x = target_x - ship_x
        v1_y = target_z - ship_z
        mag_v1 = math.sqrt(v1_x**2 + v1_y**2)
        v2_x = shipf_x - ship_x
        v2_y = shipf_z - ship_z
        mag_v2 = math.sqrt(v2_x**2 + v2_y**2)
        v1_dot_v2 = (v1_x * v2_x) + (v1_y * v2_y)
        v1_cross_v2 = (v2_x * v1_y) - (v2_y * v1_x)
        
        # Determine the position of the target relative to the ship's direction
        if v1_cross_v2 < 0:
            self.point_loc = 1
        elif v1_cross_v2 > 0:
            self.point_loc = -1
        else:
            self.point_loc = 0
        
        # Calculate the required angle
        req_theta = math.acos(v1_dot_v2 / (mag_v1 * mag_v2)) * (180 / 3.14)
        return req_theta

    def dist_calc(self, p1_x, p1_y, p2_x, p2_y):
        # Calculate the distance between two points
        return math.sqrt((p1_x - p2_x)**2 + (p1_y - p2_y)**2)
    
    def nav_start(self):
        # Start navigation based on the current and target positions
        angle = self.angle_calc(self.pos_data[0], self.pos_data[2], self.pos_data[7], self.pos_data[9],
                                self.ship_square_nav_coords[self.pos_mark][0], self.ship_square_nav_coords[self.pos_mark][1])
        
        dist = self.dist_calc(self.pos_data[0], self.pos_data[2],
                              self.ship_square_nav_coords[self.pos_mark][0], self.ship_square_nav_coords[self.pos_mark][1])
        
        # Calculate the linear speed using PD control
        self.linear_speed = (self.kp * (dist - 0)) + (self.kd * (dist - self.prev_dist))
        print(self.linear_speed)
        
        # Calculate the left and right speeds based on the angle and linear speed
        if self.point_loc == -1:
            self.right_speed = abs((self.turn_kp * (angle - 0)) + (self.turn_kd * (angle - self.prev_angle))) + self.linear_speed
            self.left_speed = self.linear_speed
        elif self.point_loc == 1:
            self.left_speed = abs((self.turn_kp * (angle - 0)) + (self.turn_kd * (angle - self.prev_angle))) + self.linear_speed
            self.right_speed = self.linear_speed
        
        self.prev_angle = angle
        self.prev_dist = dist

        print(self.right_speed, self.left_speed)

        # Construct the control message
        self.msg.data = "R" + str(self.right_speed) + "*" + "L" + str(self.left_speed)
        self.right_speed = 0
        self.left_speed = 0

        # Check if the current target waypoint has been reached
        if (abs(self.ship_square_nav_coords[self.pos_mark][0] - self.pos_data[0]) < self.pos_thresh) and (abs(self.ship_square_nav_coords[self.pos_mark][1] - self.pos_data[2]) < self.pos_thresh):
            self.pos_mark += 1
            print("Point " + str(self.pos_mark) + " Reached")
            time.sleep(1)
            if self.pos_mark >= len(self.ship_square_nav_coords):
                print("Navigation Done")
                exit()

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create the node and start spinning
    node = ShipControl()
    rclpy.spin(node)

    # Destroy the node and shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
