# Python Script to create an example camera subscription and display it

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # ROS 2 Node class
from std_msgs.msg import String  # ROS 2 String message type
import cv2  # OpenCV library for image processing
import numpy as np  # Numpy library for numerical operations
import base64  # Library for encoding and decoding base64 strings

class CamFeedSubscriber(Node):

    def __init__(self):
        super().__init__('cam_feed_subscriber')
        
        # Get the drone number from the user
        self.drone_no = input("Drone No For Cam Feed: ")
        
        # Create a subscription to the camera feed topic
        self.subscription = self.create_subscription(
            String,
            'cam_feed_datadrone' + self.drone_no,  # Topic name
            self.listener_callback,  # Callback function
            10)  # Queue size
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Callback function to handle received camera feed data
        try:
            # Decode the base64 string to image bytes
            img_bytes = base64.b64decode(msg.data)
            
            # Convert image bytes to numpy array
            np_arr = np.frombuffer(img_bytes, np.uint8)
            
            # Decode the numpy array to OpenCV image
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img = cv2.resize(img, (256, 256))

            # Display the image
            cv2.imshow('Camera Feed Drone ' + self.drone_no, img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error decoding image: {e}")

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create the camera feed subscriber node and start spinning
    cam_feed_subscriber = CamFeedSubscriber()
    rclpy.spin(cam_feed_subscriber)
    
    # Destroy the node and shutdown ROS 2
    cam_feed_subscriber.destroy_node()
    rclpy.shutdown()
    
    # Close all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
