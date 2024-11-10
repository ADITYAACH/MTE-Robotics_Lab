# red_ball_tracker.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedBallTracker(Node):

    def __init__(self):
        super().__init__('red_ball_tracker')
        self.declare_parameter('image_topic', '/camera1/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.get_logger().info('Red Ball Tracker Node Initialized')

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Define the range of red color in HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)

        lower_red = np.array([160, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red, upper_red)

        mask = mask1 + mask2

        # Find contours of the red ball
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
                self.get_logger().info(f'Red Ball Detected at ({cX}, {cY})')

                # Draw the center of the ball
                cv2.circle(cv_image, (cX, cY), 10, (0, 255, 0), -1)
                cv2.imshow('Red Ball Tracker', cv_image)
                cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RedBallTracker()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
