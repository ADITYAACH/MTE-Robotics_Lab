#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import transforms3d
import math

class GotoGoalNode(Node):
    def __init__(self):
        super().__init__("move_robot")
        self.target_x = 2.0
        self.target_y = 2.0
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscriber = self.create_subscription(Odometry, "odom", self.control_loop, 10)
        
        # Add PID parameters for better control
        self.linear_kp = 0.3
        self.angular_kp = 0.4
        self.distance_threshold = 0.2
        self.angle_threshold = 0.1
        
        self.get_logger().info(f"Moving to target: ({self.target_x}, {self.target_y})")

    def control_loop(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        dist_x = self.target_x - current_x
        dist_y = self.target_y - current_y
        
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        
        # Get current orientation
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = transforms3d.euler.quat2euler([quat.w, quat.x, quat.y, quat.z])
        
        # Calculate target angle
        goal_theta = math.atan2(dist_y, dist_x)
        
        # Calculate angle difference and normalize to [-pi, pi]
        angle_diff = goal_theta - yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # Create and populate Twist message
        vel = Twist()
        
        # Log current state
        self.get_logger().debug(f"Current position: ({current_x:.2f}, {current_y:.2f})")
        self.get_logger().debug(f"Distance to goal: {distance:.2f}")
        self.get_logger().debug(f"Angle difference: {math.degrees(angle_diff):.2f} degrees")
        
        if distance < self.distance_threshold:
            # We've reached the goal
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.get_logger().info("Goal reached!")
        else:
            if abs(angle_diff) > self.angle_threshold:
                # First rotate to face the goal
                vel.linear.x = 0.0
                vel.angular.z = self.angular_kp * angle_diff
            else:
                # Move towards the goal
                vel.linear.x = self.linear_kp * distance
                vel.angular.z = self.angular_kp * angle_diff  # Keep adjusting angle while moving
        
        # Publish velocity commands
        self.publisher.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = GotoGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()