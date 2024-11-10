import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("pi_control")
        self.target_x = 9.0
        self.target_y = 9.0
        self.pose_ = None
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose",self.callback_turtle_pose, 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
        self.prev_error=0.0
        self.integral=0.0
    def callback_turtle_pose(self,msg):
        self.pose_ = msg
    def control_loop(self):
        if self.pose_ == None:
            return
        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)
        msg = Twist()
        if distance > 0.2:
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            kp = 2.5
            ki = 0.03
            kd = 0.0

            print(diff)
            error = diff
            self.integral += error
            derivative = error - self.prev_error
            self.prev_error = error
            control_signal = kp * error + ki*self.integral + kd*derivative
            msg.linear.x = distance*(abs(3.1-abs(diff))/3.1)
            msg.angular.z = control_signal
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(msg)
    def callback_get_distance(self, request, response):
        x = request.loc_x - self.pose_.x
        y = request.loc_y - self.pose_.y
        response.distance = math.sqrt(x * x + y * y)
        return response
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()