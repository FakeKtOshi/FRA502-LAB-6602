#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
#!/usr/bin/python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')

        # Variables to store positions
        self.pose = None        # My position (Turtle 2)
        self.target_pose = None # Target position (Turtle 1 / Eater)

        # Publisher for Killer's movement
        self.cmd_vel_pul = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Subscriber 1: Where am I?
        self.create_subscription(Pose, '/turtle2/pose', self.pose_callback, 10)

        # Subscriber 2: Where is the target? (NEW)
        # We listen to turtle1 so we can chase it
        self.create_subscription(Pose, '/turtle1/pose', self.target_pose_callback, 10)

        # Timer for the control loop (The Brain)
        self.create_timer(0.01, self.timer_callback)

    def pose_callback(self, data):
        self.pose = [data.x, data.y, data.theta]

    def target_pose_callback(self, data):
        # Update target location whenever turtle1 moves
        self.target_pose = [data.x, data.y]

    def timer_callback(self):
        if self.pose is None or self.target_pose is None:
            return
        
        dx = self.target_pose[0] - self.pose[0]
        dy = self.target_pose[1] - self.pose[1]

        distance = math.hypot(dx, dy)
        
        goal_theta = math.atan2(dy, dx)

        alpha = goal_theta - self.pose[2]

        e_theta = math.atan2(math.sin(alpha), math.cos(alpha))

        Kp_linear = 2.0
        Kp_angular = 6.0

        vx = Kp_linear * distance
        wz = Kp_angular * e_theta

        max_v = 2.5
        max_w = 4.0
        
        vx = max(min(vx, max_v), -max_v)
        wz = max(min(wz, max_w), -max_w)

        if distance < 0.1:
            vx = 0.0
            wz = 0.0

        # 5. Publish
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        self.cmd_vel_pul.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()