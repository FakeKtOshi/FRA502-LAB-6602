#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

import numpy as np
import math as math

from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')

        self.spawn_turtle_cilent = self.create_client(Spawn, '/spawn_turtle')
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback1, 10) 
        self.create_subscription(Pose, '/turtle2/pose', self.pose_callback2, 10)
        self.remove_turtle_cilent = self.create_client(Kill, '/remove_turtle')

        self.robot1_pose = np.array([0.0, 0.0, 0.0])
        self.robot2_pose = np.array([0.0, 0.0, 0.0])
        self.kp_d = 1
        self.kp_0 = 3

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.create_timer(0.01, self.timer_callback) #100hz

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def spawn_turtle(self, x, y):
        spawn_turtle_request = Spawn.Request()
        spawn_turtle_request.x = x
        spawn_turtle_request.y = y
        # spawn_turtle_request.name = 'turtle2' this make a copy of another when we run
        self.spawn_turtle_cilent.call_async(spawn_turtle_request)

    def remove_turtle(self):
        remove_turtle_cilent = Kill.Request()
        remove_turtle_cilent.name = 'turtle1'
        self.remove_turtle_cilent.call_async(remove_turtle_cilent)

    def pose_callback1(self, msg):
        self.robot1_pose[0] = msg.x
        self.robot1_pose[1] = msg.y
        self.robot1_pose[2] = msg.theta

    def pose_callback2(self, msg):
        self.robot2_pose[0] = msg.x
        self.robot2_pose[1] = msg.y
        self.robot2_pose[2] = msg.theta

    def calculate_distant_turtle(self):
        x1 = self.robot2_pose[0]
        y1 = self.robot2_pose[1]
        x2 = self.robot1_pose[0]
        y2 = self.robot1_pose[1]

        dx = x2 - x1
        dy = y2 - y1

        d = math.hypot(dx, dy)
        alpha = math.atan2(dy, dx)
        theta = self.robot2_pose[2]
        e = alpha - theta
        e0 = math.atan2(math.sin(e), math.cos(e))

        return d, e0
    
    def timer_callback(self):
        # self.cmdvel(0.1, 0.1) #linear and angular velocity

        d, e0 = self.calculate_distant_turtle()

        vx = self.kp_d * d
        w = self.kp_0 * e0

        self.cmdvel(vx, w)
        if d < 0.3:
            self.cmdvel(0.0, 0.0)
            self.remove_turtle()

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    node.spawn_turtle(0.0, 0.0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
