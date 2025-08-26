#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

import numpy as np
import math as math

from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose

from std_msgs.msg import Int64

from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10) 
        self.create_subscription(Point, '/mouse_position', self.mouse_position_callback, 10)
        self.create_subscription(Int64, '/turtle1/pizza_count', self.count_pizza, 10)
        self.create_timer(0.01, self.timer_callback) #100hz
        
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.mouse_pose = np.array([0.0, 0.0])
        self.count_pizza = np.array([0.0])

        self.Logic = False #Closed
        self.kp_d = 1
        self.kp_0 = 3

        self.spawn_pizza_cilent = self.create_client(GivePosition, 'spawn_pizza')
        self.eat_pizza_cilent = self.create_client(Empty, '/turtle1/eat')

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def pose_callback(self, msg):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta

    def mouse_position_callback(self, msg): #Goal_pose function
        self.mouse_pose[0] = msg.x
        self.mouse_pose[1] = msg.y

        self.Logic = True #Start

        if self.count_pizza[0] < 20:
            self.spawn_pizza(self.mouse_pose[0], self.mouse_pose[1])
            self.count_pizza[0] = self.count_pizza[0] + 1
        # print(self.mouse_pose)

    def spawn_pizza(self, x ,y):
        position_request = GivePosition.Request()
        position_request.x = x
        position_request.y = y
        self.spawn_pizza_cilent.call_async(position_request)

    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_cilent.call_async(eat_request)   

    def count_pizza(self, msg):
        self.count_pizza[0] = msg.data
        print(self.count_pizza[0])
    
    def calculate_distant_pizza(self):
        xr = self.robot_pose[0]
        yr = self.robot_pose[1]
        xm = self.mouse_pose[0]
        ym = self.mouse_pose[1]

        dx = xm - xr
        dy = ym - yr

        d = math.hypot(dx, dy)
        alpha = math.atan2(dy, dx)
        theta = self.robot_pose[2]
        e = alpha - theta
        e0 = math.atan2(math.sin(e), math.cos(e))

        return d, e0
        
    def timer_callback(self):
        self.eat_pizza()
        
        self.cmdvel(0.1, 0.5) #linear and angular velocity
        
        if not self.Logic:
            return
        else:
            d, e0 = self.calculate_distant_pizza()

            vx = self.kp_d * d
            w = self.kp_0 * e0

            self.cmdvel(vx, w)
            if d < 0.3:
                self.cmdvel(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
