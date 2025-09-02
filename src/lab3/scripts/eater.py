#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np, math
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import Int64, Float64, String, Bool
from controller_interfaces.srv import SetMaxPizza, SetParam

class EaterTurtle(Node):
    def __init__(self):
        super().__init__('eater_node')

        self.kp_d = 1.0
        self.kp_0 = 3.0
        self.maxpizza = 5

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, '/turtle1/eating_done', 10)

        self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10) 
        self.create_subscription(Point, '/mouse_position', self.mouse_cb, 10)
        self.create_subscription(Int64, '/turtle1/pizza_count', self.count_cb, 10)

        self.spawn_cli = self.create_client(GivePosition, '/spawn_pizza')
        self.eat_cli   = self.create_client(Empty, '/turtle1/eat')

        self.create_service(SetParam, 'turtle1/set_param', self.set_param_cb)
        self.create_service(SetMaxPizza, 'max_pizza', self.maxpizza_cb)

        self.declare_parameter('sampling_frequency', 100.0)
        rate = self.get_parameter('sampling_frequency').get_parameter_value().double_value 
        self.timer = self.create_timer(1.0/rate, self.timer_cb)

        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.mouse_pose = np.array([0.0, 0.0])
        self.pizza_count = 0
        self.active = False

    def pose_cb(self, msg):
        self.robot_pose[:] = [msg.x, msg.y, msg.theta]

    def mouse_cb(self, msg):
        self.mouse_pose[:] = [msg.x, msg.y]
        self.active = True
        if self.pizza_count < self.maxpizza:
            self.spawn_pizza(msg.x, msg.y)

    def count_cb(self, msg):
        self.pizza_count = msg.data

    def set_param_cb(self, req, res):
        self.kp_d = req.kp_linear.data
        self.kp_0 = req.kp_angular.data
        return res

    def maxpizza_cb(self, req, res):
        self.maxpizza = int(req.max_pizza.data)
        self.done_pub.publish(Bool(data=False))
        self.get_logger().info(f'max_pizza updated to {self.maxpizza}')
        return res

    def spawn_pizza(self, x, y):
        req = GivePosition.Request()
        req.x, req.y = float(x), float(y)
        self.spawn_cli.call_async(req)

    def eat_pizza(self):
        self.eat_cli.call_async(Empty.Request())

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_vel_pub.publish(msg)

    def calc_d_e(self):
        dx, dy = self.mouse_pose[0]-self.robot_pose[0], self.mouse_pose[1]-self.robot_pose[1]
        d = math.hypot(dx,dy)
        alpha = math.atan2(dy, dx)
        e = alpha - self.robot_pose[2]
        e0 = math.atan2(math.sin(e), math.cos(e))
        return d, e0

    def timer_cb(self):
        self.eat_pizza()
        if self.pizza_count >= self.maxpizza:
            self.done_pub.publish(Bool(data=True))
        else:
            self.done_pub.publish(Bool(data=False))

        if not self.active:
            return
        d, e0 = self.calc_d_e()
        v, w = self.kp_d * d, self.kp_0 * e0
        self.cmdvel(v, w)
        if d < 0.3:
            self.cmdvel(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = EaterTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
