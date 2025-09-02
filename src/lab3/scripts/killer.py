#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np, math
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class KillerTurtle(Node):
    def __init__(self):
        super().__init__('killer_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose1_cb, 10) 
        self.create_subscription(Pose, '/turtle2/pose', self.pose2_cb, 10)
        self.create_subscription(Bool, '/turtle1/eating_done', self.done_cb, 10)

        self.spawn_cli = self.create_client(Spawn, '/spawn_turtle')
        self.kill_cli  = self.create_client(Kill,  '/remove_turtle')

        self.robot1_pose = np.array([0.0, 0.0, 0.0])
        self.robot2_pose = np.array([0.0, 0.0, 0.0])
        self.kp_d, self.kp_0 = 1.0, 3.0
        self.hunt = False
        self.spawn_attempted = False  

        self.declare_parameter('sampling_frequency', 100.0)
        rate = self.get_parameter('sampling_frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0/rate, self.timer_cb)

        self.create_timer(0.1, self.try_spawn_once) 

    def pose1_cb(self, msg): self.robot1_pose[:] = [msg.x, msg.y, msg.theta]
    def pose2_cb(self, msg): self.robot2_pose[:] = [msg.x, msg.y, msg.theta]

    def done_cb(self, msg: Bool):
        self.hunt = msg.data
        if not self.hunt:
            self.cmdvel(0.0, 0.0)

    def try_spawn_once(self):
        if self.spawn_attempted:
            return
        if not self.spawn_cli.service_is_ready():
            return
        self.spawn_attempted = True
        self.spawn_turtle(4.0, 3.0, 0.0)

    def spawn_turtle(self, x, y, theta):
        req = Spawn.Request()
        req.name  = 'turtle2'
        req.x     = float(x)
        req.y     = float(y)
        req.theta = float(theta)  

        future = self.spawn_cli.call_async(req)
        future.add_done_callback(self.on_spawn_done)

    def on_spawn_done(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.spawn_attempted = False
            self.create_timer(0.5, self.try_spawn_once)

    def remove_turtle(self):
        if not self.kill_cli.service_is_ready():
            return
        req = Kill.Request()
        req.name = 'turtle1'
        self.kill_cli.call_async(req)

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_vel_pub.publish(msg)

    def calc_d_e(self):
        dx = self.robot1_pose[0]-self.robot2_pose[0]
        dy = self.robot1_pose[1]-self.robot2_pose[1]
        d = math.hypot(dx,dy)
        alpha = math.atan2(dy, dx)
        e = alpha - self.robot2_pose[2]
        e0 = math.atan2(math.sin(e), math.cos(e))
        return d, e0

    def timer_cb(self):
        if not self.hunt or (self.robot2_pose[0] == 0.0 and self.robot2_pose[1] == 0.0 and not self.spawn_attempted):
            return

        d, e0 = self.calc_d_e()
        v, w = self.kp_d * d, self.kp_0 * e0
        self.cmdvel(v, w)
        if d < 0.3:
            self.cmdvel(0.0, 0.0)
            self.remove_turtle()

def main(args=None):
    rclpy.init(args=args)
    node = KillerTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
