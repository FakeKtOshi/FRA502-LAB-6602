#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var # Commented out to prevent import errors
import rclpy
from rclpy.node import Node

# Topics
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from std_msgs.msg import Int64

# Services
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty

# Other
import math
import numpy as np

class EaterNode(Node):
    def __init__(self):
        super().__init__('eater_node')

        # Variables
        self.pizza_count_spawn = 0
        self.pizza_max = 5
        
        # --- FIX 1: Initialize these to prevent "AttributeError" ---
        self.pose = None
        self.mouse_pose = None
        self.pizza_count_all = 0 # Default to 0 until callback updates it
        # -----------------------------------------------------------

        self.current_waypoint = None
        self.next_waypoint = None

        # Topics
        self.cmd_vel_pul = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Point, '/mouse_position', self.mouse_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Int64, '/turtle1/pizza_count', self.pizza_count_callback, 10)

        # Services
        self.spawn_cilent = self.create_client(GivePosition, '/spawn_pizza')
        self.eat_cilent = self.create_client(Empty, '/turtle1/eat')

        # Timers
        self.create_timer(0.01, self.timer_callback)

    # Callbacks and variables
    def pose_callback(self, data):
        self.pose = [data.x, data.y, data.theta]

    def goal_callback(self, data):
        self.goal_pose = [data.x, data.y]

    def pizza_count_callback(self, data):
        self.pizza_count_all = data.data

    # Actions
    def vel_pub(self, v, w):
        data = Twist()
        data.linear.x = float(v)
        data.angular.z = float(w)
        self.cmd_vel_pul.publish(data)

    def eat_pizza(self):
        req = Empty.Request()
        self.eat_cilent.call_async(req)

    def spawn_pizza(self, x, y):
        req = GivePosition.Request()
        req.x = float(x)
        req.y = float(y)
        if self.pizza_count_spawn < self.pizza_max: 
            self.spawn_cilent.call_async(req)
            self.pizza_count_spawn += 1

    def mouse_callback(self, data):
        self.mouse_pose = [data.x, data.y]
        self.spawn_pizza(self.mouse_pose[0], self.mouse_pose[1])
        
        if self.pizza_count_all == self.pizza_max and self.pizza_count_spawn == self.pizza_max:
            self.current_waypoint = self.mouse_pose
            self.get_logger().info('Max pizza reached!')
            
        if self.current_waypoint is None:
            self.current_waypoint = self.mouse_pose
        else:
            self.next_waypoint = self.mouse_pose

    # Timer
    def timer_callback(self):
        # --- FIX 2: Safety Check ---
        # Don't run logic if we haven't seen the robot or clicked the mouse yet
        if self.pose is None or self.mouse_pose is None:
            return
        
        x_turtle = self.pose[0]
        y_turtle = self.pose[1]
        x_mouse = self.mouse_pose[0]
        y_mouse = self.mouse_pose[1]

        d_x = x_mouse - x_turtle
        d_y = y_mouse - y_turtle
        
        alpha = math.atan2(d_y, d_x)
        e = alpha - self.pose[2]

        distance = math.hypot(d_x, d_y)
        e0 = math.atan2(math.sin(e), math.cos(e))   

        linear_gain = 1.0
        angular_gain = 3.0

        max_linear_speed = 1.5
        max_angular_speed = 6.0

        vx = min(max_linear_speed, max(-max_linear_speed, distance * linear_gain))
        wz = min(max_angular_speed, max(-max_angular_speed, e0 * angular_gain))

        # --- FIX 3: Use the correct function ---
        self.vel_pub(vx, wz) # Was self.cmd_vel_pul(vx, wz) which is an error
        
        if distance < 0.3:
            self.vel_pub(0.0, 0.0)
            if self.pizza_count_all != self.pizza_max:
                self.current_waypoint = self.next_waypoint
                self.next_waypoint = None
                if self.pizza_count_all <= self.pizza_max:
                    self.eat_pizza()

def main(args=None):
    rclpy.init(args=args)
    node = EaterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()