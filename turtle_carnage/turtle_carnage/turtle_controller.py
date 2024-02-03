#!/usr/bin/env python3

from functools import partial
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from my_robot_interfaces.srv import TargetTurtleParams
from my_robot_interfaces.srv import KillServerParams

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.target_turtle_pose_ = []
        self.target_turtle_name_ = None
        self.turtle1_pose_ = None

        self.target_pose_server_ = self.create_service(TargetTurtleParams, "target_pose", self.callback_turtle_pose_server)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.turtle1_pose_sub_ = self.create_subscription(Pose, "turtle1/pose", self.callback_turtle1_pose_sub, 10)

        self.control_loop_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Turtle Controller Node has been started.")

    
    def callback_turtle_pose_server(self, request, response):
        self.target_turtle_pose_ = request.target_pose
        self.target_turtle_name_ = request.name
        
        response.success = True

        self.get_logger().info(str(self.target_turtle_pose_) + ' and ' + str(self.target_turtle_name_))

        return response

    
    def callback_turtle1_pose_sub(self, msg):
        self.turtle1_pose_ = msg

    def control_loop(self):

        if self.turtle1_pose_ == None or self.target_turtle_name_ == None:
            return
        self.get_logger().info(str(self.turtle1_pose_.x) + " and " + str(self.turtle1_pose_.y) + ' and ' + str(self.target_turtle_pose_[0]))
        diff_x = self.target_turtle_pose_[0] - self.turtle1_pose_.x
        diff_y = self.target_turtle_pose_[1] - self.turtle1_pose_.y

        distance = math.sqrt((diff_x * diff_x) + (diff_y * diff_y))

        self.get_logger().info(str(distance) + " and " + str(diff_x) + ' and ' + str(diff_y))

        msg  = Twist()

        if distance > 0.25:
            msg.linear.x = 2*distance
            goal_theta = math.atan2(diff_y, diff_x)
            diff = goal_theta - self.turtle1_pose_.theta
            if diff>math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            msg.angular.z = 6*diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_kill_server(self.target_turtle_name_)
            self.target_turtle_name_ == None

        self.cmd_vel_publisher_.publish(msg)

    def call_kill_server(self, name):
        client = self.create_client(KillServerParams, "kill_turtle")
        
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server kill_turtle...")
        
        request = KillServerParams.Request()
        request.name = name
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill_server))
        

    def callback_kill_server(self, future):
        try:
            response = future.result()
            
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))



def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()