#!/usr/bin/env python3

from functools import partial
import math
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import TargetTurtleParams
from my_robot_interfaces.srv import KillServerParams

from turtlesim.srv import Spawn
import random

from turtlesim.srv import Kill

class TurtleSpawnerNode(Node):

    def __init__(self):
        super().__init__("node_name")
        self.target_pose_ = [2.0, 9.5, 3.14]
        self.target_turtle_counter_ = 2
        self.name_ = None
        self.spawn_new_turtle()
        self.kill_server_ = self.create_service(KillServerParams, "kill_turtle", self.callback_kill_server)
        self.get_logger().info('Turtle spawner node has been started.')
    
    def callback_kill_server(self, request, response):
        #self.get_logger().info(str(request.name))
        if str(request.name) == self.name_:
            #self.get_logger().info(str(request.name))
            self.call_kill_service(str(request.name))
            response.success = True
            self.get_logger().info('Killing {} : {}'.format(str(request.name), str(response.success)))
            self.get_logger().info('spawning new turtle after killing old one...')
            self.spawn_new_turtle()
        else:
            response.success = False
            self.get_logger().info('Killing {} : {}'.format(str(request.name), str(response.success)))
        

        return response
    
    def call_kill_service(self, name):
        client = self.create_client(Kill, "kill")
        
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server kill...")
        
        request = Kill.Request()
        request.name = name
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill_service))
        

    def callback_kill_service(self, future):
        try:
            response = future.result()
            
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def spawn_new_turtle(self):
        x = random.uniform(0.0, 10.0)
        y = random.uniform(0.0, 10.0)
        theta = random.uniform(0.0, 2*math.pi)
        name = 'turtle'+str(self.target_turtle_counter_)
        self.call_spawn_server(x, y, theta, name)
    
    def call_spawn_server(self, x, y, theta, name):
        client = self.create_client(Spawn, "spawn")
        
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server spawn...")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        future = client.call_async(request)
        self.get_logger().info(str(request.x))
        self.get_logger().info(str(request.y))
        self.get_logger().info(name)
        future.add_done_callback(partial(self.callback_spawn_server, x=x, y=y, theta=theta, name = name))
        #future.add_done_callback(lambda future: self.callback_spawn_server(future, x, y))

        

    def callback_spawn_server(self, future, x, y, theta, name):
        try:
            response = future.result()
            if response.name != "":
                self.name_ = name
                self.get_logger().info('new turtle has been spawned: '+self.name_)
                self.target_pose_ = [x, y, theta]
                self.call_turtle_pose_server(self.target_pose_, self.name_)
                self.target_turtle_counter_ += 1
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
    
    def call_turtle_pose_server(self, target_pose, name):
        client = self.create_client(TargetTurtleParams, "target_pose")
        
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server target_pose...")
        
        request = TargetTurtleParams.Request()
        request.target_pose = self.target_pose_
        request.name = self.name_

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_turtle_pose_server, target_pose=target_pose, name=name))

    def callback_turtle_pose_server(self, future, target_pose, name):
        try:
            response = future.result()
            self.get_logger().info(
                str(target_pose) + ' + ' + str(name))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))



def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()