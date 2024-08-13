#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

from trajectory_msgs.msg import JointTrajectoryPoint

from rclpy.duration import Duration
from copy import deepcopy

from motoman_communication.motoman_interface import MotomanInterface

from control_msgs.action import FollowJointTrajectory

from motoros2_interfaces.srv import QueueTrajPoint, StartTrajMode
from motoros2_interfaces.msg import MotionReadyEnum
from time import sleep
import subprocess
import threading

def main(args = None):
    rclpy.init(args = args)
    motoman_interface = MotomanInterface()
    
    # executor = MultiThreadedExecutor()
    # executor.add_node(motoman_interface)
    # spin_thread = threading.Thread(target=executor.spin)
    # spin_thread.start()
    
    motoman_interface.get_logger().info("Starting up...")
    
    start = motoman_interface.get_clock().now()
    while(motoman_interface.get_clock().now() - start < Duration(seconds=3)):
        rclpy.spin_once(motoman_interface)
        sleep(0.1)
    
    request = StartTrajMode.Request()

    future = motoman_interface.start_traj_mode_client_.call_async(request)

    rclpy.spin_until_future_complete(motoman_interface, future, timeout_sec=5)
    
    start_traj_response: StartTrajMode.Response = future.result()
    
    motoman_interface.get_logger().info(f"Start traj mode result code: {start_traj_response.result_code}")  
    
    motoman_interface.get_logger().info("Waiting after starting trajectory mode...")
    
    start = motoman_interface.get_clock().now()
    while(motoman_interface.get_clock().now() - start < Duration(seconds=3)):
        rclpy.spin_once(motoman_interface)
        sleep(0.1)

    goal = FollowJointTrajectory.Goal()
    
    goal.trajectory.joint_names = motoman_interface.most_recent_joint_state.name
    
    start = JointTrajectoryPoint()
    start.positions = motoman_interface.most_recent_joint_state.position
    start.velocities = [0.0] * 7
    start.time_from_start.sec = 0
    
    new_point = deepcopy(start)
    new_point.positions[0] -= 0.1
    # new_point.positions[5] += 0.1
    new_point.time_from_start.sec = 3

    goal.trajectory.points.append(start)
    goal.trajectory.points.append(new_point)
    
    motoman_interface.get_logger().info("Waiting for server")
    
    motoman_interface._action_client.wait_for_server()
    
    motoman_interface.get_logger().info("Sending Goal")
    
    try:
        future = motoman_interface._action_client.send_goal_async(goal)
        
        future.add_done_callback(motoman_interface.goal_response_callback)
        
        while not motoman_interface.goal_finished:
            rclpy.spin_once(motoman_interface)
            sleep(0.1)
        
    except KeyboardInterrupt:
        pass
    
if __name__ == "__main__":
    main()