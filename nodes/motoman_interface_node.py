#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

from motoman_communication.motoman_interface import MotomanInterface
from time import sleep
import subprocess
import threading

def main(args = None):
    rclpy.init(args = args)
    motoman_interface = MotomanInterface()
    
    executor = MultiThreadedExecutor()
    executor.add_node(motoman_interface)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    
    sleep(1)
    motoman_interface.start_trajectory_mode_()
    motoman_interface.move_robot(-1, 0.25)
    motoman_interface.stop_trajectory_mode_()
    
    motoman_interface.destroy_node()
    rclpy.shutdown()

    
if __name__ == "__main__":
    main()