#!/usr/bin/env python3

import threading
import rclpy
from motoman_communication.moveit_py_interface import CompetitionInterface
from rclpy.executors import MultiThreadedExecutor
from time import sleep

def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    
    while True:
        interface.print_current_pose()
    
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()