import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionClient

from copy import deepcopy

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from industrial_msgs.msg import RobotStatus
from motoros2_interfaces.srv import StartTrajMode
from std_srvs.srv import Trigger

class MotomanInterface(Node):
    def __init__(self):
        super().__init__('competition_interface')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.joint_state_subscriber_ = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb_,
            qos_profile=qos_profile)

        self.robot_status_subscriber_ = self.create_subscription(
            RobotStatus,
            '/robot_status',
            self.robot_status_cb_,
            qos_profile=qos_profile)
        
        self.start_traj_mode_client_ = self.create_client(
            StartTrajMode,
            '/start_traj_mode'
        )
        
        self.stop_traj_mode_client_ = self.create_client(
            Trigger,
            '/stop_traj_mode'
        )
        
        self.most_recent_joint_state: JointState = None
                
        self._action_client = ActionClient(self, FollowJointTrajectory, "/follow_joint_trajectory")

    def joint_state_cb_(self, msg: JointState):
        """Saves the most recent joint state

        Args:
            msg (JointState): joint state message received from the /joint_states topic
        """
        self.most_recent_joint_state = msg
    
    def robot_status_cb_(self, msg: RobotStatus):
        """Prints an error message if there is an error

        Args:
            msg (RobotStatus): Robot status message received from the /robot_status topic
        """
        if msg.error_codes != 0 and msg.in_error.val == 1:
            self.get_logger().info(f"Robot status error code: {msg.error_codes}")
            
        
    def start_trajectory_mode_(self):
        """Starts the Motoros2 trajectory mode
        """
        request = StartTrajMode.Request()

        future = self.start_traj_mode_client_.call_async(request)

        while not future.done():
            pass
        
        response: StartTrajMode.Response = future.result()
        
        self.get_logger().info(f"Start traj mode result code: {response.result_code}")

    def stop_trajectory_mode_(self):
        """Stops the trajectory mode
        """
        request = Trigger.Request()

        future = self.stop_traj_mode_client_.call_async(request)

        while not future.done():
            pass
    
    def move_robot(self, joint_index: int, degree: float):
        """Moves the robot. The joint to move is selected through joint index.

        Args:
            joint_index (int): Index of which joint will be moved
            degree (float): Degree that the joint will be moved by
        """
        if self.most_recent_joint_state == None:
            print("No joint states received yet")
            return
                
        new_joint_state = deepcopy(self.most_recent_joint_state)
        
        new_joint_state.position[joint_index] += degree
                
        self._action_client.wait_for_server()
        
        new_follow_joint_trajectory = FollowJointTrajectory.Goal()
        new_follow_joint_trajectory.trajectory = self.generate_joint_trajectory(new_joint_state)
        
        self.get_logger().info(f"Goal:\n\n{new_follow_joint_trajectory}\n\n")
        
        future = self._action_client.send_goal_async(new_follow_joint_trajectory)
        
        while not future.done():
            pass
        
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code != 0:
            self.get_logger().error(f'Error code: {result.error_string}')
        else:
            self.get_logger().info('Movement successful!')
        
    def generate_joint_trajectory(self, joint_state: JointState):
        """Generates a joint trajectory from the joint state parameter

        Args:
            joint_state (JointState): Target joint state

        Returns:
            JointTrajectory: Joint trajectory from the input joint state
        """
        new_joint_trajectory = JointTrajectory()
        new_joint_trajectory.joint_names = joint_state.name
        
        original_point = JointTrajectoryPoint()
        original_point.positions = self.most_recent_joint_state.position
        original_point.velocities = [0.0] * 7
        original_point.time_from_start = rclpy.duration.Duration(seconds=0).to_msg()
        new_joint_trajectory.points.append(original_point)
        
        new_point = JointTrajectoryPoint()
        new_point.positions = joint_state.position
        new_point.velocities = [0.0] * 7
        new_point.time_from_start = rclpy.duration.Duration(seconds=20).to_msg()
        new_joint_trajectory.points.append(new_point)
        
        return new_joint_trajectory