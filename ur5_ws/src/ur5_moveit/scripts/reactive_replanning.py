#!/usr/bin/env python3
"""
Reactive Replanning Pick and Place using Kinematic Redundancy
ME5250 Final Project - Abdul Rahman

This node demonstrates Strategy 1: Reactive replanning by exploiting
kinematic redundancy of the UR5 manipulator. When obstacles appear,
the system computes multiple IK solutions for the same goal pose
and evaluates alternative configurations to find collision-free paths.
"""
# print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.msg import (
    CollisionObject, 
    PlanningScene,
    RobotState,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes
)
from moveit_msgs.srv import GetPositionIK, GetPlanningScene, GetMotionPlan
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from geometry_msgs.msg import Pose, PoseStamped, Point
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import numpy as np
import time
import threading
from typing import List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum


class RobotState(Enum):
    IDLE = 0
    MOVING_TO_PICK = 1
    PICKING = 2
    MOVING_TO_PLACE = 3
    PLACING = 4
    REPLANNING = 5


@dataclass
class IKSolution:
    """Stores an IK solution with metadata"""
    joint_positions: List[float]
    manipulability: float
    is_valid: bool


class ReactiveReplanner(Node):
    def __init__(self):
        super().__init__('reactive_replanner')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        self.current_joint_state = None
        self.robot_state = RobotState.IDLE
        self.obstacle_detected = False
        self.current_trajectory = None
        
        self.pick_poses = [
            self._create_pose(0.45, 0.1, 0.38, 0.0, 1.0, 0.0, 0.0),
            self._create_pose(0.5, 0.0, 0.38, 0.0, 1.0, 0.0, 0.0),
            self._create_pose(0.55, -0.1, 0.38, 0.0, 1.0, 0.0, 0.0),
        ]
        
        self.place_pose = self._create_pose(0.3, -0.5, 0.58, 0.0, 1.0, 0.0, 0.0)
        
        self.home_joints = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.collision_object_sub = self.create_subscription(
            CollisionObject,
            '/collision_object',
            self._collision_object_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.collision_object_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )
        
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self.callback_group
        )
        
        self.planning_scene_client = self.create_client(
            GetPlanningScene,
            '/get_planning_scene',
            callback_group=self.callback_group
        )
        
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            'move_action',
            callback_group=self.callback_group
        )
        
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            'execute_trajectory',
            callback_group=self.callback_group
        )
        
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Reactive Replanner initialized')
        self.get_logger().info('Waiting for services and action servers...')
        
        self._wait_for_services()
        
        self.get_logger().info('All services ready!')
        self.get_logger().info('='*50)
        self.get_logger().info('ME5250 Reactive Replanning Demo')
        self.get_logger().info('Abdul Rahman - Northeastern University')
        self.get_logger().info('='*50)
    
    def _create_pose(self, x, y, z, qx, qy, qz, qw) -> Pose:
        """Create a Pose message"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose
    
    def _wait_for_services(self):
        """Wait for all required services to be available"""
        self.get_logger().info('Waiting for /compute_ik service...')
        self.ik_client.wait_for_service()
        
        self.get_logger().info('Waiting for /get_planning_scene service...')
        self.planning_scene_client.wait_for_service()
        
        self.get_logger().info('Waiting for move_action server...')
        self.move_group_client.wait_for_server()
        
        self.get_logger().info('Waiting for execute_trajectory server...')
        self.execute_trajectory_client.wait_for_server()
    
    def _joint_state_callback(self, msg: JointState):
        """Store current joint state"""
        self.current_joint_state = msg
    
    def _collision_object_callback(self, msg: CollisionObject):
        """Detect when obstacles are added to the scene"""
        if msg.operation == CollisionObject.ADD:
            if 'obstacle' in msg.id.lower() or 'red' in msg.id.lower():
                self.get_logger().warn(f'OBSTACLE DETECTED: {msg.id}')
                self.obstacle_detected = True
    
    def compute_multiple_ik(self, target_pose: Pose, num_attempts: int = 20) -> List[IKSolution]:
        """
        Compute multiple IK solutions for the same target pose
        by using different seed states (exploiting kinematic redundancy)
        """
        solutions = []
        seen_configs = set()
        
        for i in range(num_attempts):
            seed_state = self._generate_random_seed()
            
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'ur5_arm'
            request.ik_request.robot_state.joint_state.name = self.joint_names
            request.ik_request.robot_state.joint_state.position = seed_state
            request.ik_request.pose_stamped.header.frame_id = 'world'
            request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            request.ik_request.pose_stamped.pose = target_pose
            request.ik_request.timeout.sec = 0
            request.ik_request.timeout.nanosec = 50000000
            request.ik_request.avoid_collisions = True
            
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is not None:
                result = future.result()
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    joint_positions = list(result.solution.joint_state.position)[:6]
                    
                    config_key = tuple(round(p, 2) for p in joint_positions)
                    if config_key not in seen_configs:
                        seen_configs.add(config_key)
                        
                        manipulability = self._compute_manipulability(joint_positions)
                        
                        solutions.append(IKSolution(
                            joint_positions=joint_positions,
                            manipulability=manipulability,
                            is_valid=True
                        ))
        
        solutions.sort(key=lambda s: s.manipulability, reverse=True)
        
        self.get_logger().info(f'Found {len(solutions)} unique IK solutions')
        return solutions
    
    def _generate_random_seed(self) -> List[float]:
        """Generate random joint configuration as seed for IK"""
        limits = [
            (-6.28, 6.28),
            (-6.28, 6.28),
            (-3.14, 3.14),
            (-6.28, 6.28),
            (-6.28, 6.28),
            (-6.28, 6.28),
        ]
        return [np.random.uniform(low, high) for low, high in limits]
    
    def _compute_manipulability(self, joint_positions: List[float]) -> float:
        """
        Compute manipulability index for a given configuration
        Using simplified Yoshikawa manipulability measure
        """
        a2 = 0.425
        a3 = 0.39225
        d4 = 0.10915
        
        q2 = joint_positions[1]
        q3 = joint_positions[2]
        
        s2 = np.sin(q2)
        c2 = np.cos(q2)
        s23 = np.sin(q2 + q3)
        c23 = np.cos(q2 + q3)
        
        det_approx = abs(a2 * a3 * np.sin(q3))
        
        distance_from_limits = min(
            abs(q + 3.14) + abs(q - 3.14) for q in joint_positions
        )
        
        return det_approx * (1 + 0.1 * distance_from_limits)
    
    def plan_to_joint_target(self, joint_target: List[float]) -> Optional[JointTrajectory]:
        """Plan a trajectory to joint target using MoveGroup"""
        goal = MoveGroup.Goal()
        goal.request.group_name = 'ur5_arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        
        goal.request.goal_constraints.append(Constraints())
        for i, (name, position) in enumerate(zip(self.joint_names, joint_target)):
            from moveit_msgs.msg import JointConstraint
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal.request.goal_constraints[0].joint_constraints.append(jc)
        
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return None
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        result = result_future.result()
        if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            return result.result.planned_trajectory.joint_trajectory
        else:
            self.get_logger().error(f'Planning failed: {result.result.error_code.val}')
            return None
    
    def execute_trajectory_with_monitoring(self, trajectory: JointTrajectory) -> bool:
        """
        Execute trajectory while monitoring for obstacles
        Returns True if completed successfully, False if interrupted
        """
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = trajectory
        
        self.obstacle_detected = False
        self.current_trajectory = trajectory
        
        future = self.execute_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory execution rejected')
            return False
        
        self.get_logger().info('Trajectory execution started...')
        
        result_future = goal_handle.get_result_async()
        
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.obstacle_detected:
                self.get_logger().warn('OBSTACLE DETECTED - Canceling trajectory!')
                goal_handle.cancel_goal_async()
                time.sleep(0.5)
                return False
        
        result = result_future.result()
        return result.result.error_code.val == MoveItErrorCodes.SUCCESS
    
    def replan_with_redundancy(self, target_pose: Pose) -> bool:
        """
        Replan to target using kinematic redundancy
        Computes multiple IK solutions and tries each until finding collision-free path
        """
        self.robot_state = RobotState.REPLANNING
        self.get_logger().info('='*50)
        self.get_logger().info('REPLANNING using kinematic redundancy...')
        self.get_logger().info('='*50)
        
        ik_solutions = self.compute_multiple_ik(target_pose, num_attempts=25)
        
        if not ik_solutions:
            self.get_logger().error('No IK solutions found!')
            return False
        
        self.get_logger().info(f'Testing {len(ik_solutions)} configurations...')
        
        for idx, solution in enumerate(ik_solutions):
            self.get_logger().info(f'Trying configuration {idx + 1}/{len(ik_solutions)} '
                                   f'(manipulability: {solution.manipulability:.4f})')
            
            trajectory = self.plan_to_joint_target(solution.joint_positions)
            
            if trajectory is not None:
                self.get_logger().info(f'Configuration {idx + 1} found collision-free path!')
                
                self.obstacle_detected = False
                success = self.execute_trajectory_with_monitoring(trajectory)
                
                if success:
                    self.get_logger().info('Replanned trajectory executed successfully!')
                    return True
                else:
                    self.get_logger().warn(f'Configuration {idx + 1} execution failed, trying next...')
            else:
                self.get_logger().info(f'Configuration {idx + 1} blocked, trying next...')
        
        self.get_logger().error('All configurations blocked!')
        return False
    
    def control_gripper(self, position: float) -> bool:
        """Control gripper: 0.0 = closed, 0.8 = open"""
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Gripper action server not available')
            return False
        
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        return True
    
    def open_gripper(self):
        """Open the gripper"""
        self.get_logger().info('Opening gripper...')
        self.control_gripper(0.8)
        time.sleep(1.0)
    
    def close_gripper(self):
        """Close the gripper"""
        self.get_logger().info('Closing gripper...')
        self.control_gripper(0.0)
        time.sleep(1.0)
    
    def move_to_home(self) -> bool:
        """Move robot to home position"""
        self.get_logger().info('Moving to home position...')
        trajectory = self.plan_to_joint_target(self.home_joints)
        if trajectory:
            return self.execute_trajectory_with_monitoring(trajectory)
        return False
    
    def pick_object(self, pick_pose: Pose) -> bool:
        """Execute pick operation"""
        self.robot_state = RobotState.MOVING_TO_PICK
        self.get_logger().info('Moving to pick position...')
        
        approach_pose = Pose()
        approach_pose.position.x = pick_pose.position.x
        approach_pose.position.y = pick_pose.position.y
        approach_pose.position.z = pick_pose.position.z + 0.1
        approach_pose.orientation = pick_pose.orientation
        
        ik_solutions = self.compute_multiple_ik(approach_pose)
        if not ik_solutions:
            return False
        
        trajectory = self.plan_to_joint_target(ik_solutions[0].joint_positions)
        if not trajectory:
            return False
        
        if not self.execute_trajectory_with_monitoring(trajectory):
            if self.obstacle_detected:
                return self.replan_with_redundancy(approach_pose)
            return False
        
        self.robot_state = RobotState.PICKING
        self.open_gripper()
        
        ik_solutions = self.compute_multiple_ik(pick_pose)
        if ik_solutions:
            trajectory = self.plan_to_joint_target(ik_solutions[0].joint_positions)
            if trajectory:
                self.execute_trajectory_with_monitoring(trajectory)
        
        self.close_gripper()
        
        if ik_solutions:
            trajectory = self.plan_to_joint_target(ik_solutions[0].joint_positions)
        
        return True
    
    def place_object(self, place_pose: Pose) -> bool:
        """Execute place operation with reactive replanning"""
        self.robot_state = RobotState.MOVING_TO_PLACE
        self.get_logger().info('Moving to place position...')
        self.get_logger().info('>>> Insert obstacle now using: ros2 run ur5_moveit insert_obstacle --x 0.35 --y -0.25 --z 0.5')
        
        approach_pose = Pose()
        approach_pose.position.x = place_pose.position.x
        approach_pose.position.y = place_pose.position.y
        approach_pose.position.z = place_pose.position.z + 0.1
        approach_pose.orientation = place_pose.orientation
        
        ik_solutions = self.compute_multiple_ik(approach_pose)
        if not ik_solutions:
            self.get_logger().error('No IK solutions for place approach')
            return False
        
        self.get_logger().info(f'Found {len(ik_solutions)} IK solutions for place')
        
        trajectory = self.plan_to_joint_target(ik_solutions[0].joint_positions)
        if not trajectory:
            return False
        
        success = self.execute_trajectory_with_monitoring(trajectory)
        
        if not success and self.obstacle_detected:
            self.get_logger().warn('Obstacle detected during place motion!')
            success = self.replan_with_redundancy(approach_pose)
            if not success:
                return False
        
        self.robot_state = RobotState.PLACING
        
        ik_solutions = self.compute_multiple_ik(place_pose)
        if ik_solutions:
            trajectory = self.plan_to_joint_target(ik_solutions[0].joint_positions)
            if trajectory:
                self.execute_trajectory_with_monitoring(trajectory)
        
        self.open_gripper()
        
        return True
    
    def run_demo(self):
        """Run the full pick and place demo"""
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('STARTING REACTIVE REPLANNING PICK AND PLACE DEMO')
        self.get_logger().info('='*60)
        self.get_logger().info('')
        
        time.sleep(2.0)
        
        self.move_to_home()
        self.open_gripper()
        
        for i, pick_pose in enumerate(self.pick_poses):
            self.get_logger().info(f'\n--- Picking object {i + 1} of {len(self.pick_poses)} ---')
            
            if self.pick_object(pick_pose):
                self.get_logger().info(f'Pick {i + 1} successful, now placing...')
                
                if self.place_object(self.place_pose):
                    self.get_logger().info(f'Place {i + 1} successful!')
                else:
                    self.get_logger().error(f'Place {i + 1} failed!')
            else:
                self.get_logger().error(f'Pick {i + 1} failed!')
            
            self.move_to_home()
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('DEMO COMPLETE')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    
    node = ReactiveReplanner()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    demo_thread = threading.Thread(target=node.run_demo)
    demo_thread.start()
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
