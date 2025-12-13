#!/usr/bin/env python3
"""
Reactive Replanning Pick and Place using Kinematic Redundancy
ME5250 Final Project - Abdul Rahman

Strategy 1: Reactive replanning by exploiting kinematic redundancy
of the UR5 manipulator. When obstacles appear, computes multiple IK 
solutions for the same goal pose and evaluates alternative configurations.
"""
# print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.msg import (
    CollisionObject,
    MoveItErrorCodes,
    Constraints,
    JointConstraint,
)
from moveit_msgs.srv import GetPositionIK, GetPlanningScene
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

import numpy as np
import time
import threading
from typing import List, Optional
from dataclasses import dataclass


@dataclass 
class IKSolution:
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
        self.obstacle_detected = False
        
        # Place pose - on the higher table to the right
        # Table at x=0.3, y=-0.5, surface z=0.50
        self.place_pose = self._create_pose(0.25, -0.45, 0.58, 0.707, 0.707, 0.0, 0.0)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10,
            callback_group=self.callback_group
        )
        
        self.collision_object_sub = self.create_subscription(
            CollisionObject, '/collision_object', self._collision_object_callback, 10,
            callback_group=self.callback_group
        )
        
        # Publisher
        self.collision_object_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        # Service clients
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.callback_group)
        self.planning_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene', callback_group=self.callback_group)
        
        # Action clients
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.callback_group)
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory', callback_group=self.callback_group)
        
        self.get_logger().info('Reactive Replanner initialized')
        self._wait_for_services()
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('ME5250 Reactive Replanning Demo')
        self.get_logger().info('Abdul Rahman - Northeastern University')
        self.get_logger().info('=' * 50)
    
    def _create_pose(self, x, y, z, qx, qy, qz, qw) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose
    
    def _wait_for_services(self):
        self.get_logger().info('Waiting for services...')
        self.ik_client.wait_for_service()
        self.planning_scene_client.wait_for_service()
        self.move_group_client.wait_for_server()
        self.execute_trajectory_client.wait_for_server()
        self.get_logger().info('All services ready!')
    
    def _joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
    
    def _collision_object_callback(self, msg: CollisionObject):
        if msg.operation == CollisionObject.ADD:
            if 'obstacle' in msg.id.lower() or 'red' in msg.id.lower():
                self.get_logger().warn(f'!!! OBSTACLE DETECTED: {msg.id} !!!')
                self.obstacle_detected = True
    
    def get_current_joints_ordered(self) -> List[float]:
        """Get current joint positions in correct order for MoveIt"""
        if self.current_joint_state is None:
            return [0.0] * 6
        
        positions = []
        for name in self.joint_names:
            try:
                idx = list(self.current_joint_state.name).index(name)
                positions.append(self.current_joint_state.position[idx])
            except (ValueError, IndexError):
                positions.append(0.0)
        return positions
    
    def compute_multiple_ik(self, target_pose: Pose, num_attempts: int = 30) -> List[IKSolution]:
        """Compute multiple IK solutions exploiting kinematic redundancy"""
        solutions = []
        seen_configs = set()
        
        self.get_logger().info(f'Computing IK for pose: x={target_pose.position.x:.2f}, y={target_pose.position.y:.2f}, z={target_pose.position.z:.2f}')
        
        for i in range(num_attempts):
            if i == 0:
                seed = self.get_current_joints_ordered()
            else:
                seed = self._random_seed()
            
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'ur5_arm'
            request.ik_request.robot_state.joint_state.name = self.joint_names
            request.ik_request.robot_state.joint_state.position = seed
            request.ik_request.pose_stamped.header.frame_id = 'base_link'
            request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            request.ik_request.pose_stamped.pose = target_pose
            request.ik_request.timeout.sec = 0
            request.ik_request.timeout.nanosec = 200000000
            request.ik_request.avoid_collisions = True
            
            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() and future.result().error_code.val == MoveItErrorCodes.SUCCESS:
                result = future.result()
                
                joint_pos = []
                for name in self.joint_names:
                    try:
                        idx = list(result.solution.joint_state.name).index(name)
                        joint_pos.append(result.solution.joint_state.position[idx])
                    except:
                        break
                
                if len(joint_pos) == 6:
                    key = tuple(round(p, 1) for p in joint_pos)
                    if key not in seen_configs:
                        seen_configs.add(key)
                        manip = self._manipulability(joint_pos)
                        solutions.append(IKSolution(joint_pos, manip, True))
        
        solutions.sort(key=lambda s: s.manipulability, reverse=True)
        self.get_logger().info(f'Found {len(solutions)} unique IK solutions')
        return solutions
    
    def _random_seed(self) -> List[float]:
        return [
            np.random.uniform(-3.14, 3.14),
            np.random.uniform(-2.5, 0.0),
            np.random.uniform(-2.0, 2.0),
            np.random.uniform(-3.14, 3.14),
            np.random.uniform(-3.14, 3.14),
            np.random.uniform(-3.14, 3.14),
        ]
    
    def _manipulability(self, joints: List[float]) -> float:
        score = 1.0
        for j in joints:
            dist = min(abs(j - 3.14), abs(j + 3.14))
            score *= (dist / 3.14)
        return score
    
    def plan_to_joints(self, target: List[float]) -> Optional[JointTrajectory]:
        """Plan to joint configuration"""
        goal = MoveGroup.Goal()
        goal.request.group_name = 'ur5_arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        
        constraints = Constraints()
        for name, pos in zip(self.joint_names, target):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.result() or not future.result().accepted:
            return None
        
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        result = result_future.result()
        if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            return result.result.planned_trajectory.joint_trajectory
        return None
    
    def execute_trajectory(self, traj: JointTrajectory) -> bool:
        """Execute with obstacle monitoring"""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = traj
        
        self.obstacle_detected = False
        
        future = self.execute_trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.result() or not future.result().accepted:
            return False
        
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        
        self.get_logger().info('Executing trajectory...')
        
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.obstacle_detected:
                self.get_logger().warn('!!! OBSTACLE DETECTED - ABORTING !!!')
                goal_handle.cancel_goal_async()
                time.sleep(0.5)
                return False
        
        return result_future.result().result.error_code.val == MoveItErrorCodes.SUCCESS
    
    def move_to_place_with_replanning(self) -> bool:
        """Move to place position with reactive replanning on obstacle"""
        self.get_logger().info('Moving to PLACE position...')
        self.get_logger().info('>>> INSERT OBSTACLE NOW TO TEST REPLANNING <<<')
        
        solutions = self.compute_multiple_ik(self.place_pose)
        if not solutions:
            self.get_logger().error('No IK solutions for place pose!')
            return False
        
        self.get_logger().info(f'Got {len(solutions)} IK solutions for place pose')
        
        # Try first solution
        self.get_logger().info('Trying solution 1...')
        traj = self.plan_to_joints(solutions[0].joint_positions)
        
        if traj is None:
            self.get_logger().error('Planning failed for solution 1')
            return False
        
        success = self.execute_trajectory(traj)
        
        if success:
            self.get_logger().info('Reached place position!')
            return True
        
        # If obstacle detected, replan using kinematic redundancy
        if self.obstacle_detected:
            self.get_logger().info('')
            self.get_logger().info('=' * 50)
            self.get_logger().info('REPLANNING USING KINEMATIC REDUNDANCY')
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'Testing {len(solutions)-1} alternative configurations...')
            
            for i, sol in enumerate(solutions[1:], 2):
                self.get_logger().info(f'Trying configuration {i}/{len(solutions)} (manipulability: {sol.manipulability:.4f})')
                self.obstacle_detected = False
                
                traj = self.plan_to_joints(sol.joint_positions)
                if traj is None:
                    self.get_logger().info(f'  Config {i}: Planning failed (blocked by obstacle)')
                    continue
                
                self.get_logger().info(f'  Config {i}: Planning succeeded, executing...')
                if self.execute_trajectory(traj):
                    self.get_logger().info(f'  Config {i}: SUCCESS! Found collision-free path!')
                    return True
                else:
                    self.get_logger().info(f'  Config {i}: Execution failed')
            
            self.get_logger().error('All configurations exhausted!')
        
        return False
    
    def return_to_start(self) -> bool:
        """Return to starting/pick position"""
        self.get_logger().info('Returning to start position...')
        
        # Store current position as target (robot starts at pick position)
        start_joints = [0.0, 0.0, 1.42, 0.24, 4.69, 1.63]  # The default position
        
        traj = self.plan_to_joints(start_joints)
        if traj:
            return self.execute_trajectory(traj)
        return False
    
    def run_demo(self):
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('REACTIVE REPLANNING DEMO - KINEMATIC REDUNDANCY')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('The robot starts at PICK position (above blue cubes)')
        self.get_logger().info('It will move to PLACE position (higher table on right)')
        self.get_logger().info('')
        self.get_logger().info('To test replanning, insert obstacle DURING motion:')
        self.get_logger().info('  python3 ~/me5250/src/ur5_ws/src/ur5_moveit/ur5_moveit/insert_obstacle.py --x 0.3 --y -0.2 --z 0.5')
        self.get_logger().info('')
        self.get_logger().info('Press ENTER to start the demo...')
        input()
        
        # Wait for joint states
        while self.current_joint_state is None:
            self.get_logger().info('Waiting for joint states...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        current = self.get_current_joints_ordered()
        self.get_logger().info(f'Current joints: {[f"{j:.2f}" for j in current]}')
        
        # Do 3 pick-place cycles
        for cycle in range(3):
            self.get_logger().info('')
            self.get_logger().info(f'===== CYCLE {cycle+1}/3 =====')
            self.get_logger().info('Robot is at PICK position (simulating grasp)')
            time.sleep(1.0)
            
            # Move to place
            if self.move_to_place_with_replanning():
                self.get_logger().info('At PLACE position (simulating release)')
                time.sleep(1.0)
            else:
                self.get_logger().warn('Failed to reach place position')
            
            # Return to pick
            self.return_to_start()
            time.sleep(1.0)
            
            # Remove obstacle for next cycle
            self.obstacle_detected = False
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('DEMO COMPLETE')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveReplanner()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    thread = threading.Thread(target=node.run_demo)
    thread.start()
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
