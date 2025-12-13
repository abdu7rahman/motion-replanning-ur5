#!/usr/bin/env python3
"""
Pick and Place with Reactive Replanning
ME5250 Final Project - Abdul Rahman
"""

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
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

import numpy as np
import time
import threading
from typing import List
from dataclasses import dataclass


@dataclass
class IKSolution:
    joint_positions: List[float]


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        
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
        
        self.GRIPPER_OPEN = 0.0
        self.GRIPPER_CLOSE = 0.3
        
        self.GRIPPER_LENGTH = 0.22
        
        self.pick_orientation = {
            'x': 0.0,
            'y': -0.707,
            'z': 0.707,
            'w': 0.0
        }
        
        # Cube positions - pick table top at z=0.20
        self.cube_positions = [
            {'name': 'blue_cube_1', 'x': 0.45, 'y': 0.1, 'z': 0.325},
            {'name': 'blue_cube_2', 'x': 0.5, 'y': 0.0, 'z': 0.325},
            {'name': 'blue_cube_3', 'x': 0.55, 'y': -0.1, 'z': 0.325},
        ]
        
        # Place position - LOWERED to be within UR5 reach
        # Place table top at z=0.30, place cube at z=0.34 (same height as pick)
        self.place_position = {'x': 0.4, 'y': -0.35, 'z': 0.34}
        
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10,
            callback_group=self.callback_group
        )
        
        self.collision_sub = self.create_subscription(
            CollisionObject, '/collision_object', self._collision_cb, 10,
            callback_group=self.callback_group
        )
        
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.callback_group)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action', callback_group=self.callback_group)
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory', callback_group=self.callback_group)
        self.gripper_client = ActionClient(self, GripperCommand, '/hand_controller/gripper_cmd', callback_group=self.callback_group)
        
        self.get_logger().info('Waiting for services...')
        self.ik_client.wait_for_service()
        self.move_group_client.wait_for_server()
        self.execute_client.wait_for_server()
        self.gripper_client.wait_for_server()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Pick and Place with Reactive Replanning')
        self.get_logger().info('ME5250 - Abdul Rahman')
        self.get_logger().info('=' * 60)
    
    def _joint_state_cb(self, msg):
        self.current_joint_state = msg
    
    def _collision_cb(self, msg):
        if msg.operation == CollisionObject.ADD and 'obstacle' in msg.id.lower():
            self.get_logger().warn(f'!!! OBSTACLE DETECTED: {msg.id} !!!')
            self.obstacle_detected = True
    	    print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))
    def get_current_joints(self) -> List[float]:
        if self.current_joint_state is None:
            return [0.0] * 6
        positions = []
        for name in self.joint_names:
            try:
                idx = list(self.current_joint_state.name).index(name)
                positions.append(self.current_joint_state.position[idx])
            except:
                positions.append(0.0)
        return positions
    
    def add_collision_box(self, name, x, y, z, sx, sy, sz):
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = name
        
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [sx, sy, sz]
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        
        self.collision_pub.publish(obj)
        self.get_logger().info(f'Added collision: {name}')
    
    def setup_scene(self):
        self.get_logger().info('Setting up planning scene...')
        time.sleep(0.5)
        self.add_collision_box('ground', 0.0, 0.0, -0.01, 2.0, 2.0, 0.01)
        time.sleep(0.1)
        # Pick table - top at z=0.20
        self.add_collision_box('pick_table', 0.5, 0.0, 0.15, 0.4, 0.6, 0.3)
        time.sleep(0.1)
        
        # Place table - top at z=0.30
        self.add_collision_box('place_table', 0.4, -0.50, 0.15, 0.3, 0.4, 0.3)
        time.sleep(0.1)
        
        self.get_logger().info('Scene setup complete!')
    
    def gripper(self, position: float) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.result():
            return False
        
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        return True
    
    def open_gripper(self):
        self.get_logger().info('Opening gripper...')
        self.gripper(self.GRIPPER_OPEN)
        time.sleep(0.5)
    
    def close_gripper(self):
        self.get_logger().info('Closing gripper...')
        self.gripper(self.GRIPPER_CLOSE)
        time.sleep(0.5)
    
    def create_pose(self, x, y, z) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z + self.GRIPPER_LENGTH
        pose.orientation.x = self.pick_orientation['x']
        pose.orientation.y = self.pick_orientation['y']
        pose.orientation.z = self.pick_orientation['z']
        pose.orientation.w = self.pick_orientation['w']
        return pose
    
    def compute_ik(self, pose: Pose, attempts: int = 100) -> List[IKSolution]:
        solutions = []
        seen = set()
        
        self.get_logger().info(f'  Computing IK ({attempts} attempts)...')
        
        for i in range(attempts):
            if i == 0:
                seed = self.get_current_joints()
            else:
                seed = [
                    np.random.uniform(-3.14, 3.14),
                    np.random.uniform(-3.14, 0.0),
                    np.random.uniform(-3.14, 3.14),
                    np.random.uniform(-3.14, 3.14),
                    np.random.uniform(-3.14, 3.14),
                    np.random.uniform(-3.14, 3.14),
                ]
            
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'ur5_arm'
            req.ik_request.robot_state.joint_state.name = self.joint_names
            req.ik_request.robot_state.joint_state.position = seed
            req.ik_request.pose_stamped.header.frame_id = 'world'
            req.ik_request.pose_stamped.pose = pose
            req.ik_request.timeout.sec = 0
            req.ik_request.timeout.nanosec = 500000000
            req.ik_request.avoid_collisions = True
            
            future = self.ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() and future.result().error_code.val == MoveItErrorCodes.SUCCESS:
                joints = []
                for name in self.joint_names:
                    try:
                        idx = list(future.result().solution.joint_state.name).index(name)
                        joints.append(future.result().solution.joint_state.position[idx])
                    except:
                        break
                
                if len(joints) == 6:
                    key = tuple(round(j, 0) for j in joints)
                    if key not in seen:
                        seen.add(key)
                        solutions.append(IKSolution(joints))
        
        return solutions
    
    def plan_and_execute(self, joints: List[float]) -> bool:
        goal = MoveGroup.Goal()
        goal.request.group_name = 'ur5_arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        constraints = Constraints()
        for name, pos in zip(self.joint_names, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        
        goal.planning_options.plan_only = False
        
        self.obstacle_detected = False
        
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.result() or not future.result().accepted:
            self.get_logger().warn('Goal not accepted')
            return False
        
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.obstacle_detected:
                self.get_logger().warn('OBSTACLE DETECTED! Canceling...')
                goal_handle.cancel_goal_async()
                time.sleep(0.5)
                return False
        
        result = result_future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            return True
        else:
            self.get_logger().warn(f'Execution failed: {result.error_code.val}')
            return False
    
    def move_to_pose(self, pose: Pose, desc: str) -> bool:
        self.get_logger().info(f'Moving to {desc}... (EE: x={pose.position.x:.2f}, y={pose.position.y:.2f}, z={pose.position.z:.2f})')
        
        solutions = self.compute_ik(pose)
        self.get_logger().info(f'  Found {len(solutions)} IK solutions')
        
        if not solutions:
            self.get_logger().error('  No IK solutions!')
            return False
        
        for i, sol in enumerate(solutions, 1):
            self.get_logger().info(f'  Trying solution {i}/{len(solutions)}...')
            if self.plan_and_execute(sol.joint_positions):
                self.get_logger().info(f'  Success!')
                return True
            
            if self.obstacle_detected:
                self.get_logger().info('  Obstacle, trying next...')
                self.obstacle_detected = False
        
        self.get_logger().error('  All solutions failed!')
        return False
    
    def pick(self, cube) -> bool:
        name = cube['name']
        x, y, z = cube['x'], cube['y'], cube['z']
        
        self.get_logger().info(f'')
        self.get_logger().info(f'=== PICKING {name} ===')
        self.get_logger().info(f'Cube at: ({x}, {y}, {z})')
        
        self.open_gripper()
        
        if not self.move_to_pose(self.create_pose(x, y, z + 0.15), 'above cube'):
            return False
        
        if not self.move_to_pose(self.create_pose(x, y, z), 'grasp'):
            return False
        
        self.close_gripper()
        time.sleep(0.5)
        
        if not self.move_to_pose(self.create_pose(x, y, z + 0.20), 'lift'):
            return False
        
        self.get_logger().info(f'Picked up {name}!')
        return True
    
    def place(self, cube_name: str) -> bool:
        x, y, z = self.place_position['x'], self.place_position['y'], self.place_position['z']
        
        self.get_logger().info(f'')
        self.get_logger().info(f'=== PLACING {cube_name} ===')
        self.get_logger().info(f'Place at: ({x}, {y}, {z})')
        self.get_logger().info('>>> INSERT OBSTACLE NOW TO TEST REPLANNING <<<')
        
        if not self.move_to_pose(self.create_pose(x, y, z + 0.10), 'above place'):
            return False
        
        if not self.move_to_pose(self.create_pose(x, y, z), 'place'):
            return False
        
        self.open_gripper()
        
        self.move_to_pose(self.create_pose(x, y, z + 0.2), 'retreat')
        
        self.get_logger().info(f'Placed {cube_name}!')
        return True
    
    def run(self):
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('PICK AND PLACE DEMO')
        self.get_logger().info('=' * 60)
        
        time.sleep(2.0)
        
        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        
        self.setup_scene()
        time.sleep(1.0)
        
        cube = self.cube_positions[0]
        
        if self.pick(cube):
            self.place(cube['name'])
        else:
            self.get_logger().error('Pick failed!')
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('DONE!')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    thread = threading.Thread(target=node.run)
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
