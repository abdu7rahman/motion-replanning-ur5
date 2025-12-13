#!/usr/bin/env python3
"""
PointCloud to Planning Scene - Adds detected objects to MoveIt planning scene
ME5250 Final Project - Abdul Rahman

Uses depth camera pointcloud to detect obstacles and add them to MoveIt
for collision-aware motion planning.
"""
# print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from std_msgs.msg import Header
import struct
import numpy as np
from collections import defaultdict


class PointCloudToPlanningScene(Node):
    def __init__(self):
        super().__init__('pointcloud_to_planning_scene')
        
        # Publisher for collision objects
        self.collision_object_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )
        
        # Subscriber to pointcloud
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )
        
        self.scene_initialized = False
        self.update_rate = 2.0  # Update planning scene every 2 seconds
        self.last_update_time = self.get_clock().now()
        
        # Known static objects (we'll add these once)
        self.static_objects_added = False
        
        self.get_logger().info('PointCloud to Planning Scene node started')
        self.get_logger().info('Waiting for pointcloud data...')
        
        # Add static objects immediately
        self.add_static_objects()
    
    def add_box(self, name: str, x: float, y: float, z: float,
                size_x: float, size_y: float, size_z: float,
                frame_id: str = 'world'):
        """Add a box collision object to planning scene"""
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, size_z]
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        
        self.collision_object_pub.publish(collision_object)
        self.get_logger().info(f'Added collision object: {name} at ({x:.2f}, {y:.2f}, {z:.2f})')
    
    def add_cylinder(self, name: str, x: float, y: float, z: float,
                     radius: float, height: float, frame_id: str = 'world'):
        """Add a cylinder collision object to planning scene"""
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [height, radius]  # height, radius
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        
        self.collision_object_pub.publish(collision_object)
        self.get_logger().info(f'Added cylinder: {name} at ({x:.2f}, {y:.2f}, {z:.2f})')
    
    def add_static_objects(self):
        """Add known static objects (tables) to planning scene"""
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Adding static objects to planning scene')
        self.get_logger().info('=' * 50)
        
        # Ground plane
        self.add_box('ground', 0.0, 0.0, -0.01, 3.0, 3.0, 0.02)
        
        # Pick table - from world file: pose(0.5, 0, 0.15), size(0.4, 0.6, 0.3)
        self.add_box('pick_table', 0.5, 0.0, 0.15, 0.4, 0.6, 0.3)
        
        # Place table - from world file: pose(0.3, -0.5, 0.25), size(0.3, 0.4, 0.5)
        self.add_box('place_table', 0.3, -0.5, 0.25, 0.3, 0.4, 0.5)
        
        # Blue cubes on pick table (surface at z=0.30)
        cube_size = 0.04
        cube_z = 0.32  # On top of table
        
        self.add_box('blue_cube_1', 0.45, 0.1, cube_z, cube_size, cube_size, cube_size)
        self.add_box('blue_cube_2', 0.5, 0.0, cube_z, cube_size, cube_size, cube_size)
        self.add_box('blue_cube_3', 0.55, -0.1, cube_z, cube_size, cube_size, cube_size)
        
        self.static_objects_added = True
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Static objects added to planning scene!')
        self.get_logger().info('You should now see green collision objects in RViz')
        self.get_logger().info('=' * 50)
    
    def pointcloud_callback(self, msg: PointCloud2):
        """Process pointcloud and detect dynamic obstacles"""
        
        # Rate limit updates
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_update_time).nanoseconds / 1e9
        
        if time_diff < self.update_rate:
            return
        
        self.last_update_time = current_time
        
        if not self.scene_initialized:
            self.get_logger().info('Receiving pointcloud data!')
            self.scene_initialized = True
        
        # Parse pointcloud to detect new obstacles
        # For now, we rely on the insert_obstacle.py script to add dynamic obstacles
        # This callback can be extended to do actual obstacle detection from pointcloud


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToPlanningScene()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
