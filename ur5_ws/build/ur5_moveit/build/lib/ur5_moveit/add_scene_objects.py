#!/usr/bin/env python3
"""
Add environment objects (tables, cubes) to MoveIt planning scene
ME5250 Final Project - Abdul Rahman
"""
# print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from std_msgs.msg import Header
import time


class SceneSetup(Node):
    def __init__(self):
        super().__init__('scene_setup')
        
        self.collision_object_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )
        
        # Wait for publisher to be ready
        time.sleep(1.0)
        self.get_logger().info('Scene Setup Node initialized')
    
    def add_box(self, name: str, x: float, y: float, z: float, 
                size_x: float, size_y: float, size_z: float,
                frame_id: str = 'base_link'):
        """Add a box collision object"""
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        # Define the box primitive
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, size_z]
        
        # Define pose
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        
        self.collision_object_pub.publish(collision_object)
        self.get_logger().info(f'Added {name} at ({x}, {y}, {z}) size ({size_x}, {size_y}, {size_z})')
    
    def setup_scene(self):
        """Add all environment objects to planning scene"""
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Adding environment to MoveIt Planning Scene')
        self.get_logger().info('=' * 50)
        
        # Ground plane
        self.add_box('ground', 0.0, 0.0, -0.025, 2.0, 2.0, 0.05)
        time.sleep(0.2)
        
        # Pick table (in front of robot)
        # Position: x=0.5, y=0, z=0.15 (center), size: 0.4 x 0.6 x 0.3
        self.add_box('pick_table', 0.5, 0.0, 0.15, 0.4, 0.6, 0.3)
        time.sleep(0.2)
        
        # Place table (to the right, higher)
        # Position: x=0.3, y=-0.5, z=0.25 (center), size: 0.3 x 0.4 x 0.5
        self.add_box('place_table', 0.3, -0.5, 0.25, 0.3, 0.4, 0.5)
        time.sleep(0.2)
        
        # Blue cubes on pick table (surface at z=0.30)
        cube_size = 0.04
        cube_z = 0.30 + cube_size/2  # On top of table
        
        self.add_box('blue_cube_1', 0.45, 0.1, cube_z, cube_size, cube_size, cube_size)
        time.sleep(0.1)
        
        self.add_box('blue_cube_2', 0.5, 0.0, cube_z, cube_size, cube_size, cube_size)
        time.sleep(0.1)
        
        self.add_box('blue_cube_3', 0.55, -0.1, cube_z, cube_size, cube_size, cube_size)
        time.sleep(0.1)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Scene setup complete!')
        self.get_logger().info('Tables and cubes are now visible in RViz')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    
    node = SceneSetup()
    node.setup_scene()
    
    # Spin briefly to ensure messages are sent
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
