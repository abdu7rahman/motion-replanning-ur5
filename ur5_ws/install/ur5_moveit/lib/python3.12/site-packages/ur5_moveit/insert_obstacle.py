#!/usr/bin/env python3
"""
Obstacle Inserter for ME5250 Bin Picking Project
Abdul Rahman - ME5250 Northeastern University
"""
# print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
import time


class ObstacleInserter(Node):
    def __init__(self):
        super().__init__('obstacle_inserter')
        
        self.collision_object_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )
        time.sleep(0.5)
    
    def add_obstacle(self, name, x, y, z, radius, height):
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [height, radius]
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        
        for _ in range(5):
            self.collision_object_pub.publish(collision_object)
            time.sleep(0.1)
        
        self.get_logger().info(f'Added obstacle: {name} at ({x}, {y}, {z})')
    
    def remove_obstacle(self, name):
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        collision_object.operation = CollisionObject.REMOVE
        
        for _ in range(5):
            self.collision_object_pub.publish(collision_object)
            time.sleep(0.1)
        
        self.get_logger().info(f'Removed obstacle: {name}')


def main():
    parser = argparse.ArgumentParser(description='Insert/remove obstacles in MoveIt')
    parser.add_argument('--x', type=float, default=0.3, help='X position')
    parser.add_argument('--y', type=float, default=-0.2, help='Y position')
    parser.add_argument('--z', type=float, default=0.5, help='Z position')
    parser.add_argument('--radius', type=float, default=0.04, help='Cylinder radius')
    parser.add_argument('--height', type=float, default=0.25, help='Cylinder height')
    parser.add_argument('--name', type=str, default='obstacle', help='Obstacle name')
    parser.add_argument('--remove', action='store_true', help='Remove obstacle')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = ObstacleInserter()
    
    try:
        if args.remove:
            node.remove_obstacle(args.name)
        else:
            node.add_obstacle(args.name, args.x, args.y, args.z, args.radius, args.height)
        
        rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
