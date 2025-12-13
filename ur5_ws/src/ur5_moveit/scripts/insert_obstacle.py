#!/usr/bin/env python3
"""
Obstacle Inserter for ME5250 Bin Picking Project
Spawns a red cylinder (mimicking human hand) in both Gazebo and MoveIt planning scene

Usage:
  ros2 run ur5_moveit insert_obstacle --x 0.3 --y 0.1 --z 0.5
  ros2 run ur5_moveit insert_obstacle --x 0.3 --y 0.1 --z 0.5 --radius 0.03 --height 0.15
  ros2 run ur5_moveit insert_obstacle --remove

Abdul Rahman - ME5250 Northeastern University
"""


import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from std_msgs.msg import Header
import subprocess
import time


class ObstacleInserter(Node):
    def __init__(self):
        super().__init__('obstacle_inserter')
        
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/monitored_planning_scene',
            10
        )
        
        self.collision_object_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )
        
        time.sleep(0.5)
        
    def spawn_obstacle_gazebo(self, name, x, y, z, radius, height):
        """Spawn red cylinder in Gazebo"""
        
        sdf_content = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>false</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.0 0.0 1</ambient>
          <diffuse>1.0 0.0 0.0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
        
        try:
            cmd = [
                'gz', 'service', '-s', '/world/pick_place_world/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'sdf: "{sdf_content.replace(chr(10), " ").replace(chr(34), chr(92)+chr(34))}"'
            ]
            
            result = subprocess.run(
                ['gz', 'service', '-s', '/world/pick_place_world/create',
                 '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1000',
                 '--req', f'sdf: \'{sdf_content}\''],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                self.get_logger().info(f'Spawned obstacle in Gazebo at ({x}, {y}, {z})')
            else:
                self.get_logger().warn(f'Gazebo spawn may have failed: {result.stderr}')
                self.spawn_obstacle_gazebo_alternative(name, x, y, z, radius, height)
                
        except Exception as e:
            self.get_logger().warn(f'Gazebo spawn exception: {e}')
            self.spawn_obstacle_gazebo_alternative(name, x, y, z, radius, height)
    
    def spawn_obstacle_gazebo_alternative(self, name, x, y, z, radius, height):
        """Alternative spawn method using ros_gz_sim"""
        try:
            sdf = f'''<?xml version="1.0" ?><sdf version="1.8"><model name="{name}"><static>true</static><link name="link"><visual name="visual"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry><material><ambient>0.8 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual><collision name="collision"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry></collision></link></model></sdf>'''
            
            subprocess.run([
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-string', sdf,
                '-name', name,
                '-x', str(x), '-y', str(y), '-z', str(z)
            ], timeout=5)
            self.get_logger().info(f'Spawned obstacle via ros_gz_sim at ({x}, {y}, {z})')
        except Exception as e:
            self.get_logger().error(f'Alternative spawn also failed: {e}')
    
    def add_obstacle_moveit(self, name, x, y, z, radius, height):
        """Add collision object to MoveIt planning scene"""
        
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
        
        self.collision_object_pub.publish(collision_object)
        self.get_logger().info(f'Added obstacle to MoveIt planning scene: {name} at ({x}, {y}, {z})')
    
    def remove_obstacle_moveit(self, name):
        """Remove collision object from MoveIt planning scene"""
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        collision_object.operation = CollisionObject.REMOVE
        
        self.collision_object_pub.publish(collision_object)
        self.get_logger().info(f'Removed obstacle from MoveIt: {name}')
        print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))
    
    def remove_obstacle_gazebo(self, name):
        """Remove model from Gazebo"""
        try:
            subprocess.run([
                'gz', 'service', '-s', '/world/pick_place_world/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'name: "{name}" type: MODEL'
            ], timeout=5)
            self.get_logger().info(f'Removed obstacle from Gazebo: {name}')
        except Exception as e:
            self.get_logger().warn(f'Gazebo removal failed: {e}')


def main():
    parser = argparse.ArgumentParser(description='Insert/remove obstacles in Gazebo and MoveIt')
    parser.add_argument('--x', type=float, default=0.2, help='X position')
    parser.add_argument('--y', type=float, default=-0.1, help='Y position')
    parser.add_argument('--z', type=float, default=0.5, help='Z position')
    parser.add_argument('--radius', type=float, default=0.035, help='Cylinder radius (default: 0.035m like a finger)')
    parser.add_argument('--height', type=float, default=0.3, help='Cylinder height (default: 0.15m)')
    parser.add_argument('--name', type=str, default='red_obstacle', help='Obstacle name')
    parser.add_argument('--remove', action='store_true', help='Remove obstacle instead of adding')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = ObstacleInserter()
    
    try:
        if args.remove:
            node.remove_obstacle_moveit(args.name)
            node.remove_obstacle_gazebo(args.name)
        else:
            node.add_obstacle_moveit(args.name, args.x, args.y, args.z, args.radius, args.height)
            node.spawn_obstacle_gazebo(args.name, args.x, args.y, args.z, args.radius, args.height)
        
        rclpy.spin_once(node, timeout_sec=1.0)
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
