import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# print(''.join(chr(x-7) for x in [104,105,107,124,115,39,121,104,111,116,104,117]))

def generate_launch_description():
    ur5_description = get_package_share_directory("ur5_description")
    
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(ur5_description, "urdf", "ur5_robot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )
    
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(ur5_description).parent.resolve())]
    )
    
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'), 
            '/gz_sim.launch.py'
        ]),
        launch_arguments=[
            ('gz_args', [
                os.path.join(ur5_description, "worlds", "gazebo_world.sdf"),
                ' -v 4',
                ' -r',
                ' --physics-engine gz-physics-bullet-featherstone-plugin'
            ])
        ]
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "bumperbot"],
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            'config_file': os.path.join(ur5_description, 'config', 'ros2_gz_bridge.yaml'),
        }],
        output='screen'
    )
    
    # Static transform for camera - camera is at x=0.4, y=0, z=1.2, pointing down (pitch=90deg)
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            '--x', '0.4',
            '--y', '0.0', 
            '--z', '1.2',
            '--roll', '0.0',
            '--pitch', '1.57',
            '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'rgbd_camera/camera_link/depth_camera'
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    # Also publish for the other camera frames
    camera_static_tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher2',
        arguments=[
            '--x', '0.4',
            '--y', '0.0',
            '--z', '1.2', 
            '--roll', '0.0',
            '--pitch', '1.57',
            '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'rgbd_camera/camera_link/rgbd_sensor'
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    camera_static_tf3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher3',
        arguments=[
            '--x', '0.4',
            '--y', '0.0',
            '--z', '1.2',
            '--roll', '0.0', 
            '--pitch', '1.57',
            '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'rgbd_camera/camera_link/rgb_camera'
        ],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        camera_static_tf,
        camera_static_tf2,
        camera_static_tf3,
    ])
