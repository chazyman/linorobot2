# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True
    robot_base = os.getenv('LINOROBOT2_BASE')

    world_name = "trajectory_follower"
    #world_name = 'playground2'

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "worlds", "trajectory_follower.sdf"]
        #[FindPackageShare("linorobot2_gazebo"), "worlds", "elevator.sdf"]
        #[FindPackageShare("linorobot2_gazebo"), "worlds", "playground2.sdf"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'gz_args': world_path
            }.items()
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description', 
                '-name', f'linorobot2_{robot_base}'
            ],
        ),
    # trajectory_follower
    # playground2
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                f'/world/{world_name}/model/linorobot2_{robot_base}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',            
                f'/model/linorobot2_{robot_base}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/odom/unfiltered@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            ],
            remappings=[
                (f'/world/{world_name}/model/linorobot2_{robot_base}/joint_state', '/joint_states'),
                (f'/model/linorobot2_{robot_base}/tf','/tf'), 
                ('/camera/camera_info', '/camera/color/camera_info'),
                ('/camera/image', '/camera/color/image_raw'),
                ('/camera/depth_image', '/camera/depth/image_rect_raw'),
                ('/camera/points', '/camera/depth/color/points'),
            ]
        ),

        Node(
            package="tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0.", "0.", "0.", "0.", "0.", "0.", "camera_bottom_screw_frame", "linorobot2/base_footprint/camera"]
        ),

        # Node(
        #     package='linorobot2_gazebo',
        #     executable='command_timeout',
        #     name='command_timeout'
        # ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940