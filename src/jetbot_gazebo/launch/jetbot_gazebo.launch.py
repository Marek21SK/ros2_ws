from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    # Nastavenie cesty ku svetu
    world_path = os.path.join(
        get_package_share_directory("jetbot_gazebo"),
        "worlds",
        "test_world.sdf"
    )

    # Cesta k JetBot SDF
    model_path = os.path.join(
        get_package_share_directory("jetbot_gazebo"),
        "models",
        "jetbot",
        "jetbot.sdf"
    )

    return LaunchDescription([

        # Spustenie Gazebo so svetom
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])),
            launch_arguments={
                "gz_args": world_path,
                "on_exit_shutdown": "True"
            }.items(),
        ),

        # Most medzi Gazebo a ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/image@sensor_msgs/msg/Image[gz.msgs.Image'
            ],
            output='screen'
        ),

        # robot_state_publisher na zobrazenie v Rviz2
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{'robot_description': open(model_path).read()}],
            output="screen"
        ),

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', '/home/mpastor2/ros2_ws/src/jetbot_description/rviz/jetbot_config.rviz']
        ),

        # # teleop_twist_keyboard
        # Node(
        #     package="teleop_twist_keyboard",
        #     executable="teleop_twist_keyboard",
        #     output="screen",
        # ),
    ])
