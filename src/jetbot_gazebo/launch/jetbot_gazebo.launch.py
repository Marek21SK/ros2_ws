from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
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

    # Cesta k Rviz configu
    rviz_config_path = os.path.join(
        get_package_share_directory("jetbot_description"),
        "rviz",
        "jetbot_config.rviz"
    )

    # Nody pre robota
    collision_test = Node(
        package='jetbot_gazebo',
        executable='collision_test',
        name='collision_test',
        output='screen'
    )

    obstacle_stop = Node(
        package='jetbot_gazebo',
        executable='obstacle_stop',
        name='obstacle_stop',
        output='screen'
    )

    obstacle_avoid = Node(
        package='jetbot_gazebo',
        executable='obstacle_avoid',
        name='obstacle_avoid',
        output='screen'
    )

    run_to_goal = Node(
        package='jetbot_gazebo',
        executable='run_to_goal',
        name='run_to_goal',
        output='screen'
    )

    follow_wall = Node(
        package='jetbot_gazebo',
        executable='follow_wall',
        name='follow_wall',
        output='screen'
    )

    bug0_algoritmus = Node(
        package='jetbot_gazebo',
        executable='bug0_algoritmus',
        name='bug0_algoritmus',
        output='screen'
    )

    bug1_algoritmus = Node(
        package='jetbot_gazebo',
        executable='bug1_algoritmus',
        name='bug1_algoritmus',
        output='screen'
    )

    tracking_node = Node(
        package='jetbot_gazebo',
        executable='tracking_node',
        name='tracking_node',
        output='screen'
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
            arguments=['-d', rviz_config_path]
        ),

        # Path Publisher
        Node(
            package='jetbot_gazebo',
            executable='path_publisher',
            name='path_publisher',
            output='log'
        ),

        # Spúšťanie skriptov
        tracking_node,
        # collision_test,
        # obstacle_stop,
        # obstacle_avoid,
        # run_to_goal,
        # follow_wall,
        # bug0_algoritmus,
        # bug1_algoritmus,
    ])
