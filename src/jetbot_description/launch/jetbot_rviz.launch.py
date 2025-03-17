from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, Shutdown, ExecuteProcess
from launch.event_handlers import OnProcessExit
import os

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory('jetbot_description'), 'models/urdf', 'jetbot.urdf')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/mpastor2/ros2_ws/src/jetbot_description/rviz/jetbot_config.rviz']
    )

    # Spustenie ovládania pre robota
    teleop_node = ExecuteProcess(
        cmd=['x-terminal-emulator', '-e', 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'],
        name='teleop_twist_keyboard',
        output='screen'
    )

    return LaunchDescription([
        # Static Transform Publishers (odom -> base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'odom', '--child-frame-id', 'base_link'
            ],
            name='odom_to_base_link'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),

        # Path Publisher
        Node(
            package='jetbot_description',
            executable='path_publisher',
            name='path_publisher',
            output='log'
        ),

        # Cmd_vel to Odom
        Node(
            package='jetbot_description',
            executable='cmd_vel_to_odom',
            name='cmd_vel_to_odom',
            output='log'
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='log'
        ),

        # Cam2Image Publisher (kamera)
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image',
            output='log',
            parameters=[{'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', 'WARN']
        ),

        # Spustenie Rviz-u
        rviz_node,

        # Spustenie ovládanie robota
        teleop_node,

        # Automatické vypnutie Rviz
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=[Shutdown()]
            )
        ),

        # Automatické vypnutie Teleop_twist_keyboard
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=[Shutdown()]
            )
        ),
    ])
