from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Cesta k súboru sveta pre Gazebo
    world_path = os.path.join(
        '/home/mpastor2/ros2_ws/src/jetbot_gazebo/worlds',
        'test_world.sdfworld'
    )

    # Cesta k JetBot SDF
    urdf_path = os.path.join(
        '/home/mpastor2/ros2_ws/src/jetbot_gazebo/models/jetbot/sdf',
        'jetbot.sdf'
    )

    # Spustenie Gazebo Harmony
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", world_path],
        output="screen"
    )

    # Spustenie robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}],
        output='screen'
    )
    
    # Ovládanie robota cez klávesnicu
    teleop_twist_keyboard = Node(
      package='teleop_twist_keyboard',
      executable='teleop_twist_keyboard',
      output='screen',
      remappings=[('/cmd_vel', '/jetbot/cmd_vel')]
    )

    # Spustenie controller_manager s YAML konfiguráciou
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': open(urdf_path).read()},
            os.path.join('/home/mpastor2/ros2_ws/src/jetbot_gazebo/config', 'jetbot_controller.yaml')
        ]
    )

    # Spustenie ros_gz_bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/model/jetbot/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/jetbot/odom/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ]
    )

    # Spawnovanie robota v Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', urdf_path, '-name', 'jetbot', '-x', '1.0', '-y', '1.0', '-z', '0.1'],
        output='screen'
    )
    
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    spawn_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        #teleop_twist_keyboard, //// musím otvoriť fyzický terminál
        controller_manager,
        ros_gz_bridge,
        spawn_robot,
        spawn_joint_state_broadcaster,
        spawn_diff_drive_controller
    ])
