import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'custom_robot_sim'

    # -- PATHS --
    xacro_file_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'robot_description.urdf.xacro'
    )
    controllers_yaml = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'controller_config.yaml'
    )

    # -- SPAWN POSE --
    robot_pos = ['0.0', '0.0', '0.0']
    robot_yaw = '0.0'

    # -- XACRO -> URDF --
    robot_description_raw = xacro.process_file(xacro_file_path).toxml()

    # -- SIM TIME ARG --
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # -- RSP --
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # -- GAZEBO --
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # -- SPAWN ENTITY --
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_manipulator',
            '-x', robot_pos[0], '-y', robot_pos[1], '-z', robot_pos[2], '-Y', robot_yaw
        ],
        output='screen'
    )

    # -- TWIST MAPPER (cmd_vel -> controllere)
    twist_mapper = Node(
        package='custom_robot_sim',
        executable='twist_to_fwdrear_group',
        name='twist_to_fwdrear_group',
        output='screen',
        parameters=[{
            'wheel_radius': 0.08,
            'wheelbase': 0.4,
            'max_steer': 0.6,
            'max_wheel_speed': 30.0,
            'vel_topic': '/front_wheel_velocity_controller/commands',
            'steer_topic': '/back_steer_position_controller/commands',
            # invertert tegn på styring (du sa den var feil vei)
            'steer_sign_left': -1.0,
            'steer_sign_right': -1.0,
        }]
    )

    # ====== FOXY-SIKKER KONTROLLER-OPPSTART ======
    # NB: TimerAction-perioder er ABSOLUTTE fra launch-start,
    # derfor legger vi dem sekvensielt 6s, 7s, 8s, 9s, 10s.

    # 1) Last YAML-parametre til controller_manager
    load_params = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/controller_manager', controllers_yaml],
        output='screen'
    )

    # 2) Start joint_state_broadcaster
    start_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--controller-manager', '/controller_manager',
             '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    # 3) Start bakhjuls-styring (posisjon)
    start_steer = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--controller-manager', '/controller_manager',
             '--set-state', 'start', 'back_steer_position_controller'],
        output='screen'
    )

    # 4) Start forhjuls-drift (hastighet)
    start_drive = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--controller-manager', '/controller_manager',
             '--set-state', 'start', 'front_wheel_velocity_controller'],
        output='screen'
    )

    # 5) Start arm_hold_controller (låser armen i fast posisjon)
    start_arm_hold = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--controller-manager', '/controller_manager',
             '--set-state', 'start', 'arm_hold_controller'],
        output='screen'
    )

    start_arm_pose = ExecuteProcess(
    cmd=[
        'ros2', 'topic', 'pub',
        '--once',
        '/arm_hold_controller/commands',
        'std_msgs/Float64MultiArray',
        '{data: [0.0, 3.1416, -1.2217, 0.0]}'
    ],
    output='screen'
    )

    return LaunchDescription([
        sim_time_arg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        twist_mapper,  # start mapper-noden
        TimerAction(period=6.0, actions=[load_params]),
        TimerAction(period=7.0, actions=[start_jsb]),
        TimerAction(period=8.0, actions=[start_steer]),
        TimerAction(period=9.0, actions=[start_drive]),
        TimerAction(period=10.0, actions=[start_arm_hold]),
        TimerAction(period=11.0, actions=[start_arm_pose]),
    ])
