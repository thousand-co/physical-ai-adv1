from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path
import yaml


def generate_launch_description():
    package_share = Path(get_package_share_directory('final_project_ros'))
    world_path = LaunchConfiguration('world')
    gazebo_launch = str(Path(get_package_share_directory('gazebo_ros')) / 'launch' / 'gazebo.launch.py')
    crane_description = Path(get_package_share_directory('crane_plus_description'))
    xacro_file = str(crane_description / 'urdf' / 'crane_plus.urdf.xacro')
    moveit_config = Path(get_package_share_directory('crane_plus_moveit_config'))

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' use_gazebo:=true',
            ' use_ignition:=false',
            ' gz_control_config_package:=crane_plus_control',
            ' gz_control_config_file_path:=config/crane_plus_controllers.yaml',
        ]),
        value_type=str,
    )
    robot_description_semantic = (moveit_config / 'config' / 'crane_plus.srdf').read_text()
    with (moveit_config / 'config' / 'kinematics.yaml').open(encoding='utf-8') as config_file:
        kinematics_config = yaml.safe_load(config_file)
    kinematics_config['arm']['kinematics_solver_timeout'] = 0.1
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic,
            'robot_description_kinematics': kinematics_config,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'crane_plus',
            '-x', '0.10',
            '-y', '-0.05',
            '-z', '0.05',
        ],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'crane_plus_arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'crane_plus_gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    perception_node = Node(
        package='final_project_ros',
        executable='gazebo_delivery_demo',
        name='gazebo_delivery_demo',
        output='screen',
        parameters=[{
            'target_color': LaunchConfiguration('target_color'),
            'image_topic': '/camera/color/image_raw',
            'target_point_topic': '/target_point',
            'base_frame': 'world',
            'min_contour_area': 80.0,
            'use_sim_time': True,
            'hover_height': 0.15,
        }],
    )
    delivery_node = Node(
        package='final_project_ros',
        executable='delivery_controller',
        name='delivery_controller',
        output='screen',
        parameters=[{
            'target_color': LaunchConfiguration('target_color'),
            'base_x': 0.10,
            'base_y': -0.05,
            'base_z': 0.05,
            'box_x': 0.10,
            'box_y': -0.19,
            'box_drop_z': 0.15,
            'approach_height': 0.03,
            'gripper_open_position': 0.50,
            'gripper_closed_position': -0.5236,
            'box_capacity': 20,
            'startup_homing': ParameterValue(LaunchConfiguration('startup_homing'), value_type=bool),
            'approach_max_velocity': ParameterValue(LaunchConfiguration('approach_max_velocity'), value_type=float),
            'approach_max_acceleration': ParameterValue(LaunchConfiguration('approach_max_acceleration'), value_type=float),
            'descent_max_velocity': ParameterValue(LaunchConfiguration('descent_max_velocity'), value_type=float),
            'descent_max_acceleration': ParameterValue(LaunchConfiguration('descent_max_acceleration'), value_type=float),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('startup_homing', default_value='false'),
        DeclareLaunchArgument('approach_max_velocity', default_value='0.8'),
        DeclareLaunchArgument('approach_max_acceleration', default_value='1.4'),
        DeclareLaunchArgument('descent_max_velocity', default_value='0.35'),
        DeclareLaunchArgument('descent_max_acceleration', default_value='0.7'),
        DeclareLaunchArgument('record_demo', default_value='false'),
        DeclareLaunchArgument('record_output', default_value=str(Path.cwd() / 'artifacts/demo_raw.mp4')),
        Node(package='final_project_ros', executable='record_demo',
             condition=IfCondition(LaunchConfiguration('record_demo')),
             parameters=[{'output': LaunchConfiguration('record_output')}], output='screen'),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Run the Gazebo client for visualization',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=str(package_share / 'worlds' / 'simple_delivery.world'),
            description='SDF world file to load',
        ),
        DeclareLaunchArgument(
            'target_color',
            default_value='blue',
            description='Tab10 color to sort: blue, orange, green, red, purple, brown, pink, gray, olive, or cyan',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world_path,
                'verbose': 'true',
                'gui': LaunchConfiguration('gui'),
            }.items(),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[gripper_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gripper_controller_spawner,
                on_exit=[perception_node, delivery_node],
            )
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),
        move_group,
        spawn_robot,
    ])
