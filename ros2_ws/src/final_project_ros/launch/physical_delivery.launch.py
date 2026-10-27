from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,RegisterEventHandler,EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share=Path(get_package_share_directory('final_project_ros'))
    description=(share/'config/physical_crane.urdf').read_text()
    spawn=Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-topic','robot_description','-entity','crane_plus','-x','.1','-y','-.05','-z','.05'],output='screen')
    broadcaster=Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','--controller-manager-timeout','90'],output='screen')
    arm=Node(package='controller_manager',executable='spawner',arguments=['crane_plus_arm_controller','--controller-manager-timeout','90'],output='screen')
    hand=Node(package='controller_manager',executable='spawner',arguments=['crane_plus_gripper_controller','--controller-manager-timeout','90'],output='screen')
    perception=Node(package='final_project_ros',executable='gazebo_delivery_demo',parameters=[{'use_sim_time':True,'target_color':'blue','min_contour_area':40.}],output='screen')
    sorting=LaunchConfiguration('sorting')
    controller=Node(package='final_project_ros',executable=PythonExpression(["'conveyor_sorting' if '",sorting,"' == 'true' else 'physical_delivery'"]),parameters=[{'use_sim_time':True,'output':LaunchConfiguration('output'),'disable_grip':ParameterValue(LaunchConfiguration('disable_grip'),value_type=bool)}],output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='false'),
        DeclareLaunchArgument('sorting',default_value='false'),
        DeclareLaunchArgument('output',default_value=str(Path.cwd()/'artifacts/physical_delivery/episode.json')),
        DeclareLaunchArgument('disable_grip',default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('gazebo_ros'))/'launch/gazebo.launch.py')),launch_arguments={'world':PythonExpression(["'",str(share/'worlds'),"/' + ('conveyor_sorting.world' if '",sorting,"' == 'true' else 'physical_delivery.world')"]),'gui':LaunchConfiguration('gui'),'verbose':'true'}.items()),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description,'use_sim_time':True}],output='screen'),spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=[broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster,on_exit=[arm])),
        RegisterEventHandler(OnProcessExit(target_action=arm,on_exit=[hand])),
        RegisterEventHandler(OnProcessExit(target_action=hand,on_exit=[perception,controller])),
        RegisterEventHandler(OnProcessExit(target_action=controller,on_exit=[EmitEvent(event=Shutdown(reason='Physical delivery evidence saved'))])),
    ])
