from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,TimerAction,DeclareLaunchArgument,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes,ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():
    chase_dir = get_package_share_directory('calculate_attack_pose_if_available')
    params_file = os.path.join(chase_dir, 'config', 'chase_params.yaml')
    self_color = LaunchConfiguration('self_color', default='red')


    invincible_checker = Node(
        package="calculate_attack_pose_if_available",
        executable="invincible_checker",
        namespace='',
        output="screen",
        parameters=[{'self_color': self_color}]
    )

    decide_chase_pos = Node(
        package="calculate_attack_pose_if_available",
        executable="dec_shoot_and_pose",
        namespace='',
        emulate_tty = True,
        output="screen",
        parameters=[params_file],
    )


    return LaunchDescription(
        [
            invincible_checker,
            decide_chase_pos,           
        ]
    )
    