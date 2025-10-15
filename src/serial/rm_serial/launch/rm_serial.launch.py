from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    use_fake_referee = LaunchConfiguration('use_fake_referee', default='true')

    receive_from_c = Node(
        package="rm_serial",
        executable="receive_from_c",
        name="receive_from_c",
        output='screen',
        emulate_tty = True,
        parameters=[{'use_fake_referee': use_fake_referee}]
    )
    send_to_c = Node(
        package="rm_serial",
        executable="send_to_c",
        name="send_to_c",
        output='screen',
    )
    return LaunchDescription(
        [
            receive_from_c,
            send_to_c
        ]
    )