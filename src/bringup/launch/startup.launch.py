from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,TimerAction,DeclareLaunchArgument,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes,ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import yaml
from launch.substitutions import PythonExpression
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    nav_params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    globol_config = os.path.join(bringup_dir, 'params', 'global_config.yaml')

    with open(globol_config,'r') as f:
        global_params = yaml.safe_load(f)
    use_fake_referee_yaml = global_params.get('use_fake_referee',False)
    self_color_yaml = global_params.get('self_color','red')

    use_fake_referee = LaunchConfiguration('use_fake_referee', default='false')
    use_fake_referee_cmd = DeclareLaunchArgument(
        'use_fake_referee',
        default_value=str(use_fake_referee_yaml).lower(),
        description='Whether to use fake referee'
    )

    self_color = LaunchConfiguration('self_color', default='red')
    self_color_yaml_cmd = DeclareLaunchArgument(
        'self_color',
        default_value=str(self_color_yaml).lower(),
        description='Self Color'
    )

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        parameters=[nav_params_file, {'autostart': True}],
    )
    
    tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('bringup')+"/launch/tf.launch.py"
        )
    )

    livox_ros_driver2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('livox_ros_driver2')+"/launch/msg_MID360_launch.py"
        )
    )

    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('point_lio_with_grid_map')+"/launch/point_lio.launch.py"
        )
    )

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('fast_lio')+"/launch/mapping.launch.py"
        )
    )

    point_lio_rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace='',
        arguments=["-d", os.path.join(bringup_dir, 'rviz', 'loam_livox.rviz')],
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    start_nav2_rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace='',
        arguments=["-d", os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    referee_fake = Node(
            package="dec_tree",
            executable="referee_fake",
            namespace='',
            output="screen",
            condition=IfCondition(use_fake_referee),
    )

    dec = Node(
            package="dec_tree",
            executable="root",
            namespace='',
            output="both",
            parameters=[{'self_color': self_color}]
    )
    

    dec_radical = Node(
            package="dec_tree",
            executable="root_radical",
            namespace='',
            output="both",
            parameters=[{'self_color': self_color}]
        )
    
    dec_simple = Node(
            package="dec_tree",
            executable="root_simple",
            namespace='',
            output="both",
            parameters=[{'self_color': self_color}]
        )
    seg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('linefit_ground_segmentation_ros')+"/launch/segmentation.launch.py"
        )
    )

    load_map_server = LoadComposableNodes(
        target_container='container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                parameters=[{'yaml_filename': os.path.join(bringup_dir, 'map', '601_left.yaml')}],
                # parameters=[{'yaml_filename': os.path.join(bringup_dir, 'map', '6floor_mid.yaml')}],
                # parameters=[{'yaml_filename': os.path.join(bringup_dir, 'map', 'blank.yaml')}],
                # parameters=[{'yaml_filename': os.path.join(bringup_dir, 'map', 'rmuc_2025_normalized.yaml')}],
                name='map_server',),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'autostart': True,
                             'node_names': ['map_server']}]),
         ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nav')+"/launch/bringup_launch.py"        
        )
    )

    fake_baselink = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('fake_baselink')+"/launch/fake_baselink.launch.py"
        )
    )
    p_to_l = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        namespace='',
        output="screen",
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': 0.2,
            'max_height': 1.0,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,  # M_PI/2
            'angle_increment': 0.0043,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.4,
            'range_max': 15.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        remappings=[('cloud_in',  ['/segmentation/obstacle']),
                ('scan',  ['/scan'])],
    )

    dp_a = Node(
        package="dip_angle",
        executable="dip_angle",
        namespace='',
        output="screen",
    )

    gicp=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('small_gicp_localization')+"/launch/small_gicp_localization.launch.py"
        )
    )

    rm_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('rm_serial')+"/launch/rm_serial.launch.py"
        ),
        launch_arguments={'use_fake_referee': use_fake_referee}.items()
    )

    icp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            
            get_package_share_directory('icp_registration')+'/launch/icp.launch.py'
        )
    )

    modify=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('modify_map_to_odom')+"/launch/modify.launch.py"
        )
    )

    calculatepose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('calculate_attack_pose_if_available')+"/launch/calculatepose.launch.py"
        ),
        launch_arguments={'our_color': self_color}.items()
    )

    return LaunchDescription(
        [
            use_fake_referee_cmd,
            self_color_yaml_cmd ,
            modify,
            # icp,
            rm_serial,
            #  pointlio,
            fastlio ,
            livox_ros_driver2,
            container,
            referee_fake,
            tf,
            start_nav2_rviz,
            seg,
            load_map_server,
            p_to_l,
            fake_baselink,
            dp_a, 
            # TimerAction(period=4.0, actions=[icp]),
            # TimerAction(period=4.0, actions=[nav2]),
            # TimerAction(period=4.0, actions=[dec]),
            # TimerAction(period=8.0, actions=[dec_simple]),
            TimerAction(period=8.0, actions=[calculatepose]),


            
        ]
    )
    