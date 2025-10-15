from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)

# def generate_launch_description():

#     rviz_config_file = get_package_share_directory('bringup')+'/rviz/mapping.rviz'
#     map_dir = get_package_share_directory('bringup')+'/map/'
#     params_file = get_package_share_directory('nav')+'/params/mapper_params_online_async.yaml'

#     container = ComposableNodeContainer(
#         name='container',
#         namespace='',
#         package='rclcpp_components',
#         executable='component_container',
#         output='screen',
#         #parameters=[params_file, {'autostart': True}],
#     )
#     slam_toolbox = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(get_package_share_directory('nav')+'/launch/online_async_launch.py'),
#         launch_arguments={'map':map_dir+'rmuc'}.items()
#     )

#     tf_bringup = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             get_package_share_directory('bringup')+"/launch/tf.launch.py"
#         )
#     )

#     livox = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             get_package_share_directory('livox_ros_driver2')+"/launch/msg_MID360_launch.py"
#         )
#     )

#     pointlio = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             get_package_share_directory('point_lio_with_grid_map')+"/launch/point_lio.launch.py"
#         ),
#         launch_arguments={
#                         'preprocess.blind':'2.0',
#                           'point_filter_num': '4',  
#                         #   'space_down_sample': 'True',
#                           'filter_size_surf': '0.3',  
#                           'filter_size_map': '0.3',
#                           'pcd_save.pcd_save_en': 'True', 
#                           }.items()
#     )
    
#     p_to_l = LoadComposableNodes(
#         target_container='container',
#         composable_node_descriptions=[
#             ComposableNode(
#                 package='pointcloud_to_laserscan',
#                 plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
#                 name='pointcloud_to_laserscan_node',
#                 parameters=[{
#                     'target_frame': 'base_link',
#                     'transform_tolerance': 0.1,
#                     'min_height': 0.2,
#                     'max_height': 1.0,
#                     'angle_min': -3.14159,  # -M_PI/2
#                     'angle_max': 3.14159,  # M_PI/2
#                     'angle_increment': 0.0043,  # M_PI/360.0
#                     'scan_time': 0.3333,
#                     'range_min': 0.4,
#                     'range_max': 15.0,
#                     'use_inf': True,
#                     'inf_epsilon': 1.0
#                 }],
#                 remappings=[('cloud_in',  ['/segmentation/obstacle']),
#                         ('scan',  ['/scan'])],
#             )
#         ]
#     )
#     segment = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             get_package_share_directory('linefit_ground_segmentation_ros')+"/launch/segmentation.launch.py"
#         )
#     )

#     lidar_transform = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             get_package_share_directory('lidar_transform')+"/launch/lidar_transform.launch.py"
#         )
#     )

#     start_rviz_cmd = Node(
#         package='rviz2',
#         executable='rviz2',
#         arguments=['-d', rviz_config_file],
#         output='screen')

#     return LaunchDescription([
#         container,
#         # lidar_transform,
#         # tf_bringup,
#         # livox,
#         pointlio,
#         p_to_l,
#         segment,
#         # slam_toolbox,
#         start_rviz_cmd,
#     ])


def generate_launch_description():

    # sc_liorf_localization_share_dir = get_package_share_directory('sc_liorf_localization')
    # sc_liorf_localization_parameter_file = LaunchConfiguration('lio_config_params_file')
    # rviz_config_file = os.path.join(sc_liorf_localization_share_dir, 'rviz', 'mapping.rviz')

    # lio_config_file = LaunchConfiguration('lio_config_params_file')

    # sc_liorf_localization_parameter_file = PathJoinSubstitution([sc_liorf_localization_share_dir, 'config', lio_config_file]), ".yaml"

    # lio_config_declare = DeclareLaunchArgument(
    #     'lio_config_params_file',
    #     default_value='lio_sam_mid360_mapping',
    #     description='FPath to the ROS2 parameters file to use.')

    # Livox 驱动
    livox_ros_driver2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('livox_ros_driver2')+"/launch/msg_MID360_launch.py"
        )
    )

    bringup_cmd_group = GroupAction([

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        #     parameters=[sc_liorf_localization_parameter_file],
        #     output='screen'
        # ),
        # Node(
        #     package='imu_complementary_filter',
        #     executable='complementary_filter_node',
        #     name='complementary_filter_gain_node',
        #     output='screen',
        #     parameters=[
        #         {'do_bias_estimation': True},
        #         {'do_adaptive_gain': True},
        #         {'use_mag': False},
        #         {'gain_acc': 0.00001},
        #         {'gain_mag': 0.001},
        #     ],
        #     remappings=[
        #         ('/imu/data_raw', '/livox/imu'),
        #     ]
        # ),
        Node(
            package='sc_liorf_localization',
            executable='sc_liorf_localization_imuPreintegration',
            name='sc_liorf_localization_imuPreintegration',
            parameters=[sc_liorf_localization_parameter_file],
            output='screen'
        ),
        Node(
            package='sc_liorf_localization',
            executable='sc_liorf_localization_imageProjection',
            name='sc_liorf_localization_imageProjection',
            parameters=[sc_liorf_localization_parameter_file],
            output='screen'
        ),
        Node(
            package='sc_liorf_localization',
            executable='sc_liorf_localization_featureExtraction',
            name='sc_liorf_localization_featureExtraction',
            parameters=[sc_liorf_localization_parameter_file],
            output='screen'
        ),
        Node(
            package='sc_liorf_localization',
            executable='sc_liorf_localization_sc_mapOptmization',
            name='sc_liorf_localization_sc_mapOptmization',
            parameters=[sc_liorf_localization_parameter_file],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),                     
    ])

    return LaunchDescription(
        [
            
            livox_ros_driver2,
            lio_config_declare,
            TimerAction(period=4.0, actions=[bringup_cmd_group]),
            
            
        ]
    )