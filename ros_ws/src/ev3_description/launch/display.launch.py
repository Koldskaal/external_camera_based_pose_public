import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ev3_description').find('ev3_description')
    default_model_path = os.path.join(pkg_share, 'src/description/ev3_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_rviz.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    odoamne = launch_ros.actions.Node(
        package='tcp_client',
        executable='odoamne',
        name='odoamne',
        output='screen',
    )
    static_map_odom_pub = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0','0','0','0','0','0','map','odom']
    )
    params_file = os.path.join(pkg_share,'config',"nav2_params.yaml")
    map_file = os.path.join(pkg_share,'config',"map01.yaml")
    ekf_config_file = os.path.join(pkg_share,'config',"ekf.yaml")
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'false','params_file': params_file,'map': map_file}.items()
    )
    ekf = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )

    img_pose = launch_ros.actions.Node(
        package='pose_estimator',
        executable='pose_server',
        name='pose_server',
        output='screen')


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        static_map_odom_pub,
        rviz_node,
        odoamne,
        nav2_bringup,
        img_pose,
        ekf
    ])