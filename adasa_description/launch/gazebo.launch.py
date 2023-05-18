import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():

    package_share = get_package_share_directory('adasa_description')
    # URDF file
    urdf_file_name = 'urdf/adasa.xacro'
    model_path = os.path.join(package_share, urdf_file_name)
    robot_description_config = xacro.process_file(model_path)
    urdf = robot_description_config.toxml()

    # Rviz config file
    rviz_file_name = '/home/arpit/ros2/adasa_ws/src/adasa_rover/adasa_description/rviz/adasa.rviz'
    rviz_config = rviz_file_name  # os.path.join(package_share, rviz_file_name)

    # World file
    world_file_name = 'empty.world'
    world_path = os.path.join(package_share, 'worlds', world_file_name)

    # Map file
    map_file_name = 'map.yaml'
    map_path = os.path.join(package_share, 'maps', map_file_name)

    world = LaunchConfiguration('world')
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load'
    )

    declare_map_cmd = DeclareLaunchArgument(
        name='map',
        default_value=map_path,
        description='Full path to map yaml file to load'
    )

    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py'
            ])
        ]),
        launch_arguments={'map': map_path}.items()
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'world': world}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': urdf
        }, {'use_sim_time': True}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", "adasa"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_world_cmd,
        map_server,
        declare_sim_time_cmd,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        joint_state_publisher,
        # joint_state_publisher_gui,
        spawn_entity,
        rviz2
    ])
