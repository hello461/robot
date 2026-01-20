from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
import os
import launch

def generate_launch_description():
    get_nav2_pkg = get_package_share_directory("go2_navigation2")
    get_bringup_pkg = get_package_share_directory("nav2_bringup")
    go2_core_pkg = get_package_share_directory("go2_core")
    go2_perception_pkg = get_package_share_directory("go2_perception")
    client_service_pkg = get_package_share_directory("client_service")
    # Get shared package for docking
    get_pose_docking_pkg = get_package_share_directory("go2_docking")

    # Paramter for simulation
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='False')

    # Map file name
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join('map1.yaml'))
    
    # Nav2 configuration file
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(get_nav2_pkg, 'config', 'nav2_params_dwb.yaml'))
    
    # Docking configuration file
    docking_param_path = launch.substitutions.LaunchConfiguration(
        'docking_file', default=os.path.join(get_pose_docking_pkg, 'config', 'dock_params.yaml'))
    
    # Rviz2 configuration file
    rviz_config_dir = os.path.join(get_nav2_pkg, 'config', 'nav2_config.rviz')

    # Launch file containing nav2
    # nav2_compose_launch = Node(
    #     package='rclcpp_components',
    #     executable='component_container',
    #     name='nav2_container'
    # ),
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_bringup_pkg, "launch", "navigation_launch.py")),
        launch_arguments=[
            ("map", map_yaml_path),
            ("params_file", nav2_param_path), 
            ("use_sim_time", use_sim_time), 
        ]
    )

    # Launch client service
    client_service_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(client_service_pkg, "launch", "pub2server.launch.py"))
    )

    # =========== map_server ===========
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'use_sim_time': use_sim_time,
            'topic_name': 'map'
        }]
    )

    # ========== AMCL ============
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_param_path, {'use_sim_time': use_sim_time}]
    )

    # ========== Docking ===========
    docking = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[docking_param_path, {'use_sim_time': use_sim_time}]
    )

    # ======== lifecycle_manager ==========
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl', 'docking_server']
        }]
    )

    # ==============Detected dock pose publisher===============
    detected_apritag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_pose_docking_pkg, "launch", "go2_docking.launch.yaml"))
    )

    # Odometer Fusion IMU
    go2_robot_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_core_pkg, "launch", "go2_robot_localization.launch.py")
            )
        )
    
    # ============= RViz2 ============
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Point cloud processing
    go2_pointcloud_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                go2_perception_pkg,
                "launch",
                "go2_pointcloud.launch.py",
            )
        )
    )

    # TF broadcaster IMU -> Base_link
    # Publish IMU message, Battery information
    lowStateToIMU = Node(
        package="go2_driver",
        executable="lowstate_to_imu",
        name="lowSate_to_IMU"
    )

    # TF broadcaster Base_link -> Base_footprint
    footprintToLink = Node(
        package="go2_driver",
        executable="footprint_to_link",
        name="footprint_to_link"
    )

    # TF broadcaster Base_footprint -> Odom
    # Publish PostStamp Base_footprint -> Odom
    # (Not run) Publish Joint status
    driver = Node(
        package="go2_driver",
        executable="driver",
        name="driver"
    )

    # Real robot control
    robot_control = Node(
        package="go2_control",
        executable="go2_control"
    )

    # Initialize pos
    init_pos = Node(
        package="go2_control",
        executable="init_position"
    )

    return LaunchDescription([
        driver,
        footprintToLink,
        lowStateToIMU,
        robot_control,
        docking,
        map_server,
        amcl,
        lifecycle_manager,
        nav2_launch,
        go2_robot_localization,
        rviz2,
        go2_pointcloud_launch,
        # detected_apritag_launch,
        # client_service_launch
        # init_pos,
    ])
