from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import IfCondition

def generate_launch_description():

    go2_desc_pkg = get_package_share_directory("go2_description")
    go2_driver_pkg = get_package_share_directory("go2_driver")

    ## Add a switch to start rviz2
    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="false"      # Use it for debugging, close it when running SLAM, and open the corresponding rviz2 in the main program.
    )

    return LaunchDescription([
        use_rviz,
        # Robot Model Visualization
        IncludeLaunchDescription(
            launch_description_source = PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(go2_desc_pkg, "launch", "display.launch.py")
            ),
            launch_arguments=[("use_joint_state_publisher", "false")]   #Joint states are published via the driver node, so the default settings do not need to be used.
        ),
        # Include rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(go2_driver_pkg, "rviz", "display.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz"))
        ),
    
        # Speed ​​Message Bridge
        Node(
            package="go2_twist_bridge",
            executable="twist_bridge"
        ),

        # Add dynamic coordinate transformation from base_link to base_footprint
        Node(
            package="go2_driver",
            executable="footprint_to_link"
        ),


        # Odometer message release, broadcast odometer coordinates, release joint status information
        Node(
            package="go2_driver",
            executable="driver",
            parameters=[os.path.join(go2_driver_pkg, "params", "driver.yaml")]
        ),

        # imu
        Node(
            package="go2_driver",
            executable="lowstate_to_imu"
        )
    ])


