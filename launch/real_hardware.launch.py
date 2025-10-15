import os 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # --- UR Driver Launch ---
    ur_driver_launch_file = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'launch',
        'ur_control.launch.py'
    )

    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_driver_launch_file),
        launch_arguments={
            'ur_type': 'ur3e',
            'robot_ip': '10.10.0.60',
            'kinematics_params_file': os.path.join(os.environ['HOME'], 'my_robot_calibration.yaml')
        }.items()
    )

    # --- UR MoveIt Launch ---
    ur_moveit_launch_file = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'launch',
        'ur_moveit.launch.py'
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_launch_file),
        launch_arguments={
            'ur_type': 'ur3e',
            'launch_rviz': 'true',
            'use_sim_time': 'false'
        }.items()
    )

    # --- Custom Nodes ---
    custom_node_launch_file = os.path.join(
        get_package_share_directory('ur_yt_sim'),
        'launch',
        'custom_node_real_hardware.launch.py'
    )

    custom_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_node_launch_file)
    )

    return LaunchDescription([
        ur_driver_launch,
        ur_moveit_launch,
        custom_node_launch
    ])