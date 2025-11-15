import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # --- Declare Args ---
    pddl_arg = DeclareLaunchArgument("pddl", default_value="false")
    use_ollama_arg = DeclareLaunchArgument(
        "use_ollama",
        default_value="false",
        description="If true, use local Ollama LLM instead of Google Gemini."
    )
    real_hardware_arg = DeclareLaunchArgument("real_hardware", default_value="true")

    # Add declare statements
    ld.add_action(pddl_arg)
    ld.add_action(use_ollama_arg)
    ld.add_action(real_hardware_arg)

    # LaunchConfigurations for forwarding
    pddl = LaunchConfiguration("pddl")
    use_ollama = LaunchConfiguration("use_ollama")
    real_hardware = LaunchConfiguration("real_hardware")

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
    ld.add_action(ur_driver_launch)

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
    ld.add_action(ur_moveit_launch)

    # --- Custom Node Launch ---
    custom_node_launch_file = os.path.join(
        get_package_share_directory('ur_yt_sim'),
        'launch',
        'custom_node_real_hardware.launch.py'
    )

    custom_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_node_launch_file),
        launch_arguments={
            'pddl': pddl,
            'use_ollama': use_ollama,
            'real_hardware': real_hardware,
        }.items()
    )
    ld.add_action(custom_node_launch)

    return ld
