from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    ld = LaunchDescription()

    # --- share dirs ---
    uryt_share     = get_package_share_directory("ur_yt_sim")
    robotiq_share  = get_package_share_directory("robotiq_description")
    ur_share       = get_package_share_directory("ur_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    world_file = os.path.join(get_package_share_directory('ur_yt_sim'), 'worlds', 'world5.world')

    cartesian_relative_action_server_node = Node(
        package='low_level_planner_executor',
        executable='cartesian_relative_action_server',
        name='cartesian_relative_action_server_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'real_hardware': True}],
    )
    ld.add_action(cartesian_relative_action_server_node)

    return ld
