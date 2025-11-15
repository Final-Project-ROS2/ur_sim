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

    # --- args ---
    pddl = DeclareLaunchArgument("pddl", default_value="false")
    use_ollama = DeclareLaunchArgument(
    "use_ollama",
    default_value="false",
    description="If true, use local Ollama LLM instead of Google Gemini."
    )

    moveit_pose_action_server_node = Node(
        package='low_level_planner_executor',
        executable='moveit_pose_action_server',
        name='moveit_pose_action_server_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'real_hardware': True}],
    )
    ld.add_action(moveit_pose_action_server_node)

    low_level_planner_executor_node = Node(
        package='low_level_planner_executor',
        executable='low_level_planner_executor',
        name='low_level_planner_executor_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'real_hardware': True}],
    )
    ld.add_action(low_level_planner_executor_node)

    cartesian_path_action_server_node = Node(
        package='low_level_planner_executor',
        executable='cartesian_path_action_server',
        name='cartesian_path_action_server_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'real_hardware': True}],
    )
    ld.add_action(cartesian_path_action_server_node)

    cartesian_relative_action_server_node = Node(
        package='low_level_planner_executor',
        executable='cartesian_relative_action_server',
        name='cartesian_relative_action_server_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'real_hardware': True}],
    )
    ld.add_action(cartesian_relative_action_server_node)

    gripper_service_node = Node(
        package='gripper_helper',
        executable='gripper_service_server',
        name='gripper_service_node',
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(gripper_service_node)

    medium_level_planner_node = Node(
        package='medium_level_planner',
        executable='medium_level_planner',
        name='medium_level_planner_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            "use_ollama": LaunchConfiguration("use_ollama"),
            "real_hardware": True,
        }]
    )
    ld.add_action(medium_level_planner_node)

    high_level_planner_node = Node(
        package='high_level_planner',
        executable='high_level_planner',
        name='high_level_planner_node',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration("pddl")),
        parameters=[{
            "use_ollama": LaunchConfiguration("use_ollama"),
            "real_hardware": True,
        }]
    )
    ld.add_action(high_level_planner_node)

    high_level_planner_pddl_node = Node(
        package='high_level_pddl_planner',
        executable='pddl_planner_node',
        name='high_level_planner_pddl_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("pddl")),
        parameters=[{
            "use_ollama": LaunchConfiguration("use_ollama"),
            "real_hardware": True,
        }]
    )
    ld.add_action(high_level_planner_pddl_node)

    pddl_state_node = Node(
        package='high_level_pddl_planner',
        executable='pddl_state_node',
        name='pddl_state_node',
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(pddl_state_node)

    plan_complex_cartesian_steps_node = Node(
        package='complex_low_level_planner',
        executable='plan_complex_cartesian_steps_node',
        name='plan_complex_cartesian_steps_node',
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(plan_complex_cartesian_steps_node)

    return ld
