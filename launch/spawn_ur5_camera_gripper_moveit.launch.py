from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction, AppendEnvironmentVariable,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
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
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    world_file = os.path.join(get_package_share_directory('ur_yt_sim'), 'worlds', 'world3.sdf')

    # --- ENV Gazebo ---
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=":".join(["/usr/share/gz", uryt_share, robotiq_share, ur_share])
    ))
    # ld.add_action(SetEnvironmentVariable(
    #     name="GAZEBO_MODEL_PATH",
    #     value=":".join([
    #         os.path.join(uryt_share,"models"),
    #         os.path.join(robotiq_share,"models"),
    #         os.path.expanduser("~/.gazebo/models")
    #     ])
    # ))
    ld.add_action(AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        ":".join([
            os.path.join(uryt_share,"models"),
            os.path.join(robotiq_share,"models"),
            os.path.expanduser("~/.gazebo/models")
        ])
    ))
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=":".join([
            "/opt/ros/humble/lib",
            os.path.normpath(os.path.join(uryt_share, "..", "..", "lib")),
        ])
    ))

    # --- args ---
    with_rviz     = DeclareLaunchArgument("with_rviz", default_value="true")
    with_octomap  = DeclareLaunchArgument("with_octomap", default_value="true")  # << NEW
    x_arg = DeclareLaunchArgument("x", default_value="0")
    y_arg = DeclareLaunchArgument("y", default_value="0")
    z_arg = DeclareLaunchArgument("z", default_value="0")
    ld.add_action(with_rviz); ld.add_action(with_octomap)
    ld.add_action(x_arg); ld.add_action(y_arg); ld.add_action(z_arg)

    # --- MoveIt config ---
    joint_controllers_file = os.path.join(uryt_share, "config", "ur5_controllers_gripper.yaml")
    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur5_camera_gripper_moveit_config")
        .robot_description(
            file_path="config/ur.urdf.xacro",
            mappings={
                "ur_type": "ur5",
                "sim_gazebo": "true",
                "sim_ignition": "false",
                "use_fake_hardware": "false",
                "simulation_controllers": joint_controllers_file,
                "initial_positions_file": os.path.join(uryt_share, "config", "initial_positions.yaml"),
            },
        )
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True
        )
        .to_moveit_configs()
    )

    # --- Gazebo ---
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
    #     # launch_arguments={"use_sim_time":"true", "gui":"true", "paused":"true", "world": world_file}.items()
    #     launch_arguments={'gz_args': ['-r -g -v4'], 'on_exit_shutdown': 'true'}.items(),
    #     # launch_arguments={"use_sim_time":"true", "gui":"true", "paused":"true"}.items()
    # )
    # ld.add_action(gazebo)

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world model file to load'
    )

    ld.add_action(world_arg)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', ], 'on_exit_shutdown': 'true'}.items(),
    )
    ld.add_action(gazebo)

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={'gz_args': '-g -v4'}.items(),
    )
    ld.add_action(gzclient_cmd)

    # --- RSP ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="screen",
    )
    ld.add_action(robot_state_publisher)

    # --- Spawn ---
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name","cobot",
            "-topic","robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )
    ld.add_action(TimerAction(period=3.0, actions=[spawn]))

    # jsb  = Node(package="controller_manager", executable="spawner",
    #             arguments=["joint_state_broadcaster","--controller-manager","/controller_manager"], output="screen")
    # arm  = Node(package="controller_manager", executable="spawner",
    #             arguments=["joint_trajectory_controller","--controller-manager","/controller_manager"], output="screen")
    # grip = Node(package="controller_manager", executable="spawner",
    #             arguments=["gripper_position_controller","--controller-manager","/controller_manager"], output="screen")
    jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    arm = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    grip = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # ld.add_action(RegisterEventHandler(
    #     OnProcessStart(target_action=spawn, on_start=[
    #         TimerAction(period=2.0, actions=[jsb]),
    #         TimerAction(period=3.0, actions=[arm, grip]),
    #     ])
    # ))
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[
            jsb, arm, grip
        ])
    ))

    bridge_params = os.path.join(get_package_share_directory("ur_yt_sim"), "config", "gz_bridge.yaml")
    
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ld.add_action(ros_gz_bridge)

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    ld.add_action(ros_gz_image_bridge)

    rviz_cfg = os.path.join(get_package_share_directory("ur5_camera_gripper_moveit_config"),
                            "config", "moveit.rviz")
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        condition=IfCondition(LaunchConfiguration("with_rviz"))
    )
    ld.add_action(rviz)

    mg_params = moveit_config.to_dict()
    mg_params.update({"use_sim_time": True})

    move_group_with_octomap = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[mg_params],
        arguments=["--ros-args","--log-level","info"],
        condition=IfCondition(LaunchConfiguration("with_octomap")),   # << NEW
    )

    mg_params_no_sensors = dict(mg_params)
    mg_params_no_sensors.pop("sensors", None)

    move_group_no_octomap = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[mg_params_no_sensors],
        arguments=["--ros-args","--log-level","info"],
        condition=UnlessCondition(LaunchConfiguration("with_octomap")),
    )

    

    ld.add_action(move_group_with_octomap)
    ld.add_action(move_group_no_octomap)

    return ld