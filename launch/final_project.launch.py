from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AndSubstitution
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, NotEqualsSubstitution
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
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
    world_file = PathJoinSubstitution([
        get_package_share_directory("ur_yt_sim"),
        "worlds",
        LaunchConfiguration("world_file")
    ])

    # --- ENV Gazebo ---
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=":".join(["/usr/share/gazebo-11", uryt_share, robotiq_share, ur_share])
    ))
    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=":".join([
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
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="sim",
        description="Mode: 'sim' (simulation), 'cam' (real camera), 'real' (real hardware)"
    )
    with_rviz     = DeclareLaunchArgument("with_rviz", default_value="true")
    with_octomap  = DeclareLaunchArgument("with_octomap", default_value="true")  # << NEW
    pddl = DeclareLaunchArgument("pddl", default_value="false")
    world_arg = DeclareLaunchArgument("world_file", default_value="test_world_find_object.world")
    use_ollama = DeclareLaunchArgument(
        "use_ollama",
        default_value="false",
        description="If true, use local Ollama LLM instead of Google Gemini."
    )
    ollama_model = DeclareLaunchArgument(
        "ollama_model",
        default_value="qwen3:8b"
    )
    real_hardware = DeclareLaunchArgument(
        "real_hardware",
        default_value=PythonExpression([
            "'true' if '", LaunchConfiguration("mode"), "' == 'real' else 'false'"
        ])
    )
    real_camera = DeclareLaunchArgument(
        "real_camera",
        default_value=PythonExpression([
            "'true' if '", LaunchConfiguration("mode"), "' in ['cam', 'real'] else 'false'"
        ])
    )
    confirm = DeclareLaunchArgument("confirm", default_value="true")
    tcp_offset = DeclareLaunchArgument("tcp_offset", default_value="false")
    
    # PDDL initial state args
    is_home_arg = DeclareLaunchArgument("is_home", default_value="true")
    is_ready_arg = DeclareLaunchArgument("is_ready", default_value="false")
    is_handover_arg = DeclareLaunchArgument("is_handover", default_value="false")
    gripper_is_open_arg = DeclareLaunchArgument("gripper_is_open", default_value="true")

    # Add declare statements
    x_arg = DeclareLaunchArgument("x", default_value="0")
    y_arg = DeclareLaunchArgument("y", default_value="0")
    z_arg = DeclareLaunchArgument("z", default_value="0")
    ld.add_action(mode_arg)
    ld.add_action(with_rviz); ld.add_action(with_octomap)
    ld.add_action(pddl)
    ld.add_action(real_hardware)
    ld.add_action(real_camera)
    ld.add_action(tcp_offset)
    ld.add_action(world_arg)
    ld.add_action(use_ollama)
    ld.add_action(ollama_model)
    ld.add_action(confirm)
    ld.add_action(is_home_arg)
    ld.add_action(is_ready_arg)
    ld.add_action(is_handover_arg)
    ld.add_action(gripper_is_open_arg)
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

    # --- Simulation Launch Nodes ---

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")),
        launch_arguments={"use_sim_time":"true", "gui":"true", "paused":"true", "world": world_file}.items(),
        # launch_arguments={"use_sim_time":"true", "gui":"true", "paused":"true"}.items()
        condition=UnlessCondition(LaunchConfiguration("real_hardware"))
    )
    ld.add_action(gazebo)

    # --- RSP ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("real_hardware"))
    )
    ld.add_action(robot_state_publisher)

    # --- Spawn ---
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity","cobot",
            "-topic","robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("real_hardware"))
    )
    ld.add_action(TimerAction(period=3.0, actions=[spawn]))

    jsb  = Node(package="controller_manager", executable="spawner",
                arguments=["joint_state_broadcaster","--controller-manager","/controller_manager"], output="screen",
                condition=UnlessCondition(LaunchConfiguration("real_hardware"))
            )
    arm  = Node(package="controller_manager", executable="spawner",
                arguments=["joint_trajectory_controller","--controller-manager","/controller_manager"], output="screen",
                condition=UnlessCondition(LaunchConfiguration("real_hardware"))
            )
    grip = Node(package="controller_manager", executable="spawner",
                arguments=["gripper_position_controller","--controller-manager","/controller_manager"], output="screen",
                condition=UnlessCondition(LaunchConfiguration("real_hardware"))
            )

    ld.add_action(RegisterEventHandler(
        OnProcessStart(target_action=spawn, on_start=[
            TimerAction(period=2.0, actions=[jsb]),
            TimerAction(period=3.0, actions=[arm, grip]),
        ])
    ))

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
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration("with_rviz"),
                NotEqualsSubstitution(LaunchConfiguration("real_hardware"), "true")
            )
        )
    )
    ld.add_action(rviz)

    mg_params = moveit_config.to_dict()
    mg_params.update({"use_sim_time": True})

    mg_params_no_sensors = dict(mg_params)
    mg_params_no_sensors.pop("sensors", None)

    move_group_no_octomap = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[mg_params_no_sensors],
        arguments=["--ros-args","--log-level","info"],
        condition=UnlessCondition(LaunchConfiguration("real_hardware"))
    )

    ld.add_action(move_group_no_octomap)

    gripper_wrapper = Node(
            package='gripper_helper',
            executable='wrapper_gripper_node',
            name='gripper_wrapper_node',
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(LaunchConfiguration("real_hardware"))
    )
    ld.add_action(gripper_wrapper)

    contact_listener = Node(
            package='gripper_helper',
            executable='contact_listener',
            name='contact_listener_node',
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(LaunchConfiguration("real_hardware"))
    )
    ld.add_action(contact_listener)

    # --- Real Hardware Launch Nodes ---

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
        }.items(),
        condition=IfCondition(LaunchConfiguration("real_hardware"))
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
        }.items(),
        condition=IfCondition(LaunchConfiguration("real_hardware"))
    )
    ld.add_action(ur_moveit_launch)

    gripper_service_node = Node(
        package='gripper_helper',
        executable='gripper_service_server',
        name='gripper_service_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("real_hardware")),
    )
    ld.add_action(gripper_service_node)

    moveit_pose_action_server_node = Node(
        package='low_level_planner_executor',
        executable='moveit_pose_action_server',
        name='moveit_pose_action_server_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": NotEqualsSubstitution(
                    LaunchConfiguration("real_hardware"), "true"
                ),
                "real_hardware": LaunchConfiguration("real_hardware"),
            }
        ],
    )
    ld.add_action(moveit_pose_action_server_node)

    low_level_planner_executor_node = Node(
        package='low_level_planner_executor',
        executable='low_level_planner_executor',
        name='low_level_planner_executor_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": NotEqualsSubstitution(
                    LaunchConfiguration("real_hardware"), "true"
                ),
                "real_hardware": LaunchConfiguration("real_hardware"),
            }
        ],
    )
    ld.add_action(low_level_planner_executor_node)

    cartesian_path_action_server_node = Node(
        package='low_level_planner_executor',
        executable='cartesian_path_action_server',
        name='cartesian_path_action_server_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": NotEqualsSubstitution(
                    LaunchConfiguration("real_hardware"), "true"
                ),
                "real_hardware": LaunchConfiguration("real_hardware"),
            }
        ],
    )
    ld.add_action(cartesian_path_action_server_node)

    cartesian_relative_action_server_node = Node(
        package='low_level_planner_executor',
        executable='cartesian_relative_action_server',
        name='cartesian_relative_action_server_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": NotEqualsSubstitution(
                    LaunchConfiguration("real_hardware"), "true"
                ),
                "real_hardware": LaunchConfiguration("real_hardware"),
            }
        ],
    )
    ld.add_action(cartesian_relative_action_server_node)

    medium_level_planner_node = Node(
        package='medium_level_planner',
        executable='medium_level_planner',
        name='medium_level_planner_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": NotEqualsSubstitution(
                    LaunchConfiguration("real_hardware"), "true"
                ),
                "real_hardware": LaunchConfiguration("real_hardware"),
                "use_ollama": LaunchConfiguration("use_ollama"),
            }
        ],
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
            "real_hardware": LaunchConfiguration("real_hardware"),
            "confirm": LaunchConfiguration("confirm"),
            "ollama_model": LaunchConfiguration("ollama_model"),
        }]
    )

    high_level_planner_pddl_node = Node(
        package='high_level_pddl_planner',
        executable='pddl_planner_node',
        name='high_level_planner_pddl_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("pddl")),
        parameters=[{
            "use_ollama": LaunchConfiguration("use_ollama"),
            "real_hardware": LaunchConfiguration("real_hardware"),
            "confirm": LaunchConfiguration("confirm"),
        }]
    )

    pddl_state_node = Node(
        package='high_level_pddl_planner',
        executable='pddl_state_node',
        name='pddl_state_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            "is_home": LaunchConfiguration("is_home"),
            "is_ready": LaunchConfiguration("is_ready"),
            "is_handover": LaunchConfiguration("is_handover"),
            "gripper_is_open": LaunchConfiguration("gripper_is_open"),
        }]
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

    simple_sam_detector_node = Node(
        package='vision',
        executable='simple_sam_detector',
        name='simple_sam_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "real_hardware": LaunchConfiguration("real_camera"),
            }
        ],
    )
    ld.add_action(simple_sam_detector_node)

    clip_classifier_node = Node(
        package='vision',
        executable='clip_classifier',
        name='clip_classifier_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "real_hardware": LaunchConfiguration("real_camera"),
            }
        ],
    )
    ld.add_action(clip_classifier_node)

    graspnet_detector_node = Node(
        package='vision',
        executable='graspnet_detector',
        name='graspnet_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "real_hardware": LaunchConfiguration("real_camera"),
            }
        ],
    )
    ld.add_action(graspnet_detector_node)

    scene_understanding_node = Node(
        package='vision',
        executable='scene_understanding',
        name='scene_understanding_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "real_hardware": LaunchConfiguration("real_camera"),
            }
        ],
    )
    ld.add_action(scene_understanding_node)

    vqa_action_server_node_sim = Node(
        package='vision',
        executable='vqa_action_server',
        name='vqa_action_server_node',
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration("real_camera")),
        parameters=[
            {
                "real_hardware": LaunchConfiguration("real_camera"),
                "image_reliability": "reliable",
            }
        ],
    )

    vqa_action_server_node_real = Node(
        package='vision',
        executable='vqa_action_server',
        name='vqa_action_server_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("real_camera")),
        parameters=[
            {
                "real_hardware": LaunchConfiguration("real_camera"),
            }
        ],
    )

    depth_camera_publisher_node = Node(
        package='depth_camera',
        executable='intel_pub',
        name='depth_camera_publisher_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("real_camera")),
        parameters=[
            {
                "logging": False,
            }
        ]
    )
    ld.add_action(depth_camera_publisher_node)

    # Start VQA after camera begins publishing when using a real camera; otherwise delay slightly.
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=depth_camera_publisher_node,
            on_start=[TimerAction(period=2.0, actions=[vqa_action_server_node_real])],
        )
    ))

    ld.add_action(TimerAction(
        period=15.0,
        actions=[vqa_action_server_node_sim],
        condition=UnlessCondition(LaunchConfiguration("real_camera")),
    ))

    # Launch high-level planners only after VQA is up.
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=vqa_action_server_node_real,
            on_start=[
                TimerAction(period=10.0, actions=[high_level_planner_node]),
                TimerAction(period=10.0, actions=[high_level_planner_pddl_node]),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("real_camera")),
    ))

    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=vqa_action_server_node_sim,
            on_start=[
                TimerAction(period=10.0, actions=[high_level_planner_node]),
                TimerAction(period=10.0, actions=[high_level_planner_pddl_node]),
            ],
        ),
        condition=UnlessCondition(LaunchConfiguration("real_camera")),
    ))

    pixel_to_real_node = Node(
        package='vision',
        executable='pixel_to_real_service',
        name='pixel_to_real_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "real_hardware": LaunchConfiguration("real_camera"),
            }
        ],
        condition=UnlessCondition(LaunchConfiguration("real_camera")),
    )
    ld.add_action(pixel_to_real_node)

    pixel_to_real_world_node = Node(
        package='vision',
        executable='pixel_to_real_world_service',
        name='pixel_to_real_world_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("real_camera")),
    )
    ld.add_action(pixel_to_real_world_node)

    find_object_node = Node(
        package='vision',
        executable='find_object_service',
        name='find_object_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "tcp_offset": LaunchConfiguration("tcp_offset"),
            }
        ]
    )
    ld.add_action(find_object_node)

    find_object_grasp_node = Node(
        package='vision',
        executable='find_object_grasp_service',
        name='find_object_grasp_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                "tcp_offset": LaunchConfiguration("tcp_offset"),
            }
        ]
    )
    ld.add_action(find_object_grasp_node)

    real_cam_info_publisher = Node(
        package='vision',
        executable='real_cam_info',
        name='real_cam_info_publisher_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("real_hardware"))
    )
    ld.add_action(real_cam_info_publisher)

    return ld
