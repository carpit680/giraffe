import os
import yaml
import xacro
from os import pathsep
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory, get_package_prefix, get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
from launch.actions import AppendEnvironmentVariable


# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

def generate_launch_description():
    giraffe_description = get_package_share_directory('giraffe_description')
    package_share = get_package_share_path('giraffe_description')

    giraffe_description_share = get_package_prefix('giraffe_description')
    gazebo_ros_dir = get_package_share_directory('ros_gz_sim')

    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
                                        giraffe_description, 'urdf', 'giraffe.urdf.xacro'
                                        ),
                                      description='Absolute path to robot urdf file'
    )

    model_path = os.path.join(giraffe_description, "models")
    model_path += pathsep + os.path.join(giraffe_description_share, "share")

    env_var = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gz_sim.launch.py')
        )
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gz_sim.launch.py')
        )
    )

    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-file', os.path.join(giraffe_description, 'models', 'giraffe.urdf.xacro'),
                '-name', 'giraffe',
                '-allow_renaming', 'true'],
        output='screen'
)

    

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ 
                    {'use_sim_time': True},
                    os.path.join(get_package_share_directory("giraffe_description"), "config", "giraffe_controller.yaml")],
        output="screen",
    )
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'arm_controller'],
        output='screen')
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'gripper_controller'],
        output='screen')
    
    # Delay start of robot controllers
    delay_controllers_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[load_joint_trajectory_controller, load_gripper_controller],
        )
    )
    # Diagnostic node to list controllers
    list_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'list_controllers'],
        output='screen',
        shell=True
    )

    # *********************** MoveIt!2 *********************** #   
    
    # Command-line argument: RVIZ file?
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", default_value="False", description="Load RVIZ file."
    )

    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF:
    robot_description_semantic_config = load_file("giraffe_moveit_config", "config/giraffe.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config }
    
    # Kinematics.yaml file:
    kinematics_yaml = load_yaml("giraffe_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": [
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
                "default_planner_request_adapters/FixStartStatePathConstraints",
            ],
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml("giraffe_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml("giraffe_moveit_config", "config/moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "use_sim_time": True,
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    # sensors_yaml = load_yaml("giraffe_moveit_config", "config/sensors_3d.yaml")
    planning_scene_monitor_parameters = {
        "planning_scene_monitor": {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        }
    }

    joint_limits_yaml = load_yaml("giraffe_moveit_config", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}
    model_path = os.path.join(package_share, 'urdf', 'giraffe.urdf.xacro')
    robot_description_config = xacro.process_file(model_path)
    robot_description_config = robot_description_config.toxml()
    robot_description_moveit = {'robot_description': robot_description_config}
    moveit_config = (
        MoveItConfigsBuilder("giraffe")
        .robot_description(file_path="config/giraffe.urdf.xacro")
        .robot_description_semantic(file_path="config/giraffe.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # START NODE -> MOVE GROUP:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # Delay move_group node until controllers are up
    delay_move_group_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[run_move_group_node]
                )
            ]
        )
    )
    # RVIZ:
    rviz_base = os.path.join(get_package_share_directory("giraffe_moveit_config"), "config")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # namespace="moveit",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description_moveit,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ]
    )

    return LaunchDescription([
        env_var,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
        rviz_node_full,
        robot_state_publisher_node,
        delay_controllers_after_joint_state_broadcaster_spawner,
        # delay_move_group_after_controllers,
        TimerAction(
            period=5.0,
            actions=[controller_manager]
        ),
        TimerAction(
            period=10.0,
            actions=[joint_state_broadcaster_spawner]
        ),
        TimerAction(
            period=5.0,
            actions=[run_move_group_node]
        ),
        TimerAction(
            period=15.0,
            actions=[list_controllers]
        )
    ])
