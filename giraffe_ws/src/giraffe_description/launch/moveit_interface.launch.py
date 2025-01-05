
import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory, get_package_share_path

from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


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
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('giraffe_description'),
                    'urdf',
                    'giraffe_hardware.xacro.urdf',
                ]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    giraffe_controllers = PathJoinSubstitution(
        [
            FindPackageShare('giraffe_description'),
            'config',
            'giraffe_hardware.yaml',
        ]
    )

    # node for controller manager which manages lifecycle of controllers defined for joints
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, giraffe_controllers],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # node which publishes robot state including the robot description
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # node to broadcasting joint states of various joints listed in urdf and configuration
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # node to start controllers associated with the joints
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
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

    package_share = get_package_share_path('giraffe_description')
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

    rviz_base = os.path.join(get_package_share_directory("giraffe_moveit_config"), "config")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
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

    # delay start of robot_controller after `joint_state_broadcaster`
    delay_motor_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    # delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node_full],
        )
    )

    giraffe_driver = Node(
        package='giraffe_control',
        executable='giraffe_driver',
        output='screen',
    )
    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_motor_controller_spawner_after_joint_state_broadcaster_spawner,
        TimerAction(
            period=5.0,
            actions=[run_move_group_node]
        ),
        giraffe_driver
    ])
