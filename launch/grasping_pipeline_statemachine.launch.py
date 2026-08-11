import sys

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from moveit_configs_utils import MoveItConfigsBuilder
import os
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetParameter
from launch.actions import TimerAction



sys.path.append(os.path.dirname(__file__))  # ensures current launch folder is in path
# Toyota's robot_description parser
try:
    import robot_description
except ImportError:
    sys.path.append(os.path.dirname(__file__))
    import robot_description

try:
    from utils import (
        get_full_path,
        load_file,
        load_yaml,
    )
except ImportError:
    sys.path.append(os.path.dirname(__file__))
    from utils import (
        get_full_path,
        load_file,
        load_yaml,
    )

def generate_launch_description():
    """Launch description for running the state‑machine tester node.

    Usage:
        ros2 launch grasping_pipeline statemachine_tester.launch.py
    """
    description_package_str = 'hsrb_description'
    description_file_str = 'hsrb4s.urdf.xacro'
    kinematics_yaml = load_yaml('config/kinematics.yaml')
    sensors_yaml = load_yaml('config/sensors_xtion.yaml')

    ompl_planning_pipeline_config = {
    "default_planning_pipeline": "ompl",

    "planning_pipelines": {
        "pipeline_names": ["ompl"],
    },
    "ompl": { 
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": " ".join([
            "default_planner_request_adapters/AddTimeOptimalParameterization",
            "default_planner_request_adapters/FixWorkspaceBounds",
            "default_planner_request_adapters/FixStartStateBounds",
            "default_planner_request_adapters/FixStartStateCollision",
            "default_planner_request_adapters/FixStartStatePathConstraints",
        ]),
        "start_state_max_bounds_error": 0.1,
    }}


    # Merge ompl_planning.yaml contents
    ompl_planning_pipeline_config.update(load_yaml("config/ompl_planning.yaml"))

    move_group_ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join(['default_planner_request_adapters/AddTimeOptimalParameterization',
                                          'default_planner_request_adapters/FixWorkspaceBounds',
                                          'default_planner_request_adapters/FixStartStateBounds',
                                          'default_planner_request_adapters/FixStartStateCollision',
                                          'default_planner_request_adapters/FixStartStatePathConstraints']),
            'start_state_max_bounds_error': 0.1}}
    move_group_ompl_planning_pipeline_config['move_group'].update(load_yaml('config/ompl_planning.yaml'))
    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': load_yaml('config/hsrb_controllers.yaml')}
    
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }

    moveit_robot_description_xml = robot_description.parse(description_package_str, description_file_str)
    moveit_robot_description = {"robot_description": moveit_robot_description_xml}
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="hsrb",   # must match SRDF robot name
            package_name="hsrb_moveit_config",
        )
        .planning_pipelines("ompl", ["ompl"])
        .moveit_cpp(
            get_package_share_directory("grasping_pipeline")
            + "/config/conf_moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )
    moveit_dict = moveit_config.to_dict()
    moveit_dict.update(sensors_yaml)
    moveit_dict.update(move_group_ompl_planning_pipeline_config)
    moveit_dict.update(moveit_controllers)
    moveit_dict.update(trajectory_execution)
    moveit_dict.update(planning_scene_monitor_parameters)
    moveit_dict.update(moveit_robot_description)
    # Inject into moveit dict properly
    moveit_dict.update(ompl_planning_pipeline_config)
    moveit_dict.update(kinematics_yaml)
    

    robot_description_semantic = {'robot_description_semantic': load_file('config/hsrb.srdf')}
    robot_description_planning = {'robot_description_planning': load_yaml('config/joint_limits.yaml')}
    moveit_dict.update(robot_description_semantic)
    moveit_dict.update(robot_description_planning)
    
    odom_joint_states_publisher = Node(package='hsrb_moveit_config',
                                       executable='odom_joint_states_publisher.py',
                                       name='odom_joint_states_publisher',
                                       parameters=[{'use_sim_time': True}],
                                       remappings=[('odom_joint_states', '/whole_body/joint_states')]
                                    )
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='screen',
                     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map'],
                     parameters=[{'use_sim_time': True}],
                    )
    
    package_share_directory = get_package_share_directory('table_plane_extractor')
    config_file = os.path.join(package_share_directory, 'config', 'config.yaml')
    table_config_file = os.path.join(package_share_directory, 'config', 'config.yaml')
    params_file = os.path.join(
    get_package_share_directory('grasping_pipeline'),
        'config',
        'config.yaml'
    )

    delayed_start_state_machine = TimerAction(
        period=5.0,   # delay in seconds
        actions=[
            Node(
                package='grasping_pipeline',
                executable='statemachine',
                name='statemachine',
                output='screen',
                emulate_tty=True,
                parameters=[moveit_dict, params_file, {'use_sim_time': True},table_config_file],
            )
        ]
    )
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),        
        delayed_start_state_machine
    ])