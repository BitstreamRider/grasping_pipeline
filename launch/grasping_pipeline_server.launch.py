import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
import os
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

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
    """Launch description for running the grasping_pipeline servers.

    Usage:
        ros2 launch grasping_pipeline grasping_pipeline_servers.launch.py
    """
    
    description_package_str = 'hsrb_description'
    description_file_str = 'hsrb4s.urdf.xacro'
    kinematics_yaml = load_yaml('./config/kinematics.yaml', package_name='grasping_pipeline')
    sensors_yaml = load_yaml('./config/sensors_xtion.yaml', package_name='hsrb_moveit_config')

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
    ompl_planning_pipeline_config.update(load_yaml("config/ompl_planning.yaml", package_name="hsrb_moveit_config"))

    move_group_ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ' '.join(['default_planner_request_adapters/AddTimeOptimalParameterization',
                                          'default_planner_request_adapters/FixWorkspaceBounds',
                                          'default_planner_request_adapters/FixStartStateBounds',
                                          'default_planner_request_adapters/FixStartStateCollision',
                                          'default_planner_request_adapters/FixStartStatePathConstraints']),
            'start_state_max_bounds_error': 0.1}}
    move_group_ompl_planning_pipeline_config['move_group'].update(load_yaml('config/ompl_planning.yaml', package_name='hsrb_moveit_config'))
    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': load_yaml('config/hsrb_controllers.yaml', package_name='hsrb_moveit_config')}
    
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
    # moveit_config = (
    #     MoveItConfigsBuilder(
    #         robot_name="hsrb",   # must match SRDF robot name
    #         package_name="hsrb_moveit_config",
    #     )
    #     .planning_pipelines("ompl", ["ompl"])
    #     .moveit_cpp(
    #         get_package_share_directory("grasping_pipeline")
    #         + "/config/conf_moveit_cpp.yaml"
    #     )
    #     .to_moveit_configs()
    # )
    # moveit_dict = moveit_config.to_dict()
    # moveit_dict.update(sensors_yaml)
    # moveit_dict.update(move_group_ompl_planning_pipeline_config)
    # moveit_dict.update(moveit_controllers)
    # moveit_dict.update(trajectory_execution)
    # moveit_dict.update(planning_scene_monitor_parameters)
    # moveit_dict.update(moveit_robot_description)
    # # Inject into moveit dict properly
    # moveit_dict.update(ompl_planning_pipeline_config)
    # moveit_dict.update(kinematics_yaml)
    

    robot_description_semantic = {'robot_description_semantic': load_file('./config/hsrb.srdf', package_name='hsrb_moveit_config')}
    robot_description_planning = {'robot_description_planning': load_yaml('./config/joint_limits.yaml', package_name='hsrb_moveit_config')}
    # moveit_dict.update(robot_description_semantic)
    # moveit_dict.update(robot_description_planning)
    robot_name={"robot_name": "hsrb"}
    move_group_node = Node(package='moveit_ros_move_group',
                           executable='move_group',
                           output='screen',
                           parameters=[moveit_robot_description,
                                       robot_description_semantic,
                                       robot_description_planning,
                                       kinematics_yaml,
                                       sensors_yaml,
                                       robot_name,
                                       move_group_ompl_planning_pipeline_config,
                                       trajectory_execution,
                                       moveit_controllers,
                                       planning_scene_monitor_parameters,
                                       {'use_sim_time': True}],
                           remappings=[('joint_states', '/whole_body/joint_states')]
                           )
    
    params_file = os.path.join(
    get_package_share_directory('grasping_pipeline'),
        'config',
        'config.yaml'
    )

    table_plane_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('table_plane_extractor'),
                'launch',
                'table_plane_extractor.launch.py'
            )
        )
    )
    image_fetcher = Node(
            package='grasping_pipeline',
            executable='image_fetcher',
            name='image_fetcher',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}],
        )
    object_detector = Node(
            package='grasping_pipeline',
            executable='object_detector',
            name='object_detector',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}],
        )
    pose_estimator = Node(
            package='grasping_pipeline',
            executable='pose_estimator',
            name='pose_estimator',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}],
        )
    find_grasppoint_server =  Node(
            package='grasping_pipeline',
            executable='find_grasppoint_server',
            name='find_grasppoint_server',
            output='screen',
            parameters=[params_file, {'model_dir': '/root/ros2_ws/src/grasping_pipeline/models'}, {'use_sim_time': True}],
        )
    execute_grasp_server = Node(
            package='grasping_pipeline',
            executable='execute_grasp_server',
            name='execute_grasp_server',
            output='screen',
            parameters=[
                        moveit_robot_description,
                        robot_description_semantic,
                        robot_description_planning,
                        kinematics_yaml,
                        sensors_yaml,
                        robot_name,
                         params_file, {'use_sim_time': True}],
                         remappings=[('joint_states', '/whole_body/joint_states')],
    )

    visualizer = Node(
        package='grasping_pipeline',
        executable='visualizer',
        name='visualizer',
        output='screen',
        arguments=['/root/ros2_ws/src/grasping_pipeline/models']
    )

    odom_joint_states_publisher = Node(package='hsrb_moveit_config',
                                       executable='odom_joint_states_publisher.py',
                                       name='odom_joint_states_publisher',
                                       parameters=[{'use_sim_time': True}],
                                       remappings=[('odom_joint_states', '/whole_body/joint_states')]
                                    )
    
    place = Node(
        package='grasping_pipeline',
        executable='place',
        name='place_object_server',
        output='screen',
        parameters=[
            moveit_robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
            sensors_yaml,
            robot_name,
            params_file, {'use_sim_time': True}],
            remappings=[('joint_states', '/whole_body/joint_states')],
    )
    
    handover = Node(
        package='sasha_handover',
        executable='handover_srv',
        name='handover',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )
    
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        odom_joint_states_publisher,        
        visualizer,        
        execute_grasp_server,
        table_plane_launch,
        image_fetcher,
        object_detector,
        pose_estimator,
        find_grasppoint_server,
        place,
        handover
    ])
