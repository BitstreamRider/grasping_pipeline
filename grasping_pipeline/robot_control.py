from math import pi
import numpy as np
from tf_transformations import quaternion_about_axis 
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
import yasmin
import time
import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from grasping_pipeline.moveit_wrapper import MoveitWrapper
from v4r_util.tf2 import TF2Wrapper
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from hsrb_interface import Robot
from hsrb_interface.exceptions import MobileBaseError
from grasping_pipeline_msgs.msg import BoundingBox3DStamped


class GoToNeutral(yasmin.State):
    """ Yasmin state that will move the robots joints to a
    predefined position, gazing at a fixed point.

    Returns
    -------
    smach-result
        'succeeded': The state only returns succeeded.
    """
    def __init__(self, node):
        super().__init__(['succeeded', 'aborted'])
        self.node = node
        # Robot initialization
        self.robot = Robot(node)
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, blackboard: yasmin.Blackboard):
        self.node.get_logger().info('Executing state GoToNeutral')
        joint_positions = {
            'arm_flex_joint': 0.0, 
            'arm_lift_joint': 0.0, 
            'arm_roll_joint': pi/2, 
            'head_pan_joint': 0.0, 
            'head_tilt_joint': -0.675, 
            'wrist_flex_joint': -pi/2, 
            'wrist_roll_joint': 0.0
            }
        self.whole_body.move_to_joint_positions(joint_positions)
        return 'succeeded'

class GoToNeutralMoveIt(yasmin.State):
    """ Yasmin state that tries to move the robot to a neutral position with moveit and orientation
    constraints, such that e.g. the water in the glass does not spill. Does not work well. GoToNeutral()
    should be used instead, unless it is necessary that the robot wrist is not rotated.

    Returns
    -------
    smach-result
        'succeeded': The state only returns succeeded, even though the movement might fail. This is 
        probably not the best way, but I decided to do it like this so you can easily replace it with
        GoToNeutral() by just changing the state name and not having to touch the transitions.
    """
    def __init__(self, node, moveit_wrapper):
        super().__init__(['succeeded'])
        self.node = node
        self.moveit_wrapper = moveit_wrapper
        # Robot initialization
        self.tf_wrapper = TF2Wrapper(self.node)

    def execute(self, blackboard: yasmin.Blackboard):
        mw = self.moveit_wrapper
        tf2 = self.tf_wrapper
        reference_frame = "map"
        end_effector_link = "hand_palm_link"
        
        eef_pose = mw.get_current_eef_pose(end_effector_link, reference_frame)
        target_eef_pose = PointStamped()
        target_eef_pose.header.frame_id = 'base_link'
        target_eef_pose.header.stamp = self.node.get_clock().now().to_msg()
        target_eef_pose.point.x = 0.14
        target_eef_pose.point.y = 0.217
        target_eef_pose.point.z = 0.673
        point = tf2.transform_point(mw.get_planning_frame(), target_eef_pose)
        mw.whole_body.set_position_target([point.point.x, point.point.y, point.point.z])

        mw.add_orientation_constraint(reference_frame, end_effector_link, eef_pose.pose.orientation, [666, 0.05, 0.05])

        plan_found, plan, planning_time, error_code  = mw.whole_body.plan()
        # Check if planning was successful
        if plan_found and len(plan.joint_trajectory.points) > 0:
            self.node.get_logger().info("Planning successful. Executing plan...")
            # Execute the plan
            mw.whole_body.execute(plan, wait=True)
            self.node.get_logger().info("Motion execution completed.")
        else:
            self.node.get_logger().error("Planning failed. Planning time: %s, Errorcode: %s", planning_time, error_code)
        return 'succeeded'
        

class MoveToJointPositions(yasmin.State):
    """ Yasmin state that will move the robots joints to the
    defined position.
    
    Returns
    -------
    yasmin-result
        'succeeded': The state only returns succeeded.
    """

    def __init__(self, node, joint_positions_dict):
        '''
        Initializes the MoveToJointPositions state. Sets the joint positions dictionary.

        Parameters
        ----------
        node: rclpy.node.Node
            The ROS2 node
        joint_positions_dict: dict
            Dictionary with joint names as keys and joint positions as values. Possible keys are:
            'arm_flex_joint', 'arm_lift_joint', 'arm_roll_joint', 'head_pan_joint', 'head_tilt_joint',
            'wrist_flex_joint', 'wrist_roll_joint'. The angles are in radians.
        '''
        super().__init__(['succeeded'])
        self.node = node
        # Robot initialization
        self.robot = Robot(node)
        self.whole_body = self.robot.try_get('whole_body')
        self.joint_positions_dict = joint_positions_dict

    def execute(self, blackboard: yasmin.Blackboard):
        self.node.get_logger().info('Executing state MoveToJointPositions')
        self.whole_body.move_to_joint_positions(self.joint_positions_dict)
        return 'succeeded'


class GoBack(yasmin.State):
    """ Yasmin state that will move the robot backwards.
    
    Returns
    -------
    yasmin-result
        'succeeded': The state only returns succeeded.
    """

    def __init__(self, node, distance):
        '''
        Initializes the GoBack state. Sets the distance to move backwards.
        
        Parameters
        ----------
        node: rclpy.node.Node
            The ROS2 node
        distance: float
            The distance to move backwards in meters.
        '''
        super().__init__(['succeeded', 'aborted'])
        self.node = node
        # Robot initialization
        self.robot = Robot(node)
        self.base = self.robot.try_get('omni_base')
        self.whole_body = self.robot.try_get('whole_body')
        self.distance = distance

    def execute(self, blackboard: yasmin.Blackboard):
        self.node.get_logger().info('Executing state GoBack')
        try:
            self.base.go_rel(-self.distance, 0.0, 0.0, timeout=20.0)
        except MobileBaseError as e:
            self.node.get_logger().error(str(e))
            return 'aborted'
        return 'succeeded'


class OpenGripper(yasmin.State):
    """ Opens the robots gripper
    
    Returns
    -------
    yasmin-result
        'succeeded': The state only returns succeeded.
    """

    def __init__(self, node):
        super().__init__(['succeeded', 'aborted'])
        self.node = node
        # Robot initialization
        self.robot = Robot(node)
        self.gripper = self.robot.try_get('gripper')

    def execute(self, blackboard: yasmin.Blackboard):
        self.node.get_logger().info('Executing state OpenGripper')
        self.gripper.command(1.0)
        return 'succeeded'

class GoToWaypoint(yasmin.State):
    '''
    Moves the robot to a given waypoint.
    
    Returns
    -------
    yasmin-result
        'succeeded': If the robot reached the waypoint.
        'aborted': If the robot could not reach the waypoint, either because action took to long and
        timed out or because the connection to the move server could not be established.
    '''
    def __init__(self, node, x, y, phi_degree, frame_id='map', timeout=30.0):
        '''
        Initializes the GoToWaypoint state. Sets the target pose and frame_id.
        
        Parameters
        ----------
        node: rclpy.node.Node
            The ROS2 node
        x: float
            x-coordinate of the target pose in meters
        y: float
            y-coordinate of the target pose in meters
        phi_degree: float
            Rotation around the z-axis in degrees of the target pose
        frame_id: str
            The frame_id of the target pose.
        timeout: float
            Timeout in seconds. The state will abort if the robot does not reach the target pose in 
            this time.
        '''
        super().__init__(['succeeded', 'aborted'])
        self.node = node
        self.move_client = ActionClient(self.node,  NavigateToPose, '/navigate_to_pose')
        node.get_logger().info("got move action server...")
        self.robot = Robot(node)
        node.get_logger().info("got robot...")
        self.whole_body = self.robot.try_get('whole_body')
        node.get_logger().info("got body...")
        self.x = x
        self.y = y
        self.phi = phi_degree
        self.frame_id = frame_id
        self.timeout = timeout

    def execute(self, blackboard: yasmin.Blackboard):
        move_goal = NavigateToPose.Goal()
        
        move_goal.pose.header.frame_id = self.frame_id
        move_goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        move_goal.pose.pose.position.x = self.x
        move_goal.pose.pose.position.y = self.y
        quat = quaternion_about_axis(self.phi * pi/180, (0, 0, 1))
        move_goal.pose.pose.orientation.x = quat[0]
        move_goal.pose.pose.orientation.y = quat[1]
        move_goal.pose.pose.orientation.z = quat[2]
        move_goal.pose.pose.orientation.w = quat[3]
        
        # Wait for action server
        while not self.move_client.wait_for_server(timeout_sec=self.timeout):
            self.node.get_logger().error("Could not connect to move server!")
            return 'aborted'    
        
        # Send goal
        self.node.get_logger().info("Waiting for result")
        future = self.move_client.send_goal_async(move_goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        
        if not future.done():
            self.node.get_logger().error("Move server execution timed out!")
            self.move_client.cancel_all_goals()
            return 'aborted'
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Move goal was rejected!")
            return 'aborted'
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=self.timeout)
        
        if not result_future.done():
            self.node.get_logger().error("Move server execution timed out!")
            return 'aborted'
        
        # wait for robot to settle down
        time.sleep(1.0)
        return 'succeeded'
        
    
class GoToAndLookAtPlacementArea(yasmin.State):
    '''
    Navigates the robot to the waypoint of the placement area and makes it look at the placement area.
    
    Returns
    -------
    yasmin-result
        'succeeded': If the robot reached the waypoint and looked at the placement area.
        'aborted': If the robot could not reach the waypoint, either because action took to long and
        timed out or because the connection to the move server could not be established, or if no
        placement area is specified for this object.
    '''
    def __init__(self, node):
        super().__init__(['succeeded', 'aborted'])
        self.node = node
        self.move_client = ActionClient(self.node,  NavigateToPose, '/navigate_to_pose')
        self.robot = Robot(node)
        self.whole_body = self.robot.try_get('whole_body')
        self.timeout = 40.0
        if not self.node.has_parameter('grasping_pipeline.dataset'):
            self.node.declare_parameter('grasping_pipeline.dataset', 'ycb_bop')
        if not self.node.has_parameter('grasping_pipeline.placement.placement_area'):
            self.node.declare_parameter('grasping_pipeline.placement.placement_area', 'table')
        try:
            package_path = get_package_share_directory('grasping_pipeline')
            config_path = Path(package_path + '/config/placement_areas.yaml')

            with open(config_path, 'r') as file:
                self.placement_config = yaml.safe_load(file)

            self.node.get_logger().info("Placement config loaded successfully")

        except Exception as e:
            self.node.get_logger().error(f"Failed to load placement config: {e}")
            self.placement_config = {}

    


    def execute(self, blackboard: yasmin.Blackboard):
        '''
        Moves the robot to the waypoint of the placement area and makes it look at the placement area.
        
        Loads the placement areas and objects from the parameter server. The placement area of the
        object can be set in the config file. Otherwise it is determined by the object name. The robot 
        moves to the waypoint of the placement area and looks at the placement area based on the data
        loaded from the parameter server.
        '''
        # Get parameters
        try:
            dataset = self.node.get_parameter('grasping_pipeline.dataset').value
        except:
            self.node.get_logger().error('Parameter dataset not found')
            return 'aborted'

        if self.node.has_parameter("grasping_pipeline.placement.placement_area"):
            placement_area_name = self.node.get_parameter("grasping_pipeline.placement.placement_area").value
        else:
            placement_area_name = 'predefined'

        if placement_area_name == 'predefined':
            self.node.get_logger().info("Loading predefined placement areas.")

            try:
                placement_objects = self.placement_config['placement_objects']
                grasp_object_name = blackboard['grasp_object_name']

                if grasp_object_name not in placement_objects[dataset]:
                    self.node.get_logger().error(f"Object {grasp_object_name} not found in placement_objects!")
                    return 'aborted'
                
                placement_area_name = placement_objects[dataset][grasp_object_name]
            except Exception as e:
                self.node.get_logger().error(f"Failed to get placement objects: {e}")
                return 'aborted'

        # Checking if placement_area_name is valid
        try:
            placement_areas = self.placement_config['placement_areas']
            if placement_area_name not in placement_areas:
                self.node.get_logger().error(f"Placement area {placement_area_name} not found!")
                return 'aborted'
                
            self.node.get_logger().info(f"Placing object on waypoint: {placement_area_name}")

            placement_area = placement_areas[placement_area_name]
        except Exception as e:
            self.node.get_logger().error(f"Failed to get placement areas: {e}")
            return 'aborted'

        frame_id = placement_area["frame_id"]
        waypoint = placement_area["waypoint"]

        # Setting method to be able to call it later during the placement
        #self.node.set_parameters([Parameter(name='/grasping_pipeline/placement/method', value=placement_area["method"])])

        # Check if the placement area has a defined center and size
        placement_area_bb = BoundingBox3DStamped()
        if "center" in placement_area.keys() and "size" in placement_area.keys() and \
            type(placement_area["center"]) == list and type(placement_area["size"]) == list:
                
            placement_area_bb.center.position.x = placement_area['center'][0]
            placement_area_bb.center.position.y = placement_area['center'][1]
            placement_area_bb.center.position.z = placement_area['center'][2]
            placement_area_bb.center.orientation.w = 1.0
            placement_area_bb.size.x = placement_area['size'][0]
            placement_area_bb.size.y = placement_area['size'][1]
            placement_area_bb.size.z = placement_area['size'][2]
            placement_area_bb.header.frame_id = frame_id

            # arm_lift moves by ~ 10 cm for each 0.2 increase and has to be between 0 and 0.6
            arm_lift_joint = max(0.0, (placement_area['center'][2] - 0.45)*2.0)
            arm_lift_joint = min(arm_lift_joint, 0.6)

            arm_flex_joint = -2.6 if arm_lift_joint > 0.1 else -0.1

            hsr_center_dist_x = np.abs(placement_area['center'][0] - waypoint[0])
            hsr_center_dist_y = np.abs(placement_area['center'][1] - waypoint[1])
            hsr_center_dist_xy = np.sqrt(hsr_center_dist_x**2 + hsr_center_dist_y**2)
            # e.g. arm_lift_joint = 0.2 -> should increase by 10cm -> 0.2*0.5 = 0.1; 0.75 is approx. the normal height of the camera
            hsr_center_dist_z = 0.75 + arm_lift_joint * 0.5 - placement_area['center'][2]
            head_tilt_angle = pi/2 - np.arctan2(hsr_center_dist_xy, hsr_center_dist_z)

            # head_tilt_joint: 0.0 is looking straight, -1.1 is about -70 degrees and the most Sasha can look down before mostly seeing his hand
            head_tilt_joint = -head_tilt_angle * 1.1 / 70 * 180 / pi - 0.15
            head_tilt_joint = max(-1.1, head_tilt_joint)
            head_tilt_joint = min(head_tilt_joint, 0.0)

        else:
            self.node.get_logger().warn("No center and/or size specified in placement area. Placement will be done on the closest plane.")

            arm_flex_joint = -0.1
            arm_lift_joint = 0.0
            head_tilt_joint = -0.7
        
        blackboard['placement_area_bb'] = placement_area_bb
        
        move_goal = NavigateToPose.Goal()
        move_goal.pose.header.frame_id = frame_id
        move_goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        move_goal.pose.pose.position.x = waypoint[0]
        move_goal.pose.pose.position.y = waypoint[1]
        quat = quaternion_about_axis(waypoint[2] * pi/180, (0, 0, 1))
        move_goal.pose.pose.orientation.x = quat[0]
        move_goal.pose.pose.orientation.y = quat[1]
        move_goal.pose.pose.orientation.z = quat[2]
        move_goal.pose.pose.orientation.w = quat[3]

        # Wait for action server
        while not self.move_client.wait_for_server(timeout_sec=self.timeout):
            self.node.get_logger().error("Could not connect to move server!")
            return 'aborted'
        
        # Send goal
        future = self.move_client.send_goal_async(move_goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        
        if not future.done():
            self.node.get_logger().error("Move server execution timed out!")
            return 'aborted'
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Move goal was rejected!")
            return 'aborted'
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=self.timeout)
        
        if not result_future.done():
            self.node.get_logger().error("Move server execution timed out!")
            return 'aborted'

        # wait for robot to settle down
        time.sleep(1.0)
        self.whole_body.move_to_joint_positions({'arm_flex_joint': arm_flex_joint, 'arm_lift_joint': arm_lift_joint, "head_tilt_joint": head_tilt_joint})
        return 'succeeded'
        

class CheckTopGrasp(yasmin.State):
    def __init__(self, node):
        super().__init__(['top_grasp', 'not_top_grasp'])
        self.node = node
    
    def execute(self, blackboard: yasmin.Blackboard):
        if blackboard['top_grasp']:
            return 'top_grasp'
        else:
            return 'not_top_grasp'