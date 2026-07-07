#! /usr/bin/env python3

import rclpy
import yasmin
from hsrb_interface import Robot
from typing import Any
import time
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import BoundingBox3D
from v4r_util.conversions import vector3_to_list
from grasping_pipeline.moveit_wrapper import MoveitWrapper
from grasping_pipeline.moveit_single_node import MoveItSingleNode
from v4r_util.tf2 import TF2Wrapper



class CollisionEnvironment(yasmin.State):
    '''
    Deals with the MoveIt collision Environment.
    
    It adds the object and the table to the MoveIt collision environment.
    The object is added as a box with the name 'object' and the same dimensions as the bounding box
    of the object. The table is added as a box with the name 'table' and the same dimensions as the 
    bounding box of the table. It also adds a floor plane to filter weird octomap points in the 
    floor that prevent the robot from moving because of 'collisions' with the floor.

    Parameters
    ----------
    grasp_object_bb: BoundingBox3D
        The bounding box of the object to grasp.
    table_bbs: BoundingBox3DArray
        The bounding boxes of the detected planes. The first one is considered the actual table.
    
    Returns
    -------
    grasp_object_name_moveit: str
        The name of the object in the MoveIt collision environment.
        Currently set to 'object', but might be changed in the future (and be an actual variable), hence the output key.
    '''

    def __init__(self, node, moveit_wrapper):
        '''
        Initializes the CollisionEnvironment state. 
        
        It creates a MoveitWrapper object and detaches all objects from the robot's end effector.
        It also adds a floor plane to filter weird octomap points in the floor that prevent the 
        robot from moving because of 'collisions' with the floor.
        '''
        super().__init__(['succeeded'])
        self.node = node
        self.tf_wrapper = TF2Wrapper(self.node)
        #timeout_cnt = 0
        #while timeout_cnt < 150: #delay so that tf buffer can fill up.
        #    rclpy.spin_once(self.node, timeout_sec=0.1)
        #    timeout_cnt += 1
        self.moveit_wrapper = moveit_wrapper
        
        
        self.moveit_wrapper.detach_all_objects()
        self.clear_octomap = self.node.create_client(Empty, '/clear_octomap')
        while not self.clear_octomap.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info('Waiting for clear_octomap service...')

        # Add floor plane to filter weird octomap points in floor that prevent the robot from moving 
        # because of 'collisions' with the floor
        self.add_floor_plane()
        # Remove floor fragments from octomap. 
        # This is necessary, because moveit does not really remove the old floor points from the octomap
        # after adding the floor plane. 
        # It only prevents adding them to the octomap in the future.
        self._call_clear_octomap()
        self.node.get_logger().info('CollEnv init')
    
    def add_floor_plane(self):
        '''
        Adds a floor plane to the MoveIt collision environment centered at the robot's current position.
        
        This is necessary to filter weird octomap points in the floor that prevent the robot from 
        moving because of 'collisions' with the floor.
        '''
        base_pose = self.moveit_wrapper.get_current_pose('map')

        pos = base_pose.pose.position
        ori = base_pose.pose.orientation

        floor = BoundingBox3D()
        floor.center.position.x = pos.x
        floor.center.position.y = pos.y
        floor.center.position.z = -0.07
        floor.center.orientation.w = ori.w

        size = [15.0, 15.0, 0.1]

        self.moveit_wrapper.add_box('floor', 'map', floor.center, size)
    
    def _call_clear_octomap(self):
        '''Call the clear_octomap service'''
        request = Empty.Request()
        future = self.clear_octomap.call_async(request)
        
        # Wait for the service to complete
        timeout_count = 0
        while not future.done() and timeout_count < 100:  # 10 seconds at 10Hz
            rclpy.spin_once(self.node, timeout_sec=0.1)
            timeout_count += 1

    def execute(self, blackboard: Any):
        '''
        Adds the object and the table to the MoveIt collision environment.
        '''
        # Detach objects from the robot eef. 
        # Needed because sometimes the object from the previous grasp is still attached to the eef 
        # if MoveIt bugs out (aka we don't handle all possible MoveIt errors properly)
        self.moveit_wrapper.detach_all_objects()
        self.node.get_logger().info("adding box")
        grasp_obj_bb = blackboard['grasp_object_bb']
        self.moveit_wrapper.add_box(
            'object',
            grasp_obj_bb.header.frame_id,
            grasp_obj_bb.center, 
            vector3_to_list(grasp_obj_bb.size))
        blackboard['grasp_object_name_moveit'] = 'object'
        self.node.get_logger().info("box added to planning scene")
        time.sleep(0.1)
        table_bb = blackboard['table_bbs'].boxes[0]
        self.moveit_wrapper.add_box(
            'table', 
            blackboard['table_bbs'].header.frame_id, 
            table_bb.center, 
            vector3_to_list(table_bb.size))

        
        return 'succeeded'

