#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2022 Hibikino-Musashi@Home
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Hibikino-Musashi@Home nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import copy
from moveit.planning import MoveItPy, PlanningComponent
from moveit_msgs.msg import Constraints
from rclpy.node import Node
import xml.etree.ElementTree as ET

#from moveit_commander.conversions import pose_to_list

import rclpy
import time
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from v4r_util.tf2 import TF2Wrapper
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlaceLocation
from moveit_msgs.action import Place
import trajectory_msgs
from moveit_msgs.msg import Constraints, OrientationConstraint,PlanningScene
from v4r_util.conversions import euler_to_quaternion
from std_srvs.srv import Empty
import numpy as np
from scipy.spatial.transform import Rotation as R

class MoveitWrapper:
    """Convenience Moveit Wrapper for Sasha."""

    def __init__(self, tf_wrapper, node: Node, planning_time=5.0):
        """
        Initializes the MoveitWrapper.

        Parameters
        ----------
        tf_wrapper : TF2Wrapper
            Instance of the TF2Wrapper class
        planning_time : float, optional
            The time to plan a trajectory, by default 5.0
        """
        self.tf_wrapper = tf_wrapper
        self.node = node
        self.node.get_logger().info(f"Starting MoveitWrapper init:")
        self.logger = node.get_logger()
        self.whole_body, self.gripper, self.scene, self.robot = self.init_moveit()
        self.path_constraints = None
        self.planning_time = planning_time

         # Storage for the latest planning scene
        self._planning_scene_msg = None

        # Subscribe to the planning scene topic
        self._scene_sub = node.create_subscription(
            PlanningScene,
            '/planning_scene',
            self._planning_scene_callback,
            10
        )

        self._planning_scene_pub = self.node.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        


    def _planning_scene_callback(self, msg: PlanningScene):
        self._planning_scene_msg = msg


    def init_moveit(self):
        '''
        Initializes the moveit commander and returns the whole_body, gripper, scene and robot objects
        
        Parameters
        ----------
        planning_time : float
            The time to plan a trajectory
        
        Returns
        -------
        whole_body : moveit_commander.MoveGroupCommander
            The whole body move group commander
        gripper : moveit_commander.MoveGroupCommander
            The gripper move group commander
        scene : moveit_commander.PlanningSceneInterface
            The planning scene interface
        robot : moveit_commander.RobotCommander
            The robot commander
        '''
        
        # MoveitPy
        self.moveit_py = MoveItPy(
            node_name="moveit_py",
            remappings={"joint_states": "/whole_body/joint_states"}
        )
        
        planning_component = PlanningComponent("whole_body", self.moveit_py)
        whole_body = self.moveit_py.get_planning_component("whole_body")
        #gripper = robot.get_planning_component("gripper", wait_for_servers=timeout_sec)
        #arm = robot.get_planning_component("arm", wait_for_servers=timeout_sec)
        gripper = None
        scene = self.moveit_py.get_planning_scene_monitor()
        scene.clear_octomap()
        robot = self.moveit_py.get_robot_model()
        
        
        return whole_body, gripper, scene, robot

    def add_box(self, name, frame, pose, size):
        """
        Add a box to the planning scene.

        Parameters
        ----------
        name : str
            The name of the box
        frame : str
            The frame of the passed pose
        pose : geometry_msgs/Point, geometry_msgs/Pose
            Pose of the box relative to the frame. If geometry_msgs/Point is passed, the orientation
            is set to (0, 0, 0, 1), otherwise the orientation is set to the passed value.
        size : list[float]
            The size of the box as (x, y, z)
        """
        box = CollisionObject()
        box.id = name
        box.header.frame_id = frame
        box.header.stamp = rclpy.time.Time().to_msg()

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(x) for x in size]  # [x, y, z]

        box.primitives.append(primitive)

        if isinstance(pose, Point):
            box_pose = Pose()
            box_pose.position = pose
            box_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        elif isinstance(pose, Pose):
            box_pose = pose
        else:
            raise TypeError("pose must be Point or Pose")

        box.primitive_poses.append(box_pose)
        box.operation = CollisionObject.ADD

        with self.scene.read_write() as scene:
            scene.apply_collision_object(box)
            scene.process_planning_scene_world(scene.planning_scene_message.world)            
            scene.current_state.update()

            msg = scene.planning_scene_message
        
        ps_msg = PlanningScene()
        ps_msg.is_diff = True
        ps_msg.world.collision_objects.append(box)

        self._planning_scene_pub.publish(ps_msg)


    def add_orientation_constraint(self, reference_frame, orientation, end_effector_link = "hand_palm_link", tolerances_xyz=[0.3, 0.3, 0.3]):
        '''
        Adds an orientation constraint to the whole body until it is cleared
        
        Parameters
        ----------
        reference_frame : str
            The name of the reference frame (i.e. the frame in which the orientation constrain is defined in)
        end_effector_link : str
            The name of the end effector link which should be constrained
        orientation : geometry_msgs/Quaternion
            The target orientation of the end effector link (i.e. the eef should have this orientation during the motion)
        tolerances_xyz : list[float], optional
            The tolerances for the orientation constraint, by default [0.3, 0.3, 0.3] (radians)
    '''
    
        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.header.frame_id = reference_frame
        orientation_constraint.orientation = orientation

        # Set tolerances for the orientation constraint
        orientation_constraint.absolute_x_axis_tolerance = tolerances_xyz[0]
        orientation_constraint.absolute_y_axis_tolerance = tolerances_xyz[1]
        orientation_constraint.absolute_z_axis_tolerance = tolerances_xyz[2]

        # Weight the constraint such that it is strictly enforced
        orientation_constraint.weight = 1.0

        # Create a Constraints object and add the orientation constraint to it
        constraints = Constraints()
        constraints.orientation_constraints.append(orientation_constraint)

        # Store the path constraints for later use in planning or execution
        self.path_constraints = constraints


    def attach_object(self, object_name, link_name = "hand_palm_link", touch_links=[]):
        '''
        Attaches an object to the robot eef

        Parameters
        ----------
        object_name : str
            The name of the object to attach. Must be a known object in the planning scene.
        link_name : str
            The name of the efector that does the gripp action (default gripper)
        touch_links : list[str], optional
            The links that are allowed to touch the object (e.g. parts of the gripper), by default []
        '''
        if touch_links is None:
            touch_links = []

        aco = AttachedCollisionObject()
        aco.link_name = link_name
        aco.touch_links = touch_links
        
        aco.object = CollisionObject()
        aco.object.header.stamp = rclpy.time.Time().to_msg()
        aco.object.id = object_name
        aco.object.operation = CollisionObject.ADD
        
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.attached_collision_objects.append(aco)
        ps.robot_state.is_diff = True

        self._planning_scene_pub.publish(ps)
        

    def clear_path_constraints(self):
        '''
        Clears the path constraints of the whole body
        '''
        empty_constraints = Constraints()
        self.whole_body.set_path_constraints(empty_constraints)
        self.whole_body.set_start_state_to_current_state()


    def current_pose_close_to_target(self, target_pose, end_effector_link="hand_palm_link", pos_tolerance=0.04, ori_tolerance=0.17):
        '''
        Checks if the current pose is close to the target pose
        
        Parameters
        ----------
        target_pose : geometry_msgs/Pose
            The target pose
        end_effector_link : str
            The eef
        pos_tolerance : float, optional
            The tolerance for the positions, by default 0.04
        ori_tolerance : float, optional
            The tolerance for the orientations, by default 0.17
        
        Returns
        -------
        bool
            True if the current pose is close to the target pose
        '''
        with self.scene.read_write() as scene:
            current_state = scene.current_state

            # Get global link transform
            link_transform = current_state.get_global_link_transform(end_effector_link)

        # Convert to geometry_msgs/Pose
        T = np.array(link_transform)
        current_pose = Pose()
        current_pose.position.x = T[0, 3]
        current_pose.position.y = T[1, 3]
        current_pose.position.z = T[2, 3]

        rot = R.from_matrix(T[:3, :3])
        quat = rot.as_quat()
        current_pose.orientation.x = quat[0]
        current_pose.orientation.y = quat[1]
        current_pose.orientation.z = quat[2]
        current_pose.orientation.w = quat[3]

        return self.all_close(target_pose, current_pose, pos_tolerance, ori_tolerance)


    def detach_all_objects(self):
        '''
        Detaches all objects from the robot eef
        
        Parameters
        ----------
        object_name : str
            The name of the object to detach. Must be a known object in the planning scene.
        '''
        pass #needs some work

    def execute(self, plan, wait=True):
        if plan:
            self.moveit_py.execute(plan)
            return True
        return False


    def plan(self):

        start_time = time.time()

        plan_result = self.whole_body.plan(
            path_constraints=self.path_constraints
        )

        planning_time = time.time() - start_time

        if plan_result and plan_result.trajectory:
            return True, plan_result.trajectory, planning_time, 1
        else:
            return False, None, planning_time, -1
    
    def get_attached_objects(self, object_names = []):
        '''
        Returns the attached objects
        
        Parameters
        ----------
        object_names : list[str], optional
            The names of the objects to return. If empty, all attached objects are returned, by 
            default []
        
        Returns
        -------
        dict
            The attached objects. The keys are the object names and the values are the objects
            (moveit_msgs/AttachedCollisionObject)    
        '''
        with self.scene.read_write() as scene:
            planning_scene = scene.planning_scene_message        
           
        attached_objects = planning_scene.robot_state.attached_collision_objects
         # Convert list -> dict
        attached_objects_dict = {
            obj.object.id: obj
            for obj in attached_objects
        }
        # Filter if object_names is given
        if object_names:
            attached_objects_dict = {name: obj for name, obj in attached_objects_dict.items() if name in object_names}
        return attached_objects_dict

    def get_current_eef_pose(self, eef_link="hand_palm_link", reference_frame="map"):
        '''
        Returns the current pose of the end effector link in the specified frame
        
        Parameters
        ----------
        eef_link : str, optional
            The name of the end effector link, by default "hand_palm_link"
        reference_frame : str, optional
            The name of the reference frame to transform the eef pose to, by default "map"
        '''
        eef_pose = self.whole_body.get_current_pose(end_effector_link=eef_link)
        eef_pose = self.tf_wrapper.transform_pose(reference_frame, eef_pose)
        return eef_pose
            
    def get_current_pose(self, frame):
        """
        Returns the current pose of the robot base in the specified frame
        
        Parameters
        ----------
        frame : str
            The frame to transform the base pose to
        
        Returns
        -------
        geometry_msgs/PoseStamped
            The current pose of the robot base in the specified frame
        """
        
        p = PoseStamped()
        p.header.frame_id = 'base_footprint' # maybe base_link not sure
        p.header.stamp = self.node.get_clock().now().to_msg()
        p.pose.orientation.w = 1.0
        base_pose = self.tf_wrapper.transform_pose(frame, p)
        return base_pose

    def get_link_names(self, group = None):
        '''
        Returns the link names of the specified group
        
        Parameters
        ----------
        group : str, optional
            The name of the group, by default None
        
        Returns
        -------
        list[str]
            All link names of the specified group.
        '''
        if group is None:
            for gname in self.robot.joint_model_group_names:
                jmg = self.robot.get_joint_model_group(gname)
                link_names = jmg.link_model_names
            # Remove duplicates
            return list(set(link_names))
        
        jmg = self.robot.get_joint_model_group(group)
        return jmg.link_model_names


    def get_object_poses(self, object_names):
        '''
        Returns the poses of the objects in the planning scene
        
        Parameters
        ----------
        object_names : list[str]
            The names of the objects to return
        
        Returns
        -------
        dict
            The poses of the objects in the planning scene. The keys are the object names and the 
            values are the poses (geometry_msgs/Pose). The frame of the poses is the planning frame.
        '''
        with self.scene.read_write() as scene:
            planning_scene = scene.planning_scene_message        

        self.node.get_logger().info(str(planning_scene.world.collision_objects))
        poses = {}
        for name in object_names:
            obj = None
            # Find the object in the current scene
            for o in planning_scene.world.collision_objects:
                if o.id == name:
                    obj = o
                    break  # stop looping once we find the object
            if obj is not None:
                # Choose first pose from primitive or mesh
                if obj.primitive_poses:
                    poses[name] = obj.primitive_poses[0]
                elif obj.mesh_poses:
                    poses[name] = obj.mesh_poses[0]
        return poses
    
    def get_planning_frame(self, group="whole_body"):
        '''
        Returns the planning frame of the specified group
        
        Parameters
        ----------
        group : str, optional
            The name of the group, by default "whole_body"
        
        Returns
        -------
        str
            The planning frame of the specified group
        '''
        # not sure if the "base_footprint" would get returned by.get_planning_frame(), maybe is odom
        # with base_footprint there where some moveit planning issues
        if group == "whole_body":
            return "odom"
        else:
            raise NotImplementedError
    def place(self, object, pose):
        '''
        Places an object at the specified pose
        
        Parameters
        ----------
        object : str
            The name of the object to place
        pose : geometry_msgs/Pose
            The pose to place the object at
        
        Returns
        -------
        not None if the object was placed successfully, None otherwise
        '''
        return self.whole_body.place(object, pose)
    
    def set_position_target(self, xyz, 
                        end_effector_link="hand_palm_link",
                        reference_frame="base_link"):

        # Create pose goal
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = reference_frame
        pose_goal.pose.position.x = xyz[0]
        pose_goal.pose.position.y = xyz[1]
        pose_goal.pose.position.z = xyz[2]

        # Preserve current orientation (ROS1 behavior)
        current_pose = self.get_current_eef_pose(
            end_effector_link,
            reference_frame
        )
        pose_goal.pose.orientation = current_pose.pose.orientation

        # Set goal state
        self.whole_body.set_goal_state(
           pose_stamped_msg=pose_goal,
           pose_link=end_effector_link,
        )
    
    def set_support_surface(self, surface_name):
        '''
        Sets the support surface for the whole body
        '''
        place_location = PlaceLocation()

        self.goal = Place.Goal()
        self.goal.group_name = "whole_body"
        self.goal.attached_object_name = "object"
        self.goal.place_locations = [place_location]
        self.goal.support_surface_name = surface_name
    
    
    def whole_body_plan_and_go(self, pose_target):
        '''
        Plans and executes a motion to the target pose
        
        Parameters
        ----------
        pose_target : geometry_msgs/PoseStamped
            The target pose
        
        Returns
        -------
        bool
            True if the plan was found and executed, False otherwise'''
        # Set goal pose
        pose_target.header.stamp = rclpy.time.Time().to_msg() # set to current time to avoid transform issues
        pose = PoseStamped()
        pose.header.frame_id = "odom"

        # small reachable offset (safe!)
        pose.pose.position.x = 0.4
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.8

        # neutral orientation (important!)
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        self.whole_body.set_goal_state(pose_stamped_msg=pose_target, pose_link="hand_palm_link")
        
        self.node.get_logger().info(f"Set goal pose to {pose_target}")
        with self.scene.read_write() as scene:
            self.whole_body.set_start_state(robot_state=scene.current_state)
            self.node.get_logger().info(f"Current state is: {scene.current_state}")
        # Plan
        plan_result = self.whole_body.plan()

        # Check if planning succeeded
        if plan_result:
            self.node.get_logger().info("got plan_result")
            # Execute
            self.moveit_py.execute("whole_body", plan_result.trajectory)
            return True

        return False
    
    def pose_to_list(self, pose):
        return [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

    def all_close(self, goal, actual, pos_tolerance, ori_tolerance):
        """
        Judges whether the actual value is within the tolerance for the goal value.

        Parameters
        ----------
        goal : list[float], geometry_msgs/Pose, geometry_msgs/PoseStamped
            The goal value
        actual : list[float], geometry_msgs/Pose, geometry_msgs/PoseStamped
            The actual value
        pos_tolerance : float
            The tolerance for the positions
        ori_tolerance : float
            The tolerance for the orientations

        Returns
        -------
        bool
            True if the value is within the tolerance
        """
        if type(goal) is list:
            for index in range(len(goal)):
                if index > 2:
                    if abs(abs(actual[index]) - abs(goal[index])) > ori_tolerance:
                        self.logger.error(f"ori: actual: {actual[index]}, goal: {goal[index]}")
                        return False
                else:
                    if abs(actual[index] - goal[index]) > pos_tolerance:
                        self.logger.error(f"pos: actual: {actual[index]}, goal: {goal[index]}")
                        return False

        elif type(goal) is PoseStamped:
            return self.all_close(goal.pose, actual, pos_tolerance, ori_tolerance)

        elif type(goal) is Pose:
            return self.all_close(self.pose_to_list(goal), self.pose_to_list(actual), pos_tolerance, ori_tolerance)

        return True