#! /usr/bin/env python3
import sys
import os
from copy import deepcopy
import numpy as np
import yaml
from yaml.loader import SafeLoader
from math import pi
from sensor_msgs.msg import Image, CameraInfo
from v4r_util.depth_pcd import convert_ros_depth_img_to_pcd
from geometry_msgs.msg import PoseStamped, Pose
import PyKDL
#import actionlib
from rclpy.action import ActionClient, ActionServer
from rclpy.qos import qos_profile_sensor_data
#import rospy
import rclpy
from rclpy.node import Node
from grasping_pipeline_msgs.action import FindGrasppoint
from tf_transformations import (quaternion_about_axis, quaternion_from_matrix,
                                quaternion_multiply)
#from tf_conversions import posemath
from visualization_msgs.msg import Marker

from grasping_pipeline.grasp_annotator import GraspAnnotator

from v4r_util.bb import create_ros_bb_stamped

import time

class FindGrasppointServer(Node):
    '''
    Computes the grasp_poses for the object to grasp based on annotated grasps.

    The FindGrasppointServer uses the grasp checker to calculate the grasp poses for the
    object, based on grasp-annotations and the estimated object poses. 
    The server returns the grasp poses, the bounding box of the
    object to grasp and the object name of the object to grasp.
    The server adds markers to the rviz visualization to show the grasp pose and the bounding box of
    the object.
    
    Attributes
    ----------
    models_metadata: dict
        The metadata of the known objects. Contains the bounding box information.
    server: actionlib.SimpleActionServer
        This action server. Computes grasp poses for the object to grasp.
    marker_pub: rospy.Publisher
        The publisher for the rviz markers.
    cam_info: sensor_msgs.msg.CameraInfo
        The camera info message containing the camera parameters.
    timeout: float
        The timeout duration for setting up the action servers and waiting for results.
    
    Parameters
    ----------
    object_to_grasp: str
        The name of the object to grasp. If specified, the server will grasp this object. If not
        specified (empty string), the server will grasp the closest object.
    object_poses: list of geometry_msgs.msg.Pose
        The poses of the objects detected by the pose estimator.
    depth: sensor_msgs.msg.Image
        The depth image of the scene.
    class_names: list of str
        The class names of the objects detected by the pose estimator.
    
    Returns
    -------
    grasping_pipeline_msgs.msg.FindGrasppointResult
        The result of the FindGrasppoint action server. Contains the grasp poses, the bounding
        box of the object and the object name.
    '''
    
    def __init__(self):
        '''
        Initializes the FindGrasppointServer.

        Loads the model metadata from the models_metadata.yml file in the model_dir. This file 
        contains the bounding box information for known objects. The server then waits for the
        camera info message. After that it initializes the action server and the rviz marker
        publisher.
        Finally it starts the action server.
        
        Parameters
        ----------
        model_dir: str
            Path to the directory containing the model metadata file.
        '''
        super().__init__('find_grasppoint_server')
        self.declare_parameter('model_dir', '/root/ros2_ws/src/grasping_pipeline/models')
        model_dir = self.get_parameter('model_dir').value
        with open(os.path.join(model_dir, "models_metadata.yml")) as f:
            self.models_metadata = yaml.load(f, Loader=SafeLoader)

        self.declare_parameter('object_to_grasp', None)
        self.declare_parameter('dataset', 'ycb_ichores')
        self.dataset = self.get_parameter('dataset').value

        self._action_server = ActionServer(
            self,
            FindGrasppoint,
            'find_grasppoint',
            execute_callback=self.execute_callback
        )
        
        self.marker_pub = self.create_publisher(Marker, '/grasping_pipeline/grasp_marker', 10)
        

        self.get_logger().debug('Waiting for camera info')
        
        self.declare_parameter('cam_info_topic', '/head_rgbd_sensor/depth_registered/camera_info')
        self.cam_topic = self.get_parameter('cam_info_topic').value
        self.cam_info = None
        self.wait_for_camera_info()

        self.declare_parameter('timeout_duration',40.0)
        self.timeout = float(self.get_parameter('timeout_duration').value)
        self.grasp_annotator = GraspAnnotator(self)
      
        self.get_logger().info('Initializing FindGrasppointServer done')

    '''
     Waits for the camera info message to be received and stores it in the cam_info attribute.
        This function replaces the old rospy.wait_for_message for the camera info, since we need the camera info to convert the depth image
    '''    
    def wait_for_camera_info(self):
        self.cam_info = None

        def cb(msg):
            self.cam_info = msg

        self.create_subscription(
            CameraInfo,
            self.cam_topic,
            cb,
            qos_profile_sensor_data
        )

        self.get_logger().info(f'Waiting for CameraInfo on {self.cam_topic}...')

        while rclpy.ok() and self.cam_info is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.cam_info is None:
            raise RuntimeError('CameraInfo not received')

        self.get_logger().info('CameraInfo received')

    def execute_callback(self, goal_handle):
        '''
        Executes the FindGrasppoint action server.
        
        If an object to grasp is specified in the goal, the server will estimate the grasp pose for
        the specified object. Otherwise, it will estimate the grasp pose for the closest object.
        The server uses the grasp checker which uses grasp-annotations to calculate grasp poses from
        the estimated object pose.
        In the end the server will add markers to the rviz visualization to show the grasp pose and the
        bounding box of the object.
        
        Parameters
        ----------
        goal_handle: grasping_pipeline_msgs.msg.FindGrasppointGoal
            The goal of the FindGrasppoint action server. Contains the object to grasp.
        
        Returns
        -------
        grasping_pipeline_msgs.msg.FindGrasppointResult
            The result of the FindGrasppoint action server. Contains the grasp poses, the bounding 
            box of the object and the object name.
        '''
        self.get_logger().info('Received FindGrasppoint request')
        goal = goal_handle.request
        result = FindGrasppoint.Result()
        
        try:
            scene_cloud, scene_cloud_o3d = convert_ros_depth_img_to_pcd(
                goal.depth, 
                self.cam_info, 
                project_valid_depth_only=False)
            self.get_logger().info(f"Width: {scene_cloud.width}")
            self.get_logger().info(f"Height: {scene_cloud.height}")
            self.get_logger().info(f"is_dense: {scene_cloud.is_dense}")
            self.get_logger().info(f"Fields: {[f.name for f in scene_cloud.fields]}")
            if goal.object_to_grasp != None and goal.object_to_grasp != '':
                param_object_to_grasp = goal.object_to_grasp
            else:
                param_object_to_grasp = self.get_parameter('object_to_grasp').value
                
            # Check if object to grasp is specified and detected
            if param_object_to_grasp != None and param_object_to_grasp != '' and param_object_to_grasp != 'None':
                self.get_logger().info(f'Object to grasp specified. Will grasp specified object {param_object_to_grasp}')
                if param_object_to_grasp in goal.class_names:
                    object_idxs = [goal.class_names.index(param_object_to_grasp)]     # changed           
                else:
                    self.get_logger().warn(f'Object to grasp ({param_object_to_grasp}) not detected. Grasping closest object.')
                    object_idxs = self.get_closest_objects(goal.object_poses)             
            else:
                self.get_logger().info('No object to grasp specified. Will grasp closest object')
                object_idxs = self.get_closest_objects(goal.object_poses)    # changed to return a list of object indices sorted by distance

            for object_idx in object_idxs:

                object_to_grasp = goal.object_poses[object_idx]
                object_name = goal.class_names[object_idx]

                object_to_grasp_stamped = PoseStamped(pose = object_to_grasp, header = goal.depth.header)
                self.get_logger().info(f"Object pose frame: {object_to_grasp_stamped.header.frame_id}")
                self.get_logger().info(f"Object Z before: {object_to_grasp_stamped.pose.position.z}")
                self.get_logger().info(f"Annoting grasps for object {object_name}")

                self.get_logger().debug('Generating grasp poses')

                grasp_poses = self.grasp_annotator.annotate(object_to_grasp_stamped, scene_cloud, object_name)
                self.get_logger().info(f"got grasp_poses")
                object_bb_stamped = self.get_bb_for_known_objects(object_to_grasp_stamped, object_name, goal.depth.header.frame_id, goal.depth.header.stamp)
                self.get_logger().info(f"got object_bb_stamped")
                if grasp_poses is None or len(grasp_poses) < 1:
                    self.get_logger().error(f"No grasp pose found for object {object_name}")
                    continue
                self.get_logger().info(f"Object pose frame: {grasp_poses[0].header.frame_id}")
                self.get_logger().info(f"Object Z after: {grasp_poses[0].pose.position.z}")
                result.grasp_poses = grasp_poses
                result.grasp_object_bb = object_bb_stamped
                result.grasp_object_name = object_name

                self.add_marker(grasp_poses[0])
                self.add_bb_marker(object_bb_stamped)
                goal_handle.succeed()
                return result
            
            self.get_logger().error("No valid grasp found for any object")
            goal_handle.abort()
            return result

        except (ValueError, TimeoutError) as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return result


    def transform_to_kdl(self, pose):
        '''
        Converts a geometry_msgs.msg.Pose to a PyKDL.Frame.
        
        Parameters
        ----------
        pose: geometry_msgs.msg.Pose
            The pose to convert.
        
        Returns
        -------
        PyKDL.Frame
            The converted pose.
        '''
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                                 pose.orientation.z, pose.orientation.w),
                       PyKDL.Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z))

    def get_bb_for_known_objects(self, object_pose, object_name, frame_id, stamp):
        '''
        Generates a bounding box for known objects based on the object pose and object name.
        
        Loads the bounding box information for the object from the models_metadata.yml file.
        
        Parameters
        ----------
        object_pose: geometry_msgs.msg.PoseStamped
            The pose of the object.
        object_name: str
            The name of the object. Used to look up the bounding box information.
        frame_id: str
            The frame id of the object pose.
        stamp: rospy.Time
            The timestamp of the object pose.

        Returns
        -------
        grasping_pipeline_msgs.msg.BoundingBoxStamped
            The bounding box for the object with frame id and timestamp.
        '''
        dataset = self.dataset        
        metadata = self.models_metadata[dataset][object_name]
        center = metadata['center']
        extent = metadata['extent']
        rot_mat = metadata['rot']
        bb_stamped = create_ros_bb_stamped(center, extent, rot_mat, frame_id, rclpy.time.Time().to_msg()) # maybe back to stamp
        t1 = self.transform_to_kdl(bb_stamped.center)
        t2 = self.transform_to_kdl(object_pose.pose)
        t_res = t2 * t1
        #t_res_ros = posemath.toMsg(t_res)
        t_res_ros =  self.kdl_to_pose(t_res)
        bb_stamped.center = t_res_ros
        return bb_stamped
    '''
    def kdl_to_pose (self, pose):
        return posemath.toMsg(t_res) from PyKDL.Frame to  geometry_msgs.msg.Pose
    '''
    def kdl_to_pose(self, frame: PyKDL.Frame) -> Pose:
        pose = Pose()

        x, y, z = frame.p
        qx, qy, qz, qw = frame.M.GetQuaternion()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        return pose

    def get_closest_objects(self, object_poses):
        '''
        Returns a sorted list with indices of the closest objects. The closest object is at index 0.
        
        The closest object is determined by the distance to the camera. The distance is calculated
        as the euclidean distance from the camera to the object pose.
        
        Parameters
        ----------
        estimator_result: GenericImgProcAnnotatorResult
            The result of the pose estimator.
        
        Raises
        ------
        ValueError
            If no close object is found.
        
        Returns
        -------  
        list of int
            The indices of the closest objects in the estimator result.
        '''

        object_idx = []
        distances = []

        for i, pose in enumerate(object_poses):
            pose_pos = pose.position
            dist = pose_pos.x*pose_pos.x + pose_pos.y*pose_pos.y + pose_pos.z*pose_pos.z
            distances.append(dist)
            object_idx.append(i)
        
        distances_np = np.array(distances)
        sort_idx = np.argsort(distances_np)
        return np.array(object_idx)[sort_idx]

    
        
    def add_marker(self, pose_goal):
        """ 
        Adds a marker to the rviz visualization to show the grasp pose.
        
        The marker is an arrow pointing in the direction of the grasp pose. The arrow is red.
        The topic of the marker is '/grasping_pipeline/grasp_marker'. The marker is published
        in the namespace 'grasp_marker' with id 0.
        
        Parameters
        ----------
        pose_goal: geometry_msgs.msg.PoseStamped
            The pose of the grasp.
        """
        marker = Marker()
        marker.header.frame_id = pose_goal.header.frame_id
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = 'grasp_marker'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        q2 = [pose_goal.pose.orientation.w, pose_goal.pose.orientation.x,
              pose_goal.pose.orientation.y, pose_goal.pose.orientation.z]
        q = quaternion_about_axis(pi / 2, (0, 1, 0))
        q = quaternion_multiply(q, q2)

        marker.pose.orientation.w = q[0]
        marker.pose.orientation.x = q[1]
        marker.pose.orientation.y = q[2]
        marker.pose.orientation.z = q[3]
        marker.pose.position.x = pose_goal.pose.position.x
        marker.pose.position.y = pose_goal.pose.position.y
        marker.pose.position.z = pose_goal.pose.position.z

        marker.scale.x = 0.1
        marker.scale.y = 0.05
        marker.scale.z = 0.01

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        self.get_logger().info('grasp_marker')

    
    def add_bb_marker(self, object_bb_stamped):
        '''
        Adds a marker to the rviz visualization to show the bounding box of the object.

        The marker is a blue bounding box.
        The topic of the marker is '/grasping_pipeline/grasp_marker'. The marker is published
        in the namespace 'bb_marker' with id 0.

        Parameters
        ----------
        object_bb_stamped: grasping_pipeline_msgs.msg.BoundingBoxStamped
            The bounding box of the object.
        '''
        marker = Marker()
        marker.header.frame_id = object_bb_stamped.header.frame_id
        marker.header.stamp = rclpy.time.Time().to_msg()
        marker.ns = 'bb_marker'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = deepcopy(object_bb_stamped.center)
        marker.scale = deepcopy(object_bb_stamped.size)

        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = FindGrasppointServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()