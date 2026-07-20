#! /usr/bin/env python3
#!/usr/bin/env python3

import traceback

import rclpy
from rclpy.node import Node, MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from copy import deepcopy

from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from actionlib_msgs.msg import GoalStatus

from grasping_pipeline_msgs.srv import CallDirectGraspPoseEstimator
from robokudo_msgs.action import GenericImgProcAnnotator

from v4r_util.depth_pcd import convert_np_depth_img_to_o3d_pcd
from v4r_util.bb import get_minimum_oriented_bounding_box, o3d_bb_to_ros_bb_stamped

from tf_transformations import quaternion_about_axis, quaternion_multiply


class DirectGraspposeEstimatorCaller(Node):
    '''Calls a service that directly estimates the grasppose.
    
    Calls a service that directly estimates the grasppose without needing an 
    object pose and annotations. The topic of the service is defined in the
    parameter '/grasping_pipeline/grasppoint_estimator_topic'. The results are 
    published as a marker in the rviz visualization with the topic 
    '/grasping_pipeline/grasp_marker'.

    If an object to grasp is specified, the service will only estimate the
    grasppose of the specified object. If no object is specified, the service
    will only estimate the grasppose of the closest object to the camera.

    Parameters
    ----------
    rgb: sensor_msgs.msg.Image
        The RGB image of the scene.
    depth: sensor_msgs.msg.Image
        The depth image of the scene.
    mask_detections: list of sensor_msgs.msg.Image
        The object masks.
    bb_detections: list of sensor_msgs.msg.RegionOfInterest
        The 2D bounding boxes of the objects.
    class_names: list of str
        The class names of the objects.
    object_to_grasp: str
        The name of the object to grasp. If empty, the closest object to the camera is grasped.
    
    Attributes
    ----------
    bridge: CvBridge
        The OpenCV bridge to convert images.
    srv: rospy.Service
        This service. Calls the direct grasppose estimator when called.
    cam_info: sensor_msgs.msg.CameraInfo
        The camera information/intrinsics.
    marker_pub: rospy.Publisher
        The publisher that publishes the markers in the rviz visualization.
    
    Raises
    ------
    rospy.ServiceException
        If the direct grasppose estimator service times out or fails to estimate the grasppose
        or if no mask or bounding box detections are provided.
    
    Returns
    -------
    grasp_poses: list of geometry_msgs.msg.PoseStamped
        The estimated graspposes.
    grasp_object_bb: grasping_pipeline_msgs.msg.BoundingBoxStamped
        The bounding box of the object to grasp.
    grasp_object_name: str
        The name of the object to grasp.
    '''
    def __init__(self):
        super().__init__('grasppose_estimator')

        self.bridge = CvBridge()

        self.declare_parameter('cam_info_topic', '/head_rgbd_sensor/depth_registered/camera_info')
        self.declare_parameter('grasping_pipeline.grasppoint_estimator_topic', '/pose_estimator/find_grasppose_haf')
        self.declare_parameter('grasping_pipeline.timeout_duration', 40.0)
        self.cam_topic = self.get_parameter('cam_info_topic').value
        self.action_topic = self.get_parameter('grasping_pipeline.grasppoint_estimator_topic').value
        self.timeout = float(self.get_parameter('grasping_pipeline.timeout_duration').value)

        # replaces the old wait_for_message in rospy to get the camera info, since we need the camera info to convert the depth image 
        # to a point cloud to extract the 3D bounding boxes of the objects
        self.cam_info = None
        self.wait_for_camera_info()

        self.cbgroup = MutuallyExclusiveCallbackGroup()
        # Service
        self.srv = self.create_service(
            CallDirectGraspPoseEstimator,
            'call_direct_grasppose_estimator',
            self.execute
        )

         # Publisher
        self.marker_pub = self.create_publisher(
            Marker,
            '/grasping_pipeline/grasp_marker',
            10
        )

        # Action client
        self.action_client = ActionClient(
            self,
            GenericImgProcAnnotator,
            self.action_topic,
            callback_group=self.cbgroup
        )

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


    async def execute(self, request, response):
        '''
        Calls the direct grasppose estimator service and returns the grasppose.

        If an object to grasp is specified, the service will only estimate the
        grasppose of the specified object. If no object is specified, the service
        will only estimate the grasppose of the closest object to the camera.

        If no mask or bounding box detections are provided, the service will raise
        an error.

        Parameters
        ----------
        request: grasping_pipeline_msgs.srv.CallDirectGraspPoseEstimatorRequest
            The request to the service. Contains the RGB image, depth image, mask detections,
            bounding box detections, class names, and (optionally) the name of the object to grasp.

        Raises
        ------
        rospy.ServiceException
            If the direct grasppose estimator service times out or fails to estimate the grasppose
            or if no mask or bounding box detections are provided.
        
        Returns
        -------
        grasping_pipeline_msgs.srv.CallDirectGraspPoseEstimatorResponse
            The response of the service. Contains the name of the object to grasp, the bounding box
            of the object, and the estimated graspposes.
        '''

        self.get_logger().info(f'Waiting for direct-grasppose-estimator server with topic: {self.action_topic}')
        if not self.action_client.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error(f'Connection to direct-grasppose_estimator \'{self.action_topic}\' timed out!')
            return response

        self.get_logger().info('Connected to direct-grasppose-estimator server')

        bbs_3D = self.get_3D_bbs(request.depth, request.mask_detections, request.bb_detections)
        center_poses = self.get_bb_center_poses(bbs_3D)
        
        if request.object_to_grasp != None and request.object_to_grasp != '':
            if request.object_to_grasp in request.class_names:
                self.get_logger().info(f'Object to grasp specified. Will grasp specified object {request.object_to_grasp}')
                object_idx = request.class_names.index(request.object_to_grasp)               
            else:
                self.get_logger().warn(f'Object to grasp {request.object_to_grasp} not detected. Aborting')
                return response
        else:
            self.get_logger().info('No object to grasp specified. Will grasp closest object')
            object_idx = self.get_closest_object(center_poses)
            self.get_logger().info(f'Closest object to camera is {request.class_names[object_idx]}')
        
        object_mask, object_bb = [], []
        if len(request.mask_detections) > 0:
            object_mask = [request.mask_detections[object_idx]]
        if len(request.bb_detections) > 0:
            object_bb = [request.bb_detections[object_idx]]
        if len(object_mask) <= 0 and len(object_bb) <= 0:
            self.get_logger().error('No mask or bb detections provided! Need at least either one to proceed!')
            return response
        
        goal = GenericImgProcAnnotator.Goal()
        goal.rgb=request.rgb
        goal.depth=request.depth
        goal.mask_detections=object_mask
        goal.bb_detections=object_bb
        goal.class_names=[request.class_names[object_idx]]
        

        self.get_logger().info('Sending goal to direct-grasppose-estimator')
        send_goal_future = self.action_client.send_goal_async(goal)
        self.get_logger().info('Waiting for direct-grasppose-estimator')
        await send_goal_future
        

        goal_handle_client = send_goal_future.result()

        if not goal_handle_client.accepted:
            self.get_logger().error("direct-grasppose-estimator rejected goal")
            return response

        result_future = goal_handle_client.get_result_async()
        await result_future

        result = result_future.result()

        if result is None:
            self.get_logger().error("No result received")
            return response

        pose_result = result.result

        if result.status != GoalStatus.SUCCEEDED + 1 or len(pose_result.pose_results) == 0:
            self.get_logger().error(f"direct-grasppose-estimator failed with status code: {result.status} and {len(pose_result.pose_results)} pose results")
            return response


        graspposes = result.result
        self.get_logger().info(f'Estimated the grasppose of {len(graspposes.class_names)} objects.')
        if not len(graspposes.pose_results) == 1:
            self.get_logger().error('Expected only one grasppose result, but got more than one!')
            return response

        grasp_posese = PoseStamped()
        grasp_posese.header = request.depth.header
        grasp_posese.pose = graspposes.pose_results[0]
        response.grasp_object_name = request.class_names[object_idx]
        response.grasp_object_bb = bbs_3D[object_idx]
        response.grasp_poses = [grasp_posese]

        self.add_bb_marker(response.grasp_object_bb)
        self.add_marker(response.grasp_poses[0])
        self.get_logger().info("finished direct grasppose estimation")
        return response

    def get_bb_center_poses(self, bbs):
        '''Extracts the center poses of the bounding boxes.

        Parameters
        ----------
        bbs: list of 
            A list of bounding boxes.
        
        Returns
        -------
        list
            A list of the center poses of the bounding boxes.
        '''
        center_poses = []
        for bb in bbs:
            center_poses.append(bb.center)
        return center_poses
    
    def get_3D_bbs(self, depth, masks, bbs_2d):
        '''Extracts the 3D bounding boxes of the objects.

        If masks are provided, the 3D bounding boxes are extracted from the masks 
        and the depth image. Otherwise, if 2D bounding boxes are provided, the 3D
        bounding boxes are extracted from the 2D bounding boxes. If neither masks
        nor 2D bounding boxes are provided, an error is raised.

        Parameters
        ----------
        depth: sensor_msgs.msg.Image
            The depth image.
        masks: list of sensor_msgs.msg.Image
            The object masks.
        bbs_2d: list of sensor_msgs.msg.RegionOfInterest
            The 2D bounding boxes of the objects.
        '''
        bbs = []
        depth_np = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
        if len(masks) > 0:
            for mask in masks:
                bb = self.get_bb_3D_from_mask(depth, depth_np, mask)
                bbs.append(bb)
        elif len(bbs_2d) > 0:
            for bb in bbs_2d:
                self.get_bb_3D_from_bb(depth, depth_np, bb)
                bbs.append(bb)
        else:
            self.get_logger().error('No masks or bbs provided to extract object depth values!')
        return bbs
        
    def get_bb_3D_from_mask(self, depth, depth_np, mask):
        '''Extracts the 3D bounding box of the object from the mask.

        Parameters
        ----------
        depth: sensor_msgs.msg.Image
            The depth image.
        depth_np: numpy.ndarray
            The depth image as a numpy array.
        mask: sensor_msgs.msg.Image
            The object mask.

        Returns
        -------
        grasping_pipeline_msgs.msg.BoundingBoxStamped
            The 3D bounding box of the object.
        '''
        depth_img_obj = np.full_like(depth_np, np.nan, dtype=np.float32)
        mask = self.bridge.imgmsg_to_cv2(mask, desired_encoding='passthrough')
        mask = mask != 0
        # only copy the object's depth values, rest stay NaN
        depth_img_obj[mask] = depth_np[mask]
        object_pcd = convert_np_depth_img_to_o3d_pcd(depth_img_obj, self.cam_info, project_valid_depth_only=True)
        
        # do clustering to remove parts from the background
        labels = object_pcd.cluster_dbscan(eps=0.04, min_points=100, print_progress=False)
        labels_unique, counts = np.unique(labels, return_counts=True)
        max_label = labels_unique[np.argmax(counts)]
        object_pcd = object_pcd.select_by_index(np.where(labels == max_label)[0])
        
        obj_bb_o3d = get_minimum_oriented_bounding_box(object_pcd)
        return o3d_bb_to_ros_bb_stamped(obj_bb_o3d, depth.header.frame_id, depth.header.stamp)

    def get_bb_3D_from_bb(self, depth, depth_np, bb_2d):
        '''Extracts the 3D bounding box of the object from the 2D bounding box.

        Parameters
        ----------
        depth: sensor_msgs.msg.Image
            The depth image.
        depth_np: numpy.ndarray
            The depth image as a numpy array.
        bb_2d: sensor_msgs.msg.RegionOfInterest
            The 2D bounding box of the object.
        '''
        depth_img_obj = np.full_like(depth_np, np.nan)
        bb = bb_2d
        y, x = bb.y_offset, bb.x_offset
        h, w = bb.height, bb.width
        # only copy the object's depth values, rest stay NaN
        depth_img_obj[y:y+h, x:x+w] = depth_np[y:y+h, x:x+w]
        object_pcd = convert_np_depth_img_to_o3d_pcd(depth_img_obj, self.cam_info, project_valid_depth_only=True)
        
        # do clustering to remove parts from the background
        labels = object_pcd.cluster_dbscan(eps=0.04, min_points=100, print_progress=False)
        labels_unique, counts = np.unique(labels, return_counts=True)
        max_label = labels_unique[np.argmax(counts)]
        object_pcd = object_pcd.select_by_index(np.where(labels == max_label)[0])
        
        obj_bb_o3d = get_minimum_oriented_bounding_box(object_pcd)
        return o3d_bb_to_ros_bb_stamped(obj_bb_o3d, depth.header.frame_id, depth.header.stamp)
    
    def get_closest_object(self, object_poses):
        '''
        Returns the index of the closest object in the estimator result.
        
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
        int
            The index of the closest object in the estimator result.
        '''
        min_dist_squared = float("inf")
        object_idx = None

        for i, pose in enumerate(object_poses):
            pose_pos = pose.position
            dist_squared = pose_pos.x*pose_pos.x + pose_pos.y*pose_pos.y + pose_pos.z*pose_pos.z
            if dist_squared < min_dist_squared:
                min_dist_squared = dist_squared
                object_idx = i

        if object_idx is None:
            raise ValueError("No close object found")
        
        return object_idx

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
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'grasp_marker'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        q2 = [pose_goal.pose.orientation.w, pose_goal.pose.orientation.x,
              pose_goal.pose.orientation.y, pose_goal.pose.orientation.z]
        q = quaternion_about_axis(3.1415 / 2, (0, 1, 0))
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
        marker.header.stamp = self.get_clock().now().to_msg()
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


def main():
    rclpy.init()
    node = DirectGraspposeEstimatorCaller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()