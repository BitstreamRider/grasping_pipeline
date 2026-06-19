#! /usr/bin/env python3
from urllib import response

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node, MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from actionlib_msgs.msg import GoalStatus
from robokudo_msgs.action import GenericImgProcAnnotator
from grasping_pipeline_msgs.srv import CallPoseEstimator, VisualizePoseEstimation
from sensor_msgs.msg import Image, RegionOfInterest


class CallPoseEstimatorService(Node):
    '''
    This class provides a service that calls a pose estimator to estimate the pose of objects in an image.

    The service sends an action goal to the pose estimator with the input image, bounding box detections, and
    mask detections. The pose estimator returns the estimated poses of the objects in the image.

    Parameters
    ----------
    rgb : sensor_msgs.msg.Image
        RGB image of the scene
    depth : sensor_msgs.msg.Image
        Depth image of the scene
    bb_detections : list of sensor_msgs.msg.RegionOfInterest
        Bounding box detections of the objects in the image
    mask_detections : list of sensor_msgs.msg.Image
        Masks of the objects in the image
    class_names : list of str
        Names of the detected objects
    class_confidences : list of float
        Confidence scores of the detected objects

    Attributes
    ----------
    bridge : CvBridge
        Converts between ROS Image messages and OpenCV images
    srv : rospy.Service
        Service that calls the pose estimator when called
    res_vis_service : rospy.ServiceProxy
        Service that visualizes the pose estimation result
    
    Other Parameters
    ----------------
    res_vis_service_name : str
        Topic of the result visualization service. 
        Loaded from the 'result_visualization_service_name' parameter
    
    Returns
    -------
    class_confidences : list of float
        Confidence scores of the detected objects
    class_names : list of str
        Names of the detected objects
    pose_results : list of geometry_msgs.msg.Pose
        Estimated poses of the detected objects in the image frame.
    '''

    def __init__(self):
        super().__init__('pose_estimator')
        self.declare_parameter('grasping_pipeline.result_visualization_service_name','/pose_estimator/result_visualization_service')
        self.declare_parameter('grasping_pipeline.pose_estimator_topic', '/pose_estimator/gdrnet' )
        self.declare_parameter('grasping_pipeline.timeout_duration', 40.0 )
        
        self.res_vis_service_name = self.get_parameter('grasping_pipeline.result_visualization_service_name').value
        self.topic = self.get_parameter('grasping_pipeline.pose_estimator_topic').value
        self.timeout = self.get_parameter('grasping_pipeline.timeout_duration').value
        
        self.bridge = CvBridge()
        self.action_server = self.create_service(
            CallPoseEstimator, 
            'call_pose_estimator',   
            self.execute_callback
        )
        
        self.res_vis_client = self.create_client(VisualizePoseEstimation, self.res_vis_service_name)
        
        
    
    async def execute_callback(self, request, response):
        '''
        Calls the pose estimator to estimate the pose of objects in the input image.

        Parameters
        ----------
        req : grasping_pipeline_msgs.srv.CallPoseEstimatorRequest
            Request containing the input images, bounding box detections, and mask detections,
            class names, and class confidences
        
        Returns
        -------
        grasping_pipeline_msgs.srv.CallPoseEstimatorResponse
            Response containing the estimated poses of the objects in the image, class names
            and class confidences
        '''
        self.cbgroup = MutuallyExclusiveCallbackGroup()
        self.pose_est = ActionClient(self, GenericImgProcAnnotator, self.topic, callback_group=self.cbgroup)

        self.get_logger().info('Waiting for pose estimator server with topic: %s' % self.topic)
        if not self.pose_est.wait_for_server(timeout_sec=self.timeout):
            self.get_logger().error(f'Connection to pose_estimator \'{self.topic}\' timed out!')
            return response
        
        self.get_logger().info('Connected to pose estimator server')
        
        description = ''
        for name, confidence, index in zip(request.class_names, request.class_confidences, range(0, len(request.class_names))):
            if index == 0:
                description = description + f'"{name}": "{confidence}"'
            else:
                description = description + f', "{name}": "{confidence}"'
        description = '{' + description + '}'

        goal_msg = GenericImgProcAnnotator.Goal()
        goal_msg.rgb = request.rgb
        goal_msg.depth = request.depth
        goal_msg.bb_detections = request.bb_detections
        goal_msg.mask_detections = request.mask_detections
        goal_msg.class_names = request.class_names
        goal_msg.description = str(description)

        self.get_logger().info('Sending goal to pose estimator')
        send_goal_future = self.pose_est.send_goal_async(goal_msg)
        self.get_logger().info('Waiting for pose estimation results')
        await send_goal_future
        

        goal_handle_client = send_goal_future.result()

        if not goal_handle_client.accepted:
            self.get_logger().error("Pose estimator rejected goal")
            return response

        result_future = goal_handle_client.get_result_async()
        await result_future

        result = result_future.result()

        if result is None:
            self.get_logger().error("No result received")
            return response

        pose_result = result.result

        if result.status != GoalStatus.SUCCEEDED + 1 or len(pose_result.pose_results) == 0:
            self.get_logger().error(f"Pose estimator failed with status code: {result.status} and {len(pose_result.pose_results)} pose results")
            return response

        self.get_logger().info(f'Estimated the pose of {len(pose_result.class_names)} objects.')

        self.visualize_pose_estimation_result(rgb = request.rgb, model_poses=pose_result.pose_results, model_names=pose_result.class_names)

        pose_result = result.result
        response.class_confidences = pose_result.class_confidences
        response.class_names = pose_result.class_names
        response.pose_results = pose_result.pose_results

        return response
    
    def visualize_pose_estimation_result(self, rgb, model_poses, model_names):
        '''
        Visualizes the pose estimation result using the pose estimator result visualization service.
        
        The service creates and publishes an image with a contour of the detected objects and their 
        object names. This is only possible for known objects because the object's model is used to
        determine the object's contour.
        
        Parameters
        ----------
        rgb: sensor_msgs.msg.Image
            The rgb image.
        model_poses: list of geometry_msgs.msg.Pose
            The poses of the detected objects.
        model_names: list of str
            The names of the detected objects. Used to lookup the object's model which is used to 
            determine the object's contour.
        '''
        request = VisualizePoseEstimation.Request()
        cv_image = self.bridge.imgmsg_to_cv2(rgb, desired_encoding="bgr8")
        request.rgb_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        request.model_poses = model_poses
        request.model_names = model_names

        future = self.res_vis_client.call_async(request)

        
        
    
def main(args=None):
    rclpy.init(args=args)

    node = CallPoseEstimatorService()

    #executor = MultiThreadedExecutor()
    #executor.add_node(node)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    