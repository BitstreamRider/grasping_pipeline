#! /usr/bin/env python3
import os
import sys
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import open3d as o3d
from v4r_util.rviz_visualization.image_visualization import PoseEstimationVisualizer
from v4r_util.conversions import ros_poses_to_np_transforms
from sensor_msgs.msg import Image, CameraInfo
from object_detector_msgs.srv import VisualizePoseEstimation

class PoseEstimationVisualizerRos(Node, PoseEstimationVisualizer):
    
    def __init__(
        self, 
        topic, 
        image_width, 
        image_height, 
        intrinsics_matrix, 
        model_dir,
        expose_service=False, 
        service_name=None
        ):
        '''
        Renders the pose estimation results of the object detector into an image and publishes it
        
        Renders model-contours and the corresponding modelnames of the detected objects 
        into the given image and publishes the result

        Parameters
        ----------
        topic: str
            ROS Topic to publish the rendered image to
        image_width: int
            width of input image and rendered image
        image_height: int
            height of input image and rendered image
        intrinsics_matrix: list or numpy array 
            flattened [9x1] camera matrix, e.g. [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        expose_service: bool
            whether to expose a service which can be used to trigger the visualization
        service_name: str
            name of the service that should be exposed, only needed if expose_service is True
        '''
        super().__init__('pose_estimation_visualizer')

        
        qos_profile = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.declare_parameter('dataset', 'ycb_bop')
        self.declare_parameter('result_visualization_topic', '/pose_estimator/result_visualization')
        self.declare_parameter('result_visualization_service_name', '/pose_estimator/result_visualization_service')
        self.bridge = CvBridge()
        self.image_width = image_width
        self.image_height = image_height
        self.intrinsics_matrix = intrinsics_matrix
        self.meshes = self.load_meshes(model_dir)
        self.get_logger().info(f'PoseEstimVis: Loaded {len(self.meshes)} meshes')

        self.image_pub = self.create_publisher(Image, topic, qos_profile)
        if expose_service:
            self.get_logger().info(f'PoseEstimVis: Exposing service {service_name}')
            assert service_name is not None
            self.service = self.create_service(
                VisualizePoseEstimation,
                service_name,
                self.service_callback
            )

        self.renderer_initialized = False
        self.get_logger().info(f'PoseEstimationVisualizerRos initialized, publishing to {topic}')
            
    def service_callback(self, request, response):
        '''
        Service callback for the pose estimation visualization service.
        
        Renders the contours of the objects based on the given poses into the passed image. 
        Additionally, the model names are rendered next to the contours.
        
        Parameters
        ----------
        req: object_detector_msgs.srv.VisualizePoseEstimationRequest
            The request containing: the rgb_image, a list of model_poses and a list of model_names
            
        Returns
        -------
        object_detector_msgs.srv.VisualizePoseEstimationResponse
            Empty response
        
        '''
        self.get_logger().info("PoseEstimVis: Received service call")
        # renderer needs to be initialized in the same thread that executes the callback
        # else we get some threading related cpp exceptions => most robust way to do it is to just
        # initialize the renderer in every callback and delete it afterwards 
        # This is not that bad since the renderer is quite lightweight (takes like ~20ms to initialize)
        self.get_logger().info("PoseEstimVis: Initializing renderer")
        PoseEstimationVisualizer.__init__(
            self,
            self.image_width,
            self.image_height,
            self.intrinsics_matrix
        )
        dataset = self.get_parameter('dataset').get_parameter_value().string_value
        meshes = []
        for name in request.model_names:
            mesh = self.meshes.get(dataset, {}).get(name, None)
            if mesh is None and name != 'Unknown':
                self.get_logger().warn(f'No mesh for model {name} found!')
                continue
            meshes.append(copy.deepcopy(mesh))

        try:
            self.publish_pose_estimation_result(
                request.rgb_image, 
                request.model_poses, 
                meshes, 
                request.model_names
                )
        except Exception as e:
            print(f"AAAH exception in PoseEstimVis: {e}")
        self.get_logger().info("PoseEstimVis: service call finished")

        # Delete renderer so that it properly cleans itself and is ready to get initialized next time
        del self.renderer 
        return response
    
    def publish_pose_estimation_result(self, ros_image, ros_model_poses, model_meshes, model_names):
        '''
        Renders contours of models and modelnames into an image and publishes the result

        Parameters
        ----------
        ros_image: sensor_msgs.msg.Image
            The input image to render the model contours into
        ros_model_poses: list of geometry_msgs.msg.Pose
            The poses of the models in the camera frame
        model_meshes: list of open3d.geometry.TriangleMesh
            The meshes of the models to render, scaled to meters
        model_name: list of str
            names of the models
        '''
        model_poses = ros_poses_to_np_transforms(ros_model_poses)
        np_img = self.bridge.imgmsg_to_cv2(ros_image)
        vis_img = self.create_visualization(np_img, model_poses, model_meshes, model_names)
        vis_img_ros = self.bridge.cv2_to_imgmsg(vis_img, encoding='rgb8')
        self.image_pub.publish(vis_img_ros)
      
    def load_meshes(self, model_dir):
        '''
        Load .stl meshes from the given directory and return them as a dictionary
        
        Parameters
        ----------
        model_dir: str
            The directory containing the .stl meshes that should be loaded
        
        Returns
        -------
        dict
            A dictionary containing the loaded meshes with the model name as the key. 
            The models are scaled to meters. The names are the filenames without the .stl extension.
        '''
        meshes = {}
        for dataset_name in os.listdir(model_dir):
            if not os.path.isdir(os.path.join(model_dir, dataset_name)):
                continue
            meshes[dataset_name] = {}
            for mesh_file in os.listdir(os.path.join(model_dir, dataset_name)):
                if not mesh_file.endswith('.stl') and not mesh_file.endswith('.ply'):
                    continue
                model_name = mesh_file.split('.')[0]
                path = os.path.join(model_dir, dataset_name, mesh_file)
                mesh = o3d.io.read_triangle_mesh(path)
                depth_mm_to_m = 0.001
                mesh.scale(depth_mm_to_m, center = [0, 0, 0])
                meshes[dataset_name][model_name] = mesh
        return meshes

def wait_for_camera_info(node, topic):
    future = rclpy.task.Future()

    def callback(msg):
        future.set_result(msg)

    sub = node.create_subscription(CameraInfo, topic, callback, 10)

    rclpy.spin_until_future_complete(node, future)
    node.destroy_subscription(sub)

    return future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print('No model_dir specified!')
        sys.exit(-1)

    model_dir = sys.argv[1]

    temp_node = Node('temp_node')

    cam_info_topic = '/head_rgbd_sensor/depth_registered/camera_info'
    cam_info = wait_for_camera_info(temp_node, cam_info_topic)

    image_width = cam_info.width
    image_height = cam_info.height
    intrinsics_matrix = cam_info.k

    temp_node.destroy_node()

    node = PoseEstimationVisualizerRos(
        topic='/pose_estimator/result_visualization',
        image_width=image_width,
        image_height=image_height,
        intrinsics_matrix=intrinsics_matrix,
        model_dir=model_dir,
        expose_service=True,
        service_name='/pose_estimator/result_visualization_service'
    )

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
