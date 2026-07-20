import rclpy
import numpy as np
from rclpy.node import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy
import yasmin
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from grasping_pipeline_msgs.srv import TablePlaneExtractor
from std_msgs.msg import Float32MultiArray
from v4r_util.tf2 import TF2Wrapper
from v4r_util.conversions import bounding_box_to_bounding_box_stamped
from v4r_util.bb import align_bounding_box_rotation, ros_bb_to_o3d_bb, o3d_bb_to_ros_bb
from typing import Any
from rclpy.qos import qos_profile_sensor_data


class FindTablePlanes(yasmin.State):
    '''
    Finds the table planes in the point cloud using the TablePlaneExtractor service.
    
    Returns
    -------
    table_bbs: vision_msgs/BoundingBox3DArray
        The bounding boxes of the detected planes. The first one is considered the actual table.
    table_plane_equations: list of object_detector_msgs/Plane
        The plane equations of the detected planes.
    '''

    def __init__(self, node, enlarge_table_bb_to_floor=True):
        '''
        Initializes the FindTablePlanes state. Creates a TablePlaneExtractor service client.
        
        Parameters
        ----------
        node: rclpy.node.Node
            The ROS2 node
        enlarge_table_bb_to_floor: bool
            If True, the bounding boxes of the tables are enlarged to the floor. This is useful
            to prevent the robot from colliding with the table legs by creating a box that reaches
            the floor.
        '''
        self.node = node
        super().__init__(['succeeded'])
        if not self.node.has_parameter('point_cloud_topic'):
            self.node.declare_parameter('point_cloud_topic', '/head_rgbd_sensor/depth_registered/rectified_points')
        if not self.node.has_parameter('grasping_pipeline.dataset'):
            self.node.declare_parameter('grasping_pipeline.dataset', 'ycb_ichores')
        if not self.node.has_parameter('table_extractor_server_name'):
            self.node.declare_parameter('table_extractor_server_name', '/table_plane_extractor/get_planes')
        self.table_params = {k: v.value for k, v in self.node.get_parameters_by_prefix("table_plane_extractor_server").items()}
        self.topic = self.node.get_parameter('point_cloud_topic').value
        self.table_extractor_srv_name = self.node.get_parameter('table_extractor_server_name').value
        self.cbgroup = MutuallyExclusiveCallbackGroup()
        self.table_extractor = self.node.create_client(
                TablePlaneExtractor, 
                self.table_extractor_srv_name, 
                callback_group=self.cbgroup
            )
        self.node.get_logger().info('Waiting for table plane extractor service...')
        self.table_extractor.wait_for_service(timeout_sec=10.0)
        
        self.tf_wrapper = TF2Wrapper(self.node)
        self.enlarge_table_bb_to_floor = enlarge_table_bb_to_floor
        self.point_cloud = None
        self.point_cloud_subscription = self.node.create_subscription(
            PointCloud2,
            self.topic,
            self._point_cloud_callback,
            qos_profile_sensor_data
        )

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.plane_publisher = self.node.create_publisher(
            Float32MultiArray,
            '/grasping_pipeline/plane_normal',
            qos
        )
        self.point_publisher = self.node.create_publisher(
            Point,
            '/grasping_pipeline/plane_point',
            qos
        )

    def _point_cloud_callback(self, msg: PointCloud2) -> None:
        '''Store the latest point cloud'''
        self.point_cloud = msg

    def execute(self, blackboard: yasmin.Blackboard) -> str:
        '''
        Waits for a point cloud and calls the TablePlaneExtractor service.
        
        Returns
        -------
        'succeeded': The state succeeded.
        '''
        self.node.get_logger().info('Executing state FIND_TABLE_PLANES. Waiting for point cloud.')
        
        # Wait for point cloud with timeout
        timeout_count = 0
        while self.point_cloud is None and timeout_count < 200:  # 20 seconds at 10Hz
            rclpy.spin_once(self.node, timeout_sec=0.1)
            timeout_count += 1
        
        if self.point_cloud is None:
            self.node.get_logger().error('Timeout waiting for point cloud')
            return 'succeeded'
        
        cloud = self.point_cloud
        self.node.get_logger().info('Received point cloud. Calling table plane extractor service.')
        
        # Call the service
        request = TablePlaneExtractor.Request()
        request.point_cloud = cloud
        
        future =  self.table_extractor.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=40.0)
        # Wait for the service to complete
        timeout_count = 0
        # while not future.done() and timeout_count < 600: # 60 seconds at 10Hz increased for sim testing
        #     rclpy.spin_once(self.node, timeout_sec=0.1)
        #     timeout_count += 1
        
        if not future.done():
            self.node.get_logger().error('Timeout waiting for table plane extractor service')
            return 'succeeded'
        
        try:
            response = future.result()
        except Exception as e:
            self.node.get_logger().error(f'Service call failed: {e}')
            return 'succeeded'
        
        boxes = response.plane_bounding_boxes
        
        if self.enlarge_table_bb_to_floor:
            transform_to_base = False
            if boxes.header.frame_id != 'base_link':
                transform_to_base = True

            new_boxes = []
            for ros_bb in boxes.boxes:
                if transform_to_base:
                    ros_bb = bounding_box_to_bounding_box_stamped(ros_bb, boxes.header.frame_id, self.node.get_clock().now().to_msg())
                    ros_bb = self.tf_wrapper.transform_bounding_box(ros_bb, 'base_link')
                
                aligned_bb_o3d = align_bounding_box_rotation(ros_bb_to_o3d_bb(ros_bb))
                ros_bb = o3d_bb_to_ros_bb(aligned_bb_o3d)

                center = ros_bb.center.position
                old_center_z = center.z
                size = ros_bb.size
                center.z = (center.z + size.z/2)/2
                size.x = size.x + 0.04
                size.y = size.y + 0.04
                size.z = old_center_z + size.z/2 - 0.02
                new_boxes.append(ros_bb)

            response.plane_bounding_boxes.boxes = new_boxes
            response.plane_bounding_boxes.header.frame_id = 'base_link'

        blackboard["table_bbs"] = response.plane_bounding_boxes
        blackboard["table_plane_equations"] = response.planes

        if self.node.get_parameter('grasping_pipeline.dataset') == 'tracebotcanister':
            
            # Extract plane equation
            a = response.planes[0].x
            b = response.planes[0].y
            c = response.planes[0].z
            d = response.planes[0].d 

            normal = [a, b, c]

            normal_camera = self.tf_wrapper.transform_3d_array(self.table_params['base_frame'],  cloud.header.frame_id, normal)
            
            msg = Float32MultiArray()
            msg.data = [float(n) for n in normal_camera]
            self.plane_publisher.publish(msg)
            
            # Picking center point of bbox
            bb = response.plane_bounding_boxes.boxes[0]
            x_min, y_min, z_min = bb.center.position.x - bb.size.x/2, bb.center.position.y - bb.size.y/2, bb.center.position.z - bb.size.z/2
            x_max, y_max, z_max = bb.center.position.x + bb.size.x/2, bb.center.position.y + bb.size.y/2, bb.center.position.z + bb.size.z/2
            x0, y0 = (x_min + x_max) / 2, (y_min + y_max) / 2

            # Calculate corresponding z0 using the plane equation and check if it's within the bounding box
            z0 = (-d - a*x0 - b*y0) / c
            if z_min <= z0 <= z_max:
                plane_pt = np.array([x0, y0, z0], dtype=np.float32)
                
                header = Header()
                header.frame_id = response.plane_bounding_boxes.header.frame_id
                header.stamp = rclpy.time.Time().to_msg()
                plane_pt_s = PointStamped()
                plane_pt_s.header = header
                plane_pt_s.point.x = float(plane_pt[0])
                plane_pt_s.point.y = float(plane_pt[1])
                plane_pt_s.point.z = float(plane_pt[2])
                plane_pt_s_camera = self.tf_wrapper.transform_point(cloud.header.frame_id,plane_pt_s)
                plane_pt_camera = [plane_pt_s_camera.point.x,plane_pt_s_camera.point.y,plane_pt_s_camera.point.z]
                
                self.point_publisher.publish(Point(x=plane_pt_camera[0], y=plane_pt_camera[1], z=plane_pt_camera[2]))
            else:
                self.node.get_logger().warn('Calculated z0 is outside the bounding box. Using center of bounding box instead.')

        self.node.get_logger().info('Table planes extracted. Returning succeeded.')
        return 'succeeded'