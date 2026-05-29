import rclpy
import yasmin
from grasping_pipeline.hsr_wrapper import HSR_wrapper
from sensor_msgs.msg import CameraInfo
import numpy as np
from v4r_util.conversions import quat_to_rot_mat, bounding_box_to_bounding_box_stamped
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import RegionOfInterest
from v4r_util.tf2 import TF2Wrapper


class RemoveNonTableObjects(yasmin.State):
    '''
    Checks whether any objects remain on the table.
    '''

    def __init__(self, node):
        super().__init__(['succeeded', 'failed'])
        self.node = node
        self.hsr_wrapper = HSR_wrapper()
        self.tf_wrapper = TF2Wrapper(self.node)
        self.camera_info = None
        self._setup_camera_info_subscription()
        self.bridge = CvBridge()
    
    def project_3d_bb_to_2d(self, bb_3d, camera_info):
        if camera_info.distortion_model != 'plumb_bob':
            raise ValueError('This function only supports plumb_bob distortion model.')
        camera_matrix = np.array(camera_info.k, np.float32).reshape(3, 3)
        dist_coeffs = np.array(camera_info.d, np.float32)
        
        # transform each corner of the 3D bounding box as a 3D point
        x_c = bb_3d.center.position.x
        y_c = bb_3d.center.position.y
        z_c = bb_3d.center.position.z
        rotmat = quat_to_rot_mat(bb_3d.center.orientation)
        width = rotmat @ np.array([bb_3d.size.x/2, 0, 0])
        height = rotmat @ np.array([0, bb_3d.size.y/2, 0])
        depth = rotmat @ np.array([0, 0, bb_3d.size.z/2])
        center = np.array([x_c, y_c, z_c])
        corners = np.array([center + width + height + depth,
                            center + width + height - depth,
                            center + width - height + depth,
                            center + width - height - depth,
                            center - width + height + depth,
                            center - width + height - depth,
                            center - width - height + depth,
                            center - width - height - depth])
        points_2d = cv2.projectPoints(corners, np.zeros(3), np.zeros(3), camera_matrix, dist_coeffs)[0].reshape(-1, 2)
                
        # create a mask based on the projected corner points
        mask = np.zeros((camera_info.height, camera_info.width), np.uint8)
        hull = cv2.convexHull(points_2d.astype(np.int32))
        cv2.fillConvexPoly(mask, hull, 1)
        
        # remove invalid points which are outside the image
        points_2d = points_2d[(points_2d[:, 0] >= 0) & (points_2d[:, 0] <= camera_info.width) & (points_2d[:, 1] >= 0) & (points_2d[:, 1] <= camera_info.height)]
        
        x_min = min(points_2d[:, 0])
        x_max = max(points_2d[:, 0])
        y_min = min(points_2d[:, 1])
        y_max = max(points_2d[:, 1])
        x_min = max(0, x_min)
        x_max = min(camera_info.width, x_max)
        y_min = max(0, y_min)
        y_max = min(camera_info.height, y_max)

        bb_2d = RegionOfInterest()
        bb_2d.x_offset = int(x_min)
        bb_2d.y_offset = int(y_min)
        bb_2d.width = int(x_max - x_min)
        bb_2d.height = int(y_max - y_min)
        return bb_2d, mask
    
    def filter_bb_on_table(self, bbs, table_bb):
        idxs_to_keep = []
        for i, bb in enumerate(bbs):
            overlap_area = self.calculate_overlap(bb, table_bb)
            detection_area = bb.width * bb.height
            if overlap_area / detection_area > 0.1:
                idxs_to_keep.append(i)
        return idxs_to_keep
            
    def calculate_overlap(self, bb1, bb2):
        '''
        Calculates the overlap of two bounding boxes.
        
        Parameters
        ----------
        bb1: sensor_msgs/RegionOfInterest
            The first bounding box.
        bb2: sensor_msgs/RegionOfInterest
            The second bounding box.
        '''
        x1 = max(bb1.x_offset, bb2.x_offset)
        y1 = max(bb1.y_offset, bb2.y_offset)
        x2 = min(bb1.x_offset + bb1.width, bb2.x_offset + bb2.width)
        y2 = min(bb1.y_offset + bb1.height, bb2.y_offset + bb2.height)
        w = max(0, x2 - x1)
        h = max(0, y2 - y1)
        return w * h
    
    def bb_to_mask(self, bb, width, height):
        '''
        Converts a bounding box to a mask.
        
        Parameters
        ----------
        bb: sensor_msgs/RegionOfInterest
            The bounding box.
        width: int
            The width of the mask. Generally the width of the image.
        height: int
            The height of the mask. Generally the height of the image.
        
        Returns
        -------
        np.array
            The mask.
        '''
        mask = np.zeros((height, width), np.uint8)
        mask[bb.y_offset:bb.y_offset+bb.height, bb.x_offset:bb.x_offset+bb.width] = 1
        return mask        
    
    def filter_masks_on_table(self, masks, table_mask):
        idxs_to_keep = []
        for i, mask in enumerate(masks):
            self.node.get_logger().warning(f"mask unique: {np.unique(mask)}")
            self.node.get_logger().warning(f"table_mask unique: {np.unique(table_mask)}")
            self.node.get_logger().warning(f"mask.shape={mask.shape}, table_mask.shape={table_mask.shape}")
            intersection =  mask & table_mask
            area = np.sum(intersection)
            table_area = np.sum(table_mask)
            total_area = np.sum(mask)
            if table_area / total_area > 0.1:
                idxs_to_keep.append(i)
            self.node.get_logger().warning(f"mask {i}: overlap={area}, table_total={table_area}, ratio={area/table_area:.2f}")
        return idxs_to_keep
        
    def filter_table_detection(self, class_names):
        table_idxs = []
        for i, class_name in enumerate(class_names):
            if 'table' in class_name:
                table_idxs.append(i)
        return table_idxs
    
    def _setup_camera_info_subscription(self):
        '''Setup subscription to camera info topic'''
        if not self.node.has_parameter('cam_info_topic'):
            self.node.declare_parameter('cam_info_topic', '/head_rgbd_sensor/depth_registered/camera_info')
        cam_info_topic = self.node.get_parameter('cam_info_topic').value
        self.camera_info_subscription = self.node.create_subscription(
            CameraInfo,
            cam_info_topic,
            self._camera_info_callback,
            10
        )
    
    def _camera_info_callback(self, msg: CameraInfo) -> None:
        '''Store the latest camera info'''
        self.camera_info = msg

    def execute(self, blackboard : yasmin.Blackboard):
        # Wait for camera info with timeout
        timeout_count = 0
        while self.camera_info is None and timeout_count < 150:  # 15 seconds at 10Hz
            rclpy.spin_once(self.node, timeout_sec=0.1)
            timeout_count += 1
        
        if self.camera_info is None:
            self.node.get_logger().error('Timeout waiting for camera info')
            return 'failed'
        
        camera_info = self.camera_info

        if len(blackboard["table_bbs"].boxes) == 0:
            self.node.get_logger().warning('No table bounding box detected.')
            return 'failed'
        
        if len(blackboard["bb_detections"]) == 0 and len(blackboard["mask_detections"]) == 0:
            return 'succeeded'
        
        table_bb = blackboard["table_bbs"].boxes[0]
        
        table_bb = bounding_box_to_bounding_box_stamped(table_bb, blackboard["table_bbs"].header.frame_id, blackboard["table_bbs"].header.stamp)
        table_bb_camera_frame = self.tf_wrapper.transform_bounding_box(table_bb, camera_info.header.frame_id)
        table_bb_2d, table_mask = self.project_3d_bb_to_2d(table_bb_camera_frame, camera_info)
        self.node.get_logger().warning(f"table_bb original frame = {table_bb.header.frame_id}")
        self.node.get_logger().warning(f"table_bb_camera_frame = {table_bb_camera_frame.header.frame_id}")

        mask_detections = blackboard["mask_detections"]
        bb_detections = blackboard["bb_detections"]
        class_names = blackboard["class_names"]
        
        table_idxs = self.filter_table_detection(class_names)
        # use masks if available, otherwise use bounding boxes
        if len(mask_detections) != 0:
            if len(mask_detections) != len(class_names):
                raise ValueError(f'The number of masks and class names must be the same. {len(mask_detections) = }, {len(class_names) = }')
            np_masks = [self.bridge.imgmsg_to_cv2(mask) for mask in mask_detections]
            idxs_to_keep = self.filter_masks_on_table(np_masks, table_mask)
        elif len(bb_detections) != 0:
            if len(bb_detections) != len(class_names):
                raise ValueError(f'The number of bounding boxes and class names must be the same. {len(bb_detections) = }, {len(class_names) = }')
            idxs_to_keep = self.filter_bb_on_table(bb_detections, table_bb_2d)
        
        # remove table detections
        idxs_to_keep = set(idxs_to_keep) - set(table_idxs)
        
        # only keep the objects that are on the table
        objects_left = len(idxs_to_keep)
        class_names = [class_names[i] for i in idxs_to_keep]
        if len(mask_detections) != 0:
            mask_detections = [mask_detections[i] for i in idxs_to_keep]
        if len(bb_detections) != 0:
            bb_detections = [bb_detections[i] for i in idxs_to_keep]
        
        self.node.get_logger().warning(f'{objects_left} objects left on the table.')
        self.node.get_logger().warning(f'{blackboard["class_names"] = }, {idxs_to_keep = }')
        
        blackboard["bb_detections"] = bb_detections
        blackboard["mask_detections"] = mask_detections
        blackboard["class_names"] = class_names
        return 'succeeded'


class CheckTableClean(yasmin.State):
    '''
    Checks whether any objects remain on the table.
    '''

    def __init__(self):
        super().__init__(['clean', 'not_clean'])
        self.hsr_wrapper = HSR_wrapper()

    def execute(self, blackboard : yasmin.Blackboard):
        objects_left = max(len(blackboard["bb_detections"]), len(blackboard["mask_detections"]))
        
        if objects_left == 0:
            self.hsr_wrapper.tts_say('The table is clean. I am finally done.')
            return 'clean'
        else:
            if objects_left == 1:
                self.hsr_wrapper.tts_say(f'The table is not clean. There is 1 object left on the table.')
            else:
                self.hsr_wrapper.tts_say(f'The table is not clean. There are {objects_left} objects left on the table.')
            
            return 'not_clean'
