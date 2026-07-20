
import rclpy

import yasmin

import os
import glob
from ament_index_python.packages import get_package_share_directory

class GraspMethodSelector(yasmin.State):
    '''
    Returns 'pose_based_grasp' if there are no 'Unknown' objects in class_names, otherwise 'direct_grasp'.
    This assumes that pose_based_grasp methods require the object class to be known.
    
    Parameters
    ----------
    class_names : list of str
        List of object class names detected in the scene.
    
    Returns
    -------
    smach-result
        'pose_based_grasp' if there are no 'Unknown' objects in class_names, otherwise 'direct_grasp'.
    '''

    def __init__(self, node):
        super().__init__(["pose_based_grasp", "direct_grasp"])
        self.node = node

    def execute(self, blackboard : yasmin.Blackboard):
        '''
        Decides which grasp method to use based on whether there are 'Unknown' objects in class_names or objects without annotated grasp points.

        Returns
        -------
        str
            'pose_based_grasp' if there are no 'Unknown' objects in class_names, otherwise 'direct_grasp'.
        '''
        if not self.node.has_parameter('grasping_pipeline.dataset'):
            self.node.declare_parameter('grasping_pipeline.dataset', "ycb_bop")
        self.dataset = self.node.get_parameter('grasping_pipeline.dataset').value
        
        share_dir = get_package_share_directory("grasping_pipeline")
        grasps_path = os.path.join(share_dir, "grasps", self.dataset)

        files = glob.glob(os.path.join(grasps_path, '*.npy'))
        objects = [os.path.basename(f).split('.')[0] for f in files]
        self.node.get_logger().info(f"Grasps path: {grasps_path}")
        self.node.get_logger().info(f"Exists: {os.path.exists(grasps_path)}")
        self.node.get_logger().info(f"there are this {len(objects)}")
        
        class_names = blackboard["class_names"]
        self.node.get_logger().info("there are {} objects in the dataset".format(len(class_names)))
        #for the moment test just with pose based grasped until haf grasping ported to ros2
        for class_name in class_names:
           if class_name not in objects or class_name == 'Unknown':
               self.node.get_logger().info('Detected unknown object class: {}'.format(class_name))
               return 'direct_grasp'
        return 'pose_based_grasp'
   

        