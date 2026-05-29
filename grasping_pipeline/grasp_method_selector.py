
import rclpy

import yasmin

import os
import glob
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
        if not self.node.has_parameter('dataset'):
            self.node.declare_parameter('dataset', "ycb_bop")
        self.dataset = self.node.get_parameter('dataset').value
        
        grasps_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, 'grasps', self.dataset)

        files = glob.glob(os.path.join(grasps_path, '*.npy'))
        objects = [os.path.basename(f).split('.')[0] for f in files]
        
        class_names = blackboard["class_names"]
        #for the moment test just with pose based grasped until haf grasping ported to ros2
        #for class_name in class_names:
        #    if class_name not in objects or class_name == 'Unknown':
        #        self.node.get_logger().info('Detected unknown object class: {}'.format(class_name))
        #        return 'direct_grasp'
        return 'pose_based_grasp'
   

        