#! /usr/bin/env python3


import copy
from math import pi
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

import tf_transformations
#import tf
from v4r_util.tf2 import TF2Wrapper
from v4r_util.alignment import align_pose_rotation, get_best_aligning_axis, Axis
from v4r_util.util import rotmat_around_axis
from v4r_util.conversions import ros_pose_to_np_transform, np_transform_to_ros_pose, point_to_vector3
from grasping_pipeline.moveit_wrapper import MoveitWrapper
from grasping_pipeline.hsr_wrapper import HSR_wrapper
from geometry_msgs.msg import Pose, PoseStamped, Transform
from visualization_msgs.msg import Marker
from grasping_pipeline_msgs.action import ExecuteGrasp


class ExecuteGraspServer(Node):
    def __init__(self):
        '''
        Initialize and starts the action server.
        '''
        super().__init__('execute_grasp_server',  automatically_declare_parameters_from_overrides=True)

        self.safety_distance = .1

        self.tf_wrapper = TF2Wrapper(self)
        self.get_logger().info("Execute grasp: Waiting for moveit")
        self.moveit_wrapper = MoveitWrapper(self.tf_wrapper, self)
        self.get_logger().info("Execute grasp: Got Moveit")
        self.hsr_wrapper = HSR_wrapper()
               
        self.get_logger().info("Execute grasp: Init")

        self.timer = self.create_timer(1.0, self.move)
        self.get_logger().info("Execute grasp: Init ?????")

    def move(self):
        self.get_logger().info("Timer callback")
        approach_pose = PoseStamped()
        approach_pose.header.stamp = rclpy.time.Time().to_msg()
        approach_pose.header.frame_id = "map"
        approach_pose.pose.position.x = 0.2
        approach_pose.pose.position.y = 0.6
        approach_pose.pose.position.z = .6

        planning_frame = self.moveit_wrapper.get_planning_frame("whole_body")
        self.get_logger().info(f"Planing frame: {planning_frame}")
        approach_pose = self.tf_wrapper.transform_pose(planning_frame,  approach_pose)

        plan_found = self.moveit_wrapper.whole_body_plan_and_go(approach_pose)
        if not plan_found:
            self.get_logger().warn("FAILED AAAAAAAAAAAAAA")
    
        self.get_logger().info("Done -- shutdown")

        # 1. Cleanly cancel the current timer so it doesn't fire again
        if self.timer is not None:
            self.timer.cancel()

        # 2. Give the Action Client 1 second to exchange cleanup packets with move_group
        self.shutdown_timer = self.create_timer(1.0, self.clean_exit)
        
    def clean_exit(self):
        self.get_logger().info("Shutting down node now.")
        # Breaking out of rclpy.spin() gracefully
        raise SystemExit
    

def main():
    rclpy.init()
    node = ExecuteGraspServer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass # Expected exit path
    except KeyboardInterrupt:
        pass # Ctrl+C exit path    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

