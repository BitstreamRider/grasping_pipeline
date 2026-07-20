#!/usr/bin/env python3
import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from grasping_pipeline_msgs.srv import FetchImages
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SynchronizedImageFetcher(Node):
    '''
    This class provides a service that fetches synchronized RGB and Depth images from the robot.
    Instead of subscribing to the RGB and Depth topics directly, it uses ApproximateTimeSynchronizer
    to ensure that the images are captured at the same time. Additionally, it unregisters the subscribers
    after the images are captured to save bandwidth.

    Attributes
    ----------
    service : rospy.Service
        This service. Fetches synchronized images when called
    rgb_image : sensor_msgs.msg.Image
        RGB image captured by the robot
    depth_image : sensor_msgs.msg.Image
        Depth image captured by the robot
    
    Other Parameters
    ----------------
    rgb_topic : str
        Name of the RGB image topic. Loaded from the 'rgb_topic' parameter
    depth_topic : str
        Name of the Depth image topic. Loaded from the 'depth_topic' parameter
    
    Returns
    -------
    rgb : sensor_msgs.msg.Image
        RGB image captured by the robot
    depth : sensor_msgs.msg.Image
        Depth image captured by the robot
    '''
    def __init__(self):
        super().__init__('synchronized_image_fetcher')

        self.declare_parameter('rgb_topic', '/head_rgbd_sensor/rgb/image_rect_color')
        self.declare_parameter('depth_topic', '/head_rgbd_sensor/depth_registered/image_rect_raw')

        self.srv = self.create_service(FetchImages, 'fetch_synchronized_images', self.fetch)
        self.rgb_image = None
        self.depth_image = None

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value

        self.rgb_sub = Subscriber(self, Image, rgb_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.ats = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=5, slop=2.0)
        self.ats.registerCallback(self.callback)

    def fetch(self, req, response):
        '''Fetches synchronized RGB and Depth images from the robot.

        After the images are captured, the service unregisters the subscribers to save bandwidth.

        Returns
        -------
        FetchImagesResponse
            Response containing the synchronized RGB and Depth images
        '''


        self.get_logger().info('Waiting for synchronized images...')
        timeout_cnt = 0
        while  (self.rgb_image is None or self.depth_image is None) and timeout_cnt < 200:
           rclpy.spin_once(self, timeout_sec=0.1)
           timeout_cnt += 1

        self.get_logger().info('Synchronized Images captured!')

        # Unregister subscribers to save bandwidth
        #rgb_sub.subscriber.destroy()
        #depth_sub.subscriber.destroy()

        response = FetchImages.Response()
        response.rgb = self.rgb_image
        response.depth = self.depth_image

        # Reset stored images for next request
        self.rgb_image = None
        self.depth_image = None

        return response

    def callback(self, rgb_msg, depth_msg):
        '''
        Callback function for ApproximateTimeSynchronizer. Stores the synchronized RGB and Depth images.
        '''
        self.rgb_image = rgb_msg
        self.depth_image = depth_msg

def main(args=None):
    rclpy.init(args=args)
    fetcher = SynchronizedImageFetcher()
    rclpy.spin(fetcher)
    fetcher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()