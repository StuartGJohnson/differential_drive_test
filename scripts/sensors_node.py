#!/usr/bin/env python3
# 2025 Stuart Johnson
#
# Sensor test for simulation and real robots as a ros2 node. This node places the
# robot at one (or more) poses and records sensor values.
#
from typing import Tuple, Iterator
import time
import sys
import rclpy
import scipy.io
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from laser_geometry import LaserProjection
from rclpy.service import Service
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.spatial.transform.rotation import Rotation as rot
import matplotlib.pyplot as plt
import os
import yaml
import numpy as np
import copy
import cv2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

class SensorsNode(Node):
    def __init__(self):
        super().__init__('sensors_node')
        self.depth_image_valid = False
        self.rgb_image_valid = False
        self.scan_valid = False
        self.done = False
        self.done_depth = False
        self.done_rgb = False
        self.done_scan = False
        self.declare_parameter('sim_type', 'gazebo') # Declare the parameter with a default value
        self.sim_type = self.get_parameter('sim_type').get_parameter_value().string_value
        print("sim type is: ", self.sim_type)
        default_report_dir = os.path.join(os.getenv("HOME"),'sim_check_test')
        self.declare_parameter('report_dir', default_report_dir) # Declare the parameter with a default value
        self.report_dir = self.get_parameter('report_dir').get_parameter_value().string_value
        print("report_dir is: ", self.report_dir)
        if self.sim_type == 'isaacsim':
            self.adj_omega = 180.0 / np.pi
            self.source_xform_frame = 'odom'
            self.depth_topic = '/depth_camera/depth'
            self.rgb_topic = '/rgb_camera/rgb'
        elif self.sim_type == 'gazebo':
            self.adj_omega = 1.0
            self.source_xform_frame = 'differential_drive_robot_4wheel/odom'
            self.depth_topic = '/d435_depth_camera/image_raw'
            self.rgb_topic = '/d435_rgb_camera/image_raw'
        elif self.sim_type == 'robot':
            self.adj_omega = 1.0
            self.source_xform_frame = 'differential_drive_robot_4wheel/odom'
            self.depth_topic = '/d435_depth_camera/image_raw'
            self.rgb_topic = '/d435_rgb_camera/image_raw'
            # temporary - the camera is not plugged in yet
            self.done_depth = True
            self.done_rgb = True
        else:
            print("unknown simulator type")
            self.done=True
            return
        self.listener_depth = self.create_subscription(Image,self.depth_topic, self.process_depth, 10)
        self.listener_rgb = self.create_subscription(Image, self.rgb_topic, self.process_rgb, 10)
        self.listener_scan = self.create_subscription(LaserScan, '/scan', self.process_scan, 10)
        self.bridge = CvBridge()

    def process_depth(self, image_msg: Image):
        # grab an image
        if not self.done_depth:
            self.depth_image = copy.deepcopy(image_msg)
            self.done_depth = True
            self.rgb_image_valid = True
        self.done = self.done_depth and self.done_rgb and self.done_scan

    def process_rgb(self, image_msg: Image):
        # grab an image
        if not self.done_rgb:
            self.rgb_image = copy.deepcopy(image_msg)
            self.done_rgb = True
            self.depth_image_valid = True
        self.done = self.done_depth and self.done_rgb and self.done_scan

    def process_scan(self, scan_msg: LaserScan):
        # grab a scan
        if not self.done_scan:
            self.scan = copy.deepcopy(scan_msg)
            self.done_scan = True
            self.scan_valid = True
        self.done = self.done_depth and self.done_rgb and self.done_scan

    def serialize(self):
        os.makedirs(self.report_dir, exist_ok=True)
        # write the time series to a matlab .mat file - for matlab analysis
        if self.rgb_image_valid:
            cv_rgb = self.bridge.imgmsg_to_cv2(self.rgb_image, desired_encoding='bgr8')
            #cv2.imwrite('/home/sjohnson/depth_image.jpg', self.depth_image)
            cv2.imwrite(os.path.join(self.report_dir, 'rgb_image.jpg'), cv_rgb)
        if self.depth_image_valid:
            depth_image = self.bridge.imgmsg_to_cv2(self.depth_image, desired_encoding='passthrough')
            depth_image_np = np.array(depth_image,dtype=np.float32)
            # for close-ups
            depth_image_max = np.nanmax(depth_image_np)
            # not-so-close-up max (20m)
            depth_image_max = min(20.0, depth_image_max)
            depth_array = np.clip(depth_image_np,0, depth_image_max)
            cv_depth = (depth_array / depth_image_max * 255).astype(np.uint8)
            cv2.imwrite(os.path.join(self.report_dir, 'depth_image.jpg'), cv_depth)
        if self.scan_valid:
            scan_converter = LaserProjection()
            pc2 = scan_converter.projectLaser(self.scan, range_cutoff=100.0)
            pc2_extracted = point_cloud2.read_points(pc2,field_names=['x','y','z'], skip_nans=True)
            pc2_np = np.array(pc2_extracted.tolist(), dtype=np.float32)
            # plot away!
            plt.figure()
            plt.plot(pc2_np[:,0],pc2_np[:,1],'r.')
            plt.xlabel('lidar x')
            plt.ylabel('lidar y')
            plt.title('lidar point cloud')
            plt.axis('equal')
            # for debugging in pycharm
            #plt.show()
            plt.savefig(os.path.join(self.report_dir, 'lidar.jpg'))

def main(args=None):
    rclpy.init(args=args)
    node = SensorsNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.serialize()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

