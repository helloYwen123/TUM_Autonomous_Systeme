#!/usr/bin/env python3

from __future__ import division
from __future__ import print_function

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import PointCloud2
from color_pcl_generator import ColorPclGenerator
import message_filters
import cv2
from matplotlib import pyplot as plt

def extract_semantic_info(semantic_img_cv):
    non_black_mask = np.any(semantic_img_cv != [0, 0, 0], axis=-1)
    return non_black_mask
    
class SemanticCloud:
    def __init__(self):
        # 初始化 ROS
        self.bridge = CvBridge()

        # 320 X 240 pixel
        fx = 120.00000000000001
        fy = fx
        cx = 160.0
        cy = 120.0
        
        # intrinsic matrix
        intrinsic = np.matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype = np.float32)
        self.pcl_pub = rospy.Publisher("/semantic_pcl/semantic_pcl", PointCloud2, queue_size = 1)
        
        self.semantic_sub = message_filters.Subscriber('/realsense/semantic/image_raw', Image, queue_size = 1, buff_size = 20*320*240)
        self.depth_sub = message_filters.Subscriber('semantic_registered/depth/image_raw', Image, queue_size = 1, buff_size = 30*320*240)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.semantic_sub, self.depth_sub], queue_size = 1, slop = 0.5)
        self.ts.registerCallback(self.semantic_depth_callback)
        self.cloud_generator = ColorPclGenerator(intrinsic, 320, 240, 'camera')


    def semantic_depth_callback(self, semantic_img_ros, depth_img_ros):
    
        try:
            semantic_img = self.bridge.imgmsg_to_cv2(semantic_img_ros, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_img_ros, "32FC1") #32FC1  16UC1
        except CvBridgeError as e:
            rospy.logerr(e)
            return


        height, width, channels = semantic_img.shape # Width Height Channels
        height_d, width_d = depth_img.shape 
        # size = semantic_img.size
        non_black_mask = extract_semantic_info(semantic_img)
        print(f"Semantic image size: Width={width}, Height={height}, Channels={channels}")
        print(f"depth image size: Width={width_d}, Height={height_d}, shape={depth_img.shape}")


        #使用ColorPclGenerator仅生成彩色点云
        cloud_ros = self.cloud_generator.generate_cloud_color(semantic_img, depth_img, non_black_mask ,semantic_img_ros.header.stamp)
        

        # 发布点云
        self.pcl_pub.publish(cloud_ros)

def main():
    rospy.init_node('semantic_cloud', anonymous=True)
    try:
        SemanticCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Semantic Cloud Node.")

if __name__ == '__main__':
    main()