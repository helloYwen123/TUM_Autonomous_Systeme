from __future__ import division
from __future__ import print_function
# import rospy
# import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
#from enum import Enum
#import time

class ColorPclGenerator:
    '''
    Generate a ros point cloud given a semantic image and a depth image
    \author Wenjie Xie
    '''
    def __init__(self, intrinsic, width = 640, height = 480, frame_id = "camera_rgb_optical_frame",):
        
        '''
        width: (int) width of input images
        height: (int) height of input images
        '''
        self.intrinsic = intrinsic
        self.width = width
        self.height = height
        x_index, y_index = np.meshgrid(np.arange(width), np.arange(height), indexing='xy')
        x_index = x_index.ravel().astype('<f4')  # 将二维数组展平并转换为指定的数据类型
        y_index = y_index.ravel().astype('<f4')
        self.xy_index = np.vstack((x_index, y_index)).T # x,y
        self.cloud_ros = PointCloud2()
        self.cloud_ros.header.frame_id = frame_id
        
       

    def generate_cloud_data_common(self, bgr_img, depth_img, non_black_mask):
        bgr_img = bgr_img.astype(np.uint8)
        depth_img = depth_img.astype(np.float32)

        # 应用非黑色掩码
        non_black_indices = np.where(non_black_mask.flatten())[0]  # 获取非黑色像素的索引
        filtered_depth_img = depth_img.flatten()[non_black_indices] / 1000.0
        print(bgr_img.shape)
        print(depth_img.shape)
        print(non_black_indices.shape)
        bgr_imgfiltered_bgr_img = bgr_img.reshape(-1, 3)[non_black_indices]

        # 计算实际的世界坐标
        filtered_xy_index = self.xy_index[non_black_indices]
        filtered_xyz_data = np.zeros((len(non_black_indices), 3), dtype=np.float32)
        filtered_xyz_data[:, 0] = (filtered_xy_index[:, 0] - self.intrinsic[0, 2]) * filtered_depth_img / self.intrinsic[0, 0]
        filtered_xyz_data[:, 1] = (filtered_xy_index[:, 1] - self.intrinsic[1, 2]) * filtered_depth_img / self.intrinsic[1, 1]
        filtered_xyz_data[:, 2] = filtered_depth_img
        filtered_xyz_data_meters = (filtered_xyz_data).astype(np.float32) # m / mm / cm

        # 初始化 ros_data 以匹配过滤后的点的数量
        self.ros_data = np.ones((len(non_black_indices), 8), dtype=np.float32)  # 注意，这里假设每个点8个float32

        self.ros_data[:,:3] = filtered_xyz_data_meters
        # 直接将rgb的每个颜色通道分配到ros_data的相应位置
        packed_rgb = self.pack_bgr_to_float32(bgr_imgfiltered_bgr_img) #bgr_img filtered_bgr_img
        
        self.ros_data[:,4:5] = packed_rgb
        
        # save output for checking
        np.savetxt("depth_flattened.txt", filtered_depth_img, fmt="%f")
        np.savetxt("filtered_bgr_img.txt", bgr_imgfiltered_bgr_img, fmt="%f")
        np.savetxt("packed_rgb.txt", packed_rgb, fmt="%f")
        np.savetxt("filtered_xyz_data.txt", filtered_xyz_data, fmt="%f")
        np.savetxt("ros_data.txt", self.ros_data, fmt="%f")



    def make_ros_cloud(self, stamp):
        self.cloud_ros.header.stamp = stamp
        self.cloud_ros.height = 1
        self.cloud_ros.width = len(self.ros_data)# semantic 只有灯
        # self.cloud_ros.width = self.width*self.height
        self.cloud_ros.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        self.cloud_ros.is_bigendian = False
        self.cloud_ros.point_step = 32  # 由于每个点现在是12字节 20 12 16 
        self.cloud_ros.row_step = self.cloud_ros.point_step * self.cloud_ros.width
        self.cloud_ros.is_dense = False  # 假设所有点都是有效的
        self.cloud_ros.data = np.asarray(self.ros_data).tobytes()
        return self.cloud_ros

    def generate_cloud_color(self, bgr_img, depth_img, non_black, stamp):
        """
        Generate color point cloud
        \param bgr_img (numpy array bgr8) input color image
        \param depth_img (numpy array float32) input depth image
        """
        self.generate_cloud_data_common(bgr_img, depth_img, non_black)
        return self.make_ros_cloud(stamp)
    


    
    def calculate_point_cloud_center(ros_data):
        
        # 提取XYZ坐标
        points_xyz = ros_data[:, :3]
        
        # 计算中心坐标
        center = np.mean(points_xyz, axis=0)
        
        return center
    
    def pack_bgr_to_float32(self, bgr_img):
        rgb = np.zeros((len(bgr_img), 4), dtype = '<u1')
        # rgb[:, 0] = bgr_img[:, 0]  # R
        # rgb[:, 1] = bgr_img[:, 1]  # G
        # rgb[:, 2] = bgr_img[:, 2]  # B
        # rgb[:, 3] = 255  # Alpha
        rgb[:, 0] = bgr_img[:, 0]  # R
        rgb[:, 1] = bgr_img[:, 1]  # G
        rgb[:, 2] = bgr_img[:, 2]  # B
        rgb[:, 3] = 255  # Alpha

        packed_rgb = rgb.view('<f4')
        return packed_rgb


# Test
if __name__ == "__main__":
    pass


    # def generate_cloud_color_data(self, bgr_img, depth_img):
    #     bgr_img = bgr_img.astype(np.uint8)
    #     depth_img = depth_img.astype(np.float32)


       
    #     filtered_depth_img = depth_img.flatten() / 1000.0
    #     bgr_imgfiltered_bgr_img = bgr_img.reshape(-1, 3)
    #     print(bgr_img.shape)

    #     # 计算实际的世界坐标
    #     filtered_xy_index = self.xy_index
    #     filtered_xyz_data = np.zeros((self.width*self.height, 3), dtype=np.float32)
    #     filtered_xyz_data[:, 0] = (filtered_xy_index[:, 0] - self.intrinsic[0, 2]) * filtered_depth_img / self.intrinsic[0, 0]
    #     filtered_xyz_data[:, 1] = (filtered_xy_index[:, 1] - self.intrinsic[1, 2]) * filtered_depth_img / self.intrinsic[1, 1]
    #     filtered_xyz_data[:, 2] = filtered_depth_img
    #     filtered_xyz_data_meters = (filtered_xyz_data).astype(np.float32) # m / mm / cm

    #     # 初始化 ros_data 以匹配过滤后的点的数量
    #     self.ros_data = np.ones((self.width*self.height, 5), dtype=np.float32)  # 注意，这里假设每个点5个float32

    #     self.ros_data[:,:3] = filtered_xyz_data_meters
    #     # 直接将rgb的每个颜色通道分配到ros_data的相应位置
    #     packed_rgb = self.pack_bgr_to_float32(bgr_imgfiltered_bgr_img) #bgr_img filtered_bgr_img
    #     self.ros_data[:,4:5] = packed_rgb
        
    #     # save output for checking
    #     np.savetxt("depth_flattened.txt", filtered_depth_img, fmt="%f")
    #     np.savetxt("filtered_bgr_img.txt", bgr_imgfiltered_bgr_img, fmt="%f")
    #     np.savetxt("filtered_xyz_data.txt", filtered_xyz_data, fmt="%f")
    #     # np.savetxt("packed_rgb.txt", packed_rgb, fmt="%f")
    #     np.savetxt("ros_data.txt", self.ros_data, fmt="%f")



    # def generate_cloud_color_non_semantic(self, bgr_img, depth_img, stamp):
    #     """
    #     Generate color point cloud
    #     \param bgr_img (numpy array bgr8) input color image
    #     \param depth_img (numpy array float32) input depth image
    #     """
    #     self.generate_cloud_color_data(bgr_img, depth_img)  # non semantic image
    #     return self.make_ros_cloud(stamp)