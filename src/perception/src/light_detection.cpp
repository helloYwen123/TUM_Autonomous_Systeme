#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <depth_image_proc/depth_traits.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <geometry_msgs/PointStamped.h>

class LightDetectionNode {
  ros::NodeHandle ros_handle_;
  ros::Subscriber seg_img_sub_;
  ros::Subscriber depth_img_sub_;
  ros::Subscriber cam_info_sub_;

  tf::TransformListener tf_listener_;

  std::vector<pcl::PointXYZ> detected_points_;
  ros::Publisher point_pub_;

  sensor_msgs::Image latest_depth_img_;
  sensor_msgs::CameraInfo latest_cam_info_;
  image_geometry::PinholeCameraModel cam_model_;

public:
  LightDetectionNode() {
    seg_img_sub_ = ros_handle_.subscribe("/realsense/semantic/image_raw", 5,
                                         &LightDetectionNode::handleSegImage, this);
    depth_img_sub_ = ros_handle_.subscribe("/realsense/depth/image", 5,
                                           &LightDetectionNode::handleDepthImage, this);
    cam_info_sub_ = ros_handle_.subscribe("/realsense/depth/camera_info", 5,
                                          &LightDetectionNode::handleCamInfo, this);

    point_pub_ = ros_handle_.advertise<geometry_msgs::PointStamped>("detected_points", 10);
  }

  void handleSegImage(const sensor_msgs::ImageConstPtr &seg_img_msg) {
    auto masked_depth_mat = applyMaskToDepth(seg_img_msg);

    sensor_msgs::PointCloud2::Ptr pc2_msg(new sensor_msgs::PointCloud2);
    buildPointCloudFromDepth(pc2_msg, masked_depth_mat);

    auto world_cloud = convertPointCloudToWorld(pc2_msg);
    if (world_cloud.points.empty()) {
      return;
    }
    auto centroid = computeCloudCentroid(world_cloud);

    if (!std::isnan(centroid.x) && !std::isnan(centroid.y) && !std::isnan(centroid.z)) {
      if (isUniqueDetection(centroid)) {
        ROS_WARN("Light detected at %f, %f, %f", centroid.x, centroid.y, centroid.z);
        detected_points_.push_back(centroid);

        geometry_msgs::PointStamped pt_msg;
        pt_msg.header.stamp = ros::Time::now();
        pt_msg.header.frame_id = "world";
        pt_msg.point.x = centroid.x;
        pt_msg.point.y = centroid.y;
        pt_msg.point.z = centroid.z;
        point_pub_.publish(pt_msg);
      }
    }
  }

  void handleDepthImage(const sensor_msgs::ImageConstPtr &depth_img_msg) {
    latest_depth_img_ = *depth_img_msg;
  }

  void handleCamInfo(const sensor_msgs::CameraInfo &cam_info_msg) {
    latest_cam_info_ = cam_info_msg;
    cam_model_.fromCameraInfo(cam_info_msg);
  }

private:
  void buildPointCloudFromDepth(sensor_msgs::PointCloud2::Ptr &pc2_msg, const cv::Mat &depth_mat) {
    pc2_msg->header = latest_depth_img_.header;
    pc2_msg->height = latest_depth_img_.height;
    pc2_msg->width = latest_depth_img_.width;
    pc2_msg->is_dense = false;
    pc2_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(*pc2_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    convertDepthToCloud(depth_mat, pc2_msg, cam_model_);
  }

  pcl::PointCloud<pcl::PointXYZ> convertPointCloudToWorld(sensor_msgs::PointCloud2::Ptr pc2_msg) {
    pcl::PointCloud<pcl::PointXYZ> local_cloud;
    pcl::fromROSMsg(*pc2_msg, local_cloud);

    tf::StampedTransform tf_trans;
    try {
      tf_listener_.waitForTransform("world", pc2_msg->header.frame_id, ros::Time(0), ros::Duration(10.0));
      tf_listener_.lookupTransform("world", pc2_msg->header.frame_id, ros::Time(0), tf_trans);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
    }

    pcl::PointCloud<pcl::PointXYZ> world_cloud;
    pcl_ros::transformPointCloud(local_cloud, world_cloud, tf_trans);
    return world_cloud;
  }

  pcl::PointXYZ computeCloudCentroid(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    pcl::PointXYZ center(0, 0, 0);
    int valid_count = 0;
    for (const auto &pt : cloud.points) {
      if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) &&
          (pt.x != 0 || pt.y != 0 || pt.z != 0)) {
        center.x += pt.x;
        center.y += pt.y;
        center.z += pt.z;
        valid_count++;
      }
    }
    if (valid_count > 0) {
      center.x /= valid_count;
      center.y /= valid_count;
      center.z /= valid_count;
    } else {
      center.x = std::numeric_limits<float>::quiet_NaN();
      center.y = std::numeric_limits<float>::quiet_NaN();
      center.z = std::numeric_limits<float>::quiet_NaN();
    }
    return center;
  }

  void convertDepthToCloud(const cv::Mat &depth_mat, sensor_msgs::PointCloud2::Ptr &pc2_msg,
                             const image_geometry::PinholeCameraModel &cam_model, double max_range = 0.0) {
    float c_x = cam_model.cx();
    float c_y = cam_model.cy();

    double scale_unit = depth_image_proc::DepthTraits<uint16_t>::toMeters(uint16_t(1));
    float scale_x = scale_unit / cam_model.fx();
    float scale_y = scale_unit / cam_model.fy();
    float invalid_val = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Iterator<float> itr_x(*pc2_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> itr_y(*pc2_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> itr_z(*pc2_msg, "z");

    const uint16_t *depth_row = reinterpret_cast<const uint16_t *>(depth_mat.data);
    int row_inc = depth_mat.step / sizeof(uint16_t);

    for (int v = 0; v < (int)pc2_msg->height; ++v, depth_row += row_inc) {
      for (int u = 0; u < (int)pc2_msg->width; ++u, ++itr_x, ++itr_y, ++itr_z) {
        uint16_t depth_val = depth_row[u];

        if (!depth_image_proc::DepthTraits<uint16_t>::valid(depth_val)) {
          if (max_range != 0.0) {
            depth_val = depth_image_proc::DepthTraits<uint16_t>::fromMeters(max_range);
          } else {
            *itr_x = *itr_y = *itr_z = invalid_val;
            continue;
          }
        }

        *itr_x = (u - c_x) * depth_val * scale_x;
        *itr_y = (v - c_y) * depth_val * scale_y;
        *itr_z = depth_image_proc::DepthTraits<uint16_t>::toMeters(depth_val);
      }
    }
  }

  cv::Mat applyMaskToDepth(const sensor_msgs::ImageConstPtr &seg_img_msg) {
    auto seg_img = cv_bridge::toCvCopy(seg_img_msg, sensor_msgs::image_encodings::BGR8);
    auto depth_img = cv_bridge::toCvCopy(latest_depth_img_, sensor_msgs::image_encodings::TYPE_16UC1);

    cv::Mat mask;
    cv::inRange(seg_img->image, cv::Scalar(4, 235, 255), cv::Scalar(4, 235, 255), mask);
    depth_img->image.setTo(cv::Scalar(std::numeric_limits<double>::quiet_NaN()), ~mask);

    return depth_img->image;
  }

  bool isUniqueDetection(const pcl::PointXYZ &pt) {
    for (const auto &det : detected_points_) {
      double dist_sq = std::pow(det.x - pt.x, 2) +
                       std::pow(det.y - pt.y, 2) +
                       std::pow(det.z - pt.z, 2);
      double thresh = 100;
      if (dist_sq < thresh) {
        return false;
      }
    }
    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "light_detection_node");
  LightDetectionNode detector;
  ros::spin();
}
