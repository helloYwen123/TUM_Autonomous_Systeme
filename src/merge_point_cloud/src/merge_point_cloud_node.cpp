#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher pub;

void mergeClouds(const sensor_msgs::PointCloud2ConstPtr& cloud1_msg,
                 const sensor_msgs::PointCloud2ConstPtr& cloud2_msg,
                 const sensor_msgs::PointCloud2ConstPtr& cloud3_msg) {
    pcl::PCLPointCloud2 pcl_cloud1;
    pcl::PCLPointCloud2 pcl_cloud2;
    pcl::PCLPointCloud2 pcl_cloud3;
    pcl::PCLPointCloud2 merged_cloud;
    pcl::PCLPointCloud2 final_merged_cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud1_msg, pcl_cloud1);
    pcl_conversions::toPCL(*cloud2_msg, pcl_cloud2);
    pcl_conversions::toPCL(*cloud3_msg, pcl_cloud3);

    // Merge the two point clouds
    pcl::concatenatePointCloud(pcl_cloud1, pcl_cloud2, merged_cloud);

    // 现在再将merged_cloud和pcl_cloud3合并
    pcl::concatenatePointCloud(merged_cloud, pcl_cloud3, final_merged_cloud);
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(final_merged_cloud, output);

    // Publish the merged cloud
    pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_point_cloud");
    ros::NodeHandle nh;

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("merged_color_point_cloud", 1);

    // Create message_filters subscribers
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub(nh, "/r_color_point_cloud", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub(nh, "/l_color_point_cloud", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud3_sub(nh, "/semantic_pcl/semantic_pcl",1);
    // Set up the synchronizer policy
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud1_sub, cloud2_sub, cloud3_sub); // Include cloud3_sub in synchronizer
    sync.registerCallback(boost::bind(&mergeClouds, _1, _2, _3)); // Add cloud3_msg as a parameter to the callback function
    ros::spin();
    return 0;
}
