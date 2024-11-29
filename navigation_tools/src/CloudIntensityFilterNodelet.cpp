//
// Created by lsy on 24-11-29.
//

/**
* @file IntensityFilterNodelet.cpp
* @brief A nodelet to filter point cloud based on intensity
*/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace cloud_filter {

class IntensityFilterNodelet : public nodelet::Nodelet {
private:
 ros::Publisher pub_;
 ros::Subscriber sub_;
 double intensity_threshold_;
 std::string input_topic_;
 std::string output_topic_;

 virtual void onInit() override {
   auto& nh = getNodeHandle();
   auto& private_nh = getPrivateNodeHandle();

   intensity_threshold_ = private_nh.param("intensity_threshold", 100.0);
   input_topic_ = private_nh.param<std::string>("input_topic", "points");
   output_topic_ = private_nh.param<std::string>("output_topic", "points_filtered");

   pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
   sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
       input_topic_, 10, &IntensityFilterNodelet::cloudCallback, this);

   NODELET_INFO("Intensity filter nodelet initialized:");
   NODELET_INFO("  - Input topic: %s", input_topic_.c_str());
   NODELET_INFO("  - Output topic: %s", output_topic_.c_str());
   NODELET_INFO("  - Intensity threshold: %f", intensity_threshold_);
 }

 void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input) {
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
   pcl::fromROSMsg(*input, *cloud);

   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
   cloud_filtered->reserve(cloud->size());

   for (const auto& point : *cloud) {
     if (point.intensity >= intensity_threshold_) {
       cloud_filtered->push_back(point);
     }
   }

   cloud_filtered->header = cloud->header;
   cloud_filtered->height = 1;
   cloud_filtered->width = cloud_filtered->size();
   cloud_filtered->is_dense = false;

   sensor_msgs::PointCloud2 output;
   pcl::toROSMsg(*cloud_filtered, output);
   output.header = input->header;

   pub_.publish(output);
 }
};

}  // namespace cloud_filter

PLUGINLIB_EXPORT_CLASS(cloud_filter::IntensityFilterNodelet, nodelet::Nodelet)