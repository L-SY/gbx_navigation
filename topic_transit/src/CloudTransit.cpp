//
// Created by lsy on 24-10-23.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudFilter {
public:
  PointCloudFilter() {

    ros::NodeHandle private_nh("~");
    private_nh.param("min_z", min_z_, 0.0);
    private_nh.param("max_z", max_z_, 1.0);
    private_nh.param("max_radius", max_radius_, 5.0);
    private_nh.param("input_topic", input_topic_, std::string("/cloud_registered"));
    private_nh.param("output_topic", output_topic_, std::string("/cloud_registered_filter"));
    private_nh.param("frequency", frequency_, 10.0);

    point_cloud_sub_ = nh_.subscribe(input_topic_, 1, &PointCloudFilter::pointCloudCallback, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    // 将ROS点云数据转换为PCL格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    // 创建过滤器对象，首先基于Z轴（高度）进行过滤
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z_, max_z_);
    pass.filter(*cloud);  // 过滤后的点云仍保存在cloud中

    // 对半径进行过滤（从传感器中心到点的距离）
    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& point : cloud->points) {
      float radius = std::sqrt(point.x * point.x + point.y * point.y);
      if (radius <= max_radius_) {
        radius_filtered_cloud->points.push_back(point);
      }
    }

    // 将处理后的点云转换回ROS消息格式并发布
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*radius_filtered_cloud, output);
    output.header = input->header;  // 保持原始消息的时间戳和坐标系
    point_cloud_pub_.publish(output);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;

  double min_z_;  // 最小高度
  double max_z_;  // 最大高度
  double max_radius_;  // 最大半径
  std::string input_topic_;
  std::string output_topic_;
  double frequency_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_transit");
  PointCloudFilter filter_node;
  ros::spin();
  return 0;
}

