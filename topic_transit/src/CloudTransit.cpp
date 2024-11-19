#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <topic_transit/CloudFilterConfig.h>
#include <pcl/filters/extract_indices.h>

class PointCloudFilter {
public:
  PointCloudFilter();
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);

private:
  void configCallback(topic_transit::CloudFilterConfig &config, uint32_t level);
  void removeGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud);

  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;
  ros::Publisher ground_cloud_pub_;
  ros::Rate* rate_;

  // 动态参数服务器
  dynamic_reconfigure::Server<topic_transit::CloudFilterConfig>* server_;
  dynamic_reconfigure::Server<topic_transit::CloudFilterConfig>::CallbackType f_;

  // 参数
  double min_z_;
  double max_z_;
  double max_radius_;
  int min_neighbors_;
  double leaf_size_;
  int max_window_size_;    // 新增：PMF参数
  double slope_;           // 新增：PMF参数
  double initial_distance_;// 新增：PMF参数
  double max_distance_;    // 新增：PMF参数
  std::string input_topic_;
  std::string output_topic_;
  std::string ground_topic_;  // 新增：地面点云话题
  double frequency_;
  std::string frame_id_;
  bool use_ground_filter_;  // 是否使用地面滤波
  bool publish_ground_;     // 是否发布地面点云
};

PointCloudFilter::PointCloudFilter() {
  ros::NodeHandle private_nh("~");

  // 加载参数
  private_nh.param("min_z", min_z_, -0.5);
  private_nh.param("max_z", max_z_, 0.5);
  private_nh.param("max_radius", max_radius_, 0.5);
  private_nh.param("min_neighbors", min_neighbors_, 10);
  private_nh.param("leaf_size", leaf_size_, 0.1);
  private_nh.param("max_window_size", max_window_size_, 5);
  private_nh.param("slope", slope_, 0.3);
  private_nh.param("initial_distance", initial_distance_, 0.5);
  private_nh.param("max_distance", max_distance_, 1.0);
  private_nh.param("input_topic", input_topic_, std::string("/cloud_registered"));
  private_nh.param("output_topic", output_topic_, std::string("/cloud_registered_filter"));
  private_nh.param("ground_topic", ground_topic_, std::string("/ground_cloud"));
  private_nh.param("frequency", frequency_, 10.0);
  private_nh.param("frame_id", frame_id_, std::string("base_link"));
  private_nh.param("use_ground_filter", use_ground_filter_, false);
  private_nh.param("publish_ground", publish_ground_, false);

  // 设置订阅和发布
  point_cloud_sub_ = nh_.subscribe(input_topic_, 1, &PointCloudFilter::pointCloudCallback, this);
  point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
  ground_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_, 1);

  rate_ = new ros::Rate(frequency_);

  // 设置动态参数服务器
  server_ = new dynamic_reconfigure::Server<topic_transit::CloudFilterConfig>(private_nh);
  f_ = boost::bind(&PointCloudFilter::configCallback, this, _1, _2);
  server_->setCallback(f_);
}

void PointCloudFilter::configCallback(topic_transit::CloudFilterConfig &config, uint32_t level) {
  min_z_ = config.min_z;
  max_z_ = config.max_z;
  max_radius_ = config.max_radius;
  min_neighbors_ = config.min_neighbors;
  leaf_size_ = config.leaf_size;
  max_window_size_ = config.max_window_size;
  slope_ = config.slope;
  initial_distance_ = config.initial_distance;
  max_distance_ = config.max_distance;
  use_ground_filter_ = config.use_ground_filter;
  publish_ground_ = config.publish_ground;
}

void PointCloudFilter::removeGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud) {
  // 使用PMF进行地面分割
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud(cloud);
  pmf.setMaxWindowSize(max_window_size_);
  pmf.setSlope(slope_);
  pmf.setInitialDistance(initial_distance_);
  pmf.setMaxDistance(max_distance_);

  pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
  pmf.extract(ground_indices->indices);

  // 分离地面点和非地面点
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(ground_indices);

  // 提取地面点
  extract.setNegative(false);
  extract.filter(*ground_cloud);

  // 提取非地面点
  extract.setNegative(true);
  extract.filter(*non_ground_cloud);
}

void PointCloudFilter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
  // 将ROS点云数据转换为PCL格式
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);

  // 基础滤波处理（保持不变）
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_z_, max_z_);
  pass.filter(*cloud);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_filter.filter(*cloud);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
  radius_filter.setInputCloud(cloud);
  radius_filter.setRadiusSearch(max_radius_);
  radius_filter.setMinNeighborsInRadius(min_neighbors_);
  radius_filter.filter(*cloud);

  // TF 变换准备
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  try {
    transformStamped = tfBuffer.lookupTransform(
        frame_id_, input->header.frame_id,
        ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("TF lookup failed: %s", ex.what());
    return;
  }

  if (use_ground_filter_) {
    // 如果启用地面滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 执行地面滤波
    removeGround(cloud, ground_cloud, non_ground_cloud);

    // 发布非地面点云（主要输出）
    sensor_msgs::PointCloud2 output_non_ground;
    pcl::toROSMsg(*non_ground_cloud, output_non_ground);
    output_non_ground.header = input->header;

    sensor_msgs::PointCloud2 transformed_non_ground;
    tf2::doTransform(output_non_ground, transformed_non_ground, transformStamped);
    transformed_non_ground.header.frame_id = frame_id_;
    transformed_non_ground.header.stamp = ros::Time::now();
    point_cloud_pub_.publish(transformed_non_ground);

    // 如果需要，额外发布地面点云
    if (publish_ground_) {
      sensor_msgs::PointCloud2 output_ground;
      pcl::toROSMsg(*ground_cloud, output_ground);
      output_ground.header = input->header;

      sensor_msgs::PointCloud2 transformed_ground;
      tf2::doTransform(output_ground, transformed_ground, transformStamped);
      transformed_ground.header.frame_id = frame_id_;
      transformed_ground.header.stamp = ros::Time::now();
      ground_cloud_pub_.publish(transformed_ground);
    }
  }
  else
  {
    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud, output_cloud);
    tf2::doTransform(output_cloud, output_cloud, transformStamped);
    output_cloud.header.frame_id = frame_id_;
    output_cloud.header.stamp = ros::Time::now();
    point_cloud_pub_.publish(output_cloud);
  }

  rate_->sleep();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_transit");
  PointCloudFilter filter_node;
  ros::spin();
  return 0;
}