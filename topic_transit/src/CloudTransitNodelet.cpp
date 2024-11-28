#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
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

namespace topic_transit
{

class CloudTransitNodelet : public nodelet::Nodelet
{
public:
  CloudTransitNodelet() {}

private:
  virtual void onInit()
  {
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // 加载参数
    private_nh_.param("min_z", min_z_, -0.5);
    private_nh_.param("max_z", max_z_, 0.5);
    private_nh_.param("min_x", min_x_, -5.0);
    private_nh_.param("max_x", max_x_, 5.0);
    private_nh_.param("min_y", min_y_, -5.0);
    private_nh_.param("max_y", max_y_, 5.0);
    private_nh_.param("max_radius", max_radius_, 0.5);
    private_nh_.param("min_neighbors", min_neighbors_, 10);
    private_nh_.param("leaf_size", leaf_size_, 0.1);
    private_nh_.param("max_window_size", max_window_size_, 5);
    private_nh_.param("slope", slope_, 0.3);
    private_nh_.param("initial_distance", initial_distance_, 0.5);
    private_nh_.param("max_distance", max_distance_, 1.0);
    private_nh_.param("input_topic", input_topic_, std::string("/cloud_registered"));
    private_nh_.param("output_topic", output_topic_, std::string("/cloud_registered_filter"));
    private_nh_.param("ground_topic", ground_topic_, std::string("/ground_cloud"));
    private_nh_.param("frequency", frequency_, 10.0);
    private_nh_.param("frame_id", frame_id_, std::string("base_link"));
    private_nh_.param("use_ground_filter", use_ground_filter_, false);
    private_nh_.param("publish_ground", publish_ground_, false);
    private_nh_.param("use_voxel_filter", use_voxel_filter_, true);
    private_nh_.param("use_radius_filter", use_radius_filter_, true);

    // 设置订阅和发布
    point_cloud_sub_ = nh_.subscribe(input_topic_, 1, &CloudTransitNodelet::pointCloudCallback, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    ground_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_, 1);

    rate_ = new ros::Rate(frequency_);

    // 设置动态参数服务器
    server_ = new dynamic_reconfigure::Server<topic_transit::CloudFilterConfig>(private_nh_);
    f_ = boost::bind(&CloudTransitNodelet::configCallback, this, _1, _2);
    server_->setCallback(f_);

    NODELET_INFO("Point cloud filter nodelet initialized");
  }

  void configCallback(topic_transit::CloudFilterConfig &config, uint32_t level) {
    min_z_ = config.min_z;
    max_z_ = config.max_z;
    min_x_ = config.min_x;
    max_x_ = config.max_x;
    min_y_ = config.min_y;
    max_y_ = config.max_y;
    max_radius_ = config.max_radius;
    min_neighbors_ = config.min_neighbors;
    leaf_size_ = config.leaf_size;
    max_window_size_ = config.max_window_size;
    slope_ = config.slope;
    initial_distance_ = config.initial_distance;
    max_distance_ = config.max_distance;
    use_ground_filter_ = config.use_ground_filter;
    publish_ground_ = config.publish_ground;
    use_voxel_filter_ = config.use_voxel_filter;
    use_radius_filter_ = config.use_radius_filter;
  }

  void removeGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud) {
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(max_window_size_);
    pmf.setSlope(slope_);
    pmf.setInitialDistance(initial_distance_);
    pmf.setMaxDistance(max_distance_);

    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
    pmf.extract(ground_indices->indices);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground_indices);

    extract.setNegative(false);
    extract.filter(*ground_cloud);

    extract.setNegative(true);
    extract.filter(*non_ground_cloud);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    // Z轴过滤
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(min_z_, max_z_);
    pass_z.filter(*cloud);

    // // X轴过滤
    // pcl::PassThrough<pcl::PointXYZ> pass_x;
    // pass_x.setInputCloud(cloud);
    // pass_x.setFilterFieldName("x");
    // pass_x.setFilterLimits(min_x_, max_x_);
    // pass_x.filter(*cloud);
    //
    // // Y轴过滤
    // pcl::PassThrough<pcl::PointXYZ> pass_y;
    // pass_y.setInputCloud(cloud);
    // pass_y.setFilterFieldName("y");
    // pass_y.setFilterLimits(min_y_, max_y_);
    // pass_y.filter(*cloud);

    // 体素滤波（可选）
    if (use_voxel_filter_) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      voxel_filter.setInputCloud(cloud);
      voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      voxel_filter.filter(*cloud);
    }

    // 半径滤波（可选）
    if (use_radius_filter_) {
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
      radius_filter.setInputCloud(cloud);
      radius_filter.setRadiusSearch(max_radius_);
      radius_filter.setMinNeighborsInRadius(min_neighbors_);
      radius_filter.filter(*cloud);
    }

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    try {
      transformStamped = tfBuffer.lookupTransform(
          frame_id_, input->header.frame_id,
          ros::Time(0), ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex) {
      NODELET_WARN("TF lookup failed: %s", ex.what());
      return;
    }

    if (use_ground_filter_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

      removeGround(cloud, ground_cloud, non_ground_cloud);

      sensor_msgs::PointCloud2 output_non_ground;
      pcl::toROSMsg(*non_ground_cloud, output_non_ground);
      output_non_ground.header = input->header;

      sensor_msgs::PointCloud2 transformed_non_ground;
      tf2::doTransform(output_non_ground, transformed_non_ground, transformStamped);
      transformed_non_ground.header.frame_id = frame_id_;
      transformed_non_ground.header.stamp = ros::Time::now();
      point_cloud_pub_.publish(transformed_non_ground);

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
    else {
      sensor_msgs::PointCloud2 output_cloud;
      pcl::toROSMsg(*cloud, output_cloud);
      tf2::doTransform(output_cloud, output_cloud, transformStamped);
      output_cloud.header.frame_id = frame_id_;
      output_cloud.header.stamp = ros::Time::now();
      point_cloud_pub_.publish(output_cloud);
    }

    rate_->sleep();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber point_cloud_sub_;
  ros::Publisher point_cloud_pub_;
  ros::Publisher ground_cloud_pub_;
  ros::Rate* rate_;

  dynamic_reconfigure::Server<topic_transit::CloudFilterConfig>* server_;
  dynamic_reconfigure::Server<topic_transit::CloudFilterConfig>::CallbackType f_;

  double min_z_;
  double max_z_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double max_radius_;
  int min_neighbors_;
  double leaf_size_;
  int max_window_size_;
  double slope_;
  double initial_distance_;
  double max_distance_;
  std::string input_topic_;
  std::string output_topic_;
  std::string ground_topic_;
  double frequency_;
  std::string frame_id_;
  bool use_ground_filter_;
  bool publish_ground_;
  bool use_voxel_filter_;
  bool use_radius_filter_;
};

} // namespace topic_transit

PLUGINLIB_EXPORT_CLASS(topic_transit::CloudTransitNodelet, nodelet::Nodelet)