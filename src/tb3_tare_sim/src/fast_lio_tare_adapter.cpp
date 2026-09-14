#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace
{
class FastLioTareAdapter
{
public:
  FastLioTareAdapter()
    : nh_()
    , private_nh_("~")
    , ready_cloud_count_(30)
    , cloud_count_(0)
    , has_odometry_(false)
    , gate_open_(false)
    , start_sent_(false)
  {
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("sensor_frame", sensor_frame_, "sensor");

    double lidar_x = -0.064;
    double lidar_y = 0.0;
    double lidar_z = 0.054;
    private_nh_.param("lidar_offset_x", lidar_x, lidar_x);
    private_nh_.param("lidar_offset_y", lidar_y, lidar_y);
    private_nh_.param("lidar_offset_z", lidar_z, lidar_z);
    private_nh_.param("ready_cloud_count", ready_cloud_count_, ready_cloud_count_);
    imu_to_lidar_.setIdentity();
    imu_to_lidar_.setOrigin(tf::Vector3(lidar_x, lidar_y, lidar_z));

    state_pub_ = nh_.advertise<nav_msgs::Odometry>("/state_estimation", 20);
    registered_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/registered_scan", 5);
    start_pub_ = nh_.advertise<std_msgs::Bool>("/start_exploration", 1, true);
    odom_sub_ = nh_.subscribe("/Odometry", 50, &FastLioTareAdapter::odomCallback, this);
    cloud_sub_ = nh_.subscribe("/cloud_registered", 10, &FastLioTareAdapter::cloudCallback, this);

    ROS_INFO("FAST-LIO2 TARE adapter: /Odometry -> /state_estimation, "
             "/cloud_registered -> /registered_scan");
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& odom)
  {
    tf::Transform map_to_imu;
    tf::poseMsgToTF(odom->pose.pose, map_to_imu);
    const tf::Transform map_to_lidar = map_to_imu * imu_to_lidar_;

    latest_state_ = *odom;
    latest_state_.header.frame_id = map_frame_;
    latest_state_.child_frame_id = sensor_frame_;
    tf::poseTFToMsg(map_to_lidar, latest_state_.pose.pose);
    latest_transform_ = tf::StampedTransform(
        map_to_lidar, odom->header.stamp, map_frame_, sensor_frame_);
    has_odometry_ = true;
    if (gate_open_)
    {
      publishLatestState();
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    ++cloud_count_;
    if (!gate_open_ && has_odometry_ && cloud_count_ >= ready_cloud_count_)
    {
      gate_open_ = true;
      publishLatestState();
      ROS_INFO("FAST-LIO2 warmup complete after %d registered clouds", cloud_count_);
    }
    if (!gate_open_)
    {
      return;
    }

    sensor_msgs::PointCloud2 registered = *cloud;
    registered.header.frame_id = map_frame_;
    registered_scan_pub_.publish(registered);
    publishStartWhenReady();
  }

  void publishLatestState()
  {
    state_pub_.publish(latest_state_);
    tf_broadcaster_.sendTransform(latest_transform_);
  }

  void publishStartWhenReady()
  {
    if (start_sent_ || !gate_open_)
    {
      return;
    }
    std_msgs::Bool start;
    start.data = true;
    start_pub_.publish(start);
    start_sent_ = true;
    ROS_INFO("FAST-LIO2 is ready after %d registered clouds; starting TARE exploration", cloud_count_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher state_pub_;
  ros::Publisher registered_scan_pub_;
  ros::Publisher start_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::Transform imu_to_lidar_;
  tf::StampedTransform latest_transform_;
  nav_msgs::Odometry latest_state_;
  std::string map_frame_;
  std::string sensor_frame_;
  int ready_cloud_count_;
  int cloud_count_;
  bool has_odometry_;
  bool gate_open_;
  bool start_sent_;
};
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fast_lio_tare_adapter");
  FastLioTareAdapter adapter;
  ros::spin();
  return 0;
}
