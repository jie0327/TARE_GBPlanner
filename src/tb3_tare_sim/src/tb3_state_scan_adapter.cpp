#include <cmath>
#include <string>
#include <vector>

#include <boost/bind/bind.hpp>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace
{
class Tb3StateScanAdapter
{
public:
  Tb3StateScanAdapter()
    : nh_()
    , private_nh_("~")
    , odom_filter_sub_(nh_, "/odom", 20)
    , scan_filter_sub_(nh_, "/velodyne_points", 10)
  {
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("sensor_frame", sensor_frame_, "sensor");

    double sensor_x = -0.064;
    double sensor_y = 0.0;
    double sensor_z = 0.122;
    double sensor_roll = 0.0;
    double sensor_pitch = 0.0;
    double sensor_yaw = 0.0;
    private_nh_.param("sensor_offset_x", sensor_x, sensor_x);
    private_nh_.param("sensor_offset_y", sensor_y, sensor_y);
    private_nh_.param("sensor_offset_z", sensor_z, sensor_z);
    private_nh_.param("sensor_roll", sensor_roll, sensor_roll);
    private_nh_.param("sensor_pitch", sensor_pitch, sensor_pitch);
    private_nh_.param("sensor_yaw", sensor_yaw, sensor_yaw);
    sensor_from_base_.setOrigin(tf::Vector3(sensor_x, sensor_y, sensor_z));
    sensor_from_base_.setRotation(tf::createQuaternionFromRPY(sensor_roll, sensor_pitch, sensor_yaw));

    state_pub_ = nh_.advertise<nav_msgs::Odometry>("/state_estimation", 20);
    registered_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/registered_scan", 5);
    odom_state_sub_ = nh_.subscribe("/odom", 20, &Tb3StateScanAdapter::odomCallback, this);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2>
        SyncPolicy;
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(30), odom_filter_sub_, scan_filter_sub_));
    sync_->registerCallback(boost::bind(&Tb3StateScanAdapter::scanCallback, this,
                                        boost::placeholders::_1, boost::placeholders::_2));

    ROS_INFO("TB3 TARE adapter: /odom -> /state_estimation, /velodyne_points -> /registered_scan");
    ROS_INFO("TB3 TARE sensor extrinsic from base_footprint: [%.3f %.3f %.3f] m", sensor_x, sensor_y, sensor_z);
  }

private:
  tf::Transform mapToSensor(const nav_msgs::Odometry& odom) const
  {
    tf::Transform map_to_base;
    tf::poseMsgToTF(odom.pose.pose, map_to_base);
    return map_to_base * sensor_from_base_;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& odom)
  {
    const tf::Transform map_to_sensor = mapToSensor(*odom);

    nav_msgs::Odometry state = *odom;
    state.header.frame_id = map_frame_;
    state.child_frame_id = sensor_frame_;
    tf::poseTFToMsg(map_to_sensor, state.pose.pose);
    state_pub_.publish(state);

    tf::StampedTransform transform(map_to_sensor, odom->header.stamp, map_frame_, sensor_frame_);
    tf_broadcaster_.sendTransform(transform);
  }

  void scanCallback(const nav_msgs::OdometryConstPtr& odom,
                   const sensor_msgs::PointCloud2ConstPtr& scan)
  {
    pcl::PointCloud<pcl::PointXYZI> input;
    pcl::fromROSMsg(*scan, input);

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(input, input, valid_indices);

    const tf::Transform map_to_sensor = mapToSensor(*odom);
    pcl::PointCloud<pcl::PointXYZI> registered;
    registered.reserve(input.size());
    for (const pcl::PointXYZI& input_point : input.points)
    {
      const tf::Vector3 point_in_map = map_to_sensor * tf::Vector3(input_point.x, input_point.y, input_point.z);
      pcl::PointXYZI output_point = input_point;
      output_point.x = point_in_map.x();
      output_point.y = point_in_map.y();
      output_point.z = point_in_map.z();
      registered.push_back(output_point);
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(registered, output);
    output.header.stamp = scan->header.stamp.isZero() ? odom->header.stamp : scan->header.stamp;
    output.header.frame_id = map_frame_;
    registered_scan_pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_state_sub_;
  ros::Publisher state_pub_;
  ros::Publisher registered_scan_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::Transform sensor_from_base_;
  std::string map_frame_;
  std::string sensor_frame_;

  message_filters::Subscriber<nav_msgs::Odometry> odom_filter_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> scan_filter_sub_;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2>
      SyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tb3_state_scan_adapter");
  Tb3StateScanAdapter adapter;
  ros::spin();
  return 0;
}
