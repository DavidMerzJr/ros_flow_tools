#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>

namespace ros_flow_tools
{

class PoseAdjuster
{
public:
  PoseAdjuster(ros::NodeHandle& nh, const std::string& pose_name);

  void updatePose(const std_msgs::Float64::ConstPtr& msg, const std::string dimension);

  void updateX (const std_msgs::Float64::ConstPtr& msg);
  void updateY (const std_msgs::Float64::ConstPtr& msg);
  void updateZ (const std_msgs::Float64::ConstPtr& msg);
  void updateRX(const std_msgs::Float64::ConstPtr& msg);
  void updateRY(const std_msgs::Float64::ConstPtr& msg);
  void updateRZ(const std_msgs::Float64::ConstPtr& msg);
  void updateRW(const std_msgs::Float64::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber x_sub_;
  ros::Subscriber y_sub_;
  ros::Subscriber z_sub_;
  ros::Subscriber rw_sub_;
  ros::Subscriber rx_sub_;
  ros::Subscriber ry_sub_;
  ros::Subscriber rz_sub_;
  ros::Publisher pub_;

  geometry_msgs::TransformStamped pose_;
  tf2_ros::TransformBroadcaster br_;
};

PoseAdjuster::PoseAdjuster(
    ros::NodeHandle& nh,
    const std::string& pose_name)
  : nh_(nh)
{
  pose_.header.frame_id = "map";
  pose_.child_frame_id = pose_name;
  pose_.transform.rotation.w = 1.0;

  x_sub_ = nh_.subscribe(pose_name + "_x", 5, &PoseAdjuster::updateX, this);
  y_sub_ = nh_.subscribe(pose_name + "_y", 5, &PoseAdjuster::updateY, this);
  z_sub_ = nh_.subscribe(pose_name + "_z", 5, &PoseAdjuster::updateZ, this);
  rw_sub_ = nh_.subscribe(pose_name + "_rw", 5, &PoseAdjuster::updateRW, this);
  rx_sub_ = nh_.subscribe(pose_name + "_rx", 5, &PoseAdjuster::updateRX, this);
  ry_sub_ = nh_.subscribe(pose_name + "_ry", 5, &PoseAdjuster::updateRY, this);
  rz_sub_ = nh_.subscribe(pose_name + "_rz", 5, &PoseAdjuster::updateRZ, this);
  pub_ = nh_.advertise<geometry_msgs::TransformStamped>(pose_name, 5);

  // Force the transform to broadcast
  updatePose(nullptr, "");

  return;
}

void PoseAdjuster::updateX(const std_msgs::Float64::ConstPtr& msg)
{
  if (msg != nullptr)
    updatePose(msg, "x");
  return;
}

void PoseAdjuster::updateY(const std_msgs::Float64::ConstPtr& msg)
{
  if (msg != nullptr)
    updatePose(msg, "y");
  return;
}

void PoseAdjuster::updateZ(const std_msgs::Float64::ConstPtr& msg)
{
  if (msg != nullptr)
    updatePose(msg, "z");
  return;
}

void PoseAdjuster::updateRX(const std_msgs::Float64::ConstPtr& msg)
{
  if (msg != nullptr)
    updatePose(msg, "rx");
  return;
}

void PoseAdjuster::updateRY(const std_msgs::Float64::ConstPtr& msg)
{
  if (msg != nullptr)
    updatePose(msg, "ry");
  return;
}

void PoseAdjuster::updateRZ(const std_msgs::Float64::ConstPtr& msg)
{
  if (msg != nullptr)
    updatePose(msg, "rz");
  return;
}

void PoseAdjuster::updateRW(const std_msgs::Float64::ConstPtr& msg)
{
  if (msg != nullptr)
    updatePose(msg, "rw");
  return;
}


void PoseAdjuster::updatePose(const std_msgs::Float64::ConstPtr& msg, const std::string dimension)
{
  if (msg != nullptr)
  {
    if (dimension == "x")
      pose_.transform.translation.x = msg->data;
    else if (dimension == "y")
      pose_.transform.translation.y = msg->data;
    else if (dimension == "z")
      pose_.transform.translation.z = msg->data;
    else if (dimension == "rw")
      pose_.transform.rotation.w = msg->data;
    else if (dimension == "rx")
      pose_.transform.rotation.x = msg->data;
    else if (dimension == "ry")
      pose_.transform.rotation.y = msg->data;
    else if (dimension == "rz")
      pose_.transform.rotation.z = msg->data;
  }

  pose_.header.stamp = ros::Time::now();
  br_.sendTransform(pose_);
  pub_.publish(pose_);
  return;
}

} // end namespace ros_flow_tools

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_adjuster");
  ros::NodeHandle nh;

  ros_flow_tools::PoseAdjuster pa (nh, "p");

  ros::spin();
  return 0;
}
