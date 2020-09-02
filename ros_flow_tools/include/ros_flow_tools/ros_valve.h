#ifndef ROS_VALVE_H
#define ROS_VALVE_H

#include <atomic>
#include <string>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

namespace ros_flow_tools
{

template <class MessageType>
class RosValve
{
public:
  RosValve(
      ros::NodeHandle& nh,
      const std::string& topic_in,
      const std::string& topic_out,
      const bool initial_state = false);

  bool toggleThroughput(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  void messageReceived(const std::shared_ptr<MessageType const>& msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::ServiceServer srvr_;

  std::atomic<bool> valve_open;
};

} // namespace ros_flow_tools

#endif // ROS_VALVE_H
