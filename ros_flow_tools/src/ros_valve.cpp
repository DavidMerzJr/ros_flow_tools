#include <ros_flow_tools/ros_valve.h>

namespace ros_flow_tools
{

template <class MessageType>
RosValve<MessageType>::RosValve(
    ros::NodeHandle& nh,
    const std::string& topic_in,
    const std::string& topic_out,
    const bool initial_state)
  : nh_(nh)
  , valve_open(initial_state)
{
  srvr_ = nh_.advertiseService<std_srvs::SetBool>("toggle_valve", std::bind(&RosValve::toggleThroughput, this, _1, _2));
  sub_ = nh_.subscribe(topic_in, 5, std::bind(&RosValve::messageReceived, this, _1));
  pub_ = nh_.advertise<MessageType>(topic_out, 5);

  return;
}

template <class MessageType>
bool RosValve<MessageType>::toggleThroughput(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  valve_open = req.data;
  res.success = true;
  return true;
}

template <class MessageType>
void RosValve<MessageType>::messageReceived(const std::shared_ptr<MessageType const>& msg)
{
  if (valve_open && (msg != nullptr))
  {
    pub_.publish(msg);
  }
  return;
}

} // end namespace ros_flow_tools
