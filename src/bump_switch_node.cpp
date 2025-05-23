#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mutex>

namespace bump_switch_node
{

class BumpSwitchNode : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  ros::Subscriber contact_event_sub_;
  ros::Publisher switch_pub_;

  std::mutex mutex_;
  bool bump_active_ = false;
  double switch_cooldown_ = 2.0;
  ros::Time last_switch_;
  std::string bump_controller_ = "BumpTolerantController";
  std::string normal_controller_ = "MpcController";
  std::string contact_event_topic_ = "/contact_event";
  std::string switch_topic_ = "/control_manager/switch_controller";

  void contactEventCallback(const std_msgs::Float64::ConstPtr& msg);
  void switchController(const std::string& controller_name);
};

void BumpSwitchNode::onInit()
{
  ros::NodeHandle nh = getMTPrivateNodeHandle();

  nh.param("switch_cooldown", switch_cooldown_, 2.0);
  nh.param<std::string>("bump_controller", bump_controller_, "BumpTolerantController");
  nh.param<std::string>("normal_controller", normal_controller_, "MpcController");
  nh.param<std::string>("contact_event_topic", contact_event_topic_, "/contact_event");
  nh.param<std::string>("switch_topic", switch_topic_, "/control_manager/switch_controller");

  contact_event_sub_ = nh.subscribe(contact_event_topic_, 1, &BumpSwitchNode::contactEventCallback, this);
  switch_pub_ = nh.advertise<std_msgs::String>(switch_topic_, 1);

  last_switch_ = ros::Time::now();

  ROS_INFO("[BumpSwitchNode]: Initialized. Listening on: %s", contact_event_topic_.c_str());
}

void BumpSwitchNode::contactEventCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ros::Time now = ros::Time::now();
  if ((now - last_switch_).toSec() < switch_cooldown_)
    return;

  if (msg->data == 3.0 && !bump_active_)
  {
    bump_active_ = true;
    switchController(bump_controller_);
  }
  else if (msg->data != 3.0 && bump_active_)
  {
    bump_active_ = false;
    switchController(normal_controller_);
  }
}

void BumpSwitchNode::switchController(const std::string& controller_name)
{
  std_msgs::String msg;
  msg.data = controller_name;
  switch_pub_.publish(msg);
  last_switch_ = ros::Time::now();
  ROS_INFO("[BumpSwitchNode]: Switching to controller: %s", controller_name.c_str());
}

} // namespace bump_switch_node

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bump_switch_node::BumpSwitchNode, nodelet::Nodelet)