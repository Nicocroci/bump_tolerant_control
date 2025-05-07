/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for loading dynamic parameters while the nodelet is running */
#include <dynamic_reconfigure/server.h>

/* this header file is created during compilation from python script dynparam.cfg */
#include <bump_tolerant_control/dynparamConfig.h>
/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <fstream>
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/* custom msgs of MRS group */
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/misc.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>

/* for operations with matrices */
#include <Eigen/Dense>

//}

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

namespace bump_tolerant_control
{

/* class BumpTolerantControl //{ */

class BumpTolerantControl : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  std::atomic<bool> is_initialized_ = false;

  /* ros parameters */
  std::string _uav_name_;

  // | ---------------------- msg callbacks --------------------- |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  void              callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);
  std::atomic<bool> have_goal_ = false;
  std::atomic<bool> waypoint_reached_ = false;

  // | --------------------- timer callbacks -------------------- |

  void           timerPublishDistToWaypoint(const ros::TimerEvent& te);
  ros::Publisher pub_dist_to_waypoint_;
  ros::Timer     timer_publish_dist_to_waypoint_;
  int            _rate_timer_publish_dist_to_waypoint_;

  void           timerPublishSetReference(const ros::TimerEvent& te);
  ros::Publisher pub_reference_;
  ros::Timer     timer_publisher_reference_;
  int            _rate_timer_publisher_reference_;

  void       timerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | ---------------- service server callbacks ---------------- |

  bool               callbackStartWaypointFollowing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_start_waypoints_following_;

  bool               callbackStopWaypointFollowing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_stop_waypoints_following_;

  bool               callbackFlyToFirstWaypoint(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_fly_to_first_waypoint_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_land_;
  bool               _land_end_;

  // | -------------------- loading waypoints ------------------- |

  std::vector<mrs_msgs::Reference> waypoints_;
  std::string                      _waypoints_frame_;
  bool                             waypoints_loaded_ = false;
  mrs_msgs::Reference              current_waypoint_;
  std::mutex                       mutex_current_waypoint_;
  int                              idx_current_waypoint_;
  int                              n_waypoints_;
  int                              _n_loops_;
  int                              c_loop_;
  std::mutex                       mutex_waypoint_idle_time_;
  Eigen::MatrixXd                  _offset_;

  // | ------------------- dynamic reconfigure ------------------ |

  typedef bump_tolerant_control::dynparamConfig                              Config;
  typedef dynamic_reconfigure::Server<bump_tolerant_control::dynparamConfig> ReconfigureServer;
  boost::recursive_mutex                                                      mutex_dynamic_reconfigure_;
  boost::shared_ptr<ReconfigureServer>                                        reconfigure_server_;
  void                                                                        callbackDynamicReconfigure(Config& config, uint32_t level);
  bump_tolerant_control::dynparamConfig                                      last_drs_config_;

  // | --------------------- waypoint idling -------------------- |

  bool       is_idling_ = false;
  ros::Timer timer_idling_;
  double     _waypoint_idle_time_;
  double     _waypoint_desired_dist_;
  void       timerIdling(const ros::TimerEvent& te);

  // | -------------------- support functions ------------------- |

  std::vector<mrs_msgs::Reference> matrixToPoints(const Eigen::MatrixXd& matrix);

  void offsetPoints(std::vector<mrs_msgs::Reference>& points, const Eigen::MatrixXd& offset);

  double distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose);
};

//}

/* onInit() //{ */

void BumpTolerantControl::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here

  have_goal_        = false;
  is_idling_        = false;
  waypoints_loaded_ = false;

  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  mrs_lib::ParamLoader param_loader(nh, "BumpTolerantControl");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("land_at_the_end", _land_end_);
  param_loader.loadParam("n_loops", _n_loops_);
  param_loader.loadParam("waypoint_desired_distance", _waypoint_desired_dist_);
  param_loader.loadParam("waypoint_idle_time", _waypoint_idle_time_);
  param_loader.loadParam("waypoints_frame", _waypoints_frame_);
  param_loader.loadParam("rate/publish_dist_to_waypoint", _rate_timer_publish_dist_to_waypoint_);
  param_loader.loadParam("rate/publish_reference", _rate_timer_publisher_reference_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);

  /* load waypoints as a half-dynamic matrix from config file */
  Eigen::MatrixXd waypoint_matrix;
  param_loader.loadMatrixDynamic("waypoints", waypoint_matrix, -1, 4);  // -1 indicates the dynamic dimension
  waypoints_            = matrixToPoints(waypoint_matrix);
  n_waypoints_          = waypoints_.size();
  waypoints_loaded_     = true;
  idx_current_waypoint_ = 0;
  c_loop_               = 0;
  ROS_INFO_STREAM_ONCE("[BumpTolerantControl]: " << n_waypoints_ << " waypoints loaded");
  ROS_INFO_STREAM_ONCE("[BumpTolerantControl]: " << _n_loops_ << " loops requested");

  /* load offset of all waypoints as a static matrix from config file */
  param_loader.loadMatrixKnown("offset", _offset_, 1, 4);
  offsetPoints(waypoints_, _offset_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[BumpTolerantControl]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | ------------------ initialize subscribers ----------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "BumpTolerantControl";
  shopts.no_message_timeout = ros::Duration(1.0);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_odometry_             = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_uav_in");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in",
                                                                                            &BumpTolerantControl::callbackControlManagerDiag, this);

  // | ------------------ initialize publishers ----------------- |

  pub_dist_to_waypoint_ = nh.advertise<mrs_msgs::Float64Stamped>("dist_to_waypoint_out", 1);
  pub_reference_        = nh.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);

  // | -------------------- initialize timers ------------------- |

  timer_publish_dist_to_waypoint_ = nh.createTimer(ros::Rate(_rate_timer_publish_dist_to_waypoint_), &BumpTolerantControl::timerPublishDistToWaypoint, this);

  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &BumpTolerantControl::timerCheckSubscribers, this);

  // you can disable autostarting of the timer by the last argument
  timer_publisher_reference_ = nh.createTimer(ros::Rate(_rate_timer_publisher_reference_), &BumpTolerantControl::timerPublishSetReference, this, false, false);

  // | --------------- initialize service servers --------------- |

  srv_server_start_waypoints_following_ = nh.advertiseService("start_waypoints_following_in", &BumpTolerantControl::callbackStartWaypointFollowing, this);
  srv_server_stop_waypoints_following_  = nh.advertiseService("stop_waypoints_following_in", &BumpTolerantControl::callbackStopWaypointFollowing, this);
  srv_server_fly_to_first_waypoint_     = nh.advertiseService("fly_to_first_waypoint_in", &BumpTolerantControl::callbackFlyToFirstWaypoint, this);

  // | --------------- initialize service clients --------------- |

  srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("land_out");

  // | ---------- initialize dynamic reconfigure server --------- |

  reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh));
  ReconfigureServer::CallbackType f = boost::bind(&BumpTolerantControl::callbackDynamicReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  /* set the default value of dynamic reconfigure server to the value of parameter with the same name */
  {
    std::scoped_lock lock(mutex_waypoint_idle_time_);
    last_drs_config_.waypoint_idle_time = _waypoint_idle_time_;
  }
  reconfigure_server_->updateConfig(last_drs_config_);

  ROS_INFO_ONCE("[BumpTolerantControl]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackControlManagerDiag() //{ */

void BumpTolerantControl::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr diagnostics) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[BumpTolerantControl]: Received first control manager diagnostics msg");

  // get the variable under the mutex
  mrs_msgs::Reference current_waypoint = mrs_lib::get_mutexed(mutex_current_waypoint_, current_waypoint_);

  // extract the pose part of the odometry
  geometry_msgs::Pose current_pose = mrs_lib::getPose(sh_odometry_.getMsg());

  double dist = distance(current_waypoint, current_pose);
  ROS_INFO("[BumpTolerantControl]: Distance to waypoint: %.2f", dist);

  if (have_goal_ && !diagnostics->tracker_status.have_goal) {
    have_goal_ = false;

    if (dist < _waypoint_desired_dist_) {
      waypoint_reached_ = true;
      ROS_INFO("[BumpTolerantControl]: Waypoint reached.");

      /* start idling at the reached waypoint */
      is_idling_ = true;

      ros::NodeHandle nh("~");
      timer_idling_ = nh.createTimer(ros::Duration(_waypoint_idle_time_), &BumpTolerantControl::timerIdling, this,
                                     true);  // the last boolean argument makes the timer run only once

      ROS_INFO("[BumpTolerantControl]: Idling for %.2f seconds.", _waypoint_idle_time_);
    }
  }
}

//}

// | --------------------- timer callbacks -------------------- |

/* timerPublishSetReference() //{ */

void BumpTolerantControl::timerPublishSetReference([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  /* return if the uav is still flying to the previous waypoints */
  if (have_goal_) {
    return;
  }

  /* return if the UAV is idling at a waypoint */
  if (is_idling_) {
    return;
  }

  /* shutdown node after flying through all the waypoints (call land service before) */
  if (idx_current_waypoint_ >= n_waypoints_) {

    c_loop_++;

    ROS_INFO("[BumpTolerantControl]: Finished loop %d/%d", c_loop_, _n_loops_);

    if (c_loop_ >= _n_loops_) {

      ROS_INFO("[BumpTolerantControl]: Finished %d loops of %d waypoints.", _n_loops_, n_waypoints_);

      if (_land_end_) {
        ROS_INFO("[BumpTolerantControl]: Calling land service.");
        std_srvs::Trigger srv_land_call;
        srv_client_land_.call(srv_land_call);
      }

      ROS_INFO("[BumpTolerantControl]: Shutting down.");
      ros::shutdown();
      return;

    } else {
      ROS_INFO("[BumpTolerantControl]: Starting loop %d/%d", c_loop_ + 1, _n_loops_);
      idx_current_waypoint_ = 0;
    }
  }

  /* create new waypoint msg */
  mrs_msgs::ReferenceStamped new_waypoint;

  // set the frame id in which the reference is expressed
  new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
  new_waypoint.header.stamp    = ros::Time::now();

  new_waypoint.reference = waypoints_.at(idx_current_waypoint_);

  // set the variable under the mutex
  mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(idx_current_waypoint_), current_waypoint_);

  ROS_INFO("[BumpTolerantControl]: Flying to waypoint %d: x: %.2f y: %.2f z: %.2f heading: %.2f", idx_current_waypoint_ + 1, new_waypoint.reference.position.x,
           new_waypoint.reference.position.y, new_waypoint.reference.position.z, new_waypoint.reference.heading);

  try {
    pub_reference_.publish(new_waypoint);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_reference_.getTopic().c_str());
  }

  if (waypoint_reached_) {
    idx_current_waypoint_++;
    waypoint_reached_ = false;
  }

  have_goal_ = true;
}

//}

/* timerPublishDistToWaypoint() //{ */

void BumpTolerantControl::timerPublishDistToWaypoint([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  /* do not publish distance to waypoint when the uav is not flying towards a waypoint */
  if (!have_goal_) {
    return;
  }

  // this routine can not work without the odometry
  if (!sh_odometry_.hasMsg()) {
    return;
  }

  // get the variable under the mutex
  mrs_msgs::Reference current_waypoint = mrs_lib::get_mutexed(mutex_current_waypoint_, current_waypoint_);

  // extract the pose part of the odometry
  geometry_msgs::Pose current_pose = mrs_lib::getPose(sh_odometry_.getMsg());

  double dist = distance(current_waypoint, current_pose);
  ROS_INFO("[BumpTolerantControl]: Distance to waypoint: %.2f", dist);

  mrs_msgs::Float64Stamped dist_msg;
  // it is important to set the frame id correctly !!
  dist_msg.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
  dist_msg.header.stamp    = ros::Time::now();
  dist_msg.value           = dist;

  try {
    pub_dist_to_waypoint_.publish(dist_msg);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_dist_to_waypoint_.getTopic().c_str());
  }
}

//}

/* timerCheckSubscribers() //{ */

void BumpTolerantControl::timerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_odometry_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[BumpTolerantControl]: Not received uav odom msg since node launch.");
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[BumpTolerantControl]: Not received tracker diagnostics msg since node launch.");
  }
}

//}

/* timerIdling() //{ */

void BumpTolerantControl::timerIdling([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO("[BumpTolerantControl]: Idling finished");
  is_idling_ = false;
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackStartWaypointFollowing() */

bool BumpTolerantControl::callbackStartWaypointFollowing([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[BumpTolerantControl]: Cannot start waypoint following, nodelet is not initialized.");
    return true;
  }

  if (waypoints_loaded_) {

    timer_publisher_reference_.start();

    ROS_INFO("[BumpTolerantControl]: Starting waypoint following.");

    res.success = true;
    res.message = "Starting waypoint following.";

  } else {

    ROS_WARN("[BumpTolerantControl]: Cannot start waypoint following, waypoints are not set.");
    res.success = false;
    res.message = "Waypoints not set.";
  }

  return true;
}

//}

/* //{ callbackStopWaypointFollowing() */

bool BumpTolerantControl::callbackStopWaypointFollowing([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[BumpTolerantControl]: Cannot stop waypoint following, nodelet is not initialized.");
    return true;
  }

  timer_publisher_reference_.stop();

  ROS_INFO("[BumpTolerantControl]: Waypoint following stopped.");

  res.success = true;
  res.message = "Waypoint following stopped.";

  return true;
}

//}

/* //{ callbackFlyToFirstWaypoint() */

bool BumpTolerantControl::callbackFlyToFirstWaypoint([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[BumpTolerantControl]: Cannot start waypoint following, nodelet is not initialized.");

    return true;
  }

  if (waypoints_loaded_) {

    /* create new waypoint msg */
    mrs_msgs::ReferenceStamped new_waypoint;

    // it is important to set the frame id correctly !!
    new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
    new_waypoint.header.stamp    = ros::Time::now();
    new_waypoint.reference       = waypoints_.at(0);

    mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(0), current_waypoint_);

    // set the variable under the mutex

    idx_current_waypoint_ = 0;
    c_loop_               = 0;

    have_goal_ = true;

    try {
      pub_reference_.publish(new_waypoint);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", pub_reference_.getTopic().c_str());
    }

    std::stringstream ss;
    ss << "Flying to first waypoint: x: " << new_waypoint.reference.position.x << ", y: " << new_waypoint.reference.position.y
       << ", z: " << new_waypoint.reference.position.z << ", heading: " << new_waypoint.reference.heading;

    ROS_INFO_STREAM_THROTTLE(1.0, "[BumpTolerantControl]: " << ss.str());

    res.success = true;
    res.message = ss.str();

  } else {

    ROS_WARN("[BumpTolerantControl]: Cannot fly to first waypoint, waypoints not loaded!");

    res.success = false;
    res.message = "Waypoints not loaded";
  }

  return true;
}

//}

// | -------------- dynamic reconfigure callback -------------- |

/* //{ callbackDynamicReconfigure() */

void BumpTolerantControl::callbackDynamicReconfigure([[maybe_unused]] Config& config, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_)
    return;

  ROS_INFO(
      "[BumpTolerantControl]:"
      "Reconfigure Request: "
      "Waypoint idle time: %.2f",
      config.waypoint_idle_time);

  {
    std::scoped_lock lock(mutex_waypoint_idle_time_);

    _waypoint_idle_time_ = config.waypoint_idle_time;
  }
}

//}

// | -------------------- support functions ------------------- |

/* matrixToPoints() //{ */

std::vector<mrs_msgs::Reference> BumpTolerantControl::matrixToPoints(const Eigen::MatrixXd& matrix) {

  std::vector<mrs_msgs::Reference> points;

  for (int i = 0; i < matrix.rows(); i++) {

    mrs_msgs::Reference point;
    point.position.x = matrix.row(i)(0);
    point.position.y = matrix.row(i)(1);
    point.position.z = matrix.row(i)(2);
    point.heading    = matrix.row(i)(3);

    points.push_back(point);
  }

  return points;
}

//}

/* offsetPoints() //{ */

void BumpTolerantControl::offsetPoints(std::vector<mrs_msgs::Reference>& points, const Eigen::MatrixXd& offset) {

  for (size_t i = 0; i < points.size(); i++) {

    points.at(i).position.x += offset(0);
    points.at(i).position.y += offset(1);
    points.at(i).position.z += offset(2);
    points.at(i).heading += offset(3);
  }
}

//}

/* distance() //{ */

double BumpTolerantControl::distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose) {

  return mrs_lib::geometry::dist(vec3_t(waypoint.position.x, waypoint.position.y, waypoint.position.z),
                                 vec3_t(pose.position.x, pose.position.y, pose.position.z));
}

//}

}  // namespace bump_tolerant_control

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bump_tolerant_control::BumpTolerantControl, nodelet::Nodelet);
