/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <pid.hpp>

#include <mrs_uav_managers/controller.h>

#include <dynamic_reconfigure/server.h>
#include <bump_tolerant_control/bump_tolerant_controllerConfig.h>
// filepath: /home/nicolo/mrs_ws/src/bump_tolerant_control/src/bump_tolerant_controller.cpp

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>

#include <mrs_lib/subscribe_handler.h>
//}

namespace bump_tolerant_controller_plugin
{

namespace bump_tolerant_controller
{

/* //{ class BumpTolerantController */

class BumpTolerantController : public mrs_uav_managers::Controller {

public:
  bool initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput& last_control_output);

  void deactivate(void);

  void updateInactive(const mrs_msgs::UavState& uav_state, const std::optional<mrs_msgs::TrackerCommand>& tracker_command);

  ControlOutput updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState& new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& cmd);

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;
  double drone_radius_;
  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                              mutex_drs_;
  typedef bump_tolerant_controller_plugin::bump_tolerant_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>                    Drs_t;
  boost::shared_ptr<Drs_t>                                            drs_;
  void                                                                callbackDrs(bump_tolerant_controller_plugin::bump_tolerant_controllerConfig& config, uint32_t level);
  DrsConfig_t                                                         drs_params_;
  std::mutex                                                          mutex_drs_params_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

  // | --------- throttle generation and mass estimation -------- |

  double _uav_mass_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;
  std::atomic<bool> is_parallel_    = false;

  // | --------------------- state of stability ----------------- |

  bool is_stable_ = false;

  // | -------------------- contact point ----------------------- |

  Eigen::Vector3d contact_point_;
  bool contact_point_initialized_ = false;
  double theta_ = 0.0;

  // | -------------------- subscribe handlers ------------------ |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_contact_;
  mrs_lib::SubscribeHandler<std_msgs::Float64MultiArray> sh_ext_wrench_;
  mrs_lib::SubscribeHandler<sensor_msgs::Imu> sh_imu_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

bool BumpTolerantController::initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                   std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  nh_ = nh;

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();
  ROS_INFO_STREAM("[BumpTolerantController]: mass is: " << _uav_mass_); 
  last_update_time_ = ros::Time(0);

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  bool success = true;

  success *= private_handlers->loadConfigFile(ros::package::getPath("bump_tolerant_controller_plugin") + "/config/bump_tolerant_controller.yaml");

  if (!success) {
    return false;
  }

  mrs_lib::ParamLoader param_loader(nh_, "BumpTolerantController");

  // param_loader.addYamlFile(ros::package::getPath("bump_tolerant_controller_plugin") + "/config/bump_tolerant_tracker.yaml");

  param_loader.loadParam("desired_roll", drs_params_.roll);
  param_loader.loadParam("desired_pitch", drs_params_.pitch);
  param_loader.loadParam("desired_yaw", drs_params_.yaw);
  param_loader.loadParam("desired_thrust_force", drs_params_.force);
  param_loader.loadParam("radius", drone_radius_);

  // | ------------------ finish loading params ----------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[BumpTolerantController]: could not load all parameters!");
    return false;
  }

  // | --------------- subscriber handler --------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "BumpTolerantController";
  shopts.no_message_timeout = ros::Duration(.0);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();
  
  sh_contact_               = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "contact_point_in");
  sh_ext_wrench_            = mrs_lib::SubscribeHandler<std_msgs::Float64MultiArray>(shopts, "external_wrench_in");
  sh_imu_                   = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, "imu_in");
  
  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&BumpTolerantController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[BumpTolerantController]: initialized");

  is_initialized_ = true;

  return true;
}

//}

/* //{ activate() */

bool BumpTolerantController::activate(const ControlOutput& last_control_output) {

  activation_control_output_ = last_control_output;

  first_iteration_ = true;

  is_active_ = true;

  ROS_INFO("[BumpTolerantController]: activated");

  return true;
}

//}

/* //{ deactivate() */

void BumpTolerantController::deactivate(void) {

  is_active_       = false;
  first_iteration_ = false;
  is_parallel_     = false;

  ROS_INFO("[BumpTolerantController]: deactivated");
}

//}

/* updateInactive() //{ */

void BumpTolerantController::updateInactive(const mrs_msgs::UavState& uav_state, [[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand>& tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;

  is_parallel_ = false;

}

//}

/* //{ updateActive() */

BumpTolerantController::ControlOutput BumpTolerantController::updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  // clear all the optional parts of the result
  last_control_output_.desired_heading_rate          = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.control_output                = {};

  if (!is_active_) {
    return last_control_output_;
  }

  // | ---------- calculate dt from the last iteration ---------- |

  double dt;
  auto ext_wrench = sh_ext_wrench_.getMsg();
  double yaw_moment = ext_wrench->data[5];
  int clock_wise_rotation = 0;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
    if(yaw_moment > 0.8) {
      clock_wise_rotation = 1;
    }else if (yaw_moment < -0.8) {
      clock_wise_rotation = -1;
    }else{
      clock_wise_rotation = 0;
      ROS_INFO_THROTTLE(1.0, "[BumpTolerantController]: yaw moment is not significant");
      return last_control_output_;

    }   
  } else {
    dt = (uav_state.header.stamp - last_update_time_).toSec();
  }

  last_update_time_ = uav_state.header.stamp;

  if (fabs(dt) < 0.001) {

    ROS_DEBUG("[BumpTolerantController]: the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }

  // | -------- check for the available output modalities ------- |

  if (common_handlers_->control_output_modalities.attitude) {
    ROS_INFO_THROTTLE(1.0, "[BumpTolerantController]: desired attitude output modality is available");
  }

  // | ---------- extract the detailed model parameters --------- |

  if (common_handlers_->detailed_model_params) {

    mrs_uav_managers::control_manager::DetailedModelParams_t detailed_model_params = common_handlers_->detailed_model_params.value();

    ROS_INFO_STREAM_THROTTLE(1.0, "[BumpTolerantController]: UAV inertia is: " << detailed_model_params.inertia);
  }

  // | ----------------- get the external wrench ---------------- |

  if (sh_ext_wrench_.hasMsg()) {
    auto ext_wrench = sh_ext_wrench_.getMsg();

    if (ext_wrench->data.size() != 6) {
      ROS_ERROR("[BumpTolerantController]: external wrench message must have 6 elements");
      return last_control_output_;
    }
  }

  // | ---------------------- get imu data ---------------------- |

  if (sh_imu_.hasMsg()) {
    auto imu_msg = sh_imu_.getMsg();

    if (imu_msg->angular_velocity_covariance[0] < 0.01) {
      ROS_WARN_THROTTLE(1.0, "[BumpTolerantController]: IMU data is not available");
      return last_control_output_;
    }
  }

  // | -------------------- check stability --------------------- |
  if (!contact_point_initialized_) {
    // Primo urto: salva il punto di contatto (bordo del drone nella direzione di heading)
    auto odom_msg = sh_contact_.getMsg();
    auto q = odom_msg->pose.pose.orientation;
    double yaw = mrs_lib::AttitudeConverter(q).getYaw();
    contact_point_ = Eigen::Vector3d(
      odom_msg->pose.pose.position.x + drone_radius_ * cos(M_PI/4 + yaw),
      odom_msg->pose.pose.position.y + drone_radius_ * sin(M_PI/4 + yaw),
      odom_msg->pose.pose.position.z
    );
    theta_ = 0.0;
    contact_point_initialized_ = true;
  }

  // Aggiorna theta per ruotare attorno al punto di contatto
  double rotation_speed = 0.5; // rad/s, scegli tu la velocità desiderata
  theta_ += dt * rotation_speed;

  // | -------------- prepare the control reference ------------- |

  double desired_yaw = drs_params.yaw;
  double yaw = 0.0;

  geometry_msgs::PoseStamped position_reference;
  position_reference.header = tracker_command.header;
  position_reference.pose.position.x = contact_point_.x() + drone_radius_ * cos(theta_ + M_PI/2 + yaw);
  position_reference.pose.position.y = contact_point_.y() + drone_radius_ * sin(theta_ + M_PI/2 + yaw);
  position_reference.pose.position.z = contact_point_.z();
  position_reference.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(desired_yaw);

  // | ---------------- prepare the control output --------------- |

  mrs_msgs::HwApiAttitudeCmd attitude_cmd;

  attitude_cmd.orientation = mrs_lib::AttitudeConverter(drs_params.roll, drs_params.pitch, drs_params.yaw);
  attitude_cmd.throttle    = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model,
                                                                             common_handlers_->getMass() * common_handlers_->g + drs_params.force);


  // | ----------------- set the control output ----------------- |

  last_control_output_.control_output = attitude_cmd;

  // | --------------- fill in the optional parts --------------- |

  last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(drs_params.roll, drs_params.pitch, drs_params.yaw);

  last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(0, 0, 0);
  last_control_output_.desired_heading_rate          = 0;

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.controller = "BumpTolerantController";

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus BumpTolerantController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void BumpTolerantController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState& new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void BumpTolerantController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr BumpTolerantController::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& constraints) {

  if (!is_initialized_) {
    return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  ROS_INFO("[BumpTolerantController]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void BumpTolerantController::callbackDrs(bump_tolerant_controller_plugin::bump_tolerant_controllerConfig& config, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_drs_params_, config, drs_params_);

  ROS_INFO("[BumpTolerantController]: dynamic reconfigure params updated");
}

//}

}  // namespace bump_tolerant_controller

}  // namespace bump_tolerant_controller_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bump_tolerant_controller_plugin::bump_tolerant_controller::BumpTolerantController, mrs_uav_managers::Controller)