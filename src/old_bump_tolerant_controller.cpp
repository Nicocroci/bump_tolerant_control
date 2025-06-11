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
  bool debug_hover_mode_

  bool is_debuggin_mode_ = true;
  std::deque<double> last_yaw_deltas_;
  double last_yaw_ = 0.0;
  double init_yaw_; // Added declaration
  int clock_wise_rotation_; // Added declaration

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;
  double drone_radius_;
  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                                  mutex_drs_;
  typedef bump_tolerant_controller_plugin::bump_tolerant_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>                        Drs_t;
  boost::shared_ptr<Drs_t>                                                drs_;
  void                                                                    callbackDrs(bump_tolerant_controller_plugin::bump_tolerant_controllerConfig& config, uint32_t level);
  DrsConfig_t                                                             drs_params_;
  std::mutex                                                              mutex_drs_params_;

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
  mrs_lib::SubscribeHandler<std_msgs::Float64MultiArray> sh_wrench_;
  mrs_lib::SubscribeHandler<sensor_msgs::Imu> sh_imu_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_odometry_;

  // PID controller for altitude hold in hover mode
  example_controller_plugin::PIDController pid_alt_;
  double                                   hover_altitude_setpoint_;
  // Add these:
  example_controller_plugin::PIDController pid_x_;
  example_controller_plugin::PIDController pid_y_;
  double                                   hover_x_setpoint_;
  double                                   hover_y_setpoint_;
  bool                                     hover_setpoint_initialized_;
  
  ControlOutput hoveringControlOutput(double dt);

  // | ------------- diagnostics and status publishing ------------ |
  ros::Publisher pub_controller_diagnostics_;
  ros::Timer     timer_diagnostics_;
  void           timerDiagnostics(const ros::TimerEvent& event);
  mrs_msgs::ControllerDiagnostics controller_diagnostics_;
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

  success *= private_handlers->loadConfigFile(ros::package::getPath("bump_tolerant_control") + "/config/bump_tolerant_controller.yaml");

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
  param_loader.loadParam("debug_hover_mode", debug_hover_mode_, false);

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
  
  ROS_INFO_STREAM("[BumpTolerantController]: NodeHandle namespace: " << nh.getNamespace());
  sh_contact_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, nh.resolveName("/uav1/estimation_manager/contact_point"));
  sh_wrench_  = mrs_lib::SubscribeHandler<std_msgs::Float64MultiArray>(shopts, nh.resolveName("/uav1/estimation_manager/ext_wrench"));
  sh_imu_     = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, nh.resolveName("/uav1/hw_api/imu"));
  sh_odometry_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, nh.resolveName("/uav1/hw_api/odometry"));

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&BumpTolerantController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // PID controller parameters
  // Altitude PID (Force output)
  // P, I, D, output_ramp_limit (max change in force per sec), integral_limit (max force from I-term)
  pid_alt_.setParams(2.5, 0.8, 0.5, 20.0, 4.0);

  // Position PIDs (Acceleration output)
  // P, I, D, output_ramp_limit (max change in accel per sec), integral_limit (max accel from I-term)
  // Adjusted gains for X and Y:
  pid_x_.setParams(0.2, 0.0, 0.3, 10.0, 1.0); // P: 1.5->0.2, I: 0.2->0.0, D: 0.1->0.3
  pid_y_.setParams(0.2, 0.0, 0.3, 10.0, 1.0); // P: 1.5->0.2, I: 0.2->0.0, D: 0.1->0.3
  
  hover_setpoint_initialized_ = false;

  // | ------------------ diagnostics publisher ----------------- |
  pub_controller_diagnostics_ = nh_.advertise<mrs_msgs::ControllerDiagnostics>("controller_diagnostics", 1);
  timer_diagnostics_          = nh_.createTimer(ros::Duration(0.1), &BumpTolerantController::timerDiagnostics, this);


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
  hover_setpoint_initialized_ = false; // Reset hover setpoint flag
  if(is_initialized_) { // Ensure PID is initialized before resetting
    pid_alt_.reset(); // Reset PID controller states (e.g., integral term)
    // Add these:
    pid_x_.reset();
    pid_y_.reset();
  }

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

BumpTolerantController::ControlOutput BumpTolerantController::updateActive(const mrs_msgs::UavState& uav_state, [[maybe_unused]] const mrs_msgs::TrackerCommand& tracker_command) {
  // Update the member uav_state_ with the latest state
  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);
  // ROS_INFO_THROTTLE(1.0, "[BumpTolerantController]: updateActive() called");

  // | --------------- check dt validity -------------------------- |
  double dt = (ros::Time::now() - last_update_time_).toSec();
  if (first_iteration_) {
    dt = 0.01;
    first_iteration_ = false;
  } else if (dt <= 0.001) {
    ROS_WARN_THROTTLE(1.0, "[BumpTolerantController]: dt is too small (%.4f s), changing to 0.01 s.", dt);
    dt = 0.01;
  }
  last_update_time_ = ros::Time::now();

  // | --------------- check IMU data ----------------------------- |
  if (!sh_imu_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[BumpTolerantController]: No IMU message received yet.");
    return last_control_output_;  // Return last known good output or a safe default
  }
  auto imu_msg = sh_imu_.getMsg();

  // | --------------- get the yaw from the IMU ------------------ |
  if (!sh_odometry_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[BumpTolerantController]: No odometry message received yet.");
    return last_control_output_;
  }
  auto odom_msg = sh_odometry_.getMsg();
  auto q = odom_msg->pose.pose.orientation;
  double yaw = mrs_lib::AttitudeConverter(q).getYaw();

  // int clock_wise_rotation; // Moved to class member

  // | --------------- check external wrench message -------------- |
  if (!sh_wrench_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[BumpTolerantController]: No external wrench message received yet.");
    return last_control_output_;
  }
  auto ext_wrench = sh_wrench_.getMsg();
  if (ext_wrench->data.size() != 6) {
    ROS_ERROR("[BumpTolerantController]: external wrench message must have 6 elements");
    return last_control_output_;
  }

  // | --------------- check dt validity -------------------------- |
  if (fabs(dt) < 0.001) {

    ROS_DEBUG("[BumpTolerantController]: the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }

  if (debug_hover_mode_) return hoveringControlOutput(dt);

  // | --------------- check IMU data ----------------------------- |
  if (sh_imu_.hasMsg()) {
    // auto imu_msg = sh_imu_.getMsg(); // This was a re-declaration, original imu_msg is fine
  } else return last_control_output_;
    
  
  // | --------------- get the yaw from the IMU ------------------ |
  // The following block is a duplicate and will be removed.
  // auto odom_msg = sh_odometry_.getMsg();
  // auto q = odom_msg->pose.pose.orientation;
  // double yaw = mrs_lib::AttitudeConverter(q).getYaw();

  double yaw_moment = ext_wrench->data[5];


  if (is_debuggin_mode_) {
    // inertia attorno a z (dal tuo SDF: Izz = 0.315375)
    double Izz_  = 0.315375;  

    // Parametri di ammettanza per yaw
    double Kv_yaw_     = 1.0;   // [rad/s per Nm]
    double Dv_yaw_     = 0.5;   // [N·m·s/rad]
    double tau_hpf_    = 0.05;  // costante di tempo HPF [s]
    double hpf_state_  = 0.0;   // stato interno HPF
    double last_wz_    = 0.0;   // per derivata omega
    ros::Time last_imu_time_;
    double max_yaw_rate_ = 0.8; // saturazione [rad/s]

        // --- 1) calcolo dt e omega_dot ----------------
    auto imu_msg = sh_imu_.getMsg();
    ros::Time now_imu = imu_msg->header.stamp;
    double dt_imu = (now_imu - last_imu_time_).toSec();
    if (dt_imu <= 0.0 || dt_imu > 0.1) dt_imu = 0.01;


    double wz = imu_msg->angular_velocity.z;
    double domega_z = (wz - last_wz_) / dt_imu;

    // --- 2) stima momentum‐based torque esterno  ----
    double tau_e_z = Izz_ * domega_z;

    // --- 3) HPF per isolare impulso collisione ----
    // discretizzazione HPF: y[k] = α*( y[k-1] + u[k] - u[k-1] )
    double alpha = tau_hpf_ / (tau_hpf_ + dt_imu);
    hpf_state_ = alpha * (hpf_state_ + tau_e_z - hpf_state_);
    double tau_hpf = hpf_state_;

    // --- 4) comando yaw rate per ammettanza --------
    //    Mv * d r_dot + Dv * r_dot = Kv_yaw * tau_hpf
    //    in prima approssimazione:  r_dot = (Kv*tau_hpf - Dv*wz)/Izz
    double desired_yaw_rate = (Kv_yaw_ * tau_hpf - Dv_yaw_ * wz) / Izz_;

    // saturazione
    desired_yaw_rate = std::max(-max_yaw_rate_, std::min(max_yaw_rate_, desired_yaw_rate));

    // scrivo direttamente nel desired_heading_rate MRS
    last_control_output_.desired_heading_rate = desired_yaw_rate;

    // faccio partire un timeout o un flag per far tornare al normale heading dopo 0.2s
    // (opzionale)
    double roll = 0.0;
    double pitch = 0.0;
    double force = common_handlers_->getMass() * common_handlers_->g;

    mrs_msgs::HwApiAttitudeCmd attitude_cmd;
    attitude_cmd.orientation = mrs_lib::AttitudeConverter(roll, pitch, yaw);
    attitude_cmd.throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(
    common_handlers_->throttle_model, force
    ); 
    ROS_INFO_THROTTLE(1.0, "[BumpTolerantController]: Throttle command: %.2f", attitude_cmd.throttle);

    last_control_output_.control_output = attitude_cmd;
    last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(roll, pitch, yaw);
    last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(0, 0, 0);
    last_control_output_.diagnostics.controller = "BumpTolerantController[DEBUG]";
    last_control_output_.diagnostics.total_mass = common_handlers_->getMass();

    // NOTE: This block was duplicated and is removed.
    // auto odom_msg = sh_odometry_.getMsg();
    // auto q = odom_msg->pose.pose.orientation;
    // double yaw = mrs_lib::AttitudeConverter(q).getYaw();

    // Monitoraggio rotazioni
   double yaw_delta = fabs(yaw - last_yaw_);
   last_yaw_ = yaw;
   last_yaw_deltas_.push_back(yaw_delta);
   if (last_yaw_deltas_.size() > 3) last_yaw_deltas_.pop_front();

   if (last_yaw_deltas_.size() == 3) {
     double sum_rot = last_yaw_deltas_[0] + last_yaw_deltas_[1] + last_yaw_deltas_[2];
    if (sum_rot < (0.1 * M_PI / 180.0)) {
      ROS_INFO("[BumpTolerantController]: Somma rotazioni ultime 3 iterazioni: %.4f rad (%.2f deg), entro in hovering", sum_rot, sum_rot * 180.0 / M_PI);
      debug_hover_mode_ = true;
      return hoveringControlOutput(dt);
    }
  }
  
  ROS_INFO("[BumpTolerantController]: actual yaw rate: %.4f rad/s", wz);
  last_update_time_ = uav_state.header.stamp; // Update time before returning
  return last_control_output_; // Return the admittance control output
} // THIS IS THE CORRECTED CLOSING BRACE FOR updateActive if is_debuggin_mode_ is true and the inner conditions lead to this return path.

  // | --------------- handle first iteration --------------------- |
  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
    init_yaw_ = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYaw(); // Use member variable
    if (yaw_moment > 0.1) {
      clock_wise_rotation_ = 1; // Use member variable
    } else if (yaw_moment < -0.1) {
      clock_wise_rotation_ = -1; // Use member variable
    } else {
      clock_wise_rotation_ = 0; // Use member variable
      ROS_INFO_THROTTLE(1.0, "[BumpTolerantController]: yaw moment is not significant (%.4f), skipping rotation", yaw_moment);
      // first_iteration_ = true; // This would loop indefinitely if hover is not achieved
      last_update_time_ = uav_state.header.stamp; // Update time before returning
      return hoveringControlOutput(dt); // Enter hover if no significant moment
    }
    // | --------------- initialize contact point ------------------- |
    if (!contact_point_initialized_) {
      contact_point_ = Eigen::Vector3d(
        odom_msg->pose.pose.position.x + drone_radius_ * cos(M_PI/4 + init_yaw_), // Use member variable
        odom_msg->pose.pose.position.y + drone_radius_ * sin(M_PI/4 + init_yaw_), // Use member variable
        odom_msg->pose.pose.position.z
      );
      theta_ = 0.0;
      contact_point_initialized_ = true;
      ROS_INFO("[BumpTolerantController]: contact point initialized at (%.2f, %.2f, %.2f), with yaw %.2f",
               contact_point_.x(), contact_point_.y(), contact_point_.z(), init_yaw_); // Use member variable
    }
  } else {
    // dt is already calculated at the top if !first_iteration
    // The original dt = (uav_state.header.stamp - last_update_time_).toSec(); is now at the top
  }
  // This is the correct place to update last_update_time_ for the main control path
  last_update_time_ = uav_state.header.stamp;


  // | --------------- update theta for rotation ------------------ |
  double rotation_speed = 0.5; // rad/s
  theta_ += dt * rotation_speed;

  if (theta_ > (M_PI/8)*0.8) return hoveringControlOutput(dt); // Pass dt

  // | --------------- prepare control reference ------------------ |
  example_controller_plugin::PIDController pid_x, pid_y, pid_z;
  pid_x.setParams(1.5, 0.2, 0.0, 2.0, 1.0);
  pid_y.setParams(1.5, 0.2, 0.0, 3.0, 1.0);
  pid_z.setParams(2.0, 0.3, 0.0, 4.0, 1.0);

  Eigen::Vector3d pos_ref(
    contact_point_.x() + drone_radius_ * cos(theta_ - M_PI/2 + init_yaw_), // Use member variable
    contact_point_.y() + drone_radius_ * sin(theta_ - M_PI/2 + init_yaw_), // Use member variable
    contact_point_.z()
  );
  ROS_INFO("[BumpTolerantController]: position reference set to (%.2f, %.2f, %.2f)", 
             pos_ref.x(), pos_ref.y(), pos_ref.z());
  ROS_INFO("[BumpTolerantController]: actual position at (%.2f, %.2f, %.2f)", 
             uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);

  Eigen::Vector3d error = pos_ref - Eigen::Vector3d(
    uav_state.pose.position.x,
    uav_state.pose.position.y,
    uav_state.pose.position.z
  );
  ROS_INFO("[BumpTolerantController]: position error: (%.2f, %.2f, %.2f)", 
             error.x(), error.y(), error.z());


  // | --------------- compute PID acceleration ------------------- |
  double acc_x = pid_x.update(error.x(), dt);
  double acc_y = pid_y.update(error.y(), dt);
  double acc_z = pid_z.update(error.z(), dt);

  // | --------------- compute attitude command ------------------- |
  double g = common_handlers_->g;
  double des_roll  =  acc_y / g;
  double des_pitch = -acc_x / g;
  double des_yaw = atan2(pos_ref.y() - uav_state.pose.position.y,
                         pos_ref.x() - uav_state.pose.position.x);
  
  ROS_INFO("[BumpTolerantController]: desired roll: %.2f and roll from IMU: %.2f",
           des_roll, mrs_lib::AttitudeConverter(q).getRoll());
  ROS_INFO("[BumpTolerantController]: desired pitch: %.2f and pitch from IMU: %.2f",
           des_pitch, mrs_lib::AttitudeConverter(q).getPitch());
  ROS_INFO("[BumpTolerantController]: desired yaw: %.2f and yaw from IMU: %.2f",
           des_yaw, yaw); 
  ROS_INFO("[BumpTolerantController]: yaw error: %.2f",
           yaw - des_yaw);
  mrs_msgs::HwApiAttitudeCmd attitude_cmd;
  attitude_cmd.orientation = mrs_lib::AttitudeConverter(des_roll, des_pitch, des_yaw);

  // | --------------- compute throttle command ------------------- |
  double force = common_handlers_->getMass() * (g + acc_z);
  attitude_cmd.throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(
    common_handlers_->throttle_model, force
  );

  // | --------------- fill in the optional parts ----------------- |

  last_control_output_.control_output = attitude_cmd;
  last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(attitude_cmd.orientation);
  last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(acc_x, acc_y, acc_z);
  last_control_output_.desired_heading_rate = 0;
  last_control_output_.diagnostics.controller = "BumpTolerantController";
  last_control_output_.diagnostics.total_mass = common_handlers_->getMass();

  return last_control_output_;
}

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

/* hoveringControlOutput() //{ */

BumpTolerantController::ControlOutput BumpTolerantController::hoveringControlOutput(double dt) { // Added dt argument
  ControlOutput hover_output;

  mrs_msgs::UavState current_uav_state;
  {
    std::scoped_lock lock(mutex_uav_state_); // Protect access to uav_state_
    current_uav_state = uav_state_;
  }

  if (!hover_setpoint_initialized_) {
    hover_altitude_setpoint_ = current_uav_state.pose.position.z;
    hover_x_setpoint_ = current_uav_state.pose.position.x; // Initialize X setpoint
    hover_y_setpoint_ = current_uav_state.pose.position.y; // Initialize Y setpoint
    pid_alt_.reset(); 
    pid_x_.reset();   // Reset X PID
    pid_y_.reset();   // Reset Y PID
    hover_setpoint_initialized_ = true;
    ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: Hover XYZ setpoints INITIALIZED to: X: %.3f m, Y: %.3f m, Z: %.3f m", hover_x_setpoint_, hover_y_setpoint_, hover_altitude_setpoint_);
  }

  // Altitude PID (Z-axis)
  double current_altitude = current_uav_state.pose.position.z;
  double altitude_error = hover_altitude_setpoint_ - current_altitude;
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: dt: %.4f s", dt);
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: Z Alt: %.3f, Setpoint: %.3f, Error: %.3f", current_altitude, hover_altitude_setpoint_, altitude_error);
  double corrective_force_z_raw = pid_alt_.update(altitude_error, dt);
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: Z PID Output (Raw Corrective Force Z): %.3f", corrective_force_z_raw);
  double max_corrective_force_z = 0.5 * common_handlers_->getMass() * common_handlers_->g; // Max 50% of g for correction
  double corrective_force_z = std::max(-max_corrective_force_z, std::min(corrective_force_z_raw, max_corrective_force_z));
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: Z Clamped Corrective Force Z: %.3f (Max: %.3f)", corrective_force_z, max_corrective_force_z);

  // X Position PID
  double current_x = current_uav_state.pose.position.x;
  double x_error = hover_x_setpoint_ - current_x;
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: X Pos: %.3f, Setpoint: %.3f, Error: %.3f", current_x, hover_x_setpoint_, x_error);
  double corrective_accel_x_raw = pid_x_.update(x_error, dt); // PID for position outputs corrective acceleration
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: X PID Output (Raw Corrective Accel X): %.3f", corrective_accel_x_raw);
  double max_corrective_accel_xy = 2.0; // Max corrective acceleration e.g., 2.0 m/s^2 (tune as needed)
  double corrective_accel_x = std::max(-max_corrective_accel_xy, std::min(corrective_accel_x_raw, max_corrective_accel_xy));
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: X Clamped Corrective Accel X: %.3f (Max: %.3f)", corrective_accel_x, max_corrective_accel_xy);

  // Y Position PID
  double current_y = current_uav_state.pose.position.y;
  double y_error = hover_y_setpoint_ - current_y;
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: Y Pos: %.3f, Setpoint: %.3f, Error: %.3f", current_y, hover_y_setpoint_, y_error);
  double corrective_accel_y_raw = pid_y_.update(y_error, dt); // PID for position outputs corrective acceleration
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: Y PID Output (Raw Corrective Accel Y): %.3f", corrective_accel_y_raw);
  double corrective_accel_y = std::max(-max_corrective_accel_xy, std::min(corrective_accel_y_raw, max_corrective_accel_xy));
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: Y Clamped Corrective Accel Y: %.3f (Max: %.3f)", corrective_accel_y, max_corrective_accel_xy);

  // Attitude Calculation
  double actual_yaw = mrs_lib::AttitudeConverter(current_uav_state.pose.orientation).getYaw();
  double g = common_handlers_->g;

  // Convert desired accelerations to roll and pitch
  // Assuming standard ENU world frame and FLU body frame for the drone:
  // Positive X world acceleration (move forward) -> Negative Pitch
  // Positive Y world acceleration (move left)    -> Positive Roll
  double desired_roll_raw  = corrective_accel_y / g;
  double desired_pitch_raw = -corrective_accel_x / g; 

  // Clamp desired roll and pitch (e.g., +/- 20 degrees, convert to radians)
  double max_tilt_angle_rad = 20.0 * M_PI / 180.0; // 20 degrees tilt limit
  double desired_roll  = std::max(-max_tilt_angle_rad, std::min(desired_roll_raw,  max_tilt_angle_rad));
  double desired_pitch = std::max(-max_tilt_angle_rad, std::min(desired_pitch_raw, max_tilt_angle_rad));
  
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: Desired Roll_raw: %.3f, Pitch_raw: %.3f -> Clamped Roll: %.3f rad, Pitch: %.3f rad", 
           desired_roll_raw, desired_pitch_raw, desired_roll, desired_pitch);

  double roll  = desired_roll;
  double pitch = desired_pitch;
  double yaw   = actual_yaw; 

  // Thrust Calculation (from altitude PID)
  double base_force = common_handlers_->getMass() * g;
  double total_force = base_force + corrective_force_z;
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: Base Force: %.3f, Total Force (Pre-clamp): %.3f", base_force, total_force);
  total_force = std::max(0.0, std::min(total_force, 2.0 * base_force)); // e.g., max 2g thrust for safety
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: Final Total Force (Post-clamp): %.3f", total_force);
  
  ROS_INFO_THROTTLE(0.2, "[BumpTolerantController][HOVER_SUM]: ErrZ:%.2f,Fz_corr:%.2f,F_tot:%.2f | ErrX:%.2f,Ax_corr:%.2f,PitchCmd:%.2f | ErrY:%.2f,Ay_corr:%.2f,RollCmd:%.2f", 
    altitude_error, corrective_force_z, total_force, 
    x_error, corrective_accel_x, desired_pitch,
    y_error, corrective_accel_y, desired_roll);

  mrs_msgs::HwApiAttitudeCmd attitude_cmd;
  attitude_cmd.orientation = mrs_lib::AttitudeConverter(roll, pitch, yaw);
  attitude_cmd.throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(
    common_handlers_->throttle_model, total_force
  );
  ROS_INFO("[BumpTolerantController][HOVER_DEBUG]: Throttle Command: %.3f", attitude_cmd.throttle);

  hover_output.control_output = attitude_cmd;
  hover_output.desired_orientation = mrs_lib::AttitudeConverter(roll, pitch, yaw);
  // Desired unbiased acceleration should reflect the controller's intent
  // For Z, it's corrective_force_z / mass. For X & Y, it's corrective_accel_x & corrective_accel_y.
  hover_output.desired_unbiased_acceleration = Eigen::Vector3d(corrective_accel_x, corrective_accel_y, corrective_force_z / common_handlers_->getMass());
  hover_output.desired_heading_rate = 0; // Command zero heading rate to maintain the current yaw
  hover_output.diagnostics.controller = "BumpTolerantController[HOVER_XYZ]";
  hover_output.diagnostics.total_mass = common_handlers_->getMass();

  return hover_output;
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

// ------------------------- timers -------------------------
// timerDiagnostics() //{

void BumpTolerantController::timerDiagnostics([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return; // Added return to prevent access to uninitialized members
  }

  // Prepare diagnostics message
  controller_diagnostics_.controller = "BumpTolerantController"; // Changed: active_controller to controller

  // Publish the diagnostics message
  try {
    pub_controller_diagnostics_.publish(controller_diagnostics_);
  } catch (...) {
    ROS_ERROR("Exception caught during publishing topic \'%s\'.", pub_controller_diagnostics_.getTopic().c_str());
  }

}

}  // namespace bump_tolerant_controller

}  // namespace bump_tolerant_controller_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bump_tolerant_controller_plugin::bump_tolerant_controller::BumpTolerantController, mrs_uav_managers::Controller)
