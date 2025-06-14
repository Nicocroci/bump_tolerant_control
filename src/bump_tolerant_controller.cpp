/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <pid.hpp> // Assuming pid.hpp is in your include path or same directory
#include <common.h>
#include <mpc_controller.h>
#include <mrs_uav_managers/control_manager/common_handlers.h>


#include <mrs_uav_managers/controller.h>

#include <dynamic_reconfigure/server.h>
#include <bump_tolerant_control/bump_tolerant_controllerConfig.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h> 

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h> 
#include <std_srvs/SetBool.h> 

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h> 
#include <geometry_msgs/Vector3.h>

#define OUTPUT_ACTUATORS 0
#define OUTPUT_CONTROL_GROUP 1
#define OUTPUT_ATTITUDE_RATE 2
#define OUTPUT_ATTITUDE 3
//}

namespace bump_tolerant_controller_plugin
{

namespace bump_tolerant_controller
{

/* structs //{ */

typedef struct
{
  double kiwxy;          
  double kibxy;          
  double kiwxy_lim;      
  double kibxy_lim;      
  double km;             
  double km_lim;         
  double kq_roll_pitch;  
  double kq_yaw;         
  double kw_rp;          
  double kw_y;           
} Gains_t;

//}

class BumpTolerantController : public mrs_uav_managers::Controller {

public:
  BumpTolerantController(); 

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

  void holdPose(const mrs_msgs::UavState& uav_state, double dt);

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::string name_;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  boost::recursive_mutex                                      mutex_drs_;
  typedef bump_tolerant_controller_plugin::bump_tolerant_controllerConfig DrsConfig_t; 
  typedef dynamic_reconfigure::Server<DrsConfig_t>            Drs_t;
  boost::shared_ptr<Drs_t>                                    drs_;
  void                                                        callbackDrs(DrsConfig_t &config, uint32_t level);
  DrsConfig_t                                                 drs_params_;

  void BTC(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command, const double &dt, const mrs_uav_managers::control_manager::CONTROL_OUTPUT &output_modality);

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

  double _uav_mass_; 
  double uav_mass_difference_; 

  Gains_t gains_;
  Eigen::Vector2d Iw_w_;  
  Eigen::Vector2d Ib_b_;  
  std::mutex      mutex_integrals_; 
  std::mutex      mutex_gains_; 

  ros::Timer timer_gains_; 
  void       timerGains(const ros::TimerEvent &event);

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;

  ros::ServiceServer service_set_integral_terms_;
  bool               callbackSetIntegralTerms(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool               integral_terms_enabled_ = true;

  mrs_lib::SubscribeHandler<geometry_msgs::Vector3> sh_ext_force_;
  mrs_lib::SubscribeHandler<sensor_msgs::Imu> sh_imu_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_odometry_;
  
  ros::Publisher pub_controller_diagnostics_;
  ros::Publisher pub_disturbances_;
  ros::Timer     timer_diagnostics_;
  void           timerDiagnostics(const ros::TimerEvent& event);
  mrs_msgs::ControllerDiagnostics controller_diagnostics_; 

  // | ------------- Wall Interaction Controller Specifics ------------- |
  enum WallInteractionState {
    ALIGNING_TO_WALL,
    SLIDING_ALONG_WALL_1,
    SLIDING_ALONG_WALL,
  };
  WallInteractionState wall_interaction_state_;

  Eigen::Vector3d wall_normal_inertial_estimate_;
  bool            has_wall_normal_estimate_ = false;
  ros::Time       last_wall_normal_update_time_;

  double p_desired_contact_force_;      
  double p_alignment_max_yaw_rate_;     

  double p_alignment_target_roll_;      
  double p_alignment_target_pitch_;     
  double p_alignment_yaw_rate_damping_; 
  double p_alignment_yaw_rate_stop_threshold_; 
  double p_alignment_min_contact_force_; 
  int    p_alignment_yaw_rate_low_count_threshold_; 

  double p_sliding_kp_force_;
  double p_sliding_ki_force_;
  double p_sliding_force_integral_limit_;
  double force_integral_error_;

  double p_sliding_kp_pos_tangent_;
  double p_sliding_kd_pos_tangent_;

  double p_sliding_kp_pos_z_;
  double p_sliding_kd_pos_z_;

  Eigen::Vector3d sliding_tangent_inertial_cmd_;

  bool first_alignment_iteration_ = true;
  bool first_sliding_iteration_ = true;
  int  alignment_yaw_rate_low_counter_ = 0;

  ControlOutput hoveringControlOutput(const double dt); 

  bool holding_pose_ = false;
  bool holding_pose_debug_ = false;
  ros::Time hold_start_time_;
  geometry_msgs::Point hold_position_;
  geometry_msgs::Quaternion hold_orientation_;

  double m_0;
  double q_0;
  std::deque<double> m_data;
  std::deque<double> q_data;
  const size_t max_size = 200;

  double first_yaw_;
  double rotation_delta_;
  double angle_force_normal_;
  double forces_product_;
  std::string is_parallel_to_;
  const double PARALLEL_THRESHOLD_HIGH = 0.9;
  const double PARALLEL_THRESHOLD_LOW = 0.1;

  Eigen::Vector3d admittance_reference_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d admittance_reference_vel_ = Eigen::Vector3d::Zero();

  double m_wall; //slope of the wall
  double q_wall;

  ros::Time last_wall_rotation_time_ = ros::Time(0);
  bool is_rotating_wall_ = true;


  Eigen::Vector3d tangent_ref_pos;
  Eigen::Vector3d tangent_ref_vel;
  bool first_tangent_iteration_ = true;

};

/* BumpTolerantController() constructor //{ */
BumpTolerantController::BumpTolerantController() {
  Iw_w_.setZero();
  Ib_b_.setZero();
  uav_mass_difference_ = 0.0;
  wall_interaction_state_ = ALIGNING_TO_WALL; 
}
//}

/* initialize() //{ */
bool BumpTolerantController::initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                      std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {
  nh_ = nh;
  common_handlers_ = common_handlers;
  private_handlers_ = private_handlers;
  name_ = "BumpTolerantController"; 

  mrs_lib::ParamLoader param_loader(nh_, name_);

  _uav_mass_ = common_handlers_->getMass(); 

  param_loader.loadParam("gains/kiwxy", gains_.kiwxy, 0.0);
  param_loader.loadParam("gains/kibxy", gains_.kibxy, 0.0);
  param_loader.loadParam("gains/kiwxy_lim", gains_.kiwxy_lim, 0.5);
  param_loader.loadParam("gains/kibxy_lim", gains_.kibxy_lim, 0.5);
  param_loader.loadParam("gains/km", gains_.km, 0.01);
  param_loader.loadParam("gains/km_lim", gains_.km_lim, _uav_mass_ * 0.3);
  param_loader.loadParam("gains/kq_roll_pitch", gains_.kq_roll_pitch, 1.0);
  param_loader.loadParam("gains/kq_yaw", gains_.kq_yaw, 1.0);
  param_loader.loadParam("gains/kw_roll_pitch", gains_.kw_rp, 100.0); 
  param_loader.loadParam("gains/kw_yaw", gains_.kw_y, 1.0);        

  param_loader.loadParam("wall_interaction/desired_contact_force", p_desired_contact_force_, 1.0);
  param_loader.loadParam("wall_interaction/alignment_max_yaw_rate", p_alignment_max_yaw_rate_, 0.3);

  param_loader.loadParam("wall_interaction/alignment_target_roll", p_alignment_target_roll_, 0.0); 
  param_loader.loadParam("wall_interaction/alignment_target_pitch", p_alignment_target_pitch_, 0.0); 
  param_loader.loadParam("wall_interaction/alignment_yaw_rate_damping", p_alignment_yaw_rate_damping_, 0.5);
  param_loader.loadParam("wall_interaction/alignment_yaw_rate_stop_threshold", p_alignment_yaw_rate_stop_threshold_, 0.02);
  param_loader.loadParam("wall_interaction/alignment_min_contact_force", p_alignment_min_contact_force_, 0.5); 
  param_loader.loadParam("wall_interaction/alignment_yaw_rate_low_count_threshold", p_alignment_yaw_rate_low_count_threshold_, 5);

  param_loader.loadParam("wall_interaction/sliding/kp_force", p_sliding_kp_force_, 0.5);
  param_loader.loadParam("wall_interaction/sliding/ki_force", p_sliding_ki_force_, 0.1);
  param_loader.loadParam("wall_interaction/sliding/force_integral_limit", p_sliding_force_integral_limit_, 1.0);
  param_loader.loadParam("wall_interaction/sliding/kp_pos_tangent", p_sliding_kp_pos_tangent_, 1.0);
  param_loader.loadParam("wall_interaction/sliding/kd_pos_tangent", p_sliding_kd_pos_tangent_, 0.5);
  param_loader.loadParam("wall_interaction/sliding/kp_pos_z", p_sliding_kp_pos_z_, 1.0);
  param_loader.loadParam("wall_interaction/sliding/kd_pos_z", p_sliding_kd_pos_z_, 0.5);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all parameters!", name_.c_str());
    return false;
  }

  wall_interaction_state_ = ALIGNING_TO_WALL; 
  has_wall_normal_estimate_ = false;
  force_integral_error_ = 0.0;
  sliding_tangent_inertial_cmd_ = Eigen::Vector3d(1,0,0); 
  alignment_yaw_rate_low_counter_ = 0;
  first_alignment_iteration_ = true;
  first_sliding_iteration_ = true;

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh = nh_;
  shopts.node_name = name_;
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe = true;
  shopts.autostart = true;
  shopts.queue_size = 10;
  shopts.transport_hints = ros::TransportHints().tcpNoDelay();

  // Check if these topic names are available in your PrivateHandlers_t or CommonHandlers_t
  // If not, you might need to load them as parameters or adjust their source.
  sh_ext_force_   = mrs_lib::SubscribeHandler<geometry_msgs::Vector3>(shopts, nh.resolveName("/uav1/external_wrench_estimator/force_components_filt")); 
  sh_imu_      = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, nh.resolveName("/uav1/hw_api/imu"));                     
  sh_odometry_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, nh.resolveName("/uav1/hw_api/odometry"));           
  
  drs_.reset(new Drs_t(mutex_drs_, nh_));
  Drs_t::CallbackType f = boost::bind(&BumpTolerantController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);
  pub_disturbances_ = nh_.advertise<mrs_msgs::ControllerDiagnostics>("disturbances", 1);
  pub_controller_diagnostics_ = nh_.advertise<mrs_msgs::ControllerDiagnostics>("controller_diagnostics", 1);
  timer_diagnostics_ = nh_.createTimer(ros::Duration(0.1), &BumpTolerantController::timerDiagnostics, this);

  service_set_integral_terms_ = nh_.advertiseService("set_integral_terms_in", &BumpTolerantController::callbackSetIntegralTerms, this);

  is_initialized_ = true;
  ROS_INFO("[%s]: Initialized.", name_.c_str());
  return true;
}
//}

/* activate() //{ */
bool BumpTolerantController::activate(const ControlOutput& last_control_output) {
  activation_control_output_ = last_control_output;
  
  if (activation_control_output_.diagnostics.mass_estimator) {
    uav_mass_difference_ = activation_control_output_.diagnostics.mass_difference;
    ROS_INFO("[%s]: Setting mass difference from last controller: %.2f kg", name_.c_str(), uav_mass_difference_);
  } else {
    uav_mass_difference_ = 0.0;
  }

  if (activation_control_output_.diagnostics.disturbance_estimator) {
    std::scoped_lock lock(mutex_integrals_);
    Ib_b_(0) = -activation_control_output_.diagnostics.disturbance_bx_b;
    Ib_b_(1) = -activation_control_output_.diagnostics.disturbance_by_b;
    Iw_w_(0) = -activation_control_output_.diagnostics.disturbance_wx_w;
    Iw_w_(1) = -activation_control_output_.diagnostics.disturbance_wy_w;
    ROS_INFO("[%s]: Setting disturbances from last controller: Ib_b_: [%.2f, %.2f], Iw_w_: [%.2f, %.2f]",
             name_.c_str(), Ib_b_(0), Ib_b_(1), Iw_w_(0), Iw_w_(1));
  } else {
    resetDisturbanceEstimators();
  }

  wall_interaction_state_ = ALIGNING_TO_WALL; 
  first_alignment_iteration_ = true;
  first_sliding_iteration_ = true;
  has_wall_normal_estimate_ = false;
  alignment_yaw_rate_low_counter_ = 0;
  force_integral_error_ = 0.0;

  // Hold pose setup
  holding_pose_ = true;
  hold_start_time_ = ros::Time::now();
  hold_position_ = uav_state_.pose.position;
  hold_orientation_ = uav_state_.pose.orientation;

  is_active_ = true;
  first_iteration_ = true;

  ROS_INFO("[%s]: Activated. Initial state: ALIGNING_TO_WALL.", name_.c_str());
  return true;
}
//}

/* deactivate() //{ */
void BumpTolerantController::deactivate(void) {
  is_active_ = false;
  uav_mass_difference_ = 0.0; 
  ROS_INFO("[%s]: Deactivated.", name_.c_str());
}
//}

/* updateActive() //{ */
BumpTolerantController::ControlOutput BumpTolerantController::updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {
  ROS_DEBUG_THROTTLE(0.5, "[%s]: updateActive() chiamato", name_.c_str());
  uav_state_ = uav_state; 

  double dt = 0;
  if (first_iteration_) {
    last_update_time_ = uav_state.header.stamp;
    first_iteration_ = false;
    last_control_output_ = mrs_uav_managers::Controller::ControlOutput();
    last_control_output_.diagnostics.controller_enforcing_constraints = false;
    return hoveringControlOutput(0.1);
  } else {
    dt = (uav_state.header.stamp - last_update_time_).toSec();
    last_update_time_ = uav_state.header.stamp;
  }

  if (dt <= 0.001 && dt >= -0.001) { 
    ROS_WARN_THROTTLE(1.0, "[%s]: Time difference between updates is too small (%f s). Skipping update.", name_.c_str(), dt);
    return hoveringControlOutput(dt);
  }

  // --- HOLD LOGIC ---
  if (holding_pose_debug_) {
    if ((ros::Time::now() - hold_start_time_).toSec() < 700.0) {
      holdPose(uav_state, dt);
      return last_control_output_;
    } else {
      holding_pose_debug_ = false;
      ROS_INFO("[%s]: Hold finished, starting normal control logic.", name_.c_str());
    }
  }
  // --- END HOLD LOGIC ---

  mrs_uav_managers::control_manager::CONTROL_OUTPUT output_modality = mrs_uav_managers::control_manager::CONTROL_OUTPUT::ATTITUDE; 
  if (tracker_command.use_attitude_rate) { 
    output_modality = mrs_uav_managers::control_manager::CONTROL_OUTPUT::ATTITUDE_RATE;
  }

  BTC(uav_state, tracker_command, dt, output_modality);

  return last_control_output_;
}
//}

/* updateInactive() //{ */
void BumpTolerantController::updateInactive(const mrs_msgs::UavState& uav_state, [[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand>& tracker_command) {
  uav_state_ = uav_state;
  first_iteration_ = true; 
}
//}


/* resetDisturbanceEstimators() //{ */
void BumpTolerantController::resetDisturbanceEstimators(void) {
  std::scoped_lock lock(mutex_integrals_);
  Ib_b_.setZero();
  Iw_w_.setZero();
  uav_mass_difference_ = 0;
  ROS_INFO("[%s]: Disturbance and mass estimators reset.", name_.c_str());
}
//}

/* BTC() //{ */
void BumpTolerantController::BTC(const mrs_msgs::UavState &uav_state,
                                 const mrs_msgs::TrackerCommand &tracker_command,
                                 const double &dt,
                                 [[maybe_unused]] const mrs_uav_managers::control_manager::CONTROL_OUTPUT &output_modality_hint) {

  // 1) CONTROLLI PRELIMINARI (sensori, wrench, ecc.)
  if (!sh_imu_.hasMsg() || !sh_odometry_.hasMsg() || !sh_ext_force_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[%s]: BTC: Mancano dati sensori. Hover.", name_.c_str());
    last_control_output_ = hoveringControlOutput(dt);
    // Non esco subito con return; proseguo fino al blocco finale di diagnostica
    return;
  }
  auto imu_msg        = sh_imu_.getMsg();
  auto odom_msg       = sh_odometry_.getMsg();
  auto ext_wrench_msg = sh_ext_force_.getMsg();

  // 2) CALCOLI “COMUNI” PRIMA DELLE DUE BRANCH (ALIGNING vs SLIDING)
  //    – Leggo la velocità angolare corrente
  Eigen::Vector3d omega_current(
      uav_state.velocity.angular.x,
      uav_state.velocity.angular.y,
      uav_state.velocity.angular.z
  ); // [ωx, ωy, ωz]

  // FEEDFORWARD DI EMERGENZA
  Eigen::Vector3d rate_feedforward;
  rate_feedforward << -omega_current.x(),
                      -omega_current.y(),
                      0.0;

  // ORIENTAMENTO CORRENTE (quaternione targhettato)
  auto q = odom_msg->pose.pose.orientation;
  double current_yaw = 0.0; //DEBUG!!!!!
  try {
    current_yaw = mrs_lib::AttitudeConverter(q).getYaw();
  }
  catch (...) {
    ROS_WARN_THROTTLE(1.0, "[%s]: Errore getYaw(), uso yaw=0.", name_.c_str());
  }

  // MASSA E DISTURBI (come prima)
  Eigen::Vector3d uav_pos_W(uav_state.pose.position.x,
                            uav_state.pose.position.y,
                            uav_state.pose.position.z);
  Eigen::Vector3d uav_vel_W(uav_state.velocity.linear.x,
                            uav_state.velocity.linear.y,
                            uav_state.velocity.linear.z);

  Eigen::Quaterniond q_body_to_world(q.w, q.x, q.y, q.z);
  Eigen::Matrix3d R_B_to_W = q_body_to_world.toRotationMatrix();
  Eigen::Vector3d f_e_hat_body(ext_wrench_msg->x,
                               ext_wrench_msg->y,
                               ext_wrench_msg->z);
  Eigen::Vector3d f_e_hat_inertial = R_B_to_W * f_e_hat_body;

  Eigen::Vector2d f_e_comp_b_xy(f_e_hat_body.x(), f_e_hat_body.y());
  Eigen::Vector2d f_e_comp_w_xy(f_e_hat_inertial.x(), f_e_hat_inertial.y());

  double total_mass = _uav_mass_ + uav_mass_difference_;
  if (gains_.km > 1e-4 && integral_terms_enabled_) {
    double mass_err_signal = -f_e_hat_body.z();
    uav_mass_difference_ += gains_.km * mass_err_signal * dt;
    uav_mass_difference_ = std::clamp(uav_mass_difference_, -gains_.km_lim, gains_.km_lim);
  }
  total_mass = _uav_mass_ + uav_mass_difference_;

  if (integral_terms_enabled_) {
    {
      std::scoped_lock lock(mutex_integrals_);
      Ib_b_ += gains_.kibxy * f_e_comp_b_xy * dt;
      Ib_b_.x() = std::clamp(Ib_b_.x(), -gains_.kibxy_lim, gains_.kibxy_lim);
      Ib_b_.y() = std::clamp(Ib_b_.y(), -gains_.kibxy_lim, gains_.kibxy_lim);
    }
    {
      std::scoped_lock lock(mutex_integrals_);
      Iw_w_ += gains_.kiwxy * f_e_comp_w_xy * dt;
      Iw_w_.x() = std::clamp(Iw_w_.x(), -gains_.kiwxy_lim, gains_.kiwxy_lim);
      Iw_w_.y() = std::clamp(Iw_w_.y(), -gains_.kiwxy_lim, gains_.kiwxy_lim);
    }
  }

  double current_roll        = mrs_lib::AttitudeConverter(q).getRoll();
  double current_pitch       = mrs_lib::AttitudeConverter(q).getPitch();

  double current_imu_yaw_rate = imu_msg->angular_velocity.z;

  Eigen::Vector3d z_body_world = R_B_to_W.col(2);
  double tilt_rad = std::acos(z_body_world.dot(Eigen::Vector3d::UnitZ()));
  double tilt_deg = tilt_rad * 180.0 / M_PI;

  double desired_yaw_rate = (0.2 - current_imu_yaw_rate)*1.0 / (1.0 + std::exp(5 * (tilt_deg - 13)));  // rad/s
  rate_feedforward.z() = desired_yaw_rate;

  rate_feedforward.x() = std::clamp(rate_feedforward.x(), -2.0, 2.0);
  rate_feedforward.y() = std::clamp(rate_feedforward.y(), -2.0, 2.0);
  rate_feedforward.z() = std::clamp(rate_feedforward.z(), -1.0, 1.0);

  ROS_DEBUG_THROTTLE(0.1, "[%s] rate_feedforward = [%.3f, %.3f, %.3f]",
                     name_.c_str(),
                     rate_feedforward.x(),
                     rate_feedforward.y(),
                     rate_feedforward.z());

  // ========== 4) LOGICA “ALIGNING_TO_WALL” ==========
  if (wall_interaction_state_ == ALIGNING_TO_WALL) {

    if (first_alignment_iteration_) {
      ROS_INFO("[%s]: Entering ALIGNING_TO_WALL state.", name_.c_str());
      first_alignment_iteration_ = false;
      alignment_yaw_rate_low_counter_ = 0;
      first_yaw_ = mrs_lib::AttitudeConverter(q).getYaw();
      angle_force_normal_ = std::atan2(f_e_comp_b_xy.y(), f_e_comp_b_xy.x());
      forces_product_ = f_e_comp_b_xy.x() *  f_e_comp_b_xy.y();
      ROS_INFO("[%s][ALIGNING]: forces product: %f.",
                name_.c_str(),
                forces_product_);

    }

    // 4.1) Guadagni e saturazioni come nel MPC originale
    Eigen::Array3d Kq;
    Kq << gains_.kw_rp, gains_.kw_rp, gains_.kw_y;

    Eigen::Vector3d attitude_rate_saturation(
        constraints_.roll_rate,
        constraints_.pitch_rate,
        constraints_.yaw_rate
    );

    // 4.2) Costruisco “attitude_cmd” d’assetto: roll=0, pitch=0, yaw=current_yaw
    mrs_msgs::HwApiAttitudeCmd attitude_cmd;
    mrs_lib::AttitudeConverter ac_cmd(0.0, 0.0, current_yaw);
    attitude_cmd.orientation = ac_cmd;

    // Imposto throttle di “hover” nel modo con cui lo usate altrove:
    attitude_cmd.throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(
        common_handlers_->throttle_model,
        (_uav_mass_ + uav_mass_difference_) * common_handlers_->g
    );

    double desired_roll  = 0.0;
    double desired_pitch = 0.0;
    double desired_yaw   = current_yaw;
    double desired_tilt_rad = 0.0;
    double desired_tilt_deg = 0.0;

    double actual_roll  = current_roll;
    double actual_pitch = current_pitch;
    double actual_yaw   = current_yaw;
    Eigen::Vector3d z_body_world = R_B_to_W.col(2);
    double actual_tilt_rad = std::acos(z_body_world.dot(Eigen::Vector3d::UnitZ()));
    double actual_tilt_deg = actual_tilt_rad * 180.0 / M_PI;

    ROS_INFO_THROTTLE(0.5, "[%s][ALIGNING] Actual/Desired attitude: roll=%.3f/%.3f, pitch=%.3f/%.3f, yaw=%.3f/%.3f, tilt=%.3f/%.3f deg,",
                      name_.c_str(),
                      actual_roll, desired_roll,
                      actual_pitch, desired_pitch,
                      actual_yaw, desired_yaw,
                      actual_tilt_deg, desired_tilt_deg);

    // 4.3) Chiamo attitudeController (livello 1) → ritorna HwApiActuatorCmd con body_rate + throttle
    auto attitude_rate_command = mrs_uav_controllers::common::attitudeController(
        uav_state,
        attitude_cmd,
        rate_feedforward,           // feedforward custom
        attitude_rate_saturation,
        Kq,
        false
    );

    if (!attitude_rate_command) {
      ROS_ERROR_THROTTLE(1.0, "[%s][ALIGNING]: attitudeController fallito, hover.", name_.c_str());
      last_control_output_ = hoveringControlOutput(dt);
      return;
    }

    // 4.4) Assegno direttamente l’intero HwApiActuatorCmd ritornato da attitudeController
    last_control_output_.control_output = attitude_rate_command.value();
    last_control_output_.desired_orientation = ac_cmd;

    // 4.5) Debug: stampo body_rate (x,y,z) e throttle ottenuta
    ROS_INFO_THROTTLE(0.5,
      "[%s][ALIGNING] attitude_rate_command = [wx=%.3f, wy=%.3f, wz=%.3f, throttle=%.3f]",
      name_.c_str(),
      attitude_rate_command->body_rate.x,
      attitude_rate_command->body_rate.y,
      attitude_rate_command->body_rate.z,
      attitude_rate_command->throttle
    );

    // 4.6) Controllo fine allineamento: se yaw_rate corrente basso, cambio stato
    if (std::abs(current_imu_yaw_rate) < p_alignment_yaw_rate_stop_threshold_) {
      alignment_yaw_rate_low_counter_++;
    } else {
      alignment_yaw_rate_low_counter_ = 0;
    }
    //if (alignment_yaw_rate_low_counter_ >= p_alignment_yaw_rate_low_count_threshold_ || current_yaw > 45.0 * M_PI / 180.0) {
    //if (current_yaw > 45.0 * M_PI / 180.0) {
    if (alignment_yaw_rate_low_counter_ >= p_alignment_yaw_rate_low_count_threshold_) {
      ROS_INFO("[%s]: Alignment completato. Passo a SLIDING_ALONG_WALL.", name_.c_str());
      //wall_normal_inertial_estimate_ = -R_B_to_W.col(0);
      has_wall_normal_estimate_ = true;
      last_wall_normal_update_time_ = ros::Time::now();
      wall_interaction_state_ = SLIDING_ALONG_WALL;
      first_sliding_iteration_ = true;
      rotation_delta_ = mrs_lib::AttitudeConverter(q).getYaw() - first_yaw_;
    }
  }
  else if (wall_interaction_state_ == SLIDING_ALONG_WALL) {

    // ——————— LOG “ENTERING SLIDING” ———————
    if (first_sliding_iteration_) {
      first_sliding_iteration_ = false;
      admittance_reference_pos_ = uav_pos_W;
      admittance_reference_vel_ = Eigen::Vector3d::Zero();

      if (rotation_delta_ > 0.0) {
        double rotation_dir = 1.0; // Clockwise
      } else {
        double rotation_dir = -1.0; // Counter-clockwise
      }

      double cos_angle_abs = std::abs(std::cos(angle_force_normal_));
      ROS_INFO_THROTTLE(0.1, "[%s][SLIDING]: EStimated forces : x=%.3f, y=%.3f.",
              name_.c_str(), f_e_comp_b_xy.x(), f_e_comp_b_xy.y());

      if (cos_angle_abs > PARALLEL_THRESHOLD_HIGH) {
        ROS_INFO_THROTTLE(0.1, "[%s][SLIDING]: Wall normal parallel to x-axis.", name_.c_str());
        is_parallel_to_ = "X";
      } else if (cos_angle_abs < PARALLEL_THRESHOLD_LOW) {
        ROS_INFO_THROTTLE(0.1, "[%s][SLIDING]: Wall normal parallel to y-axis.", name_.c_str());
        is_parallel_to_ = "Y";
      } else {
        // Ambiguo: usa rotazione e prodotto delle forze
        bool clockwise = rotation_delta_ > 0.0;
        bool positive_product = forces_product_ > 0.0;
        ROS_INFO_THROTTLE(0.1, "[%s][SLIDING]: Rotation delta: %.3f rad, forces product: %.3f.",
                name_.c_str(), rotation_delta_, forces_product_);

        if ((clockwise && positive_product) || (!clockwise && !positive_product)) {
          ROS_INFO_THROTTLE(0.1, "[%s][SLIDING]: Wall normal parallel to x-axis, %s rotation of %.3f deg.", 
                  name_.c_str(), clockwise ? "clockwise" : "counter-clockwise", rotation_delta_* 180.0 / M_PI);
          is_parallel_to_ = "X";
        } else {
          ROS_INFO_THROTTLE(0.1, "[%s][SLIDING]: Wall normal parallel to y-axis, %s rotation of %.3f deg.", 
                  name_.c_str(), clockwise ? "clockwise" : "counter-clockwise", rotation_delta_* 180.0 / M_PI);
          is_parallel_to_ = "Y";
        }
      }
      last_wall_rotation_time_ = ros::Time::now();
    }

    // 5.0) CALCOLO POSIZIONE PARETE
    m_0 = tan(current_yaw);
    q_0 = uav_pos_W.y() - m_0 * uav_pos_W.x();
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING]: m_0 = %.3f, q_0 = %.3f", name_.c_str(), m_0, q_0);
    const double ROTATION_INTERVAL = 30.0; // secondi
    const double ROTATION_DEG = 30.0;
    const double ROTATION_RAD = ROTATION_DEG * M_PI / 180.0;
    const double ROTATION_BLOCK_TIME = 1.0; // secondi

    /*ros::Time now = ros::Time::now();
    if ((now - last_wall_rotation_time_).toSec() > ROTATION_INTERVAL && !is_rotating_wall_) {
      // Inizia la rotazione
      is_rotating_wall_ = true;
      last_wall_rotation_time_ = now;

      // Ruota tutti i valori di m_data di 30 gradi
      for (size_t i = 0; i < m_data.size(); ++i) {
        double theta = std::atan(m_data[i]);
        theta += ROTATION_RAD;
        m_data[i] = std::tan(theta);
      }
      ROS_INFO("[%s][SLIDING]: Rotated all wall slopes by 30 degrees.", name_.c_str());
    }

    // Blocca l'aggiornamento di m_data/q_data durante la rotazione per ROTATION_BLOCK_TIME secondi
    bool allow_wall_update = true;
    if (is_rotating_wall_) {
      if ((now - last_wall_rotation_time_).toSec() > ROTATION_BLOCK_TIME) {
        is_rotating_wall_ = true; // Fine rotazione, riprendi aggiornamento
      } else {
        allow_wall_update = false;
      }
    }

    // --- AGGIORNAMENTO m_data/q_data SOLO SE CONSENTITO ---
    if (allow_wall_update) {*/
    if (m_data.size() >= max_size) {
        m_data.pop_front();
        q_data.pop_front();
    }
    m_data.push_back(m_0);
    q_data.push_back(q_0);
    int n = m_data.size();
    /*
    Eigen::MatrixXd X(n, 2);
    Eigen::VectorXd Y(n);

    for (int i = 0; i < n; ++i) {
        X(i, 0) = m_data[i]
        X(i, 1) = 1.0;
        Y(i) = q_data[i];
    }*/

    if (n == 1) {
      m_wall = m_data.front();
      q_wall = q_data.front();
    } else {
      //Eigen::Vector2d beta = (X.transpose() * X).ldlt().solve(X.transpose() * Y);
      double m_sum = 0.0, q_sum = 0.0;
      for (int i = 0; i < n; ++i) {
        m_sum += m_data[i];
        q_sum += q_data[i];
      }
      //m_wall = m_sum / n;
      m_wall = 0.95;
      q_wall = q_sum / n;
    }
    //m_wall = 1; 
    Eigen::Vector3d wall_tangent(1.0, m_wall, 0.0);
    wall_tangent.normalize();
    Eigen::Vector3d wall_normal(-m_wall, 1.0, 0.0);
    wall_normal.normalize();
    wall_normal_inertial_estimate_ = wall_normal;
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING]: Wall estimate at y= %.3fx + %.3f", name_.c_str(),m_wall, q_wall);

    double wall_yaw = std::atan2(wall_tangent.y(), wall_tangent.x());

    // 5.1) CALCOLO DIREZIONE TANGENZIALE ALLO SCORRIMENTO
    Eigen::Vector3d world_z_W(0, 0, 1);
    sliding_tangent_inertial_cmd_ = world_z_W.cross(wall_normal_inertial_estimate_);
    if (sliding_tangent_inertial_cmd_.norm() < 0.1) {
      Eigen::Vector3d world_x_W(1, 0, 0);
      sliding_tangent_inertial_cmd_ = (world_x_W - world_x_W.dot(wall_normal_inertial_estimate_) * wall_normal_inertial_estimate_);
    }
    sliding_tangent_inertial_cmd_.normalize();
    if (sliding_tangent_inertial_cmd_.norm() < 0.1) {
      sliding_tangent_inertial_cmd_ = Eigen::Vector3d(1, 0, 0);
      if (std::abs(wall_normal_inertial_estimate_.dot(sliding_tangent_inertial_cmd_)) > 0.9) {
        sliding_tangent_inertial_cmd_ = Eigen::Vector3d(0, 1, 0);
      }
    }
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING]: Sliding tangent cmd: [%.3f, %.3f, %.3f] vs wall tangent: [%.3f, %.3f, %.3f]",
                      name_.c_str(),
                      sliding_tangent_inertial_cmd_.x(), sliding_tangent_inertial_cmd_.y(), sliding_tangent_inertial_cmd_.z(),
                      wall_tangent.x(),                  wall_tangent.y(),                  wall_tangent.z());

    sliding_tangent_inertial_cmd_ = wall_tangent;

    // 5.2) CALCOLO FORZA NORMALE DA APPLICARE
    Eigen::Vector3d additional_force_cmd_W = Eigen::Vector3d::Zero();
    double current_contact_force_component = f_e_hat_inertial.dot(wall_normal_inertial_estimate_); // Proiezione sulla normale
    double force_error_normal = (-p_desired_contact_force_) - current_contact_force_component;
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] Current contact force: %.3f, desired: %.3f, error: %.3f",
                      name_.c_str(),
                      current_contact_force_component,
                      -p_desired_contact_force_,
                      force_error_normal);

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] f_e_hat_inertial: [%.3f, %.3f, %.3f]", name_.c_str(),
                      f_e_hat_inertial.x(), f_e_hat_inertial.y(), f_e_hat_inertial.z());

    // --- Admittance Control Reference Update (solo NORMALE) ---
    const double M = 7.5;
    const double D = 10.0;
    Eigen::Vector3d force_error_vec = wall_normal_inertial_estimate_ * force_error_normal;
    admittance_reference_vel_ += (1.0/M) * (force_error_vec - D * admittance_reference_vel_) * dt;
    admittance_reference_pos_ += admittance_reference_vel_ * dt;




    // 5.3) CALCOLO DELLA POSIZIONE DI SLIDING TODO

    double f_cmd_tangent = 0.0;

    if (ros::Time::now() - last_wall_rotation_time_ > ros::Duration(1.0)) {
      if (first_tangent_iteration_) {
        ROS_INFO("[%s][SLIDING]: Starting sliding along wall.", name_.c_str());
        first_tangent_iteration_ = false;
        tangent_ref_pos = uav_pos_W;
        ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] Initial tangent ref pos: [%.3f, %.3f, %.3f]",
                          name_.c_str(),
                          tangent_ref_pos.x(), tangent_ref_pos.y(), tangent_ref_pos.z());
      }
      double sliding_velocity_magnitude = 0.1; // m/s, velocità di sliding desiderata
      if (is_parallel_to_ == "Y") {
        tangent_ref_vel =   R_B_to_W * Eigen::Vector3d(sliding_velocity_magnitude, 0, 0);
        tangent_ref_pos +=  Eigen::Vector3d(tangent_ref_vel.x(), tangent_ref_vel.y(), 0 )* dt;
      }else if (is_parallel_to_ == "X") {
        tangent_ref_vel =   R_B_to_W * Eigen::Vector3d(0, sliding_velocity_magnitude, 0);
        tangent_ref_pos +=  Eigen::Vector3d(tangent_ref_vel.x(), tangent_ref_vel.y(), 0 ) * dt; 
      } else {
        // Default case, use sliding tangent cmd
        tangent_ref_vel = sliding_tangent_inertial_cmd_;
        tangent_ref_pos = uav_pos_W + sliding_tangent_inertial_cmd_;
      }
      double pos_error_tangent = (tangent_ref_pos - uav_pos_W).dot(sliding_tangent_inertial_cmd_);
      double vel_error_tangent = (tangent_ref_vel - uav_vel_W).dot(sliding_tangent_inertial_cmd_);
      f_cmd_tangent = p_sliding_kp_pos_tangent_ * pos_error_tangent + p_sliding_kd_pos_tangent_ * vel_error_tangent;
      ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] Tangent ref pos: [%.3f, %.3f, %.3f], vel: [%.3f, %.3f, %.3f], pos error: %.3f, vel error: %.3f",
                        name_.c_str(),
                        tangent_ref_pos.x(), tangent_ref_pos.y(), tangent_ref_pos.z(),
                        tangent_ref_vel.x(), tangent_ref_vel.y(), tangent_ref_vel.z(),
                        pos_error_tangent, vel_error_tangent);
                    
      Eigen::Vector3d force_cmd_tangent = sliding_tangent_inertial_cmd_ * f_cmd_tangent;
      ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] f_cmd_tangent: %.3f | pos_error_tangent: %.3f | vel_error_tangent: %.3f | kp: %.3f | kd: %.3f | force_cmd : [%.3f, %.3f, %.3f]",
                        name_.c_str(),
                        f_cmd_tangent,
                        pos_error_tangent,
                        vel_error_tangent,
                        p_sliding_kp_pos_tangent_,
                        p_sliding_kd_pos_tangent_,
                        force_cmd_tangent.x(),
                        force_cmd_tangent.y(),
                        force_cmd_tangent.z());
    }
    // 5.4) SOMMA DELLE FORZE
    double f_cmd_normal = p_sliding_kp_force_ * force_error_normal;
    additional_force_cmd_W += wall_normal_inertial_estimate_ * f_cmd_normal*0.15;
    Eigen::Vector3d x_body_W = R_B_to_W.col(0); // x_body in world frame
    additional_force_cmd_W += x_body_W * f_cmd_tangent;
    //additional_force_cmd_W += sliding_tangent_inertial_cmd_ * f_cmd_tangent;

    // --- Controllo quota invariato ---
    double pos_error_z = tracker_command.position.z - uav_pos_W.z();
    double vel_error_z = tracker_command.velocity.z - uav_vel_W.z();
    double f_cmd_z = p_sliding_kp_pos_z_ * pos_error_z + p_sliding_kd_pos_z_ * vel_error_z;
    additional_force_cmd_W.z() += f_cmd_z;
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING][ALT] z_des: %.3f | z: %.3f | err: %.3f | verr: %.3f | f_cmd_z: %.3f",
                      name_.c_str(),
                      tracker_command.position.z,
                      uav_pos_W.z(),
                      pos_error_z,
                      vel_error_z,
                      f_cmd_z);

    Eigen::Vector3d gravity_comp_W(0, 0, total_mass * common_handlers_->g);
    Eigen::Vector3d feedforward_acc_force_W = total_mass * Eigen::Vector3d(
        tracker_command.acceleration.x,
        tracker_command.acceleration.y,
        tracker_command.acceleration.z);

    Eigen::Vector3d disturbance_comp_W = Eigen::Vector3d::Zero();
    {
      std::scoped_lock lock(mutex_integrals_);
      disturbance_comp_W.head<2>() = Iw_w_;
      Eigen::Matrix3d R_B_to_W_yaw_only = mrs_lib::AttitudeConverter(0, 0, current_yaw);
      disturbance_comp_W += R_B_to_W_yaw_only * Eigen::Vector3d(Ib_b_.x(), Ib_b_.y(), 0);
    }

    Eigen::Vector3d total_force_cmd_W = gravity_comp_W
                                      + additional_force_cmd_W
                                      - disturbance_comp_W;

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] Total force cmd: [%.3f, %.3f, %.3f], gravity comp: [%.3f, %.3f, %.3f], additional force: [%.3f, %.3f, %.3f], disturbance comp: [%.3f, %.3f, %.3f]",
                      name_.c_str(),
                      total_force_cmd_W.x(), total_force_cmd_W.y(), total_force_cmd_W.z(),
                      gravity_comp_W.x(), gravity_comp_W.y(), gravity_comp_W.z(),
                      additional_force_cmd_W.x(), additional_force_cmd_W.y(), additional_force_cmd_W.z(),
                      disturbance_comp_W.x(), disturbance_comp_W.y(), disturbance_comp_W.z());

    // 5.5) Calcola desired attitude per lo sliding
    Eigen::Vector3d z_body_des = total_force_cmd_W.normalized();
    /*
    // Usa la tangente della parete per x_body_des, oppure il wall_yaw
    Eigen::Vector3d x_c(std::cos(wall_yaw), std::sin(wall_yaw), 0); // oppure sliding_tangent_inertial_cmd_
    Eigen::Vector3d y_body_des = z_body_des.cross(x_c).normalized();
    Eigen::Vector3d x_body_des = y_body_des.cross(z_body_des).normalized();*/

    // Allinea x_body_des con la tangente di sliding (direzione di moto desiderata)
    Eigen::Vector3d x_body_des = sliding_tangent_inertial_cmd_.normalized();
    Eigen::Vector3d y_body_des = z_body_des.cross(x_body_des).normalized();
    x_body_des = y_body_des.cross(z_body_des).normalized(); // Ricalcola per ortogonalità

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] x_body_des: [%.3f, %.3f, %.3f], y_body_des: [%.3f, %.3f, %.3f], z_body_des: [%.3f, %.3f, %.3f]",
                      name_.c_str(),
                      x_body_des.x(), x_body_des.y(), x_body_des.z(),
                      y_body_des.x(), y_body_des.y(), y_body_des.z(),
                      z_body_des.x(), z_body_des.y(), z_body_des.z());

    Eigen::Matrix3d R_desired_att_B_to_W;
    R_desired_att_B_to_W.col(0) = x_body_des;
    R_desired_att_B_to_W.col(1) = y_body_des;
    R_desired_att_B_to_W.col(2) = z_body_des;

    mrs_lib::AttitudeConverter desired_attitude_slide(R_desired_att_B_to_W);

    Eigen::Vector3d z_body_des_check = R_desired_att_B_to_W.col(2);
    double desired_tilt_rad = std::acos(z_body_des_check.dot(Eigen::Vector3d(0, 0, 1)));
    double desired_tilt_deg = desired_tilt_rad * 180.0 / M_PI;

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] Actual/Desired attitude: roll=%.3f/%.3f, pitch=%.3f/%.3f, yaw=%.3f/%.3f, tilt=%.3f/%.3f deg,",
                      name_.c_str(),
                      current_roll, desired_attitude_slide.getRoll(),
                      current_pitch, desired_attitude_slide.getPitch(),
                      current_yaw, desired_attitude_slide.getYaw(),
                      tilt_deg, desired_tilt_deg);

    double thrust_mag = total_force_cmd_W.dot(z_body_des);
    //thrust_mag = std::max(thrust_mag, 0.1 * total_mass * common_handlers_->g);

    // 5.6) Costruisco il comando di attitude + thrust per lo sliding

    double desired_yaw_rate_sliding = -1.3; // rad/s, positivo = senso orario
    rate_feedforward.z() = desired_yaw_rate_sliding;

    mrs_msgs::HwApiAttitudeCmd attitude_cmd_slide;
    attitude_cmd_slide.orientation = desired_attitude_slide;
    attitude_cmd_slide.throttle    = mrs_lib::quadratic_throttle_model::forceToThrottle(
        common_handlers_->throttle_model,
        thrust_mag);

    // ASSEGNO SEMPRE qua il control_output per SLIDING
    last_control_output_.control_output = attitude_cmd_slide;
    last_control_output_.desired_orientation = Eigen::Quaterniond(
        attitude_cmd_slide.orientation.w,
        attitude_cmd_slide.orientation.x,
        attitude_cmd_slide.orientation.y,
        attitude_cmd_slide.orientation.z);

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] thrust=%.3f, att_quat=[%.3f, %.3f, %.3f, %.3f]",
                      name_.c_str(),
                      attitude_cmd_slide.throttle,
                      attitude_cmd_slide.orientation.x,
                      attitude_cmd_slide.orientation.y,
                      attitude_cmd_slide.orientation.z,
                      attitude_cmd_slide.orientation.w);
    
    /*
    // 5.7) Se contatto viene perso, torno ad ALIGNING
    if (std::abs(current_contact_force_component) < 0.3 * p_desired_contact_force_ &&
        (ros::Time::now() - last_wall_normal_update_time_).toSec() > 0.5) {
      ROS_INFO("[%s]: Forza contatto persa. Passo a ALIGNING.", name_.c_str());
      wall_interaction_state_ = ALIGNING_TO_WALL;
      first_alignment_iteration_ = true;
      has_wall_normal_estimate_ = false;
    }
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] desired_att_body_x_W: [%.3f, %.3f, %.3f]", name_.c_str(),
        desired_att_body_x_W.x(), desired_att_body_x_W.y(), desired_att_body_x_W.z());
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] desired_att_body_y_W: [%.3f, %.3f, %.3f]", name_.c_str(),
        desired_att_body_y_W.x(), desired_att_body_y_W.y(), desired_att_body_y_W.z());
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] desired_att_body_z_W: [%.3f, %.3f, %.3f]", name_.c_str(),
        desired_att_body_z_W.x(), desired_att_body_z_W.y(), desired_att_body_z_W.z());
    */

    double det = R_desired_att_B_to_W.determinant();
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] Determinant of R_desired_att_B_to_W: %.3f", name_.c_str(), det);

    if (det < 0.0) {
      ROS_WARN_THROTTLE(0.1, "[%s][SLIDING] WARNING: Desired attitude matrix is left-handed (det < 0)!", name_.c_str());
    }
  }
  else {
    // Qualunque stato sconosciuto → forzare hover
    ROS_ERROR_THROTTLE(1.0, "[%s]: Stato wall_interaction sconosciuto! Hover.", name_.c_str());
    last_control_output_ = hoveringControlOutput(dt);
    wall_interaction_state_ = ALIGNING_TO_WALL;
    first_alignment_iteration_ = true;
  }

  // ——————— BLOCCO COMUNE DI DIAGNOSTICA ———————
  last_control_output_.diagnostics.controller          = name_
      + (wall_interaction_state_ == ALIGNING_TO_WALL ? "[ALIGNING]" : "[SLIDING]");
  last_control_output_.diagnostics.mass_estimator      = true;
  last_control_output_.diagnostics.mass_difference     = uav_mass_difference_;
  last_control_output_.diagnostics.total_mass          = total_mass;
  last_control_output_.diagnostics.disturbance_estimator = true;
  {
    std::scoped_lock lock(mutex_integrals_);
    last_control_output_.diagnostics.disturbance_bx_b = -Ib_b_.x();
    last_control_output_.diagnostics.disturbance_by_b = -Ib_b_.y();
    last_control_output_.diagnostics.disturbance_wx_w = -Iw_w_.x();
    last_control_output_.diagnostics.disturbance_wy_w = -Iw_w_.y();
  }

  controller_diagnostics_.mass_estimator   = true;
  controller_diagnostics_.mass_difference  = uav_mass_difference_;
  controller_diagnostics_.total_mass       = total_mass;

  // FINE BTC()
  /*pub_controller_diagnostics_.publish(last_control_output_.diagnostics);
  ROS_INFO("[%s]: Body dusturbances: bx=%.3f, by=%.3f, wx=%.3f, wy=%.3f",
           name_.c_str(),
           last_control_output_.diagnostics.disturbance_bx_b,
           last_control_output_.diagnostics.disturbance_by_b,
           last_control_output_.diagnostics.disturbance_wx_w,
           last_control_output_.diagnostics.disturbance_wy_w);*/
}

//}

/* getStatus() //{ */
const mrs_msgs::ControllerStatus BumpTolerantController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}
//}

/* switchOdometrySource() //{ */
void BumpTolerantController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState& new_uav_state) {
  ROS_INFO("[%s]: Odometry source switched. Controller state maintained.", name_.c_str());
}
//}

/* setConstraints() //{ */
const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr BumpTolerantController::setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& cmd) {
  {
    std::scoped_lock lock(mutex_constraints_);
    constraints_ = cmd->constraints;
  }
  ROS_INFO("[%s]: Constraints set.", name_.c_str());
  
  mrs_msgs::DynamicsConstraintsSrvResponse::Ptr res;
  res.reset(new mrs_msgs::DynamicsConstraintsSrvResponse());
  res->success = true;
  res->message = "Constraints set successfully"; // This 'message' field is part of the service response, not ControllerStatus
  return res;
}
//}

/* callbackSetIntegralTerms //{ */
bool BumpTolerantController::callbackSetIntegralTerms(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  integral_terms_enabled_ = req.data;
  res.success = true;
  res.message = integral_terms_enabled_ ? "Integral terms enabled." : "Integral terms disabled.";
  ROS_INFO("[%s]: %s", name_.c_str(), res.message.c_str());
  if (!integral_terms_enabled_) {
    resetDisturbanceEstimators(); 
  }
  return true;
}
//}

/* callbackDrs() //{ */
void BumpTolerantController::callbackDrs(DrsConfig_t &config, [[maybe_unused]] uint32_t level) {
  std::scoped_lock lock(mutex_gains_); 
  drs_params_ = config; 

  gains_.kiwxy = drs_params_.kiwxy;
  gains_.kibxy = drs_params_.kibxy;
  gains_.kiwxy_lim = drs_params_.kiwxy_lim;
  gains_.kibxy_lim = drs_params_.kibxy_lim;
  gains_.km = drs_params_.km;
  gains_.km_lim = drs_params_.km_lim;
  gains_.kw_rp = drs_params_.kw_rp; 
  gains_.kw_y = drs_params_.kw_y; 

  p_desired_contact_force_ = drs_params_.p_desired_contact_force;
  p_alignment_max_yaw_rate_ = drs_params_.p_alignment_max_yaw_rate;
  p_alignment_target_roll_ = drs_params_.p_alignment_target_roll;
  p_alignment_target_pitch_ = drs_params_.p_alignment_target_pitch;
  p_alignment_yaw_rate_damping_ = drs_params_.p_alignment_yaw_rate_damping;
  p_alignment_yaw_rate_stop_threshold_ = drs_params_.p_alignment_yaw_rate_stop_threshold;
  p_alignment_min_contact_force_ = drs_params_.p_alignment_min_contact_force;
  p_alignment_yaw_rate_low_count_threshold_ = drs_params_.p_alignment_yaw_rate_low_count_threshold;

  p_sliding_kp_force_ = drs_params_.p_sliding_kp_force;
  p_sliding_ki_force_ = drs_params_.p_sliding_ki_force;
  p_sliding_force_integral_limit_ = drs_params_.p_sliding_force_integral_limit;
  p_sliding_kp_pos_tangent_ = drs_params_.p_sliding_kp_pos_tangent;
  p_sliding_kd_pos_tangent_ = drs_params_.p_sliding_kd_pos_tangent;
  p_sliding_kp_pos_z_ = drs_params_.p_sliding_kp_pos_z;
  p_sliding_kd_pos_z_ = drs_params_.p_sliding_kd_pos_z;

  ROS_INFO("[%s]: Dynamic reconfigure callback processed. Level %u", name_.c_str(), level);
}
//}

/* timerGains() //{ */
void BumpTolerantController::timerGains([[maybe_unused]] const ros::TimerEvent &event) {
  // Currently, gains are applied directly in callbackDrs.
}
//}

/* timerDiagnostics() //{ */
void BumpTolerantController::timerDiagnostics([[maybe_unused]] const ros::TimerEvent& event) {
    if (is_active_ && pub_controller_diagnostics_.getNumSubscribers() > 0) { 
        pub_controller_diagnostics_.publish(controller_diagnostics_);
    }
}


/* hoveringControlOutput() //{ */
BumpTolerantController::ControlOutput BumpTolerantController::hoveringControlOutput([[maybe_unused]] const double dt) { // Matched signature
  ControlOutput output;
  output.control_output = mrs_msgs::HwApiAttitudeCmd(); 
  ROS_WARN_THROTTLE(1.0, "[%s]: PUB: hoveringControlOutput (failsafe/errore dati)", name_.c_str());
  
  double desired_yaw = 0;
  if (sh_odometry_.hasMsg()){ 
    mrs_lib::AttitudeConverter ac(sh_odometry_.getMsg()->pose.pose.orientation);
    desired_yaw = ac.getYaw();
  } else if (is_initialized_ && (uav_state_.pose.orientation.w != 0 || uav_state_.pose.orientation.x != 0 || uav_state_.pose.orientation.y != 0 || uav_state_.pose.orientation.z != 0) ) { 
     mrs_lib::AttitudeConverter ac(uav_state_.pose.orientation);
     desired_yaw = ac.getYaw();
  }

  mrs_msgs::HwApiAttitudeCmd attitude_cmd;
  attitude_cmd.orientation = mrs_lib::AttitudeConverter(0, 0, desired_yaw); 
  attitude_cmd.throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model,(_uav_mass_ + uav_mass_difference_) * common_handlers_->g);
  
  output.control_output = attitude_cmd;
  output.diagnostics.controller = name_ + "[HOVERING]";
  output.diagnostics.mass_estimator = true;
  output.diagnostics.mass_difference = uav_mass_difference_;
  output.diagnostics.total_mass = _uav_mass_ + uav_mass_difference_;
  output.diagnostics.disturbance_estimator = true; 
  {
    std::scoped_lock lock(mutex_integrals_);
    output.diagnostics.disturbance_bx_b = -Ib_b_(0); 
    output.diagnostics.disturbance_by_b = -Ib_b_(1);
    output.diagnostics.disturbance_wx_w = -Iw_w_(0);
    output.diagnostics.disturbance_wy_w = -Iw_w_(1);
  }
  return output;
}
//}

void BumpTolerantController::holdPose(const mrs_msgs::UavState& uav_state, double dt) {
  const double kp_xy = 60.0;
  const double kd_xy = 60.0;
  const double kp_z  = 60.0;
  const double kd_z  = 60.0;

  Eigen::Vector3d pos(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);
  Eigen::Vector3d vel(uav_state.velocity.linear.x, uav_state.velocity.linear.y, uav_state.velocity.linear.z);

  Eigen::Vector3d pos_ref(hold_position_.x+1, hold_position_.y-1, hold_position_.z);
  Eigen::Vector3d vel_ref(0, 0, 0);

  Eigen::Vector3d pos_error = pos_ref - pos;
  Eigen::Vector3d vel_error = vel_ref - vel;

  Eigen::Vector3d hold_integral_error_ = Eigen::Vector3d::Zero();
  double hold_integral_limit_ = 1.0;

  double total_mass = _uav_mass_ + uav_mass_difference_;
  Eigen::Vector3d force_sp;
  force_sp.x() = kp_xy * pos_error.x() + kd_xy * vel_error.x();
  force_sp.y() = kp_xy * pos_error.y() + kd_xy * vel_error.y();
  force_sp.z() = kp_z  * pos_error.z() + kd_z  * vel_error.z();
  force_sp.z() += total_mass * common_handlers_->g;

  hold_integral_error_ += pos_error * dt;
  hold_integral_error_.x() = std::max(std::min(hold_integral_error_.x(), hold_integral_limit_), -hold_integral_limit_);
  hold_integral_error_.y() = std::max(std::min(hold_integral_error_.y(), hold_integral_limit_), -hold_integral_limit_);
  hold_integral_error_.z() = std::max(std::min(hold_integral_error_.z(), hold_integral_limit_), -hold_integral_limit_);

  const double ki_xy = 10.0; // Tune this!
  const double ki_z  = 10.0; // Tune this!

  force_sp.x() += ki_xy * hold_integral_error_.x();
  force_sp.y() += ki_xy * hold_integral_error_.y();
  force_sp.z() += ki_z  * hold_integral_error_.z();

  // Yaw desiderato
  mrs_lib::AttitudeConverter ac(hold_orientation_);
  double hold_yaw = ac.getYaw();

  // Direzione desiderata dell'asse z del body (verso la forza)
  Eigen::Vector3d z_body_des = force_sp.normalized();

  // Asse x del body desiderato (per mantenere il yaw)
  Eigen::Vector3d x_c(std::cos(hold_yaw), std::sin(hold_yaw), 0);
  Eigen::Vector3d y_body_des = z_body_des.cross(x_c).normalized();
  Eigen::Vector3d x_body_des = y_body_des.cross(z_body_des).normalized();

  Eigen::Matrix3d R;
  R.col(0) = x_body_des;
  R.col(1) = y_body_des;
  R.col(2) = z_body_des;

  mrs_lib::AttitudeConverter att_des(R);
  mrs_msgs::HwApiAttitudeCmd attitude_cmd;
  attitude_cmd.orientation = att_des;
  attitude_cmd.throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(
      common_handlers_->throttle_model, force_sp.norm());

  const auto& q = attitude_cmd.orientation;

  last_control_output_.control_output = attitude_cmd;
  last_control_output_.desired_orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(0,0,0);
  last_control_output_.desired_heading_rate = 0;
  last_control_output_.diagnostics.controller = "BumpTolerantController[HOLD]";
  last_control_output_.diagnostics.total_mass = total_mass;

  ROS_INFO_THROTTLE(0.05, "[%s]: Holding pose (%.2f/5.0 s) [pos err: %.2f %.2f %.2f]", name_.c_str(), (ros::Time::now() - hold_start_time_).toSec(), pos_error.x(), pos_error.y(), pos_error.z());
}

}  // namespace bump_tolerant_controller

}  // namespace bump_tolerant_controller_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bump_tolerant_controller_plugin::bump_tolerant_controller::BumpTolerantController, mrs_uav_managers::Controller)