/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <pid.hpp>
#include <common.h>
#include <mpc_controller.h>
#include <mrs_uav_managers/control_manager/common_handlers.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

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
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/SetBool.h> 

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h> 
#include <geometry_msgs/Vector3.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>
#include <mrs_msgs/String.h>

#include <angles/angles.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


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

/* //{ class BumpTolerantController */

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

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                      mutex_drs_;
  typedef bump_tolerant_controller_plugin::bump_tolerant_controllerConfig DrsConfig_t; 
  typedef dynamic_reconfigure::Server<DrsConfig_t>            Drs_t;
  boost::shared_ptr<Drs_t>                                    drs_;
  void                                                        callbackDrs(DrsConfig_t &config, uint32_t level);
  DrsConfig_t                                                 drs_params_;

  // | ----------------------- controllers ---------------------- |

  void BTC(const mrs_msgs::UavState &uav_state, const mrs_msgs::TrackerCommand &tracker_command, const double &dt, const mrs_uav_managers::control_manager::CONTROL_OUTPUT &output_modality);

  ControlOutput hoveringControlOutput(const double dt); 

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;
  
  // | -------- throttle generation and mass estimation --------- |

  double _uav_mass_; 
  double uav_mass_difference_;

  Gains_t gains_;

  // | ------------------- configurable gains ------------------- |
  
  std::mutex  mutex_integrals_; 
  std::mutex  mutex_gains_; 

  ros::Timer timer_gains_; 
  void       timerGains(const ros::TimerEvent &event);

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;

  // | ----------------- integral terms enabler ----------------- |

  ros::ServiceServer service_set_integral_terms_;
  bool               callbackSetIntegralTerms(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool               integral_terms_enabled_ = true;

  // | ------------------- subscriber handlers ------------------ |

  mrs_lib::SubscribeHandler<geometry_msgs::Vector3>      sh_ext_force_;
  mrs_lib::SubscribeHandler<geometry_msgs::Vector3>      sh_ext_torque_;
  mrs_lib::SubscribeHandler<geometry_msgs::PointStamped> sh_contact_point;
  mrs_lib::SubscribeHandler<sensor_msgs::Imu>            sh_imu_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>          sh_odometry_;

  // | -------------------- publisher handlers ------------------ |
  
  ros::Publisher pub_controller_diagnostics_;
  ros::Publisher pub_disturbances_;
  ros::Publisher pub_sliding_phase_;
  ros::Publisher pub_desired_throttle_;
  ros::Publisher pub_switch_command_;
  ros::Publisher pub_contact_cloud;
  ros::Publisher pub_1_;
  ros::Publisher pub_2_;
  ros::Publisher pub_3_;
  ros::Publisher pub_4_;
  ros::Publisher pub_5_;
  ros::Publisher pub_6_;
  ros::Publisher pub_7_;
  ros::Publisher pub_8_;
  ros::Publisher pub_9_;
  ros::Publisher pub_10_;
  ros::Publisher pub_11_;
  ros::Timer     timer_diagnostics_;

  // | ----------------------- diagnostics ---------------------- |

  void                            timerDiagnostics(const ros::TimerEvent& event);
  mrs_msgs::ControllerDiagnostics controller_diagnostics_; 

  // | ------------------------ uav specs ----------------------- |

  double radius_;
  double initial_yaw_;

  // | ----------------------------- BTC ------------------------ |
  
  // Main controller state
  enum WallInteractionState {
    ALIGNING_TO_WALL,
    SLIDING_SQUARE
  };

  // Alignment state
  enum AlignmentPhase {
    STABILIZE_ATTITUDE,
    MOVE_AWAY,
    ROTATE_TO_PARALLEL
  };

  //Actual state
  WallInteractionState wall_interaction_state_;
  AlignmentPhase       alignment_phase_ = STABILIZE_ATTITUDE;

  // Start time
  ros::Time hold_start_time_;

  // | ----------------------- structures ----------------------- |

  // Yaw data collector
  struct YawDataPoint {
    double    yaw_value;
    double    weight;
    ros::Time timestamp;
  };

  std::deque<YawDataPoint> yaw_data_points_;
  const  size_t            max_yaw_points_              = 200;
  double                   accumulated_movement_weight_ = 0.0;

  // Wall contact point
  struct WallPoint {
    Eigen::Vector3d position;
    double          weight;
    double          q;
  };
  std::vector<WallPoint> wall_points;
  const size_t           max_wall_points = 200;
  ros::Time              last_wall_point_update_time_ = ros::Time::now();

  // | ---------------- external wrench estimation -------------- |

  double forces_product_;

  // | ------------------- alignment detection ------------------ |

  bool         first_alignment_iteration_      = true;
  int          alignment_yaw_rate_low_counter_ = 0;
  ros::Time    time_limit_safety_position      = ros::Time::now();

  Eigen::Vector3d recovery_position_;
  Eigen::Vector3d direction_to_contact          = Eigen::Vector3d::Zero();
  double          attitude_stability_threshold_ = 0.01; // rad, ~3 degrees
  double          target_yaw;
  int             attitude_stable_counter_      = 0;
  double          safety_distance_              = 0.3;
  int             attitude_stable_threshold_    = 10; // iterations
  int             stuck_counter_                = 0;
  bool            first_rotation_iteration_     = true;

  double desired_altitude_;

  // | --------------------- wall estimation -------------------- |

  // Wall normal estimation
  Eigen::Vector3d wall_normal_inertial_estimate_;

  double m_wall;
  double q_wall;
  double wall_yaw;

  bool orthogonal_to_x = false;
  bool orthogonal_to_y = false;

  // | ----------------- interaction parameters ----------------- |
  double p_desired_contact_force_;      
  double p_alignment_max_yaw_rate_;  
  
  Eigen::Vector3d impedance_vel_error_B_;
  
  Eigen::Vector3d normal_ref_vel_W_= Eigen::Vector3d::Zero();

  double contact_velocity_magnitude;

  double wall_yaw_weighted_sum = 0.0;
  double wall_yaw_total_weight = 0.0;

  Eigen::Matrix3d R_OrientAdjust = Eigen::Matrix3d::Identity();

  double desired_yaw_ = 0.0;

  // | --------------------- sliding control -------------------- |

  double p_sliding_kp_force_;

  double p_sliding_kp_pos_tangent_;
  double p_sliding_kd_pos_tangent_;

  double p_sliding_kp_pos_z_;
  double p_sliding_kd_pos_z_;

  Eigen::Vector3d sliding_tangent_inertial_cmd_;

  
  bool first_sliding_iteration_ = true;
  int sliding_direction_ = 1; // 1 for forward, -1 for backward

  Eigen::Vector3d initial_position_;
  ros::Time sliding_start_time;

  Eigen::Vector3d tangent_ref_pos   = Eigen::Vector3d::Zero();
  Eigen::Vector3d tangent_ref_pos_B = Eigen::Vector3d::Zero();
  Eigen::Vector3d tangent_ref_vel;
  double pos_error_tangent          = 0.0;

  std::deque<double> slidig_impedance_data_;
  const size_t max_size_sid = 150;
  bool ready_to_sliding_    = false;

  // | --------------------- hold pose param -------------------- |

  bool holding_pose_        = false;
  bool holding_pose_debug_  = false;
  geometry_msgs::Point      hold_position_;
  geometry_msgs::Quaternion hold_orientation_;

  // | ------------------------ integrals ----------------------- |

  Eigen::Vector2d Iw_w_;  
  Eigen::Vector2d Ib_b_;

  // | --------------------- mooving on wall -------------------- |

  Eigen::Vector2d xy_ref;

  Eigen::Vector3d contact_point_W = Eigen::Vector3d::Zero();

  bool mooving_on_wall_ = false;
  bool x_target = false;

  Eigen::Vector2d xy_0_wall = Eigen::Vector2d::Zero();
  Eigen::Vector2d xy_ref_wall;
  
  
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
  _uav_mass_ = common_handlers_->getMass(); 
  
  // | ------------ loading params using ParamLoader ------------ |

  mrs_lib::ParamLoader param_loader(nh_, name_);

  private_handlers->param_loader->addYamlFile(ros::package::getPath("bump_tolerant_control") + "/config/bump_tolerant_controller.yaml");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all parameters!", name_.c_str());
    return false;
  }

  param_loader.loadParam("uav_radius", radius_, 0.5);

  param_loader.loadParam("gains/kiwxy", gains_.kiwxy, 0.0);
  param_loader.loadParam("gains/kibxy", gains_.kibxy, 0.0);
  param_loader.loadParam("gains/kiwxy_lim", gains_.kiwxy_lim, 0.5);
  param_loader.loadParam("gains/kibxy_lim", gains_.kibxy_lim, 0.5);
  param_loader.loadParam("gains/km", gains_.km, 0.01);
  param_loader.loadParam("gains/km_lim", gains_.km_lim, _uav_mass_ * 0.3);
  param_loader.loadParam("gains/kq_roll_pitch", gains_.kq_roll_pitch, 5.0);
  param_loader.loadParam("gains/kq_yaw", gains_.kq_yaw, 1.0);
  param_loader.loadParam("gains/kw_roll_pitch", gains_.kw_rp, 4.0); 
  param_loader.loadParam("gains/kw_yaw", gains_.kw_y, 4.0);        

  param_loader.loadParam("wall_interaction/desired_contact_force", p_desired_contact_force_, 1.0);
  param_loader.loadParam("wall_interaction/alignment_max_yaw_rate", p_alignment_max_yaw_rate_, 0.3);

  param_loader.loadParam("wall_interaction/sliding/kp_force", p_sliding_kp_force_, 0.5);
  param_loader.loadParam("wall_interaction/sliding/kp_pos_tangent", p_sliding_kp_pos_tangent_, 5.0);
  param_loader.loadParam("wall_interaction/sliding/kd_pos_tangent", p_sliding_kd_pos_tangent_, 2.5);
  param_loader.loadParam("wall_interaction/sliding/kp_pos_z", p_sliding_kp_pos_z_, 1.0);
  param_loader.loadParam("wall_interaction/sliding/kd_pos_z", p_sliding_kd_pos_z_, 0.5);

  // | ------------------- initial parameters ------------------- |

  wall_interaction_state_ = ALIGNING_TO_WALL;
  alignment_yaw_rate_low_counter_ = 0;
  first_alignment_iteration_ = true;
  first_sliding_iteration_ = true;

  // | ------------------- subscriber handler ------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh = nh_;
  shopts.node_name = name_;
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe = true;
  shopts.autostart = true;
  shopts.queue_size = 10;
  shopts.transport_hints = ros::TransportHints().tcpNoDelay();

  // | ------------------- topic subscription ------------------- |

  sh_ext_force_    = mrs_lib::SubscribeHandler<geometry_msgs::Vector3>(shopts, nh.resolveName("/uav1/external_wrench_estimator/force_components_filt"));
  sh_ext_torque_   = mrs_lib::SubscribeHandler<geometry_msgs::Vector3>(shopts, nh.resolveName("/uav1/external_wrench_estimator/moment_components_filt"));
  sh_contact_point = mrs_lib::SubscribeHandler<geometry_msgs::PointStamped>(shopts, nh.resolveName("/uav1/external_wrench_estimator/contact_point"));
  sh_imu_          = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, nh.resolveName("/uav1/hw_api/imu"));                     
  sh_odometry_     = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, nh.resolveName("/uav1/hw_api/odometry"));

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  Drs_t::CallbackType f = boost::bind(&BumpTolerantController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | --------------------- publisher init --------------------- |

  pub_disturbances_           = nh_.advertise<mrs_msgs::ControllerDiagnostics>("disturbances", 1);
  pub_controller_diagnostics_ = nh_.advertise<mrs_msgs::ControllerDiagnostics>("controller_diagnostics", 1);
  pub_sliding_phase_          = nh_.advertise<std_msgs::Float64>("sliding_phase", 1);
  pub_desired_throttle_       = nh_.advertise<std_msgs::Float64>("desired_throttle", 1);
  pub_switch_command_         = nh_.advertise<std_msgs::Bool>("switch_controller_command", 1);
  pub_contact_cloud           = nh_.advertise<sensor_msgs::PointCloud2>("contact_points_cloud", 1);
  pub_1_                      = nh_.advertise<geometry_msgs::PointStamped>("move_away", 1);
  pub_2_                      = nh_.advertise<std_msgs::Float64>("pub_2", 1);
  pub_3_                      = nh_.advertise<std_msgs::Float64>("pub_3", 1);
  pub_4_                      = nh_.advertise<std_msgs::Float64>("pub_4", 1);
  pub_5_                      = nh_.advertise<geometry_msgs::PointStamped>("pub_5", 1);
  pub_6_                      = nh_.advertise<geometry_msgs::PointStamped>("pub_6", 1);
  pub_7_                      = nh_.advertise<geometry_msgs::PointStamped>("pub_7", 1);
  pub_8_                      = nh_.advertise<geometry_msgs::PointStamped>("pub_8", 1);
  pub_9_                      = nh_.advertise<geometry_msgs::PointStamped>("pub_9", 1);
  pub_10_                     = nh_.advertise<std_msgs::Float64>("pub_10", 1);
  pub_11_                     = nh_.advertise<geometry_msgs::PointStamped>("projected_point", 1);

  // | ------------------------- timers ------------------------- |

  timer_diagnostics_ = nh_.createTimer(ros::Duration(0.1), &BumpTolerantController::timerDiagnostics, this);

  // | --------------------- service servers -------------------- |

  service_set_integral_terms_ = nh_.advertiseService("set_integral_terms_in", &BumpTolerantController::callbackSetIntegralTerms, this);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: Initialized.", name_.c_str());
  
  is_initialized_ = true;

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

  // Resetting all internal states
  wall_interaction_state_         = ALIGNING_TO_WALL; 
  alignment_phase_                = STABILIZE_ATTITUDE;
  first_alignment_iteration_      = true;
  first_sliding_iteration_        = true;
  is_active_                      = true;
  first_iteration_                = true;
  alignment_yaw_rate_low_counter_ = 0;

  // Hold pose setup
  holding_pose_     = true;
  hold_start_time_  = ros::Time::now();
  hold_position_    = uav_state_.pose.position;
  hold_orientation_ = uav_state_.pose.orientation;
  
  ROS_INFO("[%s]: Activated. Initial state: ALIGNING_TO_WALL.", name_.c_str());
  return true;
}
//}

/* deactivate() //{ */
void BumpTolerantController::deactivate(void) {
  is_active_           = false;
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
    last_update_time_    = uav_state.header.stamp;
    first_iteration_     = false;
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

  // DEBUG
  if (holding_pose_debug_) {
    if ((ros::Time::now() - hold_start_time_).toSec() < 700.0) {
      holdPose(uav_state, dt);
      return last_control_output_;
    } else {
      holding_pose_debug_ = false;
      ROS_INFO("[%s]: Hold finished, starting normal control logic.", name_.c_str());
    }
  }

  mrs_uav_managers::control_manager::CONTROL_OUTPUT output_modality = mrs_uav_managers::control_manager::CONTROL_OUTPUT::ATTITUDE; 
  //output_modality = mrs_uav_managers::control_manager::CONTROL_OUTPUT::ATTITUDE_RATE;

  BTC(uav_state, tracker_command, dt, output_modality);

  return last_control_output_;
}
//}

/* updateInactive() //{ */
void BumpTolerantController::updateInactive(const mrs_msgs::UavState& uav_state, [[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand>& tracker_command) {
  uav_state_       = uav_state;
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

  // 1) INITIAL CHECKS
  if (!sh_imu_.hasMsg() || !sh_odometry_.hasMsg() || !sh_ext_force_.hasMsg()) {
    ROS_WARN("[%s]: BTC: Mancano dati sensori. Hover.", name_.c_str());
    last_control_output_ = hoveringControlOutput(dt);
    return;
  }
  auto imu_msg        = sh_imu_.getMsg();
  auto odom_msg       = sh_odometry_.getMsg();
  auto ext_wrench_msg = sh_ext_force_.getMsg();

  // 2) CONTROL LOGIC
  
  Eigen::Vector3d omega_current(
      uav_state.velocity.angular.x,
      uav_state.velocity.angular.y,
      uav_state.velocity.angular.z
  ); // [ωx, ωy, ωz]

  auto q = odom_msg->pose.pose.orientation;

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

  Eigen::Vector3d uav_vel_B = R_B_to_W.transpose() * uav_vel_W;

  Eigen::Vector3d f_e_hat_inertial = R_B_to_W * f_e_hat_body;

  Eigen::Vector2d f_e_comp_b_xy(f_e_hat_body.x(), f_e_hat_body.y());
  Eigen::Vector2d f_e_comp_w_xy(f_e_hat_inertial.x(), f_e_hat_inertial.y());

  double current_roll   = mrs_lib::AttitudeConverter(q).getRoll();
  double current_pitch  = mrs_lib::AttitudeConverter(q).getPitch();
  double current_yaw    = mrs_lib::AttitudeConverter(q).getYaw();

  double current_imu_yaw_rate = imu_msg->angular_velocity.z;

  Eigen::Vector3d z_body_world = R_B_to_W.col(2);
  double tilt_rad = std::acos(z_body_world.dot(Eigen::Vector3d::UnitZ()));
  double tilt_deg = tilt_rad * 180.0 / M_PI;

  double total_mass = _uav_mass_ + uav_mass_difference_;

  if (gains_.km > 1e-4 && integral_terms_enabled_) {
    double mass_err_signal = -f_e_hat_body.z();
    uav_mass_difference_ += gains_.km * mass_err_signal * dt;
    uav_mass_difference_  = std::clamp(uav_mass_difference_, -gains_.km_lim, gains_.km_lim);
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

  Eigen::Vector3d disturbance_comp_W = Eigen::Vector3d::Zero();
    {
      std::scoped_lock lock(mutex_integrals_);
      disturbance_comp_W.head<2>() = Iw_w_;
      Eigen::Matrix3d R_B_to_W_yaw_only = mrs_lib::AttitudeConverter(0, 0, current_yaw);
      disturbance_comp_W += R_B_to_W_yaw_only * Eigen::Vector3d(Ib_b_.x(), Ib_b_.y(), 0);
    }

  // 3) Alignment to wall logic
  if (wall_interaction_state_ == ALIGNING_TO_WALL) {

    std_msgs::Float64 msg;
    msg.data = 0.0;
    pub_sliding_phase_.publish(msg);

    auto contact_point_msg = sh_contact_point.getMsg();

    if (first_alignment_iteration_ && contact_point_msg) {

      first_alignment_iteration_ = false;
      double x_projection = contact_point_msg->point.x;
      double y_projection = contact_point_msg->point.y;
      Eigen::Vector3d contact_point_B(x_projection, y_projection, uav_pos_W.z());
      if (contact_point_B.x() > 0.0) {
        contact_point_B = - contact_point_B;
      }
      contact_point_W = R_B_to_W * contact_point_B + uav_pos_W;
      contact_point_W.z() = uav_pos_W.z();

      // Contact direction from drone to contact point
      direction_to_contact = contact_point_W - uav_pos_W;
      direction_to_contact.z() = 0;
      direction_to_contact.normalize();

      recovery_position_ = uav_pos_W - direction_to_contact * safety_distance_; // 30cm
      recovery_position_.z() = uav_pos_W.z();

      alignment_phase_ = STABILIZE_ATTITUDE;
    
      Eigen::Vector2d n_xy = (contact_point_W.head<2>() - uav_pos_W.head<2>()).normalized();
      m_wall = -n_xy.x() / n_xy.y();
      q_wall = contact_point_W.y() - m_wall * contact_point_W.x();

      alignment_yaw_rate_low_counter_ = 0;
      forces_product_ = f_e_comp_b_xy.x() *  f_e_comp_b_xy.y();
      
      desired_altitude_ = uav_pos_W.z();

      ROS_INFO("[%s][ALIGNING]: forces product: %f, fx_b_: %f, fy_b_: %f",
                name_.c_str(),
                forces_product_,f_e_comp_b_xy.x(), f_e_comp_b_xy.y());
      
      ROS_INFO("[%s]: Contact point received: x=%.2f, y=%.2f, recovery position set to [%.2f, %.2f, %.2f]",
                name_.c_str(), x_projection, y_projection,
                recovery_position_.x(), recovery_position_.y(), recovery_position_.z());

      ROS_INFO("[%s]: Entering ALIGNING_TO_WALL state.", name_.c_str());

    } else if (first_alignment_iteration_ && !contact_point_msg) {

      ROS_WARN("[%s]: No contact point received, using default values.", name_.c_str());
      last_control_output_ = hoveringControlOutput(dt);
      return;
      
    }

    Eigen::Array3d Kq;
    Kq << gains_.kq_roll_pitch, gains_.kq_roll_pitch, gains_.kq_yaw;
    
    Eigen::Vector3d attitude_rate_saturation(
        constraints_.roll_rate,
        constraints_.pitch_rate,
        constraints_.yaw_rate
    );

    double total_mass = _uav_mass_ + uav_mass_difference_;
    double gravity_force = total_mass * common_handlers_->g;

    // --- PHASE 1: STABILIZE ATTITUDE ---
    if (alignment_phase_ == STABILIZE_ATTITUDE) {

      bool attitude_stable = (std::abs(current_roll) < attitude_stability_threshold_ && 
                              std::abs(current_pitch) < attitude_stability_threshold_);
      
      if (attitude_stable) {
          attitude_stable_counter_++;
          if (attitude_stable_counter_ >= attitude_stable_threshold_) {
            alignment_phase_ = MOVE_AWAY;
            ROS_INFO("[%s][ALIGNING]: Attitude stabilized, moving away from contact point", name_.c_str());
            time_limit_safety_position = ros::Time::now();
        }
      } else {
        attitude_stable_counter_ = 0;
      }
      
      mrs_msgs::HwApiAttitudeCmd attitude_cmd;
      mrs_lib::AttitudeConverter ac_cmd(0.0, 0.0, current_yaw);
      attitude_cmd.orientation = ac_cmd;
      
      double kp_z = 8.0;
      double kd_z = 4.0;
      double pos_error_z = desired_altitude_ - uav_pos_W.z();
      double vel_error_z = 0.0 - uav_vel_W.z();
      double force_pd_z  = kp_z * pos_error_z + kd_z * vel_error_z;
      
      double total_thrust = gravity_force + force_pd_z;

      attitude_cmd.throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(
        common_handlers_->throttle_model,
        total_thrust
      );

      Eigen::Vector3d rate_feedforward;
      rate_feedforward << std::clamp(-omega_current.x(), -2.0, 2.0),
                          std::clamp(-omega_current.y(), -2.0, 2.0),
                          std::clamp(-omega_current.z(), -2.0, 2.0);

      auto attitude_rate_command = mrs_uav_controllers::common::attitudeController(
        uav_state,
        attitude_cmd,
        rate_feedforward,
        attitude_rate_saturation,
        Kq,
        false
      );
      
      if (!attitude_rate_command) {
        ROS_ERROR_THROTTLE(1.0, "[%s][ALIGNING]: attitudeController failed, hover.", name_.c_str());
        last_control_output_ = hoveringControlOutput(dt);
        return;
      }
      
      last_control_output_.control_output = attitude_rate_command.value();
      last_control_output_.desired_orientation = ac_cmd;
    }
    // --- PHASE 2: MOVE AWAY FROM CONTACT ---
    else if (alignment_phase_ == MOVE_AWAY) {

      bool mooving_towards_target = std::abs((uav_pos_W.head<2>() - recovery_position_.head<2>()).norm()) < 0.2;

      if (mooving_towards_target) time_limit_safety_position = ros::Time::now();

      geometry_msgs::PointStamped pos_msg;
      pos_msg.header.stamp = ros::Time::now();
      pos_msg.header.frame_id = "uav1/world_origin";
      pos_msg.point.x = recovery_position_.x();
      pos_msg.point.y = recovery_position_.y();
      pos_msg.point.z = recovery_position_.z();
      pub_1_.publish(pos_msg);

      ROS_INFO_THROTTLE(0.5, "[%s][ALIGNING]: Moving away from contact point, moving towards target: %s, velocity magnitude: %.2f m/s, distance to target: %.2f m", 
                        name_.c_str(), mooving_towards_target ? "true" : "false", std::sqrt(uav_vel_B.x() * uav_vel_B.x() + uav_vel_B.y() * uav_vel_B.y()),
                        std::abs((recovery_position_.head<2>() - uav_pos_W.head<2>()).norm()));

      if ((!mooving_towards_target && (ros::Time::now() - time_limit_safety_position).toSec() > 5.0) || (ros::Time::now() - time_limit_safety_position).toSec() > 10.0) {

        time_limit_safety_position = ros::Time::now();
        
        // If stuck for too long, reset recovery position
        ROS_WARN("[%s][ALIGNING]: Stuck moving away, resetting recovery position", name_.c_str());
        direction_to_contact     = -direction_to_contact;
        direction_to_contact.z() = 0;
        direction_to_contact.normalize();
        Eigen::Vector3d direction_to_recovery = (recovery_position_ - uav_pos_W).normalized();
        recovery_position_       -= direction_to_contact * 2 * safety_distance_;        

        stuck_counter_           = 0;

        alignment_phase_ = STABILIZE_ATTITUDE;
      } else if (stuck_counter_ > 0) {
        stuck_counter_ -= 0.5;
      }

      Eigen::Vector3d pos_error = recovery_position_ - uav_pos_W;
      pos_error.z() = 0;

      //publish error
      geometry_msgs::PointStamped pos_error_msg;
      pos_error_msg.header.stamp = ros::Time::now();
      pos_error_msg.header.frame_id = "uav1/world_origin";
      pos_error_msg.point.x = pos_error.x();
      pos_error_msg.point.y = pos_error.y();
      pos_error_msg.point.z = pos_error.z();
      pub_7_.publish(pos_error_msg);
      
      // Create position control command
      double kp_xy = 5.0; // Position gain
      double kd_xy = std::sqrt(kp_xy); // Velocity damping
      
      // Calculate force in horizontal plane to move away
      Eigen::Vector2d force_horizontal_xy = kp_xy * pos_error.head<2>() - kd_xy * uav_vel_W.head<2>();
      Eigen::Vector3d force_horizontal(force_horizontal_xy.x(), force_horizontal_xy.y(), 0.0);

      //publish force horizontal for debugging
      geometry_msgs::PointStamped force_horizontal_msg;
      force_horizontal_msg.header.stamp = ros::Time::now();
      force_horizontal_msg.header.frame_id = "uav1/world_origin";
      force_horizontal_msg.point.x = force_horizontal.x();
      force_horizontal_msg.point.y = force_horizontal.y();
      force_horizontal_msg.point.z = force_horizontal.z();
      pub_8_.publish(force_horizontal_msg);
      
      // PD control for altitude
      double kp_z = 8.0;
      double kd_z = 4.0;
      double pos_error_z = desired_altitude_ - uav_pos_W.z();
      double vel_error_z = 0.0 - uav_vel_W.z();
      double force_pd_z = kp_z * pos_error_z + kd_z * vel_error_z;

      ROS_INFO_THROTTLE(0.5, "[%s][ALIGNING]: Force horizontal: [%.2f, %.2f, %.2f], Force PD Z: %.2f",
                        name_.c_str(), force_horizontal.x(), force_horizontal.y(), force_horizontal.z(), force_pd_z);

      Eigen::Vector3d rate_feedforward = Eigen::Vector3d::Zero();
      double yaw_error = 0;

      double proj = pos_error.dot(direction_to_contact);

      

      if (proj < 0){
        if(first_rotation_iteration_) {
          double wall_yaw = std::atan(m_wall);
        
          // Calculate the four possible parallel orientations (0°, 90°, 180°, 270° relative to wall)
          double parallel_yaws[4];
          parallel_yaws[0] = wall_yaw;                 // 0° - front face parallel
          parallel_yaws[1] = wall_yaw + M_PI/2.0;      // 90° - right face parallel
          parallel_yaws[2] = wall_yaw + M_PI;          // 180° - back face parallel
          parallel_yaws[3] = wall_yaw + 3.0*M_PI/2.0;  // 270° - left face parallel
          
          // Normalize all angles to -π to π
          for (int i = 0; i < 4; i++) {
            parallel_yaws[i] = std::fmod(parallel_yaws[i] + M_PI, 2*M_PI) - M_PI;
          }
          
          // Find the closest parallel orientation
          double min_diff = 2*M_PI;
          for (int i = 0; i < 4; i++) {
            double diff = std::abs(angles::shortest_angular_distance(current_yaw, parallel_yaws[i]));
            if (diff < min_diff) {
              min_diff = diff;
              target_yaw = parallel_yaws[i];
            }
          }
          first_rotation_iteration_ = false;
        }

        yaw_error = target_yaw - current_yaw;
        yaw_error = std::fmod(yaw_error + M_PI, 2*M_PI) - M_PI;
        double rotation_direction_ = (yaw_error > 0) ? 1.0 : -1.0;
        
        // Apply rotation with smoothing for stability
        double desired_yaw_rate = rotation_direction_ * p_alignment_max_yaw_rate_ *  (1.0 - std::exp(-std::abs(yaw_error) * 5.0));


        rate_feedforward << 0,
                            0,
                            0;

      }else if(first_rotation_iteration_) {
        target_yaw = current_yaw;
      }
      
      // Total force command
      Eigen::Vector3d total_force_cmd_W(force_horizontal.x(), force_horizontal.y(), 
                                        gravity_force + force_pd_z);
      
      // Calculate desired attitude
      Eigen::Vector3d z_body_des = total_force_cmd_W.normalized();
      Eigen::Vector3d x_c(std::cos(target_yaw), std::sin(target_yaw), 0);
      Eigen::Vector3d y_body_des = z_body_des.cross(x_c).normalized();
      Eigen::Vector3d x_body_des = y_body_des.cross(z_body_des).normalized();
      
      Eigen::Matrix3d R_desired;
      R_desired.col(0) = x_body_des;
      R_desired.col(1) = y_body_des;
      R_desired.col(2) = z_body_des;
      
      mrs_lib::AttitudeConverter desired_attitude(R_desired);
      
      double thrust_mag = total_force_cmd_W.dot(z_body_des);
      
      mrs_msgs::HwApiAttitudeCmd attitude_cmd;
      attitude_cmd.orientation = desired_attitude;
      attitude_cmd.throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(
        common_handlers_->throttle_model,
        thrust_mag
      );
      
      // Apply to controller
      auto attitude_rate_command = mrs_uav_controllers::common::attitudeController(
        uav_state,
        attitude_cmd,
        rate_feedforward,
        attitude_rate_saturation,
        Kq,
        false
      );

      //publish desitred attitude for debugging
      geometry_msgs::PointStamped desired_attitude_msg;
      desired_attitude_msg.header.stamp = ros::Time::now();
      desired_attitude_msg.header.frame_id = "uav1/world_origin";
      desired_attitude_msg.point.x = desired_attitude.getRoll();
      desired_attitude_msg.point.y = desired_attitude.getPitch();
      desired_attitude_msg.point.z = desired_attitude.getYaw();

      pub_5_.publish(desired_attitude_msg);

      //publish actual attitude for debugging
      geometry_msgs::PointStamped actual_attitude_msg;
      actual_attitude_msg.header.stamp = ros::Time::now();
      actual_attitude_msg.header.frame_id = "uav1/world_origin";
      actual_attitude_msg.point.x = current_roll;
      actual_attitude_msg.point.y = current_pitch;
      actual_attitude_msg.point.z = current_yaw;

      pub_6_.publish(actual_attitude_msg);
      
      if (!attitude_rate_command) {
        ROS_WARN("[%s][ALIGNING]: attitudeController failed, hover.", name_.c_str());
        last_control_output_ = hoveringControlOutput(dt);
        return;
      }
      
      last_control_output_.control_output = attitude_rate_command.value();
      last_control_output_.desired_orientation = desired_attitude;
      
      ROS_INFO_THROTTLE(0.5, "[%s][ALIGNING]: Moving away, distance to safe point: %.3f m", 
                        name_.c_str(), pos_error.head<2>().norm());
      
      ROS_INFO_THROTTLE(0.1, "[%s][ALIGNING]: Current position: [%.2f, %.2f, %.2f], Recovery position: [%.2f, %.2f, %.2f], error: [%.2f, %.2f, %.2f], yaw error: %.3f rad",
                        name_.c_str(),
                        uav_pos_W.x(), uav_pos_W.y(), uav_pos_W.z(),
                        recovery_position_.x(), recovery_position_.y(), recovery_position_.z(),
                        pos_error.x(), pos_error.y(), pos_error.z(),
                        yaw_error);

      Eigen::Vector3d rel = uav_pos_W - contact_point_W;
      rel.z() = 0;
      Eigen::Vector3d direction_to_recovery = -direction_to_contact;
      double s = rel.dot(direction_to_recovery);

      ROS_INFO_THROTTLE(0.5, "[%s][ALIGNING]: s: %.3f, direction to contact: [%.2f, %.2f, %.2f], rel: [%.2f, %.2f, %.2f]",
                        name_.c_str(), s,
                        direction_to_recovery.x(), direction_to_recovery.y(), direction_to_recovery.z(),
                        rel.x(), rel.y(), rel.z());

      Eigen::Vector3d projected_pt = contact_point_W + s * direction_to_recovery;

      geometry_msgs::PointStamped projected_point_msg;
      projected_point_msg.header.stamp = ros::Time::now();
      projected_point_msg.header.frame_id = "uav1/world_origin";
      projected_point_msg.point.x = projected_pt.x();
      projected_point_msg.point.y = projected_pt.y();
      projected_point_msg.point.z =0;
      pub_11_.publish(projected_point_msg);

      // Check if we've reached the recovery position
      if (s > 0.3
          && std::abs(yaw_error) < 0.1) { // ~3 degrees threshold
        alignment_yaw_rate_low_counter_++;
        if (alignment_yaw_rate_low_counter_ >= 1) { // Stable for 10 iterations
          ROS_INFO("[%s]: Alignment complete. Switching to SLIDING_SQUARE.", name_.c_str());

          wall_interaction_state_ = SLIDING_SQUARE;
          first_sliding_iteration_ = true;
        }
      } else {
        alignment_yaw_rate_low_counter_ = 0;
      }    
    }
  }
  else if (wall_interaction_state_ == SLIDING_SQUARE){

    std_msgs::Float64 msg;
    if (!ready_to_sliding_) {
      msg.data = 5.0;
    } else msg.data = 10.0;
    pub_sliding_phase_.publish(msg);

    // DEBUG MOOVING ON WALL


    if (first_sliding_iteration_) {
      tangent_ref_pos = uav_pos_W;
      initial_position_ = uav_pos_W;
      sliding_start_time = ros::Time::now();
      
      // DEBUG MOOVING ON WALL
      xy_ref.x() = -40;
      xy_ref.y() = 3;
    }

    // 0.1) Check teh relative orientation between the drone and the obstacle
    if (!orthogonal_to_x && !orthogonal_to_y) {

      double dot_x = R_B_to_W.col(0).normalized().dot(direction_to_contact);
      double cos_theta_x = std::clamp(dot_x, -1.0, 1.0);
      double angle_x = std::acos(cos_theta_x);
      
      double dot_y = R_B_to_W.col(1).normalized().dot(direction_to_contact);
      double cos_theta_y = std::clamp(dot_y, -1.0, 1.0);
      double angle_y = std::acos(cos_theta_y);

      orthogonal_to_x = std::abs(angle_x + 1) > M_PI || std::abs(angle_x) < 1;
      orthogonal_to_y = std::abs(angle_y + 1) > M_PI || std::abs(angle_y) < 1;

      ROS_INFO("[%s][SLIDING_SQUARE]: angular distanca to x:%.3f and y:%.3f, dot_x:%.3f, dot_y:%.3f",
                name_.c_str(), angle_x, angle_y,dot_x, dot_y);

      if (orthogonal_to_x){
        if(dot_x > 0.5){
          ROS_INFO("[%s][SLIDING_SQUARE]: Orthogonal positive to x-axis, angle: %.3f rad, dot: %.3f", name_.c_str(), angle_x, dot_x);
          R_OrientAdjust <<  0,  1,  0,
                            -1,  0,  0,
                             0,  0,  1;
        } else if(dot_x < -0.5) {
          ROS_INFO("[%s][SLIDING_SQUARE]: Orthogonal negative to x-axis, angle: %.3f rad, dot: %.3f", name_.c_str(), angle_x, dot_x);
          R_OrientAdjust <<  0, -1,  0,
                             1,  0,  0,
                             0,  0,  1;
        } else {
          ROS_WARN("[%s][SLIDING_SQUARE]: Orthogonal to x-axis but dot product is not high enough: %.3f", name_.c_str(), dot_x);
        }
      }else if (orthogonal_to_y){
        if(dot_y > 0.5){
          ROS_INFO("[%s][SLIDING_SQUARE]: Orthogonal to positive y-axis, angle: %.3f rad, dot: %.3f", name_.c_str(), angle_y, dot_y);
        } else if(dot_y < -0.5) {
          ROS_INFO("[%s][SLIDING_SQUARE]: Orthogonal to  negative y-axis, angle: %.3f rad, dot: %.3f", name_.c_str(), angle_y, dot_y);
          R_OrientAdjust << -1,  0,  0,
                             0, -1,  0,
                             0,  0,  1;
        } else {
          ROS_WARN("[%s][SLIDING_SQUARE]: Orthogonal to y-axis but dot product is not high enough: %.3f", name_.c_str(), dot_y);
        }
      }else {
        ROS_WARN("[%s][SLIDING_SQUARE]: Not orthogonal to any axis, angles: x=%.3f rad, y=%.3f rad", 
                  name_.c_str(), angle_x, angle_y);
      }
      initial_yaw_ = current_yaw;
    }

    if (orthogonal_to_x){
      wall_normal_inertial_estimate_ = R_B_to_W .col(0);
      sliding_tangent_inertial_cmd_ = R_B_to_W.col(1);
    }else if (orthogonal_to_y){
      wall_normal_inertial_estimate_ = R_B_to_W .col(1);
      sliding_tangent_inertial_cmd_ = R_B_to_W.col(0);
    }

    // 0.2) Verify if the obstacle is still present
    std_msgs::Bool switch_msg;
    switch_msg.data = false;
    bool mooving = std::abs((R_OrientAdjust * uav_vel_B).y()) > 0.2 * contact_velocity_magnitude &&
                   std::signbit((R_OrientAdjust * uav_vel_B).y()) == std::signbit((R_B_to_W.transpose() * normal_ref_vel_W_).y());

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: uav velocity:[%.3f, %.3f, %.3f], uav rotated velocity: [%.3f, %.3f, %.3f], mooving: %s",
                      name_.c_str(), uav_vel_B.x(), uav_vel_B.y(), uav_vel_B.z(),
                      (R_OrientAdjust * uav_vel_B).x(), (R_OrientAdjust * uav_vel_B).y(), (R_OrientAdjust * uav_vel_B).z(),
                      mooving ? "true" : "false");

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Mooving: %s, contact velocity magnitude: %.3f/%.3f, current velocity: [%.3f, %.3f, %.3f], %.3f>%.3f, q_wall: %.3f",
                      name_.c_str(), mooving ? "true" : "false", 
                      (R_OrientAdjust * uav_vel_B).y(), contact_velocity_magnitude * 0.2,
                      uav_vel_B.x(), uav_vel_B.y(), uav_vel_B.z(),
                      uav_pos_W.y(), uav_pos_W.x() * m_wall + q_wall + 1.0,
                      q_wall);

    double delta_q = std::max(0.5, -29.5 * wall_points.size()/max_wall_points + 30.0);

    bool ahead_of_wall_with_margin = uav_pos_W.y() > uav_pos_W.x() * m_wall + q_wall + delta_q && ready_to_sliding_ && mooving ;
    bool behind_wall_with_margin = uav_pos_W.y() < uav_pos_W.x() * m_wall + q_wall - delta_q && ready_to_sliding_ && mooving ;

    bool mooving_forward;
    bool switch_controller = false;
    if (std::abs(m_wall) < 10.0){
      mooving_forward = (m_wall * direction_to_contact.x() + direction_to_contact.y()) > 0.0;

      if ((mooving_forward && ahead_of_wall_with_margin) || (!mooving_forward && behind_wall_with_margin) ) {
        switch_msg.data = true;
        }
    }else{
      mooving_forward = direction_to_contact.y() > 0.0;
      if ((uav_pos_W.y() > uav_pos_W.x() * std::abs(m_wall) + q_wall + delta_q && ready_to_sliding_ && mooving) && mooving_forward) {
        switch_msg.data = true;
      }else if ((uav_pos_W.y() < uav_pos_W.x() * std::abs(m_wall) + q_wall - delta_q && ready_to_sliding_ && mooving) && !mooving_forward) {
        switch_msg.data = true;
      }
    }

    if (switch_msg.data) {
      ROS_INFO("[%s][SLIDING_SQUARE]: Total weight: %.2f, wall yaw total weight: %.2f, wall yaw weighted sum: %.3f",
              name_.c_str(), wall_yaw_total_weight, wall_yaw_total_weight, wall_yaw_weighted_sum);

      ROS_INFO("[%s][SLIDING_SQUARE]: mooving forward: %s, normal ref vel: [%.3f, %.3f, %.3f],",
                      name_.c_str(), mooving_forward ? "true" : "false", 
                      normal_ref_vel_W_.x(), normal_ref_vel_W_.y(), normal_ref_vel_W_.z());
                      
      for (const auto& point : wall_points) {
        ROS_INFO("[%s][SLIDING_SQUARE]: Wall point: [%.3f, %.3f, %.3f], weight: %.3f, q_wall: %.3f",
                name_.c_str(),
                point.position.x(), point.position.y(), point.position.z(), point.weight,
                -point.position.x() * m_wall + point.position.y());
      }
      pub_switch_command_.publish(switch_msg);
    }

    




    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: mooving forward: %s, normal ref vel: [%.3f, %.3f, %.3f],",
                      name_.c_str(), mooving_forward ? "true" : "false", 
                      normal_ref_vel_W_.x(), normal_ref_vel_W_.y(), normal_ref_vel_W_.z());

    

    // 0.3) Data collection and wall estimation

    if (!ready_to_sliding_) desired_yaw_ = current_yaw;
    double yaw_error = desired_yaw_ - current_yaw;
    yaw_error = std::fmod(yaw_error + M_PI, 2*M_PI) - M_PI;
    double estimated_wall_yaw;

    double wall_estimation_weight = 0.0;

    if (!first_sliding_iteration_ && (ros::Time::now() - sliding_start_time).toSec() > 0.2) {
      double velocity_weight = 1.0 / (1.0 + std::exp(((std::abs((R_OrientAdjust * uav_vel_B).y())-0.2) * 20.0)));
      double rotation_stability = 1.0 / (1.0 + 20.0 * std::abs(uav_state.velocity.angular.z));

      wall_estimation_weight = velocity_weight * rotation_stability;
      
      std_msgs::Float64 wall_estimation_weight_msg;
      wall_estimation_weight_msg.data = wall_yaw_total_weight;
      pub_2_.publish(wall_estimation_weight_msg);

      std_msgs::Float64 velocity_msg;
      velocity_msg.data = (R_OrientAdjust * uav_vel_B).x();
      pub_3_.publish(velocity_msg);

      std_msgs::Float64 velocity_weight_msg;
      velocity_weight_msg.data = velocity_weight;
      pub_4_.publish(velocity_weight_msg);


      ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Wall estimation weight: %.3f, velocity weight: %.3f, rotation stability: %.3f, velocity: %.3f, angular velocity: [%.3f]",
              name_.c_str(), wall_estimation_weight, velocity_weight, rotation_stability, (R_OrientAdjust * uav_vel_B).x(), uav_state.velocity.angular.z);

      if (wall_estimation_weight > 0.6){
        wall_yaw_weighted_sum += 2 *current_yaw * wall_estimation_weight;
        wall_yaw_total_weight += 2 * wall_estimation_weight;
      }

      if (wall_yaw_total_weight > 500.0) {
        if (xy_0_wall.isZero(1e-6) || first_sliding_iteration_) {
          xy_0_wall.x() = uav_pos_W.x();
          xy_0_wall.y() = uav_pos_W.y();
        }
        xy_ref_wall.x() = xy_0_wall.x() + xy_ref.x() * (1/std::sqrt(1 + m_wall * m_wall));
        xy_ref_wall.y() = m_wall * xy_ref_wall.x() + q_wall;

        geometry_msgs::PointStamped ref_point_msg;
        ref_point_msg.header.stamp = ros::Time::now();
        ref_point_msg.header.frame_id = "uav1/world_origin";
        ref_point_msg.point.x = xy_ref_wall.x();
        ref_point_msg.point.y = xy_ref_wall.y();
        ref_point_msg.point.z =0;
        pub_9_.publish(ref_point_msg);

        ready_to_sliding_ = true;
        estimated_wall_yaw = wall_yaw_weighted_sum / wall_yaw_total_weight;

        if (std::abs(estimated_wall_yaw - current_yaw) < 5* M_PI / 180.0) {
          desired_yaw_ = current_yaw;
        } else {
          desired_yaw_ = estimated_wall_yaw;
        }
      
      } else {
        desired_yaw_ = current_yaw;

        ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Wall yaw not updated yet, total weight: %.2f", name_.c_str(), wall_yaw_total_weight);
      }

    }else first_sliding_iteration_= false;


    if (ready_to_sliding_ && wall_estimation_weight > 0.8) {
      auto computeQ = [](const Eigen::Vector3d& pos, double m_wall) {
        return pos.y() - m_wall * pos.x();
      };
      
      bool new_valid_wall_point = false;
      
      if (wall_points.size() < max_wall_points) {
        m_wall = std::tan(estimated_wall_yaw);
        wall_points.push_back({uav_pos_W, wall_estimation_weight, computeQ(uav_pos_W, m_wall)});

        ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Added new wall point: [%.3f, %.3f, %.3f], weight: %.3f",
                          name_.c_str(),
                          uav_pos_W.x(), uav_pos_W.y(), uav_pos_W.z(),
                          wall_estimation_weight);
      }else{
        double q_current = computeQ(uav_pos_W, m_wall);
        double q_last    = computeQ(wall_points.back().position, m_wall);

        bool moving_forward = normal_ref_vel_W_.y() > 0.0;
        bool moving_backward = !moving_forward;

        new_valid_wall_point = (q_current > q_last && moving_forward) ||
                              (q_current < q_last && moving_backward);
      }
      
      if(new_valid_wall_point) {   
        wall_points.push_back({uav_pos_W, wall_estimation_weight, computeQ(uav_pos_W, m_wall)});

        ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: New wall point candidate: [%.3f, %.3f, %.3f], weight: %.3f",
                          name_.c_str(),
                          uav_pos_W.x(), uav_pos_W.y(), uav_pos_W.z(),
                          wall_estimation_weight);        
      }

      if (wall_points.size() < max_wall_points/3){
        m_wall = std::tan(estimated_wall_yaw);
      }

      bool time_elapsed = (ros::Time::now() - last_wall_point_update_time_).toSec() > 0.1;

      if ((new_valid_wall_point || time_elapsed) && wall_points.size() >= max_wall_points/3) {

        last_wall_point_update_time_ = ros::Time::now();

        for (auto& point : wall_points) {
          point.q = computeQ(point.position, m_wall);
        }

        //Liear regression
        double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
        size_t N = wall_points.size();
        for (const auto& point : wall_points) {
          double x = point.position.x();
          double y = point.position.y();
          sum_x  += x;
          sum_y  += y;
          sum_xx += x * x;
          sum_xy += x * y;
        }

        double denominator = N * sum_xx - sum_x * sum_x;
        double average_q;
        if (std::abs(denominator) > 1e-6) {
          m_wall = (N * sum_xy - sum_x * sum_y) / denominator;
          average_q = (sum_y - m_wall * sum_x) / N;
          ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Linear regression: m_wall=%.3f, q_wall=%.3f, average q: %.3f", name_.c_str(), m_wall, q_wall, average_q);
        } else {
          ROS_WARN_THROTTLE(1.0, "[%s][SLIDING_SQUARE]: Linear regression denominator too small, skipping update.", name_.c_str());
        }

        std::sort(wall_points.begin(), wall_points.end(),
                  [sign = normal_ref_vel_W_.y() > 0.0](const WallPoint& a, const WallPoint& b) {
                    return sign ? (a.q < b.q) : (a.q > b.q);
                  });
          
          q_wall = average_q;

          

      }
      if (wall_points.size() > max_wall_points) {
        wall_points.resize(max_wall_points-1);
      }
    }else if (ready_to_sliding_) {
      ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Wall estimation weight too low: %.3f, not adding new point.", name_.c_str(), wall_estimation_weight);
    }

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Wall points size: %zu, m_wall: %.3f, q_wall: %.3f, estimted wall m: %.3f",
                      name_.c_str(), wall_points.size(), m_wall, q_wall, std::tan(estimated_wall_yaw));

    //Point cloud

    if (wall_estimation_weight > 0.8 && ready_to_sliding_) {
      Eigen::Vector3d normal = wall_normal_inertial_estimate_.normalized();
      Eigen::Vector3d tangent = sliding_tangent_inertial_cmd_.normalized();
      Eigen::Vector3d binormal = normal.cross(tangent).normalized();

      double half = radius_ / 2.0;

      // Center
      Eigen::Vector3d p_center = uav_pos_W;

      // Corners of the square
      Eigen::Vector3d p1 = p_center +  half * tangent +  half * binormal;
      Eigen::Vector3d p2 = p_center +  half * tangent -  half * binormal;
      Eigen::Vector3d p3 = p_center -  half * tangent +  half * binormal;
      Eigen::Vector3d p4 = p_center -  half * tangent -  half * binormal;

      // Midpoints of the edges
      Eigen::Vector3d p5 = p_center +  half * tangent;
      Eigen::Vector3d p6 = p_center -  half * tangent;
      Eigen::Vector3d p7 = p_center +  half * binormal;
      Eigen::Vector3d p8 = p_center -  half * binormal;

      std::vector<Eigen::Vector3d> contact_face_points = {p_center, p1, p2, p3, p4, p5, p6, p7, p8};

      if (wall_normal_inertial_estimate_.normalized().dot(direction_to_contact) < 0.0) {
        // If the normal is facing away from the contact point, flip the points
        normal = -normal;
      }

      Eigen::Vector3d offset = normal * (radius_ / 2.0);

      for (auto& pt : contact_face_points) {
          pt += offset;
      }

      sensor_msgs::PointCloud2 cloud_msg;
      cloud_msg.header.stamp = ros::Time::now();
      cloud_msg.header.frame_id = "uav1/world_origin";
      cloud_msg.height = 1;
      cloud_msg.width = contact_face_points.size();
      cloud_msg.is_dense = false;
      cloud_msg.is_bigendian = false;

      sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      modifier.resize(contact_face_points.size());

      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_rgb(cloud_msg, "rgb");

      for (const auto& pt : contact_face_points) {
        *iter_x = pt.x();
        *iter_y = pt.y();
        *iter_z = pt.z();

        uint8_t r = 255, g = 0, b = 0; // Rosso
        uint32_t rgb = (r << 16) | (g << 8) | b;
        float rgb_float;
        std::memcpy(&rgb_float, &rgb, sizeof(float));
        *iter_rgb = rgb_float;

        ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
      }

      pub_contact_cloud.publish(cloud_msg);
      ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Published contact face point cloud with %zu points.", 
                      name_.c_str(), contact_face_points.size());
      ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Contact face points: Center: [%.3f, %.3f, %.3f], P1: [%.3f, %.3f, %.3f], P2: [%.3f, %.3f, %.3f], P3: [%.3f, %.3f, %.3f], P4: [%.3f, %.3f, %.3f]",
                      name_.c_str(),
                      p_center.x(), p_center.y(), p_center.z(),
                      p1.x(), p1.y(), p1.z(),
                      p2.x(), p2.y(), p2.z(),
                      p3.x(), p3.y(), p3.z(),
                      p4.x(), p4.y(), p4.z());
    }
    

    
    // 1. Computes the direction of impedance
    
    contact_velocity_magnitude = 1.0/(1.0 + std::exp(5 * (std::abs(yaw_error) - 0.349066))); // more than 20 deg
    Eigen::Vector3d normal_ref_pos_W_;


    // 1.2 Normal component
    
    Eigen::Vector3d normal_component_Wall(0, contact_velocity_magnitude, 0);
    normal_component_Wall = R_OrientAdjust * normal_component_Wall; 
    
    // Transform to world frame
    normal_ref_vel_W_ = R_B_to_W * normal_component_Wall;
    normal_ref_pos_W_ = uav_pos_W + (R_B_to_W * normal_component_Wall) * dt;

    // Re-transform to body frame
    Eigen::Vector3d normal_ref_pos_B_ = R_B_to_W.transpose() * (normal_ref_pos_W_ - uav_pos_W);
    Eigen::Vector3d normal_ref_vel_B_ = R_B_to_W.transpose() * normal_ref_vel_W_;

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Ref pos: [%.3f, %.3f, %.3f], ref vel: [%.3f, %.3f, %.3f], uav pos: [%.3f, %.3f, %.3f],"
                      "uav vel: [%.3f, %.3f, %.3f]",
                      name_.c_str(),
                      normal_ref_pos_W_.x(), normal_ref_pos_W_.y(), normal_ref_pos_W_.z(),
                      normal_ref_vel_W_.x(), normal_ref_vel_W_.y(), normal_ref_vel_W_.z(),
                      uav_pos_W.x(), uav_pos_W.y(), uav_pos_W.z(),
                      uav_vel_W.x(), uav_vel_W.y(), uav_vel_W.z());

    // 1.3 Impedance errors
    impedance_vel_error_B_                 = normal_component_Wall - uav_vel_B;
    Eigen::Vector3d impedance_vel_error_W_ = R_B_to_W * impedance_vel_error_B_;
    
    Eigen::Vector3d impedance_pos_error_W_ = normal_ref_pos_W_ - uav_pos_W;
    Eigen::Vector3d impedance_pos_error_B_ = R_B_to_W.transpose() * impedance_pos_error_W_;

    // 1.4 Logging
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: World normal ref pos: [%.3f, %.3f, %.3f], ref vel: [%.3f, %.3f, %.3f], pos error: [%.3f, %.3f, %.3f], vel error: [%.3f, %.3f, %.3f]",
                      name_.c_str(),
                      normal_ref_pos_W_.x(),      normal_ref_pos_W_.y(),      normal_ref_pos_W_.z(),
                      normal_ref_vel_W_.x(),      normal_ref_vel_W_.y(),      normal_ref_vel_W_.z(),
                      impedance_pos_error_W_.x(), impedance_pos_error_W_.y(), impedance_pos_error_W_.z(),
                      impedance_vel_error_W_.x(), impedance_vel_error_W_.y(), impedance_vel_error_W_.z());


    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Body normal ref pos: [%.3f, %.3f, %.3f], ref vel: [%.3f, %.3f, %.3f], pos error: [%.3f, %.3f, %.3f], vel error: [%.3f, %.3f, %.3f]",
                      name_.c_str(),
                      normal_ref_pos_B_.x(), normal_ref_pos_B_.y(), normal_ref_pos_B_.z(),
                      normal_ref_vel_B_.x(), normal_ref_vel_B_.y(), normal_ref_vel_B_.z(),
                      impedance_pos_error_B_.x(), impedance_pos_error_B_.y(), impedance_pos_error_B_.z(),
                      impedance_vel_error_B_.x(), impedance_vel_error_B_.y(), impedance_vel_error_B_.z());

    /*if (!ready_to_sliding_) {
      double initial_delta = (R_OrientAdjust * (initial_position_ - uav_pos_W)).y();
      if (std::abs(initial_delta) > 2 * safety_distance_) {
        ROS_WARN("[%s][SLIDING_SQUARE]: Initial position too far from the wall, stopping sliding square.", name_.c_str());
        orthogonal_to_x = false;
        orthogonal_to_y = false;
        direction_to_contact = -direction_to_contact;
        safety_distance_ *= 3;
      }
    }*/

    // 2. Impedance parameters

    double D = 3.0;
    double K = 5.0;

    // 3. Impedance force

    double vel_error_normal = impedance_vel_error_W_.dot(wall_normal_inertial_estimate_);
    double pos_error_normal = impedance_pos_error_W_.dot(wall_normal_inertial_estimate_);

    double f_cmd_impedance = K * pos_error_normal + D * vel_error_normal;

    Eigen::Vector3d force_normal = wall_normal_inertial_estimate_ * f_cmd_impedance;

    // 4. Tangential force
    // 4.1 Limiting the pos error
    
    pos_error_tangent = (R_OrientAdjust * (tangent_ref_pos - uav_pos_W)).x();

    // 4.2 Setting sliding tangent command
    double yaw_reduction_term         = 1 / (1.0 + std::exp(10 * (std::abs(yaw_error) - 0.349066))); // 20 deg
    double normal_reduction_term      = 1 / (1.0 + std::exp(15 * (std::abs((R_OrientAdjust * uav_vel_B).y()) - 0.2))); // 0.3 m/s
    sliding_direction_ = -1;
    double sliding_velocity_magnitude = std::clamp(sliding_direction_ * 2.5 * yaw_reduction_term * normal_reduction_term,
                                        (R_OrientAdjust * uav_vel_B).x() - 2, (R_OrientAdjust * uav_vel_B).x() + 2);

    if (mooving_on_wall_ && ready_to_sliding_){
      double min_dist = 0.5;
      double max_dist = 5.0;
      Eigen::Vector2d drone_xy(uav_pos_W.x(), uav_pos_W.y());
      Eigen::Vector2d tangent_xy(sliding_tangent_inertial_cmd_.x(), sliding_tangent_inertial_cmd_.y());
      tangent_xy.normalize();

      double distance = (xy_ref_wall - drone_xy).dot(tangent_xy);
      double distance_abs = std::abs(distance);
      ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Distance to wall along tangent: %.3f, initial sliding velocity : %.3f",
                        name_.c_str(), distance, sliding_velocity_magnitude);
      if (distance_abs <= min_dist){
        sliding_velocity_magnitude = 0.0;
        double k_additional = 2.0;
        pos_error_tangent = distance * k_additional;
        if (std::abs(uav_vel_B.head<2>().norm())< 0.05) force_normal*= 0.5;
      } else if (distance_abs <= max_dist) {
        min_dist *= 0.5;;
        double scale = (distance_abs - min_dist) / (max_dist - min_dist);
        scale = std::max(0.0, std::min(1.0, scale));
        sliding_velocity_magnitude = ((distance >= 0) ? 1.0 : -1.0) * scale * std::abs(sliding_velocity_magnitude);
        ROS_INFO("[%s][SLIDING_SQUARE]: Sliding velocity magnitude scaled to: %.3f, with scale: %.3f", name_.c_str(), sliding_velocity_magnitude, scale);
        double k_additional = 1.5;
        //pos_error_tangent = distance * k_additional;
        if (std::abs(uav_vel_B.head<2>().norm())< 0.05) force_normal*= 0.5;
        if((R_OrientAdjust * uav_vel_B).x() < 0.5){
          pos_error_tangent = distance * k_additional;
        }
      }
      if (distance_abs <= 0.1){
        x_target = true;
        ROS_INFO("[%s][SLIDING_SQUARE]: Mooving vertical, sliding phase completed", name_.c_str());
      }
    }

    pos_error_tangent = std::clamp(pos_error_tangent, -3.0 , 3.0);

    std_msgs::Float64 sliding_velocity_msg;
    sliding_velocity_msg.data = sliding_velocity_magnitude;
    pub_10_.publish(sliding_velocity_msg);

    


    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Yaw reduction term: %.3f, normal reduction term: %.3f, sliding velocity magnitude: %.3f/2.5, velocity error: %.3f",
                      name_.c_str(),
                      yaw_reduction_term,
                      normal_reduction_term,
                      sliding_velocity_magnitude,
                      (R_OrientAdjust * uav_vel_B).y());

    tangent_ref_vel   = R_B_to_W * (R_OrientAdjust * Eigen::Vector3d(sliding_velocity_magnitude, 0, 0));
    tangent_ref_pos   =   uav_pos_W + tangent_ref_vel* dt + R_B_to_W * (R_OrientAdjust * Eigen::Vector3d(pos_error_tangent, 0, 0));
    tangent_ref_pos_B = R_B_to_W.transpose() * (tangent_ref_pos - uav_pos_W);

    // 4.3 Tangent error computation
    Eigen::Vector3d tangent_ref_vel_vectorial = tangent_ref_vel - uav_vel_W;
    double vel_error_tangent = tangent_ref_vel_vectorial.dot(sliding_tangent_inertial_cmd_);

    // 4.4 Tangent force command
    double f_cmd_sliding = p_sliding_kp_pos_tangent_ * pos_error_tangent + p_sliding_kd_pos_tangent_ * vel_error_tangent;

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]:Velocity gain: %.3f, Position gain: %.3f, f_cmd_sliding: %.3f, pos error tangent: %.3f",
                      name_.c_str(),
                      p_sliding_kp_pos_tangent_, p_sliding_kd_pos_tangent_,
                      f_cmd_sliding, pos_error_tangent);

    if (!ready_to_sliding_) {
      f_cmd_sliding = 0.0;
      pos_error_tangent = 0.0;
      ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Sliding tangent command not ready, f_cmd_sliding set to 0.0", name_.c_str());
    }

    Eigen::Vector3d force_cmd_tangent = sliding_tangent_inertial_cmd_ * f_cmd_sliding;

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING] Sliding tangent cmd: [%.3f, %.3f, %.3f], f_cmd_sliding: %.3f",
                        name_.c_str(),
                        sliding_tangent_inertial_cmd_.x(), sliding_tangent_inertial_cmd_.y(), sliding_tangent_inertial_cmd_.z(),
                        f_cmd_sliding);

    // 4.5 Logging
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: World tangent ref pos: [%.3f, %.3f, %.3f], ref vel: [%.3f, %.3f, %.3f], vel error: [%.3f, %.3f, %.3f],"
                      " position error tangent: %.3f",
                      name_.c_str(),
                      tangent_ref_pos.x(), tangent_ref_pos.y(), tangent_ref_pos.z(),
                      tangent_ref_vel.x(), tangent_ref_vel.y(), tangent_ref_vel.z(),
                      tangent_ref_vel_vectorial.x(), tangent_ref_vel_vectorial.y(), tangent_ref_vel_vectorial.z(),
                      pos_error_tangent);

    Eigen::Vector3d body_tangent_ref_pos = R_B_to_W.transpose() * tangent_ref_pos;
    Eigen::Vector3d body_tangent_ref_vel = R_B_to_W.transpose() * tangent_ref_vel;
    Eigen::Vector3d body_tangent_pos_error = R_B_to_W.transpose() * (tangent_ref_pos - uav_pos_W);
    Eigen::Vector3d body_tangent_vel_error = R_B_to_W.transpose() * (tangent_ref_vel - uav_vel_W);
    
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Body tangent ref pos: [%.3f, %.3f, %.3f], ref vel: [%.3f, %.3f, %.3f], pos error: [%.3f, %.3f, %.3f], vel error: [%.3f, %.3f, %.3f]",
                      name_.c_str(),
                      body_tangent_ref_pos.x(), body_tangent_ref_pos.y(), body_tangent_ref_pos.z(),
                      body_tangent_ref_vel.x(), body_tangent_ref_vel.y(), body_tangent_ref_vel.z(),
                      body_tangent_pos_error.x(), body_tangent_pos_error.y(), body_tangent_pos_error.z(),
                      body_tangent_vel_error.x(), body_tangent_vel_error.y(), body_tangent_vel_error.z());

    // 5. Sum gravity, disturbances, etc.

    // 5.1 Gravity compensation
    Eigen::Vector3d gravity_comp_W(0, 0, total_mass * common_handlers_->g);
    
    //5.2 Altitude PD command
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Altitude tracker command: [%.3f, %.3f, %.3f],",
                      name_.c_str(),
                      tracker_command.position.x, tracker_command.position.y, tracker_command.position.z);

    double pos_error_z = tracker_command.position.z - uav_pos_W.z();
    if (mooving_on_wall_ && x_target){
      pos_error_z = xy_ref.y() - uav_pos_W.z();
      force_normal *= 0.5;
    }
    double vel_error_z = - uav_vel_W.z();
    Eigen::Vector3d pd_cmd_z_W (0, 0, p_sliding_kp_pos_z_ * pos_error_z + p_sliding_kd_pos_z_ * vel_error_z);
    Eigen::Vector3d total_force_cmd_W = gravity_comp_W + force_normal + force_cmd_tangent - disturbance_comp_W + pd_cmd_z_W;

    // 6. Attitude and throttle extraction
    
    // 6.1 Attitude extraction
    Eigen::Vector3d z_body_des = total_force_cmd_W.normalized();
    Eigen::Vector3d x_c(std::cos(std::atan(m_wall)), std::sin(std::atan(std::atan(m_wall))), 0);
    Eigen::Vector3d y_body_des = z_body_des.cross(x_c).normalized();
    Eigen::Vector3d x_body_des = y_body_des.cross(z_body_des).normalized();

    Eigen::Matrix3d R_desired_att_B_to_W;
    R_desired_att_B_to_W.col(0) = x_body_des;
    R_desired_att_B_to_W.col(1) = y_body_des;
    R_desired_att_B_to_W.col(2) = z_body_des;

    mrs_lib::AttitudeConverter desired_attitude_slide(R_desired_att_B_to_W);
    
    // 6.2 Throttle extraction
    double thrust_mag = total_force_cmd_W.dot(z_body_des);

    // 6.3 Attitude command
    mrs_msgs::HwApiAttitudeCmd attitude_cmd_slide;
    attitude_cmd_slide.orientation = desired_attitude_slide;
    attitude_cmd_slide.throttle    = mrs_lib::quadratic_throttle_model::forceToThrottle(
        common_handlers_->throttle_model,
        thrust_mag);
    
    // 6.4 Logging
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Total force cmd: [%.3f, %.3f, %.3f], gravity comp: [%.3f, %.3f, %.3f], force impedance: [%.3f, %.3f, %.3f],"
                      " force sliding: [%.3f, %.3f, %.3f], disturbance comp: [%.3f, %.3f, %.3f], altitude cmd: [%.3f, %.3f, %.3f]",
                      name_.c_str(),
                      total_force_cmd_W.x(), total_force_cmd_W.y(), total_force_cmd_W.z(),
                      gravity_comp_W.x(), gravity_comp_W.y(), gravity_comp_W.z(),
                      force_normal.x(), force_normal.y(), force_normal.z(),
                      force_cmd_tangent.x(), force_cmd_tangent.y(), force_cmd_tangent.z(),
                      disturbance_comp_W.x(), disturbance_comp_W.y(), disturbance_comp_W.z(),
                      pd_cmd_z_W.x(), pd_cmd_z_W.y(), pd_cmd_z_W.z()
                    );
    
    Eigen::Vector3d total_force_cmd_B = R_B_to_W.inverse() * total_force_cmd_W;
    Eigen::Vector3d force_normal_B = R_B_to_W.inverse() * force_normal;
    Eigen::Vector3d force_sliding_B = R_B_to_W.inverse() * force_cmd_tangent;
    Eigen::Vector3d gravity_comp_B = R_B_to_W.inverse() * gravity_comp_W;
    Eigen::Vector3d disturbance_comp_B = R_B_to_W.inverse() * disturbance_comp_W;
    Eigen::Vector3d pd_cmd_z_B = R_B_to_W.inverse() * pd_cmd_z_W;

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Body force cmd: [%.3f, %.3f, %.3f], gravity comp: [%.3f, %.3f, %.3f], force impedance: [%.8f, %.3f, %.3f],"
                      " force sliding: [%.3f, %.3f, %.3f], disturbance comp: [%.3f, %.3f, %.3f], altitude cmd: [%.3f, %.3f, %.3f]",
                      name_.c_str(),
                      total_force_cmd_B.x(), total_force_cmd_B.y(), total_force_cmd_B.z(),
                      gravity_comp_B.x(), gravity_comp_B.y(), gravity_comp_B.z(),
                      force_normal_B.x(), force_normal_B.y(), force_normal_B.z(),
                      force_sliding_B.x(), force_sliding_B.y(), force_sliding_B.z(),
                      disturbance_comp_B.x(), disturbance_comp_B.y(), disturbance_comp_B.z(),
                      pd_cmd_z_B.x(), pd_cmd_z_B.y(), pd_cmd_z_B.z()
                    );

    Eigen::Vector3d z_body_des_check = R_desired_att_B_to_W.col(2);                  
    double desired_tilt_rad = std::acos(z_body_des_check.dot(Eigen::Vector3d(0, 0, 1)));
    double desired_tilt_deg = desired_tilt_rad * 180.0 / M_PI;

    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Actual/Desired attitude: roll=%.3f/%.3f, pitch=%.3f/%.3f, yaw=%.3f/%.3f, tilt=%.3f/%.3f deg,",
                      name_.c_str(),
                      current_roll, desired_attitude_slide.getRoll(),
                      current_pitch, desired_attitude_slide.getPitch(),
                      current_yaw, desired_attitude_slide.getYaw(),
                      tilt_deg, desired_tilt_deg);

    if (std::abs(current_yaw - initial_yaw_) > (3.0/8.0) * M_PI) { // 60 degrees
      orthogonal_to_x = false;
      orthogonal_to_y = false;
      wall_yaw_weighted_sum = 0.0;
      wall_yaw_total_weight = 0.0;
      ROS_WARN("[%s][SLIDING_SQUARE]: Yaw deviation too high, resetting wall estimation, yaw difference is : %.3f", name_.c_str(), 
                      std::abs(current_yaw - initial_yaw_));
      wall_points.clear();
      ready_to_sliding_ = false;
      first_sliding_iteration_ = true;
      desired_yaw_ = current_yaw;
    }

    double kp_yaw = 10.0;
    double kd_yaw = 5.0;
    double yaw_rate_current = uav_state.velocity.angular.z;
    auto ext_torque = sh_ext_torque_.getMsg();
    double ext_mz = ext_torque->z;
    double desired_yaw_rate = 0.0;
    if (std::abs(yaw_error)>0.2){
      desired_yaw_rate = kp_yaw * yaw_error - kd_yaw * yaw_rate_current - ext_mz;
    }

    // Limita il yaw rate
    desired_yaw_rate = std::clamp(desired_yaw_rate, -1.0, 1.0);
    ROS_INFO_THROTTLE(0.1, "[%s][SLIDING_SQUARE]: Desired yaw rate: [%.3f], current yaw rate: [%.3f]",
                      name_.c_str(),
                      desired_yaw_rate,
                      yaw_rate_current);
    // 
    // Last control output
    last_control_output_.control_output = attitude_cmd_slide;
    last_control_output_.desired_heading_rate = desired_yaw_rate;
    last_control_output_.desired_orientation = Eigen::Quaterniond(
        attitude_cmd_slide.orientation.w,
        attitude_cmd_slide.orientation.x,
        attitude_cmd_slide.orientation.y,
        attitude_cmd_slide.orientation.z);
  }
  else {
    ROS_ERROR_THROTTLE(0.1, "[%s]: Stato wall_interaction sconosciuto! Hover.", name_.c_str());
    last_control_output_ = hoveringControlOutput(dt);
    wall_interaction_state_ = ALIGNING_TO_WALL;
    first_alignment_iteration_ = true;
  }

  // | ----------------- fill in the diagnostics ---------------- |

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

  p_sliding_kp_force_ = drs_params_.p_sliding_kp_force;
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
  
  double desired_yaw_hover = 0;
  if (sh_odometry_.hasMsg()){ 
    mrs_lib::AttitudeConverter ac(sh_odometry_.getMsg()->pose.pose.orientation);
    desired_yaw_hover = ac.getYaw();
  } else if (is_initialized_ && (uav_state_.pose.orientation.w != 0 || uav_state_.pose.orientation.x != 0 || uav_state_.pose.orientation.y != 0 || uav_state_.pose.orientation.z != 0) ) { 
     mrs_lib::AttitudeConverter ac(uav_state_.pose.orientation);
     desired_yaw_hover = ac.getYaw();
  }

  mrs_msgs::HwApiAttitudeCmd attitude_cmd;
  attitude_cmd.orientation = mrs_lib::AttitudeConverter(0, 0, desired_yaw_hover); 
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

//* holdPose() //{ */
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
      common_handlers_->throttle_model, force_sp.dot(z_body_des));

  const auto& q = attitude_cmd.orientation;

  last_control_output_.control_output = attitude_cmd;
  last_control_output_.desired_orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(0,0,0);
  last_control_output_.desired_heading_rate = 0;
  last_control_output_.diagnostics.controller = "BumpTolerantController[HOLD]";
  last_control_output_.diagnostics.total_mass = total_mass;

  ROS_INFO_THROTTLE(0.05, "[%s]: Holding pose (%.2f/5.0 s) [pos err: %.2f %.2f %.2f]",
                     name_.c_str(), (ros::Time::now() - hold_start_time_).toSec(), pos_error.x(), pos_error.y(), pos_error.z());
}
//}

}  // namespace bump_tolerant_controller

}  // namespace bump_tolerant_controller_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bump_tolerant_controller_plugin::bump_tolerant_controller::BumpTolerantController, mrs_uav_managers::Controller)