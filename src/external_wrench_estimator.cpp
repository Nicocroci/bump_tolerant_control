/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nodelet/nodelet.h>

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>
#include <queue>

/* for writing and reading from streams */
#include <fstream>
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/*for storing information about the accelleration state of the uav */
#include <sensor_msgs/Imu.h>

/* custom msgs of MRS group */
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/UavState.h>
// #include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_msgs/FutureTrajectory.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>
#include <mrs_msgs/String.h>

/* for operations with matrices */
#include <Eigen/Dense>

/* for std::atomic */
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

/* for visualization markers */
#include <visualization_msgs/Marker.h>

/* for math*/
#include <math.h>

/* for messagge topic*/
#include <geometry_msgs/Vector3.h>

//}

namespace external_wrench_estimator
{

/* //{ class ExternalWrenchEstimator */
class ExternalWrenchEstimator : public nodelet::Nodelet {

public:
    virtual void onInit();

private:
  std::atomic<bool> is_initialized_ = false;
  
  ros::NodeHandle nh_;

  // | --------------------- uav model parameters --------------------- |

  std::string     _uav_name_;
  mrs_msgs::UavState uav_state_;
  double          mass_;
  double          drone_radius_;
  Eigen::Matrix3d inertia_;
  Eigen::Vector3d gravity_;
  Eigen::MatrixXd allocation_matrix_;
  double          arm_length_;
  double          force_constant_;
  double          moment_constant_;
  Eigen::Matrix3d K_I_f_ = Eigen::Matrix3d::Identity() * 10;
  Eigen::Matrix3d K_I_m_ = Eigen::Matrix3d::Identity() * 36;
  Eigen::VectorXd wrench;
  double filtered_speeds[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // | ---------------------- wrench estimation ---------------------- |

  int       rate_compute_wrench_;
  double    force_threshold_;
  double    moment_threshold_;
  ros::Time prev_time_;
  Eigen::Vector3d f_ext_hat_    = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_ext_hat_    = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_p_omega_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d f_ext_hat_lp_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d f_ext_hat_hp_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_ext_hat_lp_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_ext_hat_hp_ = Eigen::Vector3d::Zero();

  // | ------------------------ subscribers ------------------------ |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_;
  mrs_lib::SubscribeHandler<sensor_msgs::Imu>                    sh_imu_;
  std::vector<mrs_lib::SubscribeHandler<std_msgs::Float64>>      sh_motor_speeds_;
  mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>       sh_hwapi_angular_velocity_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_diagnostics_;
  mrs_lib::SubscribeHandler<std_msgs::Bool>                      sh_switch_command_;
  mrs_lib::SubscribeHandler<mrs_msgs::FutureTrajectory>          sh_predicted_trajectory_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_gps_baro_;

  // | ------------------------ publishers ------------------------- |

  ros::Publisher pub_wrench_;
  ros::Publisher pub_contact_pt_;
  ros::Publisher pub_collision_marker_;
  ros::Publisher pub_filt_wrench_norm_;
  ros::Publisher pub_filt_force_;
  ros::Publisher pub_filt_moment_;
  ros::Publisher pub_contact_event_;
  ros::Publisher pub_controller_status_;
  ros::Publisher pub_activation_disturbance_b_;
  ros::Publisher pub_position;
  ros::Publisher pub_internal_wrench_;

  // | -------------------------- timers -------------------------- |

  ros::Timer timer_compute_wrench_;
  ros::Timer timer_publish_controller_status_;
  ros::Timer timer_debug_switch_;
  ros::Timer contact_point_timer_;
  
  void timerComputeWrench(const ros::TimerEvent& te);
  void timerPublishControllerStatus(const ros::TimerEvent& te);
  void timerDebugSwitch(const ros::TimerEvent& te);
  void timerCheckControllerAndSendPoint(const ros::TimerEvent& te);

  // | ---------------------- main functions ---------------------- |

  Eigen::Vector3d estimateCollisionPoint(const Eigen::Vector3d& f_e, const Eigen::Vector3d& m_e,
    const Eigen::Matrix3d& R, const Eigen::Vector3d& r_body, const std::string& r_body_frame_id);
  int marker_id_ = 0;
  
  // | ----------------- contact point management ----------------- |

  ros::ServiceClient switch_controller_client_;
  std::mutex controller_switch_mutex_;
  bool controller_switch_pending_ = false;
  bool contact_point_sent_ = false;
  std::queue<Eigen::Vector3d> contact_points_queue_;
  std::mutex queue_mutex_; // Add this for thread safety
  ros::Time controller_switch_time_;


};
//}

/* ------------------------------------------------------------ */
/* |                   controller's interface                 | */
/* ------------------------------------------------------------ */

/* //{ initialize() */

void ExternalWrenchEstimator::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  mrs_lib::ParamLoader param_loader(nh, "ExternalWrenchEstimator");

  // | --------------------- uav model parameters --------------------- |

  std::vector<double> inertia_vec;
  std::vector<double> gravity_vec;
  std::vector<double> allocation_matrix_vec;

  // | --------------------- loading parameters ---------------------- |

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("mass", mass_);
  param_loader.loadParam("drone_radius", drone_radius_);
  param_loader.loadParam("inertia", inertia_vec);
  param_loader.loadParam("gravity", gravity_vec);
  param_loader.loadParam("allocation_matrix", allocation_matrix_vec);
  param_loader.loadParam("arm_length", arm_length_);
  param_loader.loadParam("force_constant", force_constant_);
  param_loader.loadParam("moment_constant", moment_constant_);
  param_loader.loadParam("rate/compute_wrench", rate_compute_wrench_);
  param_loader.loadParam("force_threshold", force_threshold_);
  param_loader.loadParam("moment_threshold", moment_threshold_);

  // | ------------------- parameters allocation --------------------- |

  if (allocation_matrix_vec.size() != 32) {
    ROS_ERROR("[ExternalWrenchEstimator]: allocation_matrix must have 32 elements (4x8)");
    ros::shutdown();
    return;
  }
  allocation_matrix_ = Eigen::Map<const Eigen::Matrix<double, 4, 8, Eigen::RowMajor>>(allocation_matrix_vec.data());

  if (inertia_vec.size() != 9) {
    ROS_ERROR("[ExternalWrenchEstimator]: inertia must have 9 elements (3x3)");
    ros::shutdown();
    return;
  } else
  inertia_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(inertia_vec.data());

  gravity_ = Eigen::Vector3d(gravity_vec[0], gravity_vec[1], gravity_vec[2]);
  ROS_INFO("[ExternalWrenchEstimator]: gravity vector: [%.2f, %.2f, %.2f]", gravity_[0], gravity_[1], gravity_[2]);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ExternalWrenchEstimator]: Failed to load non-optional parameters!");
    ros::shutdown();
    return;
  }

  // | ------------------- subscriber handler ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "ExternalWrenchEstimator";
  shopts.no_message_timeout = ros::Duration(.0);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_odometry_               = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_uav_in");
  sh_imu_                    = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, "imu_in");
  sh_hwapi_angular_velocity_ = mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>(shopts, "/uav1/hw_api/angular_velocity");
  sh_control_diagnostics_    = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "/uav1/control_manager/diagnostics");
  sh_switch_command_         = mrs_lib::SubscribeHandler<std_msgs::Bool>(shopts, "switch_cmd_in");
  sh_predicted_trajectory_   = mrs_lib::SubscribeHandler<mrs_msgs::FutureTrajectory>(shopts, "/uav1/control_manager/mpc_tracker/predicted_trajectory");
  sh_gps_baro_               = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "gps_baro_in");

  sh_motor_speeds_.reserve(8);
  for (int i = 0; i < 8; ++i) {
    std::string topic        = "motor_speed_in" + std::to_string(i);
    sh_motor_speeds_.emplace_back(shopts, topic);
  }

  // | ------------------- publisher handler ---------------------- |

  pub_wrench_            = nh.advertise<std_msgs::Float64MultiArray>("external_wrench_estimation_out", 1);
  pub_contact_pt_        = nh.advertise<geometry_msgs::PointStamped>("contact_point_out", 1);
  pub_filt_wrench_norm_  = nh.advertise<std_msgs::Float64MultiArray>("force_moment_norms_filt_out", 1);
  pub_filt_force_        = nh.advertise<geometry_msgs::Vector3>("force_components_filt_out", 1);
  pub_filt_moment_       = nh.advertise<geometry_msgs::Vector3>("moment_components_filt_out", 1);
  pub_collision_marker_  = nh.advertise<visualization_msgs::Marker>("collision_point_marker_out", 1);
  pub_contact_event_     = nh.advertise<std_msgs::Float64>("contact_event_out", 1);
  pub_controller_status_ = nh.advertise<std_msgs::String>("current_controller_out", 1);
  pub_position           = nh.advertise<geometry_msgs::PointStamped>("position_out", 1);
  pub_internal_wrench_    = nh.advertise<std_msgs::Float64MultiArray>("internal_wrench_out", 1);

  // | ------------------------- timer ---------------------------- |

  timer_compute_wrench_            = nh.createTimer(ros::Rate(rate_compute_wrench_),&ExternalWrenchEstimator::timerComputeWrench,this);
  timer_publish_controller_status_ = nh.createTimer(ros::Duration(1.0), &ExternalWrenchEstimator::timerPublishControllerStatus, this);
  contact_point_timer_ = nh.createTimer(ros::Duration(0.02), &ExternalWrenchEstimator::timerCheckControllerAndSendPoint, this, false, false);

  // | ----------------------- parameters sanity check ------------------------ |

  bool enable_debug_switch = false;
  nh.param("enable_debug_switch", enable_debug_switch, false);

  if (enable_debug_switch) {
    timer_debug_switch_ = nh.createTimer(ros::Duration(25.0), &ExternalWrenchEstimator::timerDebugSwitch, this, true);  // oneshot=true
    
  }

  // | ---------------------- Allocation matrix scaling ----------------------- |

  Eigen::Vector4d scaling;
  scaling << 
      force_constant_ * arm_length_,
      force_constant_ * arm_length_,
      force_constant_ * moment_constant_,
      force_constant_;

  Eigen::Matrix<double, 4, 8> base_allocation = allocation_matrix_;
  for (int i = 0; i < 4; ++i) {
      allocation_matrix_.row(i) = base_allocation.row(i) * scaling(i);
  }

  // | --------------------------- Controller switch --------------------------- |

  switch_controller_client_ = nh_.serviceClient<mrs_msgs::String>("/uav1/control_manager/switch_controller");

  // | ------------------------- uav state subscriber -------------------------- |
 
  ROS_INFO("[ExternalWrenchEstimator]: Initialized");
  is_initialized_ = true;
}

//}

/* //{ timerComputeWrench(const ros::TimerEvent& te) */

void ExternalWrenchEstimator::timerComputeWrench(const ros::TimerEvent& te) {

  if (sh_switch_command_.hasMsg() && sh_switch_command_.getMsg()->data == true && sh_control_diagnostics_.getMsg()->active_controller != "MpcController") {
    static ros::ServiceClient switch_controller_client = 
    ros::NodeHandle().serviceClient<mrs_msgs::String>("/uav1/control_manager/switch_controller");
    mrs_msgs::String srv;
    srv.request.value = "MpcController";
    if (switch_controller_client.call(srv)) {
      ROS_INFO_STREAM("[ExternalWrenchEstimator]: Successfully switched to MpcController");
    } else {
      ROS_ERROR_STREAM("[ExternalWrenchEstimator]: Failed to switch controller");
    }
  }

  std::vector<double> speeds(8, 0.0);

  // | ------------ check if all motor speed messages are received ------------ |
  //double alpha = 0.95;
  for (int i = 0; i < 8; ++i) {
    if (sh_motor_speeds_[i].hasMsg()) {
      double velocity = sh_motor_speeds_[i].getMsg()->data;
      speeds[i] = velocity * velocity * 100 * 100; 
      //double thrust = velocity * velocity * 100 * 100; 
      //filtered_speeds[i] = alpha * filtered_speeds[i] + (1.0 - alpha) * thrust;
      //speeds[i] = filtered_speeds[i];
    } else {
      ROS_WARN_THROTTLE(1.0, "[ExternalWrenchEstimator]: Not received motor speed msg since node launch.");
      return;
    }
  }

  // | ------------------- compute the internal wrench ---------------------- |

  wrench = allocation_matrix_ * Eigen::Map<Eigen::VectorXd>(speeds.data(), speeds.size());
  wrench[3] += 2; //offset notce during the simulation
  std_msgs::Float64MultiArray iw_msg;
  iw_msg.data.resize(4);
  for (int i = 0; i < 4; ++i) {
    iw_msg.data[i] = wrench[i];
  }
  pub_internal_wrench_.publish(iw_msg);

  // | ------------------- aquire odometry and IMU data -------------------- |

  if (!sh_odometry_.hasMsg() || !sh_imu_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[ExternalWrenchEstimator]: Missing odometry or IMU data.");
    return;
  }
  auto odom_msg = sh_odometry_.getMsg();
  auto imu_msg  = sh_imu_.getMsg();

  // | --------------- get drone's position and orientation ---------------- |

  auto q            = odom_msg->pose.pose.orientation;
  Eigen::Quaterniond q_body_to_world(q.w, q.x, q.y, q.z);
  Eigen::Matrix3d R = q_body_to_world.toRotationMatrix();

  // | ------------------- get drone's velocity ---------------------------- |

  Eigen::Vector3d v(
    odom_msg->twist.twist.linear.x,
    odom_msg->twist.twist.linear.y,
    odom_msg->twist.twist.linear.z);

  Eigen::Vector3d w(
    odom_msg->twist.twist.angular.x,
    odom_msg->twist.twist.angular.y,
    odom_msg->twist.twist.angular.z);

  // | ------------------- get drone's acceleration ----------------------- |

  Eigen::Vector3d a(
    imu_msg->linear_acceleration.x,
    imu_msg->linear_acceleration.y,
    imu_msg->linear_acceleration.z);
  // | -------------------------- get dt ------------------------------- |

  double dt = 0.01;
  if (!prev_time_.isZero()) {
    dt = (te.current_real - prev_time_).toSec();
  }
  prev_time_ = te.current_real;
  
  // | --------- Force estimation (acceleration-based) ------------------- |

  //Eigen::Vector3d a_corrected = R.transpose() * (a - gravity_);
  Eigen::Vector3d a_corrected = a;
  Eigen::Vector3d f_body        (0, 0, wrench[3]);
  Eigen::Vector3d f           = mass_ * a_corrected - f_body;
  f_ext_hat_                 += K_I_f_ * (f - f_ext_hat_) * dt;

  // | ------------- Force HP and LP filter ----------------------------- |

  double alpha_lp_f_ = 0.98;
  f_ext_hat_lp_      = alpha_lp_f_ * f_ext_hat_lp_ + (1.0 - alpha_lp_f_) * f_ext_hat_;
  f_ext_hat_hp_      = f_ext_hat_ - f_ext_hat_lp_;

  // | ----------- Moment estimation (momentum-based) ------------------- |

  Eigen::Vector3d p_omega     = inertia_ * w;
  Eigen::Vector3d m           = wrench.head<3>();
  //Eigen::Vector3d m           = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_omega_dot = (p_omega - prev_p_omega_) / dt;
  //no offset between center and center of mass so no monment due to gravitanioal field
  //Eigen::Vector3d m_total     = m + mg + p_omega.cross(w);
  Eigen::Vector3d m_total     = m + p_omega.cross(w);
  Eigen::Vector3d m_error     = p_omega_dot - m_total - m_ext_hat_;
  m_ext_hat_                 += K_I_m_ * m_error * dt;
  prev_p_omega_               = p_omega;
  if (m_ext_hat_.hasNaN()){
    ROS_ERROR("[ExternalWrenchEstimator]: m_ext_hat_ has NaN values, resetting to zero.");
    ROS_ERROR_STREAM("[ExternalWrenchEstimator]: p_omega_dot: " << p_omega_dot.transpose());
    ROS_ERROR_STREAM("[ExternalWrenchEstimator]: dt: " << dt);

    m_ext_hat_.setZero();
  }

  // | ------------- Moment HP and LP filter ---------------------------- |

  double alpha_lp_m_ = 0.85;
  m_ext_hat_lp_      = alpha_lp_m_ * m_ext_hat_lp_ + (1.0 - alpha_lp_m_) * m_ext_hat_;
  m_ext_hat_hp_      = m_ext_hat_ - m_ext_hat_lp_;

  // | --------------- Publish external wrench -------------------------- |

  std_msgs::Float64MultiArray wrench_msg;
  wrench_msg.data.resize(6);
  for (int i = 0; i < 3; ++i) {
    wrench_msg.data[i]     = f_ext_hat_(i);
    wrench_msg.data[i + 3] = m_ext_hat_(i);
  }
  pub_wrench_.publish(wrench_msg);
  ROS_INFO_THROTTLE(1.0, "[ExternalWrenchEstimator]: Published yaw: %.2f", mrs_lib::AttitudeConverter(odom_msg->pose.pose.orientation).getYaw());

  // | --- Publish both force and moment estimates in the same plot --- |

  std_msgs::Float64MultiArray fm_msg;
  fm_msg.data.resize(2);
  fm_msg.data[0] = f_ext_hat_hp_.norm();
  fm_msg.data[1] = m_ext_hat_hp_.norm()*10;
  pub_filt_wrench_norm_.publish(fm_msg);

  // | --------------- Publish all force components --------------------- |

  geometry_msgs::Vector3 force_msg;
  force_msg.x = f_ext_hat_hp_.x();
  force_msg.y = f_ext_hat_hp_.y();
  force_msg.z = f_ext_hat_hp_.z();
  pub_filt_force_.publish(force_msg);

  // | --------------- Publish all moment components -------------------- |

  geometry_msgs::Vector3 moment_msg;
  moment_msg.x = m_ext_hat_hp_.x();
  moment_msg.y = m_ext_hat_hp_.y();
  moment_msg.z = m_ext_hat_hp_.z();
  pub_filt_moment_.publish(moment_msg);

  // | --------------- periodical message report ------------------------ |

  static ros::Time last_ok_msg_time = ros::Time::now();

  // | -------- check if the force norm is above the threshold ---------- |

  double force_norm  = f_ext_hat_hp_.norm();
  double moment_norm = m_ext_hat_hp_.norm();

  static bool collision_active = false;
  static Eigen::Vector3d last_collision_point = Eigen::Vector3d::Zero();

  std_msgs::Float64 contact_msg;

  auto position_msg = sh_gps_baro_.getMsg();
  
  Eigen::Vector3d r_body(
      position_msg->pose.pose.position.x, 
      position_msg->pose.pose.position.y,
      position_msg->pose.pose.position.z);

  ROS_INFO_THROTTLE(1.0, "[ExternalWrenchEstimator]: Drone position: (%.2f, %.2f, %.2f)", 
    r_body.x(), r_body.y(), r_body.z());

  geometry_msgs::PointStamped pos_msg;
  pos_msg.header.stamp = ros::Time::now();
  pos_msg.header.frame_id = "uav1/world_origin";
  pos_msg.point.x = r_body.x();
  pos_msg.point.y = r_body.y();
  pos_msg.point.z = r_body.z();
  pub_position.publish(pos_msg);

  if ((abs(m_ext_hat_hp_.z()) > 1.0 || abs(f_ext_hat_hp_.head<2>().norm()) > 5.0) && !collision_active) {

    collision_active = true;

    {
      std::lock_guard<std::mutex> lock(controller_switch_mutex_);
      
      auto diag_msg = sh_control_diagnostics_.getMsg();
      std_msgs::String msg;
      msg.data = diag_msg->active_controller;
      
      if (msg.data != "BumpTolerantController" && msg.data != "EmergencyController") {
        Eigen::Vector3d contact_point_body = estimateCollisionPoint(f_ext_hat_hp_, m_ext_hat_hp_, R, r_body, odom_msg->header.frame_id);
        
        // Add to queue instead of single point
        {
          std::lock_guard<std::mutex> queue_lock(queue_mutex_);
          contact_points_queue_.push(contact_point_body);
        }
        
        // Only request controller switch if it's not already pending
        if (!controller_switch_pending_) {
          controller_switch_pending_ = true;
          controller_switch_time_ = ros::Time::now();
          contact_point_timer_.start();
          
          mrs_msgs::String srv;
          srv.request.value = "BumpTolerantController";
          
          if (switch_controller_client_.call(srv)) {
            ROS_INFO("[ExternalWrenchEstimator]: Requested switch to BumpTolerantController");
          } else {
            ROS_ERROR("[ExternalWrenchEstimator]: Failed to switch controller");
            controller_switch_pending_ = false;
          }
          
          static ros::ServiceClient stop_wp = ros::NodeHandle().serviceClient<std_srvs::Trigger>("/uav1/example_waypoint_flier/stop_waypoints_following_in");
          std_srvs::Trigger srv_stop;
          stop_wp.call(srv_stop);
        }
      }
      
      contact_msg.data = 10.0;
      pub_contact_event_.publish(contact_msg);
    }
  }
  else {
    collision_active = false;
    contact_msg.data = 0.0;    // Publish 0 when no contact is detected
    pub_contact_event_.publish(contact_msg);
    ROS_INFO_THROTTLE(10.0, "[ExternalWrenchEstimator]: No contact detected");
  }

  // | --------------- Pub marker for collision detection --------------- |

  visualization_msgs::Marker cyl_marker;
  cyl_marker.header.frame_id    = "uav1/world_origin";
  cyl_marker.header.stamp       = ros::Time::now();
  cyl_marker.ns                 = "drone_cylinder";
  cyl_marker.id                 = 9999;
  cyl_marker.type               = visualization_msgs::Marker::CUBE;
  cyl_marker.action             = visualization_msgs::Marker::ADD;
  cyl_marker.scale.x            = 0.5;
  cyl_marker.scale.y            = 0.5;
  cyl_marker.scale.z            = 0.5;
  Eigen::Vector3d cyl_center    = R * Eigen::Vector3d(0, 0, -0.0195) + r_body;
  cyl_marker.pose.position.x    = cyl_center.x();
  cyl_marker.pose.position.y    = cyl_center.y();
  cyl_marker.pose.position.z    = cyl_center.z();
  cyl_marker.pose.orientation.x = odom_msg->pose.pose.orientation.x;
  cyl_marker.pose.orientation.y = odom_msg->pose.pose.orientation.y;
  cyl_marker.pose.orientation.z = odom_msg->pose.pose.orientation.z;
  cyl_marker.pose.orientation.w = odom_msg->pose.pose.orientation.w;
  cyl_marker.color.a            = 0.3;
  cyl_marker.color.r            = 0.0;
  cyl_marker.color.g            = 0.0;
  cyl_marker.color.b            = 1.0;
  pub_collision_marker_.publish(cyl_marker);

}

//}

/* //{ timerPublishControllerStatus() */

void ExternalWrenchEstimator::timerPublishControllerStatus(const ros::TimerEvent& /*te*/) {
  
  // Use diagnostics topic instead of service
  if (!sh_control_diagnostics_.hasMsg()) {
    ROS_WARN_THROTTLE(10.0, "[ExternalWrenchEstimator]: No control manager diagnostics available");
    return;
  }
  
  auto diag_msg = sh_control_diagnostics_.getMsg();
  
  std_msgs::String msg;
  msg.data = diag_msg->active_controller;
  pub_controller_status_.publish(msg);
  ROS_INFO_THROTTLE(1.0, "[ExternalWrenchEstimator]: Current controller: %s", msg.data.c_str());
}

//}

/* //{ timerDebugSwitch() */

void ExternalWrenchEstimator::timerDebugSwitch(const ros::TimerEvent& /*te*/) {
  
  ROS_WARN("[ExternalWrenchEstimator]: DEBUG - 10 seconds passed, switching to bump controller NOW!");
  
  {
    std::lock_guard<std::mutex> lock(controller_switch_mutex_);
    
    {
      std::lock_guard<std::mutex> queue_lock(queue_mutex_);
      contact_points_queue_.push(Eigen::Vector3d(0.335, -0.335, 0.0));
    }
    
    contact_point_sent_ = false;
    controller_switch_pending_ = true;
    controller_switch_time_ = ros::Time::now();

    contact_point_timer_.start();
    
    mrs_msgs::String srv;
    srv.request.value = "BumpTolerantController";
    
    if (switch_controller_client_.call(srv)) {
      ROS_INFO("[ExternalWrenchEstimator]: DEBUG - Requested switch to BumpTolerantController");
    } else {
      ROS_ERROR("[ExternalWrenchEstimator]: DEBUG - Failed to switch controller");
      controller_switch_pending_ = false;
    }

    static ros::ServiceClient stop_wp = ros::NodeHandle().serviceClient<std_srvs::Trigger>("/uav1/example_waypoint_flier/stop_waypoints_following_in");
    std_srvs::Trigger srv_stop;
    stop_wp.call(srv_stop);
    
    std_msgs::Float64 contact_msg;
    contact_msg.data = 5.0;
    pub_contact_event_.publish(contact_msg);
  }
}

//}

/* //{ timerCheckControllerAndSendPoint() */

void ExternalWrenchEstimator::timerCheckControllerAndSendPoint(const ros::TimerEvent& /*te*/) {
  std::lock_guard<std::mutex> lock(controller_switch_mutex_);
  
  if (controller_switch_pending_) {
    if (sh_control_diagnostics_.hasMsg() && 
        sh_control_diagnostics_.getMsg()->active_controller == "BumpTolerantController") {
      
      if ((ros::Time::now() - controller_switch_time_).toSec() > 0.05) {
        // Check if we have points to send
        std::lock_guard<std::mutex> queue_lock(queue_mutex_);
        
        if (!contact_points_queue_.empty()) {
          // Get the first point (FIFO)
          Eigen::Vector3d point = contact_points_queue_.front();
          contact_points_queue_.pop();
          
          // Create and send the message
          geometry_msgs::PointStamped contact_pt_msg;
          contact_pt_msg.header.stamp = ros::Time::now();
          contact_pt_msg.header.frame_id = "uav1/world_origin";
          contact_pt_msg.point.x = point.x();
          contact_pt_msg.point.y = point.y();
          contact_pt_msg.point.z = point.z();
          
          pub_contact_pt_.publish(contact_pt_msg);
          
          ROS_INFO("[ExternalWrenchEstimator]: Sent contact point: [%.3f, %.3f, %.3f] (%zu more in queue)", 
                  point.x(), point.y(), point.z(), contact_points_queue_.size());
          
          // If we have more points, keep the timer running
          if (contact_points_queue_.empty()) {
            contact_point_sent_ = true;
            controller_switch_pending_ = false;
            contact_point_timer_.stop();
          } else {
            // Set a timeout before sending the next point (100ms)
            controller_switch_time_ = ros::Time::now(); 
          }
        } else {
          // Queue is empty, stop the timer
          contact_point_sent_ = true;
          controller_switch_pending_ = false;
          contact_point_timer_.stop();
        }
      }
    } else if ((ros::Time::now() - controller_switch_time_).toSec() > 2.0) {
      // Timeout waiting for controller switch
      ROS_WARN("[ExternalWrenchEstimator]: Timeout waiting for BumpTolerantController!");
      controller_switch_pending_ = false;
      contact_point_timer_.stop();
      
      // Clear the queue
      std::lock_guard<std::mutex> queue_lock(queue_mutex_);
      std::queue<Eigen::Vector3d> empty;
      std::swap(contact_points_queue_, empty);
    }
  }
}

//}

/* //{ EstimateCollisionPoint() */

Eigen::Vector3d ExternalWrenchEstimator::estimateCollisionPoint(const Eigen::Vector3d& f_e, const Eigen::Vector3d& m_e,
  const Eigen::Matrix3d& R, const Eigen::Vector3d& r_body, const std::string& r_body_frame_id) {

    if (f_e.norm() < 1e-3 || m_e.norm() < 1e-3) {
    ROS_WARN_STREAM("[ExternalWrenchEstimator][DEBUG]: Force or moment too small for point detection. |f_e|=" << f_e.norm() << " |m_e|=" << m_e.norm());
    return Eigen::Vector3d::Zero();
  }

  // | --------------- Collision plane detection ------------------------ |

  Eigen::Vector3d f_hat  = f_e.normalized();
  Eigen::Vector3d rc_dir = f_hat.cross(m_e) / f_e.squaredNorm();

  ROS_INFO_STREAM("[ExternalWrenchEstimator][DEBUG]: f_e = " << f_e.transpose());
  ROS_INFO_STREAM("[ExternalWrenchEstimator][DEBUG]: m_e = " << m_e.transpose());
  ROS_INFO_STREAM("[ExternalWrenchEstimator][DEBUG]: f_hat = " << f_hat.transpose());
  ROS_INFO_STREAM("[ExternalWrenchEstimator][DEBUG]: rc_dir = " << rc_dir.transpose());
  ROS_INFO_STREAM("[ExternalWrenchEstimator][DEBUG]: f_e.cross(m_e).norm() = " << f_e.cross(m_e).norm());

  // | --------------- Create the line marker -------------------------- |

  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = "uav1/world_origin"; // <-- select the frame of the body
  line_marker.header.stamp    = ros::Time::now();
  line_marker.ns              = "collision_line";
  line_marker.id              = marker_id_++;
  line_marker.type            = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action          = visualization_msgs::Marker::ADD;
  line_marker.scale.x         = 0.01;
  line_marker.color.a         = 1.0;
  line_marker.color.r         = 0.0;
  line_marker.color.g         = 1.0;
  line_marker.color.b         = 1.0;

  // | --------------- Collision point detection ----------------------- |

  double alpha_start = -2.0 * drone_radius_;
  double alpha_end   = 0.0;
  double alpha_step  = 0.02;
  double tol         = 0.01;

  Eigen::Vector3d      rc;
  Eigen::Vector3d      rc_world;
  geometry_msgs::Point p;

  Eigen::Vector2d rc_dir_xy = rc_dir.head<2>();
  Eigen::Vector2d f_hat_xy  = f_hat.head<2>();

  for (double alpha = alpha_start; alpha <= alpha_end; alpha += alpha_step) {
    //rc = rc_dir - alpha * f_hat;
    //rc = rc_dir + alpha * f_hat; VERIFY THE SIGN
    
    Eigen::Vector2d rc_xy = rc_dir_xy + alpha * f_hat_xy;
    Eigen::Vector3d rc(rc_xy.x(), rc_xy.y(), 0.0);
    double z_cyl = rc.z() + 0.0195;
    double r_xy = rc.head<2>().norm();
    //ROS_INFO_STREAM("[ExternalWrenchEstimator][DEBUG]: alpha=" << alpha << " rc=" << rc.transpose() << " r_xy=" << r_xy << " z_cyl=" << z_cyl);

    // | --------------- Create the line marker ------------------------ |

    rc_world = R * rc + r_body;
    rc_world.z() = r_body.z() + 0.0195;
    p.x = rc_world.x();
    p.y = rc_world.y();
    p.z = rc_world.z();
    line_marker.points.push_back(p);

    // | --------------- Check if the point is within the cylinder --------------- |

    /*if (rc.head<2>().norm() <= drone_radius_ && std::abs(z_cyl) <= 0.155/2.0) {
      Eigen::Vector3d rc_world = R * rc + r_body;
      ROS_INFO_STREAM("[ExternalWrenchEstimator][DEBUG]: Found collision point at alpha=" << alpha << ", rc (body): " << rc.transpose());
      ROS_INFO_STREAM("[ExternalWrenchEstimator][DEBUG]: rc_world: " << rc_world.transpose());
      pub_collision_marker_.publish(line_marker);*/

    double half_side = drone_radius_; // metà lato del quadrato

    bool on_square_edge =
      (
        (std::abs(rc_xy.x() - half_side) < tol || std::abs(rc_xy.x() + half_side) < tol) &&
        (rc_xy.y() >= -half_side - tol && rc_xy.y() <= half_side + tol)
      )
      ||
      (
        (std::abs(rc_xy.y() - half_side) < tol || std::abs(rc_xy.y() + half_side) < tol) &&
        (rc_xy.x() >= -half_side - tol && rc_xy.x() <= half_side + tol)
      );

    if (on_square_edge) {
      ROS_INFO_STREAM("[ExternalWrenchEstimator][DEBUG]: Found horizontal collision point at alpha=" << alpha << ", rc (body): " << rc.transpose());
      ROS_INFO_STREAM("[ExternalWrenchEstimator][DEBUG]: rc_world: " << rc_world.transpose());
      pub_collision_marker_.publish(line_marker);

      // | --------------- Marker for collision detection ------------------------ |

      visualization_msgs::Marker marker;
      marker.header.frame_id    = "uav1/world_origin";
      marker.header.stamp       = ros::Time::now();
      marker.lifetime           = ros::Duration(0);
      marker.ns                 = "collision";
      marker.id                 = marker_id_++;
      marker.type               = visualization_msgs::Marker::SPHERE;
      marker.action             = visualization_msgs::Marker::ADD;
      marker.pose.position.x    = rc_world.x();
      marker.pose.position.y    = rc_world.y();
      marker.pose.position.z    = rc_world.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x            = 0.05;
      marker.scale.y            = 0.05;
      marker.scale.z            = 0.05;
      marker.color.a            = 1.0;
      marker.color.r            = 1.0;
      marker.color.g            = 0.0;
      marker.color.b            = 0.0;
      pub_collision_marker_.publish(marker);

      // | --------------- Green cylinder marker ------------------------------- |

      visualization_msgs::Marker cyl_marker;
      cyl_marker.header.frame_id    = "uav1/world_origin";
      cyl_marker.header.stamp       = ros::Time::now();
      cyl_marker.ns                 = "drone_cylinder_contact";
      cyl_marker.id                 = marker_id_++;
      cyl_marker.type               = visualization_msgs::Marker::CYLINDER;
      cyl_marker.action             = visualization_msgs::Marker::ADD;
      cyl_marker.scale.x            = 2 * 0.47;
      cyl_marker.scale.y            = 2 * 0.47;
      cyl_marker.scale.z            = 0.155;
      Eigen::Vector3d cyl_center    = R * Eigen::Vector3d(0, 0, -0.0195) + r_body;
      cyl_marker.pose.position.x    = cyl_center.x();
      cyl_marker.pose.position.y    = cyl_center.y();
      cyl_marker.pose.position.z    = cyl_center.z();
      cyl_marker.pose.orientation.w = 1.0;
      cyl_marker.color.a            = 0.7;
      cyl_marker.color.r            = 0.0;
      cyl_marker.color.g            = 1.0;
      cyl_marker.color.b            = 0.0;
      pub_collision_marker_.publish(cyl_marker);

      return rc;
    }
  }
  ROS_WARN_STREAM("[ExternalWrenchEstimator][DEBUG]: No valid collision point found on convex hull.");
  pub_collision_marker_.publish(line_marker);
  return Eigen::Vector3d::Zero();

}
//}


} // namespace external_wrench_estimator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(external_wrench_estimator::ExternalWrenchEstimator, nodelet::Nodelet);