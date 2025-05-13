/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nodelet/nodelet.h>

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

/*for storing information about the accelleration state of the uav */
#include <sensor_msgs/Imu.h>

/* custom msgs of MRS group */
#include <mrs_msgs/ControlManagerDiagnostics.h>
// #include <mrs_msgs/Float64Stamped.h>
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

/* for std::atomic */
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

/* for visualization markers */
#include <visualization_msgs/Marker.h>

/* for math*/
#include <math.h>

//}

namespace external_wrench_estimator
{

class ExternalWrenchEstimator : public nodelet::Nodelet {

public:
    virtual void onInit();

private:
  std::atomic<bool> is_initialized_ = false;
  std::string _uav_name_;
  double mass_;
  double drone_radius_;
  Eigen::Matrix3d inertia_;
  Eigen::Vector3d gravity_;
  Eigen::MatrixXd allocation_matrix_;
  int rate_compute_wrench_;

  Eigen::Matrix3d K_I_f_ = Eigen::Matrix3d::Identity() * 2;
  Eigen::Matrix3d K_I_m_ = Eigen::Matrix3d::Identity() * 2;
  Eigen::Vector3d f_ext_hat_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_ext_hat_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_p_omega_ = Eigen::Vector3d::Zero();
  ros::Time prev_time_;

  // Filtri per la forza stimata
  Eigen::Vector3d f_ext_hat_lp_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d f_ext_hat_hp_ = Eigen::Vector3d::Zero();

  mrs_lib::SubscribeHandler<nav_msgs::Odometry>               sh_odometry_;
  mrs_lib::SubscribeHandler<sensor_msgs::Imu>                 sh_imu_;
  std::vector<mrs_lib::SubscribeHandler<std_msgs::Float64>>   sh_motor_speeds_;
  mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>    sh_hwapi_angular_velocity_;

  ros::Publisher pub_wrench_;
  ros::Publisher pub_collision_marker_;
  ros::Publisher pub_filt_norm_;

  ros::Timer timer_compute_wrench_;

  void timerComputeWrench(const ros::TimerEvent& te);
  double force_threshold_;
  double moment_threshold_;

  void estimateCollisionPoint(const Eigen::Vector3d& f_e, const Eigen::Vector3d& m_e,
    const Eigen::Matrix3d& R, const Eigen::Vector3d& r_body);
  int marker_id_ = 0;
};

void ExternalWrenchEstimator::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  mrs_lib::ParamLoader param_loader(nh, "ExternalWrenchEstimator");

  std::vector<double> inertia_vec;
  std::vector<double> gravity_vec;
  std::vector<double> allocation_matrix_vec;

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("mass", mass_);
  param_loader.loadParam("drone_radius", drone_radius_);
  param_loader.loadParam("inertia", inertia_vec);
  param_loader.loadParam("gravity", gravity_vec);
  param_loader.loadParam("allocation_matrix", allocation_matrix_vec);
  param_loader.loadParam("rate/compute_wrench", rate_compute_wrench_);
  param_loader.loadParam("force_threshold", force_threshold_);
  param_loader.loadParam("moment_threshold", moment_threshold_);

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
  
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ExternalWrenchEstimator]: Failed to load non-optional parameters!");
    ros::shutdown();
    return;
  }

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
  
  sh_motor_speeds_.reserve(8);
  for (int i = 0; i < 8; ++i) {
    std::string topic = "motor_speed_in" + std::to_string(i);
    sh_motor_speeds_.emplace_back(shopts, topic);
  }

  pub_wrench_ = nh.advertise<std_msgs::Float64MultiArray>("external_wrench_estimation_out", 1);
  pub_collision_marker_ = nh.advertise<visualization_msgs::Marker>("collision_point_marker", 1);
  pub_filt_norm_ = nh.advertise<std_msgs::Float64MultiArray>("/external_wrench_estimator/force_norms", 1);

  timer_compute_wrench_ = nh.createTimer(ros::Rate(rate_compute_wrench_),&ExternalWrenchEstimator::timerComputeWrench,this);

  ROS_INFO("[ExternalWrenchEstimator]: Initialized");
  is_initialized_ = true;
}

void ExternalWrenchEstimator::timerComputeWrench(const ros::TimerEvent& te) {
  std::vector<double> speeds(8, 0.0);
  for (int i = 0; i < 8; ++i) {
    if (sh_motor_speeds_[i].hasMsg()) {
      speeds[i] = sh_motor_speeds_[i].getMsg()->data;
    } else {
      ROS_WARN_THROTTLE(1.0, "[ExternalWrenchEstimator]: Not received motor speed msg since node launch.");
      return;
    }
  }
  Eigen::VectorXd wrench = allocation_matrix_ * Eigen::Map<Eigen::VectorXd>(speeds.data(), speeds.size());

  if (!sh_odometry_.hasMsg() || !sh_imu_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[ExternalWrenchEstimator]: Missing odometry or IMU data.");
    return;
  }
  auto odom_msg = sh_odometry_.getMsg();
  auto imu_msg = sh_imu_.getMsg();

  Eigen::Vector3d v(
    odom_msg->twist.twist.linear.x,
    odom_msg->twist.twist.linear.y,
    odom_msg->twist.twist.linear.z);

  Eigen::Vector3d w(
    odom_msg->twist.twist.angular.x,
    odom_msg->twist.twist.angular.y,
    odom_msg->twist.twist.angular.z);

  auto q = odom_msg->pose.pose.orientation;
  Eigen::Quaterniond q_body_to_world(q.w, q.x, q.y, q.z);
  Eigen::Matrix3d R = q_body_to_world.toRotationMatrix();

  Eigen::Vector3d a(
    imu_msg->linear_acceleration.x,
    imu_msg->linear_acceleration.y,
    imu_msg->linear_acceleration.z);

  double dt = 0.01;
  if (!prev_time_.isZero()) {
    dt = (te.current_real - prev_time_).toSec();
  }
  prev_time_ = te.current_real;

  // --- Force estimation (acceleration-based) ---
  Eigen::Vector3d a_corrected = R.transpose() * (a - gravity_);
  Eigen::Vector3d f = mass_ * a_corrected;
  f_ext_hat_ += K_I_f_ * (f - f_ext_hat_) * dt;

  // --- HP Filters with different alpha values ---
  double alpha_lp_1 = 0.95;
  double alpha_lp_2 = 0.9;
  double alpha_lp_3 = 0.8;
  double alpha_lp_4 = 0.7;

  // Static variables for each filter
  static Eigen::Vector3d f_ext_hat_lp_1 = Eigen::Vector3d::Zero();
  static Eigen::Vector3d f_ext_hat_lp_2 = Eigen::Vector3d::Zero();
  static Eigen::Vector3d f_ext_hat_lp_3 = Eigen::Vector3d::Zero();
  static Eigen::Vector3d f_ext_hat_lp_4 = Eigen::Vector3d::Zero();

  // Low-pass filters (exponential moving average)
  f_ext_hat_lp_1 = alpha_lp_1 * f_ext_hat_lp_1 + (1.0 - alpha_lp_1) * f_ext_hat_;
  f_ext_hat_lp_2 = alpha_lp_2 * f_ext_hat_lp_2 + (1.0 - alpha_lp_2) * f_ext_hat_;
  f_ext_hat_lp_3 = alpha_lp_3 * f_ext_hat_lp_3 + (1.0 - alpha_lp_3) * f_ext_hat_;
  f_ext_hat_lp_4 = alpha_lp_4 * f_ext_hat_lp_4 + (1.0 - alpha_lp_4) * f_ext_hat_;

  // High-pass filters (difference between signal and low-pass)
  Eigen::Vector3d f_ext_hat_hp_1 = f_ext_hat_ - f_ext_hat_lp_1;
  Eigen::Vector3d f_ext_hat_hp_2 = f_ext_hat_ - f_ext_hat_lp_2;
  Eigen::Vector3d f_ext_hat_hp_3 = f_ext_hat_ - f_ext_hat_lp_3;
  Eigen::Vector3d f_ext_hat_hp_4 = f_ext_hat_ - f_ext_hat_lp_4;

  // --- Moment estimation (momentum-based) ---
  Eigen::Vector3d p_omega = inertia_ * w;
  Eigen::Vector3d m = Eigen::Vector3d::Zero();
  Eigen::Vector3d mg = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_omega_dot = (p_omega - prev_p_omega_) / dt;
  Eigen::Vector3d m_total = m + mg + p_omega.cross(w);
  Eigen::Vector3d m_error = p_omega_dot - m_total - m_ext_hat_;
  m_ext_hat_ += K_I_m_ * m_error * dt;

  // Moment filters (same alphas as force)
  static Eigen::Vector3d m_ext_hat_lp_1 = Eigen::Vector3d::Zero();
  static Eigen::Vector3d m_ext_hat_lp_2 = Eigen::Vector3d::Zero();
  static Eigen::Vector3d m_ext_hat_lp_3 = Eigen::Vector3d::Zero();
  static Eigen::Vector3d m_ext_hat_lp_4 = Eigen::Vector3d::Zero();

  m_ext_hat_lp_1 = alpha_lp_1 * m_ext_hat_lp_1 + (1.0 - alpha_lp_1) * m_ext_hat_;
  m_ext_hat_lp_2 = alpha_lp_2 * m_ext_hat_lp_2 + (1.0 - alpha_lp_2) * m_ext_hat_;
  m_ext_hat_lp_3 = alpha_lp_3 * m_ext_hat_lp_3 + (1.0 - alpha_lp_3) * m_ext_hat_;
  m_ext_hat_lp_4 = alpha_lp_4 * m_ext_hat_lp_4 + (1.0 - alpha_lp_4) * m_ext_hat_;

  Eigen::Vector3d m_ext_hat_hp_1 = m_ext_hat_ - m_ext_hat_lp_1;
  Eigen::Vector3d m_ext_hat_hp_2 = m_ext_hat_ - m_ext_hat_lp_2;
  Eigen::Vector3d m_ext_hat_hp_3 = m_ext_hat_ - m_ext_hat_lp_3;
  Eigen::Vector3d m_ext_hat_hp_4 = m_ext_hat_ - m_ext_hat_lp_4;

  // Publish all force and moment norms for plotting
  std_msgs::Float64MultiArray norm_msg;
  norm_msg.data.resize(18);
  // Force
  norm_msg.data[0] = f_ext_hat_.norm();
  norm_msg.data[1] = f_ext_hat_lp_1.norm();
  norm_msg.data[2] = f_ext_hat_lp_2.norm();
  norm_msg.data[3] = f_ext_hat_lp_3.norm();
  norm_msg.data[4] = f_ext_hat_lp_4.norm();
  norm_msg.data[5] = f_ext_hat_hp_1.norm();
  norm_msg.data[6] = f_ext_hat_hp_2.norm();
  norm_msg.data[7] = f_ext_hat_hp_3.norm();
  norm_msg.data[8] = f_ext_hat_hp_4.norm();
  // Moment
  norm_msg.data[9]  = m_ext_hat_.norm();
  norm_msg.data[10] = m_ext_hat_lp_1.norm();
  norm_msg.data[11] = m_ext_hat_lp_2.norm();
  norm_msg.data[12] = m_ext_hat_lp_3.norm();
  norm_msg.data[13] = m_ext_hat_lp_4.norm();
  norm_msg.data[14] = m_ext_hat_hp_1.norm();
  norm_msg.data[15] = m_ext_hat_hp_2.norm();
  norm_msg.data[16] = m_ext_hat_hp_3.norm();
  norm_msg.data[17] = m_ext_hat_hp_4.norm();
  pub_filt_norm_.publish(norm_msg);

  prev_p_omega_ = p_omega;

  // ----- Publish the wrench estimation -----
  std_msgs::Float64MultiArray wrench_msg;
  wrench_msg.data.resize(6);
  for (int i = 0; i < 3; ++i) {
    wrench_msg.data[i] = f_ext_hat_(i);
    wrench_msg.data[i + 3] = m_ext_hat_(i);
  }
  pub_wrench_.publish(wrench_msg);

  // | ----------- periodical message report ----------- |
  static ros::Time last_ok_msg_time = ros::Time::now();

  double force_norm = f_ext_hat_.norm();
  double moment_norm = m_ext_hat_.norm();

  if (force_norm > force_threshold_ || moment_norm > moment_threshold_) {
    ROS_WARN_STREAM_THROTTLE(0.5, "[ExternalWrenchEstimator]: WARNING! External force or moment above threshold! "
      << "Force: " << force_norm << " N, Moment: " << moment_norm << " Nm");
      Eigen::Vector3d r_body(
        odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z);
      estimateCollisionPoint(f_ext_hat_, m_ext_hat_, R, r_body);
  } else if ((ros::Time::now() - last_ok_msg_time).toSec() > 10.0) {
    ROS_INFO_STREAM("[ExternalWrenchEstimator]: System OK. Force: " << force_norm << " N, Moment: " << moment_norm << " Nm");
    last_ok_msg_time = ros::Time::now();
  }

  // Publish the RViz marker for the drone cylinder
  visualization_msgs::Marker cyl_marker;
  cyl_marker.header.frame_id = "uav1/world_origin";
  cyl_marker.header.stamp = ros::Time::now();
  cyl_marker.ns = "drone_cylinder";
  cyl_marker.id = 9999;
  cyl_marker.type = visualization_msgs::Marker::CYLINDER;
  cyl_marker.action = visualization_msgs::Marker::ADD;
  cyl_marker.scale.x = 2 * 0.47;
  cyl_marker.scale.y = 2 * 0.47;
  cyl_marker.scale.z = 0.155;
  Eigen::Vector3d cyl_center = R * Eigen::Vector3d(0, 0, -0.0195) + Eigen::Vector3d(
    odom_msg->pose.pose.position.x,
    odom_msg->pose.pose.position.y,
    odom_msg->pose.pose.position.z);
  cyl_marker.pose.position.x = cyl_center.x();
  cyl_marker.pose.position.y = cyl_center.y();
  cyl_marker.pose.position.z = cyl_center.z();
  cyl_marker.pose.orientation.w = 1.0;
  cyl_marker.color.a = 0.3;
  cyl_marker.color.r = 0.0;
  cyl_marker.color.g = 0.0;
  cyl_marker.color.b = 1.0;
  pub_collision_marker_.publish(cyl_marker);

}

void ExternalWrenchEstimator::estimateCollisionPoint(const Eigen::Vector3d& f_e, const Eigen::Vector3d& m_e,
  const Eigen::Matrix3d& R, const Eigen::Vector3d& r_body) {
  if (f_e.norm() < 1e-3 || m_e.norm() < 1e-3) {
    ROS_INFO_STREAM("[ExternalWrenchEstimator]: Force or moment too small for point detection");
    return;
  }

  Eigen::Vector3d f_hat = f_e.normalized();
  Eigen::Vector3d m_hat = m_e.normalized();

  Eigen::Vector3d rc_dir = (f_hat.cross(m_hat) / f_hat.dot(f_hat));

  ROS_INFO_STREAM("f_ext_hat_: " << f_e.transpose());
  ROS_INFO_STREAM("m_ext_hat_: " << m_e.transpose());
  ROS_INFO_STREAM("rc_dir: " << rc_dir.transpose());

  for (double alpha = 0.0; alpha > -1.0; alpha -= 0.01) {
    Eigen::Vector3d rc = rc_dir + alpha * f_hat;
    double z_cyl = rc.z() + 0.0195;
    if (rc.head<2>().norm() <= drone_radius_ && std::abs(z_cyl) <= 0.155/2.0) {
      Eigen::Vector3d rc_world = R * rc + r_body;
      ROS_INFO_STREAM("[ExternalWrenchEstimator]: Collision detected at (body): " << rc.transpose());
      ROS_INFO_STREAM("[ExternalWrenchEstimator]: Collision detected at (world): " << rc_world.transpose());

      // ----- Publish the collision point in RViz -----
      visualization_msgs::Marker marker;
      marker.header.frame_id = "uav1/world_origin";
      marker.header.stamp = ros::Time::now();
      marker.lifetime = ros::Duration(0);
      marker.ns = "collision";
      marker.id = marker_id_++;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = rc_world.x();
      marker.pose.position.y = rc_world.y();
      marker.pose.position.z = rc_world.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      pub_collision_marker_.publish(marker);

      // ----- Publish the cylinder in RViz -----
      visualization_msgs::Marker cyl_marker;
      cyl_marker.header.frame_id = "uav1/world_origin";
      cyl_marker.header.stamp = ros::Time::now();
      cyl_marker.ns = "drone_cylinder_contact";
      cyl_marker.id = marker_id_++;
      cyl_marker.type = visualization_msgs::Marker::CYLINDER;
      cyl_marker.action = visualization_msgs::Marker::ADD;
      cyl_marker.scale.x = 2 * 0.47;
      cyl_marker.scale.y = 2 * 0.47;
      cyl_marker.scale.z = 0.155;
      Eigen::Vector3d cyl_center = R * Eigen::Vector3d(0, 0, -0.0195) + r_body;
      cyl_marker.pose.position.x = cyl_center.x();
      cyl_marker.pose.position.y = cyl_center.y();
      cyl_marker.pose.position.z = cyl_center.z();
      cyl_marker.pose.orientation.w = 1.0;
      cyl_marker.color.a = 0.7;
      cyl_marker.color.r = 0.0;
      cyl_marker.color.g = 1.0;
      cyl_marker.color.b = 0.0;
      pub_collision_marker_.publish(cyl_marker);

      return;
    }
  }
}

} // namespace external_wrench_estimator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(external_wrench_estimator::ExternalWrenchEstimator, nodelet::Nodelet);