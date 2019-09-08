////////////////// Improvement
////// - Force filter
////// - Friction model


#include <franka_controllers/hybrid_force_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include "pseudo_inversion.h"

namespace franka_controllers {
bool HybridForceController::init(hardware_interface::RobotHW* robot_hw,
                                 ros::NodeHandle& node_handle) {
  // Read .yaml parameters
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id))
    return false;
//
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names))
    return false;

  double publish_rate;
  if (!node_handle.getParam("publish_rate", publish_rate))
    return false;

  // Joint interfaces and handles
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr)  return false;
  for (size_t i = 0; i < 7; ++i)
    joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) return false;
  state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
    state_interface->getHandle(arm_id + "_robot"));

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) return false;
  model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
    model_interface->getHandle(arm_id + "_model"));

  // Initialize value
  stiffness_m_.setZero();
  damping_m_.setZero();
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  target_stiffness_m_.setZero();
  target_damping_m_.setZero();
  S_ = Eigen::Matrix<double, 6, 6>::Identity();
  S_(2, 2) = 0;

  // Dynamic Reconfigure
  dynamic_server_node_ = ros::NodeHandle("dynamic_server_node");
  dynamic_server_param_ = std::make_unique<
    dynamic_reconfigure::Server<franka_controllers::hybrid_paramConfig>>(
    dynamic_server_node_);
  dynamic_server_param_->setCallback(
    boost::bind(&HybridForceController::hybridParamCallback, this, _1, _2));

  // Realtime Publisher
  publisher_.init(node_handle, "data", 1);

  // Init filter
  filter_size_ = 5;
  filter_buffer_ = std::make_unique<double[]>(filter_size_ * 6);
  std::fill(&filter_buffer_.get()[0], &filter_buffer_.get()[filter_size_ * 6], 0);

  // Friction model
  // friction_coeffs_ << 0.245, -0.107  , 0.0675,
  //                     0.15 , -0.155  , 0.201 ,
  //                     0.184, -0.06887, 0.0377,
  //                     0.361, -0.249  , 0.283 ,
  //                     0.272,  0.0034 , 0.096 ,
  //                     0.157,  0.0918 , 0     ,
  //                     0.21 , -0.0115 , 0.0647;

  friction_coeffs_ << 0.546,  5.118  , 0.0395,
                      0.872,  9.066  , 0.0259,
                      0.641, 10.136  ,-0.0461,
                      1.279,  5.590  , 0.0362,
                      0.839,  8.347  , 0.0262,
                      0.303,  17.133 ,-0.0210,
                      0.565, 10.336  , 0.0036;

  return true;
}

void HybridForceController::starting(const ros::Time&) {
  elapsed_time_ = 0.0;
  franka::RobotState initial_state = state_handle_->getRobotState();

  // Initial external force
  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_ext(initial_state.O_F_ext_hat_K.data());
  force_ext_initial_ = force_ext;
  force_error_.setZero();
  force_prev_.setZero();

  // Initial target position
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  force_filter_ = force_ext;

  // Initial target twist
  twist_d_.setZero();
  target_twist_.setZero();
}

void HybridForceController::update(const ros::Time&, const ros::Duration& period) {
  elapsed_time_ += period.toSec();

  franka::RobotState robot_state = state_handle_->getRobotState();
  // Update filter buffer
  // updateFitlerValue(robot_state);

  // Jacobian matrix
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // Coriolis
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_ext(robot_state.O_F_ext_hat_K.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());
  transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  Eigen::Vector3d position(transform_.translation());
  Eigen::Quaterniond orientation(transform_.linear());

  Eigen::VectorXd desired_force_torque(6), force_control(6), force_filter(6),
      tau_force(7), tau_task(7), tau_cmd(7), tau_fric(7);

  // Friction torque
  // for (size_t i = 0; i < 7; ++i){
  //   tau_fric(i) = friction_coeffs_(i, 2) * dq(i) +
  //                 friction_coeffs_(i, 0) * fabs(dq(i)) / dq(i) +
  //                 friction_coeffs_(i, 1);
  // }
  //
  // std::cout <<tau_fric <<std::endl;

  // Force control
  desired_force_torque.setZero();
  // desired_force_torque(2) = -desired_force_;
  desired_force_torque(2) = -desired_force_;
  desired_force_torque(3) = -desired_force_ * position(1);
  desired_force_torque(4) =  desired_force_ * position(0);

  // std::cout << desired_force_torque <<std::endl;

  // Filter value
  // force_filter = getFitlerValue();
  double gain = 0.8;
  force_filter_ = gain * force_ext + (1 - gain) * force_filter_;

  // if ((force_ext(2) - force_prev_(2)) / 0.001 < -200)
  //   force_error_.setZero();

  double ki{ki_f_};
  ki = ki_f_ / (1 + std::pow(200, std::fabs(desired_force_torque(2) - force_filter_(2)  + force_ext_initial_(2)) - 0.2 * desired_force_));
  // std::cout<<ki<<std::endl;


  // force_error_ = force_error_ + ki_f_ * period.toSec() * (desired_force_torque - force_ext + force_ext_initial_);
  // force_control = desired_force_torque + kp_f_ * (desired_force_torque - force_ext + force_ext_initial_) +
  //                  force_error_ - kd_f_ * jacobian * dq;

  // Filter ver
  // force_filter_ = force_ext;
  force_error_ = force_error_ + ki * period.toSec() * (desired_force_torque - force_filter_ + force_ext_initial_);
  force_error_ = force_error_ / (1 + std::pow(200, std::fabs(desired_force_torque(2) - force_filter_(2)  + force_ext_initial_(2)) - 0.5 * desired_force_));

  // force_error_ = force_error_ + ki_f_ * period.toSec() * (desired_force_torque - force_filter_ + force_ext_initial_);
  force_control = desired_force_torque + kp_f_ * (desired_force_torque - force_filter_ + force_ext_initial_) +
                   force_error_ - kd_f_ * jacobian * dq;


  force_control << 0, 0, force_control(2), 0, 0, 0;
  tau_force = jacobian.transpose() * force_control;

  // Motion control
  Eigen::Matrix<double, 6, 1> error;
  orientation_d_.coeffs() << 1.0, 0.0, 0.0, 0.0;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }

  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // Cartesian PD control
  // Eigen::MatrixXd SJ_inv;
  // pseudoInverse(S_ * jacobian, SJ_inv);

  // for (size_t i = 0; i < 7; ++i){
  //   std::cout << std::endl;
  //   for (size_t j = 0; j < 6; ++j)
  //     std::cout << SJ_inv(i, j) << ", ";
  // }

  // Eigen::Matrix<double, 6, 7> a = S_ * jacobian;
  // std::cout << SJ_inv - a.transpose() << std::endl;

  tau_task << jacobian.transpose() *
                  (-stiffness_m_ * error - damping_m_ * (jacobian * dq - twist_d_));

  // tau_task << SJ_inv *
  //                 (-stiffness_m_ * error - damping_m_ * (jacobian * dq - twist_d_));

  // COmmand torque
  tau_cmd = tau_force + tau_task + coriolis;

  // With friction compensate
  // tau_cmd = tau_force + tau_task + coriolis + tau_fric;

  tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
  for (size_t i = 0; i < 7; ++i)
    joint_handles_[i].setCommand(tau_cmd(i));

  // for (size_t i = 0; i < 7; ++i)
  // {
  //   if (i == 0) std::cout << "\n";
  //   std::cout << tau_cmd(i) << std::endl;
  // }

  // Realtime publisher
  if (rate_trigger_() && publisher_.trylock()) {
    publisher_.msg_.time = elapsed_time_;
    publisher_.msg_.x = force_filter_(2) - force_ext_initial_(2);
    // publisher_.msg_.x = force_filter(2) - force_ext_initial_(2);
    publisher_.msg_.y = desired_force_torque(2);
    publisher_.msg_.z = force_error_(2);
    // publisher_.msg_.z = (force_ext(2) - force_prev_(2)) / 0.001;

    // std::array<double, 7> tau_j = robot_state.tau_J;
    // std::array<double, 7> tau_error;
    // double error_rms{0.0};
    //
    for (size_t i = 0; i < 7; ++i) {
      // publisher_.msg_.data1[i] = last_tau_d_(i);
      publisher_.msg_.data1[i] = tau_cmd(i);
    }

    publisher_.unlockAndPublish();
  }

  // Update dynamic parameters
  updateDynamicReconfigure();
  force_prev_ << force_ext;
}

void HybridForceController::stopping(const ros::Time&) {}

Eigen::Matrix<double, 7, 1> HybridForceController::saturateTorqueRate(
     const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
     const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

// Eigen::Matrix<double, 6, 6> HybridForceController::adjoint(Eigen::Affine3d transform){}

void HybridForceController::updateDynamicReconfigure() {
  desired_force_ = filter_params_ * target_force_ + (1 - filter_params_) * desired_force_;
  stiffness_m_ = filter_params_ * target_stiffness_m_ + (1 - filter_params_) * stiffness_m_;
  damping_m_ = filter_params_ * target_damping_m_ + (1 - filter_params_) * damping_m_;
  twist_d_ = filter_params_ * target_twist_ + (1 - filter_params_) * twist_d_;
  for (size_t i =0; i<3; ++i)
    position_d_(i) += twist_d_(i) * 0.001;
}

void HybridForceController::hybridParamCallback(franka_controllers::hybrid_paramConfig& config,
                         uint32_t level) {
  // Force gains
  kp_f_ = config.kp_f;
  ki_f_ = config.ki_f;
  kd_f_ = config.kd_f;

  target_twist_ << config.v_x, config.v_y, 0, 0, 0, 0;

  // Motion gains
  target_stiffness_m_.setZero();
  target_stiffness_m_.topLeftCorner(3, 3)
      << config.trans_stiffness * Eigen::Matrix3d::Identity();
  target_stiffness_m_.bottomRightCorner(3, 3)
      << config.rot_stiffness * Eigen::Matrix3d::Identity();
  target_stiffness_m_(2,2) = 0.0;

  // Damping ratio = 1
  target_damping_m_.setZero();
  target_damping_m_.topLeftCorner(3,3)
      << 2 * sqrt(config.trans_stiffness) * Eigen::Matrix3d::Identity();
  target_damping_m_.bottomRightCorner(3,3)
      << 2 * sqrt(config.rot_stiffness) * Eigen::Matrix3d::Identity();
  target_damping_m_(2,2) = 0.0;

  // Target force
  target_force_ = config.desired_force;
}


// Filter
void HybridForceController::updateFitlerValue(const franka::RobotState& state) {
  for (size_t i = 0; i < 6; ++i) {
    filter_buffer_.get()[current_filter_pos_ * 6 + i] = state.O_F_ext_hat_K[i];
  }
  current_filter_pos_ = (current_filter_pos_ + 1) % filter_size_;
}

Eigen::Matrix<double, 6, 1> HybridForceController::getFitlerValue() {
  Eigen::Matrix<double, 6, 1> filter_val;
  filter_val.setZero();
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = i; j < 6 * filter_size_; j += 6)
      filter_val(i) += filter_buffer_.get()[j];
  }

  return filter_val / filter_size_;
}

} // end namespace

PLUGINLIB_EXPORT_CLASS(franka_controllers::HybridForceController,
                       controller_interface::ControllerBase)
