#include <franka_controllers/force_controller.h>

#include <cmath>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot_state.h>

namespace franka_controllers {
bool PandaForceController::init(hardware_interface::RobotHW* robot_hw,
                                ros::NodeHandle& node_handle) {
  // Read params in config files
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("Can't get arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("Can't get joint_names");
    return false;
  }

  double publish_rate;
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_ERROR("Can't get publisher rate");
    return false;
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  // Interfaces and handles
    // State interface
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get state interface");
    return false;
  }
    // State handle
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "Exception getting state handle from interface: " << ex.what());
    return false;
  }

    // Model interface
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get model interface");
    return false;
  }
    // Model handle
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "Exception getting model handle from interface: " << ex.what());
    return false;
  }

    // Effort joint interface
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get effort joint interface");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Dynamic reconfigure
  dynamic_reconfigure_force_controller_param_node_ =
    ros::NodeHandle("dynamic_reconfigure_force_controller_param_node_");

  dynamic_server_force_controller_param_.reset(
    new dynamic_reconfigure::Server<franka_controllers::force_controller_paramConfig>(
        dynamic_reconfigure_force_controller_param_node_));

  dynamic_server_force_controller_param_->setCallback(
    boost::bind(&PandaForceController::ForceControllerParamCallback, this, _1, _2));

  return true;

  // Realtime publisher
  publisher_.init(node_handle, "data", 1);
}

void PandaForceController::starting(const ros::Time& ) {
  franka::RobotState initial_state = state_handle_->getRobotState();
  force_ext_initial_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(initial_state.O_F_ext_hat_K.data());

  // Initialize force error sum
  force_error_int_.setZero();
  elapsed_time_ = 0.0;
  desired_force_ = 0.0;
}

void PandaForceController::update(const ros::Time& , const ros::Duration& period) {
  elapsed_time_ += period.toSec();
  // Get current robot state
  franka::RobotState robot_state = state_handle_->getRobotState();

  // Current external force w/o initial correction
  Eigen::Map<Eigen::Matrix<double, 6, 1>> force_ext(robot_state.O_F_ext_hat_K.data());

  // Current torque signal
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J(robot_state.tau_J.data());

  // Jacobian matrix
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  Eigen::VectorXd desired_force_torque(6), tau_force(7), force_control(6);

  // FOrce control
  desired_force_torque.setZero();
  desired_force_torque(2) = -desired_force_;
  force_error_int_ += period.toSec() * (desired_force_torque - force_ext + force_ext_initial_);
  // PI controller
  force_control = (desired_force_torque + k_i_ * force_error_int_
                        + k_p_ * (desired_force_torque - force_ext + force_ext_initial_));
  force_control << 0, 0, force_control(2), 0, 0, 0;
  tau_force << jacobian.transpose() * force_control;
  tau_force << saturateTorqueRate(tau_force, tau_J);

  for (size_t i = 0; i < 7; ++i)
    joint_handles_[i].setCommand(tau_force(i));

  // Update dynamic params
  updateDynamicReconfigure();

  // Realtime publisher
  if (rate_trigger_() && publisher_.trylock()) {
    publisher_.msg_.time = elapsed_time_;
    publisher_.msg_.x = force_ext(2) - force_ext_initial_(2);
    publisher_.unlockAndPublish();
  }

}

// Limit torque input
Eigen::Matrix<double, 7, 1> PandaForceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

// Update dynamic params
void PandaForceController::updateDynamicReconfigure() {
  // Gradually increase
  desired_force_ = filter_params_ * target_force_ + (1 - filter_params_) * desired_force_;
  k_p_ = filter_params_ * target_k_p_ + (1 - filter_params_) * k_p_;
  k_i_ = filter_params_ * target_k_i_ + (1 - filter_params_) * k_i_;
}

// Get Dynamic params
void PandaForceController::ForceControllerParamCallback(
  franka_controllers::force_controller_paramConfig& config,
  uint32_t /*level*/) {
    target_force_ = config.desired_force;
    target_k_p_ = config.k_p;
    target_k_i_ = config.k_i;
}
} // end namespace


PLUGINLIB_EXPORT_CLASS(franka_controllers::PandaForceController,
                       controller_interface::ControllerBase)
