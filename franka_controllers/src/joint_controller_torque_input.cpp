#include <franka_controllers/joint_controller_torque_input.h>

#include <memory>
// #include <cmath>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>
#include <ros/ros.h>

namespace franka_controllers {

bool PandaJointControllerTorqueInput::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
    // Read desired joint position
    desired_joint_position_sub_ = node_handle.subscribe(
      "/joint_d", 20, &PandaJointControllerTorqueInput::desiredJointPositionCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

    // Read parameters in /config/*.yaml file
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
      ROS_ERROR_STREAM("PandaJointControllerTorqueInput: Could not read parameter arm_id");
      return false;
    }

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names)) {
      ROS_ERROR(
          "PandaJointControllerTorqueInput: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    if (!node_handle.getParam("kp", kp)) {
      ROS_INFO_STREAM("PandaJointControllerTorqueInput: Can't read kp, set to default value");
    }

    // double ki = 0.0;
    if (!node_handle.getParam("kp", ki)) {
      ROS_INFO_STREAM("PandaJointControllerTorqueInput: Can't read kp, set to default value");
    }

    // double kd = 0.0;
    if (!node_handle.getParam("kp", kd)) {
      ROS_INFO_STREAM("PandaJointControllerTorqueInput: Can't read kp, set to default value");
    }

    double publish_rate(30.0);
    if (!node_handle.getParam("publish_rate", publish_rate)) {
      ROS_INFO_STREAM("JointImpedanceExampleController: publish_rate not found. Defaulting to "
                      << publish_rate);
    }
    rate_trigger_ = franka_hw::TriggerRate(publish_rate);


    // Initialize model interface
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
      ROS_ERROR_STREAM(
          "PandaJointControllerTorqueInput: Error getting model interface from hardware");
      return false;
    }
    try {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PandaJointControllerTorqueInput: Exception getting model handle from interface: "
          << ex.what());
      return false;
    }

    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR_STREAM(
          "PandaJointControllerTorqueInput: Error getting state interface from hardware");
      return false;
    }
    try {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PandaJointControllerTorqueInput: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
      ROS_ERROR_STREAM(
        "PandaJointControllerTorqueInput: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i) {
      try {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // rate_trigger_ = franka_hw::TriggerRate(publish_rate);

    std::fill(std::begin(desired_joint_position_), std::end(desired_joint_position_), 0);
    std::fill(std::begin(sum_error_), std::end(sum_error_), 0);
    // desired_joint_velocity_ = 0.1;

    // Realtime Publisher
    publisher_.init(node_handle, "publisher", 1);

    return true;
}

void PandaJointControllerTorqueInput::starting(const ros::Time& ) {
  franka::RobotState initial_state = state_handle_->getRobotState();
  std::array<double, 7> initial_joint_position = initial_state.q;
  for (size_t i = 0; i < 7; ++i) {
    desired_joint_position_[i] = initial_joint_position[i];
  }

  elapsed_time_ = ros::Duration(0.0);
}

void PandaJointControllerTorqueInput::update(const ros::Time& , const ros::Duration& period) {

  // Accelerate
  if (vel_current_ < vel_max_) {
    vel_current_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
  }
  vel_current_ = std::fmin(vel_current_, vel_max_);

  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  std::array<double, 7> current_joint_position = robot_state.q;
  std::array<double, 7> current_joint_velocity = robot_state.dq;
  Eigen::Matrix<double, 7, 1> tau_d;

  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  std::array<double, 7> error, d_error, cmd_joint;
  for (size_t i = 0; i < 7; ++i) {
    // Command Joint
    if (fabs(current_joint_position[i] - desired_joint_position_[i]) < period.toSec() * vel_current_)
      cmd_joint[i] = desired_joint_position_[i];
    else if (current_joint_position[i] < desired_joint_position_[i])
      cmd_joint[i] =  current_joint_position[i] + period.toSec() * vel_current_;
    else cmd_joint[i] =  current_joint_position[i] - period.toSec() * vel_current_;

    error[i] = cmd_joint[i] - current_joint_position[i];
    sum_error_[i] += error[i] * period.toSec();
    d_error[i] = -current_joint_velocity[i];
    tau_d(i) = coriolis[i] + kp * error[i] + kd * d_error[i] + ki * sum_error_[i];
  }

  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // Realtime Publisher
  // elapsed_time_ += period;
  // if (rate_trigger_() && publisher_.trylock()) {
  //   std::array<double, 7> tau_j = robot_state.tau_J;
  //   publisher_.msg_.time = elapsed_time_.toSec();
  //
  //   for (size_t i = 0; i < 7; ++i) {
  //     publisher_.msg_.tau_J[i] = tau_j[i];
  //     publisher_.msg_.tau_J_d[i] = tau_J_d(i);
  //     publisher_.msg_.coriolis[i] = coriolis[i];
  //     publisher_.msg_.gravity[i] = gravity[i];
  //   }
  //   publisher_.unlockAndPublish();
  // }
}

// don't let d_torque too big
Eigen::Matrix<double, 7, 1> PandaJointControllerTorqueInput::saturateTorqueRate(
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

void PandaJointControllerTorqueInput::desiredJointPositionCallback(
      const std_msgs::Float64MultiArrayConstPtr& msg) {
  for (size_t i = 0; i < 7; ++i) {
    desired_joint_position_[i] = msg->data[i];
  }
}

}

PLUGINLIB_EXPORT_CLASS(franka_controllers::PandaJointControllerTorqueInput,
                       controller_interface::ControllerBase)
