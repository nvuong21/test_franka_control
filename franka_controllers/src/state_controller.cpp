#include <franka_controllers/state_controller.h>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

namespace franka_controllers {

bool PandaStateController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {

  // Read parameter in config file
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("PandaStateController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR(
        "PandaStateController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("PandaStateController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  trigger_publish_ = franka_hw::TriggerRate(publish_rate);

  // Initialize interface and handle
  //// State interface
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PandaStateController: Error getting state interface from hardware");
    return false;
  }
  //// State handle
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PandaStateController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  //// Model Interface
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PandaJointControllerTorqueInput: Error getting model interface from hardware");
    return false;
  }
  //// Model handle
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
      model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PandaJointControllerTorqueInput: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // Realtime publisher
  state_publisher_.init(node_handle, "robot_state", 1);

  return true;
}

void PandaStateController::starting(const ros::Time& ) {}

void PandaStateController::update(const ros::Time&, const ros::Duration& period) {
  // Get robot state
  franka::RobotState robot_state = state_handle_->getRobotState();

  // Model state
  std::array<double, 16> pose = model_handle_->getPose(franka::Frame::kEndEffector);
  std::array<double, 42> jacobian = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity = model_handle_->getGravity();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 49> mass = model_handle_-> getMass();

  // Realtime Publisher
  if (trigger_publish_() && state_publisher_.trylock()) {

    state_publisher_.msg_.franka_state.m_total = robot_state.m_total;
    for (size_t i = 0; i < 7; ++i) {
      state_publisher_.msg_.franka_state.tau_J[i] = robot_state.tau_J[i];
      state_publisher_.msg_.franka_state.O_F_ext_hat_K[i] = robot_state.O_F_ext_hat_K[i];
      state_publisher_.msg_.franka_state.tau_ext_hat_filtered[i] = robot_state.tau_ext_hat_filtered[i];
      state_publisher_.msg_.gravity[i] = gravity[i];
      state_publisher_.msg_.coriolis[i] = coriolis[i];
    }

    for (size_t i = 0; i < 16; ++i) {
      state_publisher_.msg_.franka_state.O_T_EE[i] = robot_state.O_T_EE[i];
      state_publisher_.msg_.pose[i] = pose[i];
      state_publisher_.msg_.franka_state.tau_ext_hat_filtered[i] = robot_state.tau_ext_hat_filtered[i];
    }

    for (size_t i = 0; i < 42; ++i)
      state_publisher_.msg_.zeroJacobian[i] = jacobian[i];

    state_publisher_.unlockAndPublish();
  }

}

}


PLUGINLIB_EXPORT_CLASS(franka_controllers::PandaStateController,
                       controller_interface::ControllerBase)
