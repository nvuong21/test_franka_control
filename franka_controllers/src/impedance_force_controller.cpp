#include <franka_controllers/impedance_force_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_controllers {

bool ImpedanceForceController::init(hardware_interface::RobotHW* robot_hw,
                                    ros::NodeHandle& node_handle) {

  // Read parameters in config file
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("ImpedanceForceController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR_STREAM("ImpedanceForceController: Could not read parameter joint_names");
    return false;
  }

  std::vector<double> k_gains;
  if (!node_handle.getParam("k_gains", k_gains)) {
    ROS_ERROR_STREAM("ImpedanceForceController: Could not read parameter k_gains");
    return false;
  }

  if (!node_handle.getParam("k_collision", k_collision_)) {
    ROS_ERROR_STREAM("ImpedanceForceController: Could not read parameter k_gains");
    return false;
  }

  //// Init handles
  // model_interface
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ImpedanceForceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ImpedanceForceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }


  // state_interface
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ImpedanceForceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ImpedanceForceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // joint_effort_interface
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ImpedanceForceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "ImpedanceForceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Cartesian velocity interface
  // auto* velocity_cartesian_interface =
  //     robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  // if (velocity_cartesian_interface_ == nullptr) {
  //   ROS_ERROR(
  //       "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
  //       "hardware");
  //   return false;
  // }
  // try {
  //   velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
  //       velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  // } catch (const hardware_interface::HardwareInterfaceException& e) {
  //   ROS_ERROR_STREAM(
  //       "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
  //   return false;
  // }

  //// Init gains
  cartesian_stiffness_.setIdentity();
  cartesian_damping_.setIdentity();
  for (size_t i = 0; i < 6; ++i) {
    cartesian_stiffness_(i, i) = k_gains[i];
    cartesian_damping_(i, i) = 2 * std::sqrt(cartesian_stiffness_(i, i));
  }

  // position_d_.setZero();
  // orientation_d_.setZero();
  publisher_.init(node_handle, "read", 1);
}

void ImpedanceForceController::starting(const ros::Time& ) {
  // Read initial robot state
  franka::RobotState initial_state = state_handle_->getRobotState();

  // initial endeffector transform
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  // std::cout << orientation_d_.w() << "n";
  // Eigen::Vector3d a = orientation_d_.vec();
  // std::cout << a(0);

  elapsed_time_ = ros::Duration(0.0);

  // initial_wrench

  wrench_ext_initial_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(initial_state.O_F_ext_hat_K.data());
  // std::cout << wrench_ext_initial_ << "n";

}

void ImpedanceForceController::update(const ros::Time& ,
                                      const ros::Duration& period){
  elapsed_time_ += period;

  // Get jacobian matrix
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // Read current robot state
  franka::RobotState robot_state = state_handle_->getRobotState();

  // Joint velocity
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  //Current wrench
  Eigen::Map<Eigen::Matrix<double, 6, 1>> current_wrench(robot_state.O_F_ext_hat_K.data());

  // Previous torque command
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // Current Tee
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // Compute new desired pose
  if (elapsed_time_.toSec() > 5.0){
    orientation_d_ = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
    // position_d_(2) -= period.toSec() * vel_ * (des_force_ - fabs(current_wrench(2))) / 5.0;
    position_d_(2) += period.toSec() * vel_ ;
  }

  // if (elapsed_time_.toSec() > 12.0 && elapsed_time_.toSec() < 14.0) {
  //   position_d_(1) -= period.toSec() * vel_;
  // }

  // compute error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;  // position error

  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();      // not change the quaternion
  }

  // difference quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute Control
  Eigen::VectorXd tau_d(7);

  tau_d << jacobian.transpose() * ( -cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  // set command
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  if (rate_trigger_() && publisher_.trylock()) {
    Eigen::Map<Eigen::Matrix<double, 6, 1>> current_wrench(robot_state.O_F_ext_hat_K.data());
    publisher_.msg_.time = elapsed_time_.toSec();
    for (size_t i = 0; i < 6; ++i) {
      publisher_.msg_.data1[i] = current_wrench(i) - wrench_ext_initial_(i);

    }
    // vel += dq(6) * dq(6);
    // publisher_.msg_.x = vel;
    publisher_.unlockAndPublish();

  }
}

Eigen::Matrix<double, 7, 1> ImpedanceForceController::saturateTorqueRate(
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

}

PLUGINLIB_EXPORT_CLASS(franka_controllers::ImpedanceForceController,
                       controller_interface::ControllerBase)
