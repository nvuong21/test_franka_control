#include <franka_controllers/hybrid_controller.h>

#include <cmath>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_controllers {

bool PandaHybridController::init(hardware_interface::RobotHW* robot_hw,
                                   ros::NodeHandle& node_handle) {
    std::vector<std::string> joint_names;
    std::string arm_id;

    // Read parameters from config file
    if (!node_handle.getParam("arm_id", arm_id)) {
      ROS_ERROR("PandaHybridController: Could not read parameter arm_id");
      return false;
    }

    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
      ROS_ERROR(
          "PandaHybridController: Invalid or no joint_names parameters provided, aborting "
          "controller init!");
      return false;
    }

    if (!node_handle.getParam("kp_f", kp_f_)) {
      ROS_INFO_STREAM(
          "PandaHybridController:  Invalid or no kp_f, defaulting to "
          << kp_f_);
    }

    if (!node_handle.getParam("kd_v", kd_v_)) {
      ROS_INFO_STREAM(
          "PandaHybridController:  Invalid or no kp_d, defaulting to "
          << kd_v_);
    }
    if (!node_handle.getParam("ki_f", ki_f_)) {
      ROS_INFO_STREAM(
          "PandaHybridController:  Invalid or no kp_i, defaulting to "
          << ki_f_);
    }
    if (!node_handle.getParam("kp_c", kp_c_)) {
      ROS_INFO_STREAM(
          "PandaHybridController:  Invalid or no kc_f, defaulting to "
          << kp_c_);
    }
    if (!node_handle.getParam("kd_c", kd_c_)) {
      ROS_INFO_STREAM(
          "PandaHybridController:  Invalid or no kc_d_, defaulting to "
          << kd_c_);
    }
    if (!node_handle.getParam("ki_c", ki_c_)) {
      ROS_INFO_STREAM(
          "PandaHybridController:  Invalid or no ki_c, defaulting to "
          << ki_c_);
    }

    // Init state_handle_ and model_handle_
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR_STREAM("PandaHybridController: Error getting state interface from hardware");
      return false;
    }
    try {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PandaHybridController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
      ROS_ERROR_STREAM("PandaHybridController: Error getting model interface from hardware");
      return false;
    }
    try {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PandaHybridController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
      ROS_ERROR_STREAM("PandaHybridController: Error getting effort interface from hardware");
      return false;
    }
    for (size_t i = 0; i< 7; ++i){
      try {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "PandaHybridController: Exception getting state handle from interface: " << ex.what());
        return false;
      }
    }

    // Init velocity cartesian interface
    // velocity_cartesian_interface_ =
    //     robot_hw->get<franka_hw::FrankaVelocityCartesianInterface>();
    // if (velocity_cartesian_interface_ == nullptr) {
    //   ROS_ERROR(
    //       "HybridController: Could not get Cartesian velocity interface from "
    //       "hardware");
    //   return false;
    // }
    // try {
    //   velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
    //       velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
    // } catch (const hardware_interface::HardwareInterfaceException& e) {
    //   ROS_ERROR_STREAM(
    //       "HybridController: Exception getting Cartesian handle: " << e.what());
    //   return false;
    // }

    return true;
}

void PandaHybridController::starting(const ros::Time& ) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();

  // The torque measured by the torque sensor at each joint
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  tau_error_.setZero();

  // Current exteral torque
  tau_ext_initial_ = tau_measured - gravity;

  // Time pass
  elapsed_time_ = ros::Duration(0);

}

void PandaHybridController::update(const ros::Time& , const ros::Duration& period) {
  // Time pass
  elapsed_time_ += period;

  // Current robot state
  franka::RobotState robot_state = state_handle_->getRobotState();

  // Current joint velocity
  std::array<double, 7> joint_velocity_array = robot_state.dq;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> joint_velocity(joint_velocity_array.data());

  // Body Jacobian matrix
  std::array<double, 42> jacobian_array =
      model_handle_->getBodyJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // Gravity torque
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  // Coriolis torque
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  // Joint torque and command torque (at previous timestep)
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());

  Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);

  // Set desired Z-axis force (in the base frame)
  desired_force_torque.setZero();
  desired_force_torque(2) = -1.0;  // 1N

  // Calculate desired joint torque corresponding to the
  // desired force
  tau_d << jacobian.transpose() * desired_force_torque;

  // Current external torques
  tau_ext = tau_measured - gravity - coriolis - tau_ext_initial_;

  // The sum of torque error
  tau_error_ += (tau_d - tau_ext) * period.toSec();

  // Calculate cartesian velocity to limit the force
  Eigen::VectorXd twist(6);
  twist = jacobian * joint_velocity;

  // Slide motion
  Eigen::VectorXd tau_slide(7);
  tau_slide.setZero();

  // double rms_error{0};
  // for (size_t i = 0; i < 7; ++i){
  //   rms_error += std::pow(joint_velocity_array[i], 2);
  // }
  // rms_error = std::sqrt(rms_error);
  // if (elapsed_time_.toSec() > 2.0 && rms_error < 0.001) {
  //   double v_z = 0.005;
  //   std::array<double, 6> vel_command = {{0.0, 0.0, v_z, 0.0, 0.0, 0.0}};
  //   velocity_cartesian_handle_->setCommand(vel_command);
  //   Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_d(robot_state.dq_d.data());
  //   Eigen::Map<Eigen::Matrix<double, 7, 1>> q_d(robot_state.q_d.data());
  //   Eigen::Map<Eigen::Matrix<double, 7, 1>> q_cur(robot_state.q.data());
  //
  //   tau_slide = kp_c_ * (q_d - q_cur) + kd_c_ * (dq_d - joint_velocity);
  // }


  // Update rule
  tau_cmd = coriolis + tau_d + tau_ext_initial_
            + kp_f_ * (tau_d - tau_ext) + ki_f_ * tau_error_
            - kd_v_ * jacobian.transpose() * twist + tau_slide;

  // tau_cmd = coriolis + tau_d + kp_f_ * (tau_d - tau_ext) + ki_f_ * tau_error_
  //                     - kd_v_ * jacobian.transpose() * twist + tau_slide;

  tau_cmd << saturateTorqueRate(tau_cmd, tau_J_d);

  // Send torque command
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }



}

Eigen::Matrix<double, 7, 1> PandaHybridController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}

PLUGINLIB_EXPORT_CLASS(franka_controllers::PandaHybridController,
                       controller_interface::ControllerBase)
