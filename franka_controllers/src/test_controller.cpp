// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_controllers/test_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_controllers {

bool TestController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {

  // Read parameters from config file
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("TestController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("TestController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }

  if (!node_handle.getParam("kp", kp_)) {
    // ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("kd", kd_)) {
    // ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }

  double publish_rate(50.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("TestController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);


  // Cartesian pose interface
  auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface == nullptr) {
    ROS_ERROR_STREAM(
        "TestController: Error getting cartesian pose interface from hardware");
    return false;
  }
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "TestController: Exception getting cartesian pose handle from interface: "
        << ex.what());
    return false;
  }

  // get joint effort interface
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }


  // get model interface
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // Interactive marker
  // position_d_.setZero();
  // orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  // position_d_target_.setZero();
  // orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  sub_desire_pose_ = node_handle.subscribe(
      "/desire_pose", 20, &TestController::desirePoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Realtime publisher
  // publisher_.init(node_handle, "read", 1);

  return true;
}

void TestController::starting(const ros::Time& /* time */) {
  franka::RobotState initial_state = cartesian_pose_handle_->getRobotState();
  // Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // Current pose
  desire_pose_ = initial_state.O_T_EE;
  elapsed_time_ = ros::Duration(0.0);
}

void TestController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  cartesian_pose_handle_->setCommand(desire_pose_);

  franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  Eigen::Matrix<double, 7, 1> tau_cmd;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());

  // Calculate error and update rule
  std::array<double, 7> error, d_error;
  for (size_t i = 0; i < 7; ++i) {
    error[i] = robot_state.q_d[i] - robot_state.q[i];
    d_error[i] = robot_state.dq_d[i] - robot_state.dq[i];
    tau_cmd(i, 0) = kp_ * error[i] + kd_ * d_error[i];
  }

  tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
  for (size_t i = 0; i < 7; ++i)
    joint_handles_[i].setCommand(tau_cmd(i,0));

  //
  // if (rate_trigger_() && publisher_.trylock()) {
  //   publisher_.msg_.time = elapsed_time_.toSec();
  //   double rms{0.0};
  //   for (size_t i = 0; i < 7; ++i) {
  //     rms += error[i] * error[i]
  //     // publisher_.msg_.tau_J[i] = tau_j[i];
  //     // publisher_.msg_.tau_J_d[i] = tau_J_d(i);
  //     // publisher_.msg_.coriolis[i] = coriolis[i];
  //     // publisher_.msg_.gravity[i] = gravity[i];
  //   }
  //   publisher_.msg_.x = std::sqrt(rms)
  //   publisher_.unlockAndPublish();

  // }

  // position_joint_handles_[0].setCommand(initial_pose_[0] + 0.1);
}

void TestController::desirePoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  Eigen::Quaterniond orientation;
  orientation.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w;

  Eigen::Matrix3d rot = orientation.toRotationMatrix();
  desire_pose_[12] = msg->pose.position.x;
  desire_pose_[13] = msg->pose.position.y;
  desire_pose_[14] = msg->pose.position.z;
  desire_pose_[15] = 1.0;
  for (size_t i = 0; i < 3; ++i) {
    desire_pose_[4*i] = rot(0, i);
    desire_pose_[4*i + 1] = rot(1, i);
    desire_pose_[4*i + 2] = rot(2, i);
    desire_pose_[4*i + 3] = 0.0;
  }
}

Eigen::Matrix<double, 7, 1> TestController::saturateTorqueRate(
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

}  // namespace franka_controllers




PLUGINLIB_EXPORT_CLASS(franka_controllers::TestController,
                       controller_interface::ControllerBase)
