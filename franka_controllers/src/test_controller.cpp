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

  // if (!node_handle.getParam("kp", kp_)) {
  //   // ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
  //   return false;
  // }
  //
  // if (!node_handle.getParam("kd", kd_)) {
  //   // ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
  //   return false;
  // }
  //
  // double publish_rate(50.0);
  // if (!node_handle.getParam("publish_rate", publish_rate)) {
  //   ROS_INFO_STREAM("TestController: publish_rate not found. Defaulting to "
  //                   << publish_rate);
  // }
  // rate_trigger_ = franka_hw::TriggerRate(publish_rate);
  //
  //
  // // Cartesian pose interface
  // auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  // if (cartesian_pose_interface == nullptr) {
  //   ROS_ERROR_STREAM(
  //       "TestController: Error getting cartesian pose interface from hardware");
  //   return false;
  // }
  // try {
  //   cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
  //       cartesian_pose_interface->getHandle(arm_id + "_robot"));
  // } catch (hardware_interface::HardwareInterfaceException& ex) {
  //   ROS_ERROR_STREAM(
  //       "TestController: Exception getting cartesian pose handle from interface: "
  //       << ex.what());
  //   return false;
  // }
  //
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

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
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

  // Dynamic reconfigure
  dynamic_reconfigure_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_reconfigure_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_controllers::impedance_paramConfig>>(
      dynamic_reconfigure_param_node_);
  dynamic_reconfigure_param_->setCallback(
      boost::bind(&TestController::dynamicParamCallback, this, _1, _2));


  // Interactive marker
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;


  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  // sub_desire_pose_ = node_handle.subscribe(
  //     "/desire_pose", 20, &TestController::desirePoseCallback, this,
  //     ros::TransportHints().reliable().tcpNoDelay());

  // Realtime publisher
  publisher_.init(node_handle, "data", 1);

  return true;
}

void TestController::starting(const ros::Time& /* time */) {
  franka::RobotState initial_state = state_handle_->getRobotState();
  // Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  // Current pose
  // desire_pose_ = initial_state.O_T_EE;

  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
  initial_pos_ = initial_transform.translation();
  twist_target_.setZero();
  twist_d_.setZero();
  // std::cout << orientation_d_target_.coeffs();
  // std::cout << "\n";
  // elapsed_time_ = ros::Duration(0.0 );
}

void TestController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {

  for(size_t i = 0; i <3; ++i)
    position_d_(i) +=  period.toSec() * twist_d_(i);

  // elapsed_time_ += period;
  franka::RobotState robot_state = state_handle_->getRobotState();

  // cartesian_pose_handle_->setCommand(desire_pose_);
  //
  // franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
  // std::array<double, 7> coriolis = model_handle_->getCoriolis();
  // std::array<double, 7> gravity = model_handle_->getGravity();

  transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  Eigen::Vector3d position(transform_.translation());
  Eigen::Quaterniond orientation(transform_.linear());

  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());


  orientation_d_.coeffs() << 1.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 6, 1> error;
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


  // // Calculate error and update rule
  // std::array<double, 7> error, d_error;
  // for (size_t i = 0; i < 7; ++i) {
  //   error[i] = robot_state.q_d[i] - robot_state.q[i];
  //   d_error[i] = robot_state.dq_d[i] - robot_state.dq[i];
  //   tau_cmd(i, 0) = kp_ * error[i] + kd_ * d_error[i];
  // }
  //

  Eigen::Matrix<double, 7, 1> tau_cmd;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // tau_cmd << jacobian.transpose() *
  //                     (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));

  tau_cmd << jacobian.transpose() *
                      (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq - twist_d_));

  // Check twist


  tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
  for (size_t i = 0; i < 7; ++i)
    joint_handles_[i].setCommand(tau_cmd(i));
    // std::cout << tau_cmd(i);


  if (rate_trigger_() && publisher_.trylock()) {
    Eigen::Matrix<double, 6, 1> twist;
    twist = jacobian * dq;
    // publisher_.msg_.time = elapsed_time_.toSec();
    // double rms{0.0};
    publisher_.msg_.data1[6] = 0;
    for (size_t i = 0; i < 6; ++i) {
      publisher_.msg_.data1[i] = twist(i);
      // publisher_.msg_.tau_J[i] = tau_j[i];
      // publisher_.msg_.tau_J_d[i] = tau_J_d(i);
      // publisher_.msg_.coriolis[i] = coriolis[i];
      // publisher_.msg_.gravity[i] = gravity[i];
    }
    // publisher_.msg_.x = std::sqrt(rms)
    publisher_.unlockAndPublish();
  }

  // position_joint_handles_[0].setCommand(initial_pose_[0] + 0.1);

  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;

  // postion update
  // double filter_pos = 0.0015;
  // position_d_ = filter_pos * position_d_target_ + (1.0 - filter_pos) * position_d_;

  // velocity update
  twist_d_ = filter_params_ * twist_target_ + (1.0 - filter_params_) * twist_d_;

}

// void TestController::desirePoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
//   Eigen::Quaterniond orientation;
//   orientation.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
//     msg->pose.orientation.z, msg->pose.orientation.w;
//
//   Eigen::Matrix3d rot = orientation.toRotationMatrix();
//   desire_pose_[12] = msg->pose.position.x;
//   desire_pose_[13] = msg->pose.position.y;
//   desire_pose_[14] = msg->pose.position.z;
//   desire_pose_[15] = 1.0;
//   for (size_t i = 0; i < 3; ++i) {
//     desire_pose_[4*i] = rot(0, i);
//     desire_pose_[4*i + 1] = rot(1, i);
//     desire_pose_[4*i + 2] = rot(2, i);
//     desire_pose_[4*i + 3] = 0.0;
//   }
// }

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

// Position Callback
// void TestController::dynamicParamCallback(franka_controllers::impedance_paramConfig& config,
//                              uint32_t level) {
//
//   cartesian_stiffness_target_.setZero();
//   cartesian_stiffness_target_.topLeftCorner(3, 3)
//       << config.translational_stiffness * Eigen::Matrix3d::Identity();
//   cartesian_stiffness_target_.bottomRightCorner(3, 3)
//       << config.rotational_stiffness * Eigen::Matrix3d::Identity();
//
//   cartesian_damping_target_.setZero();
//   cartesian_damping_target_.topLeftCorner(3, 3)
//       << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
//   cartesian_damping_target_.bottomRightCorner(3, 3)
//       << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
//
//   Eigen::Vector3d dev;
//   dev << config.amplitude_x, config.amplitude_y, config.amplitude_z;
//   position_d_target_ = initial_pos_ + dev;
//   // double v = 0.03;
//   // for (size_t i = 0; i < 3; ++i){
//   //   if (position_d_target_(i) < (initial_pos_(i) + dev(i)))
//   //     position_d_target_(i) += v*0.001;
//   //   else if (position_d_target_(i) > (initial_pos_(i) + dev(i)))
//   //     position_d_target_(i) -= v * 0.001;
//   // }
// }

// Velocity callback
void TestController::dynamicParamCallback(franka_controllers::impedance_paramConfig& config,
                             uint32_t level) {

  cartesian_stiffness_target_.setZero();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();

  cartesian_damping_target_.setZero();
  cartesian_damping_target_.topLeftCorner(3, 3)
      << (2.0 * sqrt(config.translational_stiffness) + 10) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();

  // Eigen::Vector3d dev;
  // dev << config.amplitude_x, config.amplitude_y, config.amplitude_z;
  // position_d_target_ = initial_pos_ + dev;

  // Eigen::Matrix<double, 6, 1> body_twist;
  twist_target_ << config.amplitude_x, config.amplitude_y, config.amplitude_z, 0.0, 0 , 0;
  // twist_target_ = adjoint(transform_) * body_twist;
}

Eigen::Matrix<double, 6, 6> TestController::adjoint(Eigen::Affine3d transform) {
  Eigen::Matrix<double, 3, 1> trans_vector(transform.translation());
  Eigen::Matrix<double, 3, 3> rot_matrix(transform.linear());

  Eigen::Matrix<double, 6, 6> adj_mat;
  adj_mat.setZero();
  adj_mat.topLeftCorner(3,3) << rot_matrix;
  adj_mat.bottomRightCorner(3,3) << rot_matrix;

  Eigen::Matrix<double, 3, 3> bracket_p;
  bracket_p << 0.0             ,  -trans_vector(2),  trans_vector(1),
               trans_vector(2) ,  0               , -trans_vector(0),
               -trans_vector(1),  trans_vector(0) ,               0 ;
  adj_mat.bottomLeftCorner(3,3) << bracket_p * rot_matrix;
  return adj_mat;
}

}  // namespace franka_controllers




PLUGINLIB_EXPORT_CLASS(franka_controllers::TestController,
                       controller_interface::ControllerBase)
