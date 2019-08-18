#include <franka_controllers/joint_controller_torque_input.h>

#include <memory>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>
#include <ros/ros.h>

namespace franka_controllers {

bool PandaJointControllerTorqueInput::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
    // Read desired joint position
    // des_pos_sub_ = node_handle.subscribe(
    //   "/joint_d", 20, &PandaJointControllerTorqueInput::desiredJointPositionCallback, this,
    //   ros::TransportHints().reliable().tcpNoDelay());

    // sub_desire_pose_ = node_handle.subscribe(
    //       "/desire_pose", 20, &PandaJointControllerTorqueInput::desirePoseCallback, this,
    //       ros::TransportHints().reliable().tcpNoDelay());

    // Read parameters in /config/*.yaml file
    node_handle.getParam("displacement", displacement_);

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

    if (!node_handle.getParam("kp", kp_)) {
      ROS_INFO_STREAM("PandaJointControllerTorqueInput: Can't read kp_, set to default value");
    }

    if (!node_handle.getParam("kd", kd_)) {
      ROS_INFO_STREAM("PandaJointControllerTorqueInput: Can't read kp_, set to default value");
    }

    // Publish rate for publisher
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


    // std::fill(std::begin(des_pos_), std::end(des_pos_), 0);

    // interactive_marker
    // position_d_.setZero();
    // orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    // position_d_target_.setZero();
    // orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    // desired_joint_velocity_ = 0.1;

    // Realtime Publisher
    publisher_.init(node_handle, "read", 1);

    return true;
}

void PandaJointControllerTorqueInput::starting(const ros::Time& ) {
  // Read robot state
  franka::RobotState initial_state = state_handle_->getRobotState();

  // Initial state
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  // Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  //
  // // Initial joint torque
  // Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(initial_state.tau_J.data());
  //
  // // Initial exteral torque for compensation
  // tau_ext_initial_ = tau_measured - gravity;

  // Initial pose
  // position_d_ = initial_transform.translation();
  // orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  // position_d_target_ = initial_transform.translation();
  // orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // target_pos_ = cur_pos + 0.1
  for (size_t i = 0; i < 7; ++i) {
    target_pos_[i] = initial_state.q[i] + displacement_[i];
    des_pos_[i] = initial_state.q[i];
  }
  des_vel_ = 0.0;

  elapsed_time_ = ros::Duration(0);
}

void PandaJointControllerTorqueInput::update(const ros::Time& , const ros::Duration& period) {

  // Accelerate
  if (des_vel_ < vel_max_) {
    des_vel_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
  }
  des_vel_ = std::fmin(des_vel_, vel_max_);


  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J(robot_state.tau_J.data());
  std::array<double, 7> cur_pos = robot_state.q;
  std::array<double, 7> cur_vel = robot_state.dq;
  Eigen::Matrix<double, 7, 1> tau_d;

  // std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> tau_ext = robot_state.tau_ext_hat_filtered;

  // Calculate step
  std::array<double, 7> step;
  double tStep{0.001};
  double joint_distance;
  for (size_t i = 0; i < 7; ++i) {
    joint_distance = target_pos_[i] - cur_pos[i];
    step[i] = des_vel_ * tStep * (joint_distance) / fabs(joint_distance);

    // Finiish motion
    double v;
    if ((fabs(step[i]) > fabs(target_pos_[i] - des_pos_[i]))
        && step[i] * (target_pos_[i] - des_pos_[i]) >= 0) {
      tau_d(i) = coriolis[i] + kp_ * (target_pos_[i] - cur_pos[i]) - kd_ * cur_vel[i];
    } else {
      des_pos_[i] = des_pos_[i] + step[i];
      tau_d(i) = coriolis[i] + kp_ * (des_pos_[i] - cur_pos[i]) + kd_ * (des_vel_ - cur_vel[i]);
    }

    // Need to change Kp
    // if (i == 3)
    //   tau_d(i) = coriolis[i] + 100 * joint_distance - 20 * cur_vel[i];
    // else tau_d(i) = coriolis[i] + kp_ * joint_distance - kd_ * cur_vel[i];
  }

  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // Update parameter change online
  // change position and orientation target gradually
  // position_d_ = filter_params_ + position_d_target_ + (1.0 - filter_params_) * position_d_;

  // quaternion -> angle-axis -> quaternion
  // Eigen::AngleAxisd aa_orientation_d(orientation_d_);
  // Eigen::AngleAxisd aa_orientation_d_target(orientation_d_target_);
  // aa_orientation_d.axis() = filter_params_ * aa_orientation_d_target.axis() +
  //                           (1.0 - filter_params_) * aa_orientation_d.axis();
  // aa_orientation_d.angle() = filter_params_ * aa_orientation_d_target.angle() +
  //                            (1.0 - filter_params_) * aa_orientation_d.angle();
  // orientation_d_ = Eigen::Quaterniond(aa_orientation_d);

  // Realtime Publisher
  elapsed_time_ += period;
  if (rate_trigger_() && publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    publisher_.msg_.time = elapsed_time_.toSec();

    double rms{0.0};
    for (size_t i = 0; i < 7; ++i) {
      rms += pow(target_pos_[i] - cur_pos[i], 2);
      publisher_.msg_.data1[i] = target_pos_[i] - cur_pos[i];
      publisher_.msg_.data2[i] = tau_d(i);

      // publisher_.msg_.tau_J_d[i] = tau_J_d(i);
      // publisher_.msg_.coriolis[i] = coriolis[i];
      // publisher_.msg_.gravity[i] = gravity[i];
    }
    publisher_.msg_.x = sqrt(rms);
    publisher_.unlockAndPublish();
  }


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

double generateNextStep(size_t index, double current_pos, double target_pos) {

}


// void PandaJointControllerTorqueInput::desiredJointPositionCallback(
//       const std_msgs::Float64MultiArrayConstPtr& msg) {
//   for (size_t i = 0; i < 7; ++i) {
//     des_pos_[i] = msg->data[i];
//   }
// }

// Interactive marker callback
// void PandaJointControllerTorqueInput::desirePoseCallback(
//     const geometry_msgs::PoseStampedConstPtr& msg) {
//   position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//   Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
//   orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
//       msg->pose.orientation.z, msg->pose.orientation.w;
//   if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
//     orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
//   }
// }

}

PLUGINLIB_EXPORT_CLASS(franka_controllers::PandaJointControllerTorqueInput,
                       controller_interface::ControllerBase)
