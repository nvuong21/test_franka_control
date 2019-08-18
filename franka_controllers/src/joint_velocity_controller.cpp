#include <franka_controllers/joint_velocity_controller.h>
#include <cmath>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot_state.h>

namespace franka_controllers {

  bool PandaJointVelocityController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {

    // Read parameters from config file
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
      ROS_ERROR("PandaJointVelocityController: Could not read parameter arm_id");
      return false;
    }

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names)) {
      ROS_ERROR("PandaJointVelocityController: Could not read parameter arm_id");
      return false;
    }

    if (!node_handle.getParam("kp", kp_)) {
      ROS_INFO_STREAM("PandaJointVelocityController: Can't read kp_, set to default value");
    }

    if (!node_handle.getParam("kd", kd_)) {
      ROS_INFO_STREAM("PandaJointVelocityController: Can't read kp_, set to default value");
    }

    double publish_rate(30.0);
    if (!node_handle.getParam("publish_rate", publish_rate)) {
      ROS_INFO_STREAM("PandaJointVelocityController: publish_rate not found. Defaulting to "
                      << publish_rate);
    }
    rate_trigger_ = franka_hw::TriggerRate(publish_rate);

    // Initialize interface and handle
    //** Model interface
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
      ROS_ERROR_STREAM(
          "PandaJointVelocityController: Error getting model interface from hardware");
      return false;
    }
    //** Model handle
    try {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PandaJointVelocityController: Exception getting model handle from interface: "
          << ex.what());
      return false;
    }

    //** State inteface
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR_STREAM(
          "PandaJointVelocityController: Error getting state interface from hardware");
      return false;
    }
    //** State handle
    try {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PandaJointVelocityController: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }

    //** Effort Interface
    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
      ROS_ERROR_STREAM(
        "PandaJointVelocityController: Error getting effort joint interface from hardware");
      return false;
    }
    //** Effort handle
    for (size_t i = 0; i < 7; ++i) {
      try {
        effort_joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "PandaJointVelocityController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    //** Velocity Interface
    auto* velocity_joint_interface = robot_hw->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface == nullptr) {
      ROS_ERROR_STREAM(
        "PandaJointVelocityController: Error getting effort joint interface from hardware");
      return false;
    }
    //** Effort handle
    for (size_t i = 0; i < 7; ++i) {
      try {
        velocity_joint_handles_.push_back(velocity_joint_interface->getHandle(joint_names[i]));
      } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "PandaJointVelocityController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // Init publisher
    publisher_.init(node_handle, "/data", 1);

    // Filter velocity 2
    dq_buffer_ = std::make_unique<double[]>(dq_filter_size_ * 7);
    std::fill(&dq_buffer_.get()[0], &dq_buffer_.get()[dq_filter_size_ * 7], 0);
    dq_current_filter_position_ = 0;

    return true;
  }

  void PandaJointVelocityController::starting(const ros::Time&) {
    trajectory_ = PandaJointVelocityController::generateTrajectory();
    index_ = 0;
    elapsed_time_ = ros::Duration(0);

    franka::RobotState initial_state = state_handle_->getRobotState();

    for (size_t i = 0; i < 7; ++i) {
      dq_filtered_[i] = initial_state.dq[i];
    }
  }

  void PandaJointVelocityController::update(const ros::Time&, const ros::Duration& period) {
    index_ += 1;
    if (index_ >= trajectory_.size()) index_ = trajectory_.size() - 1;


    // Desired velocity
    size_t joint_number{3};           // Joint want to control
    std::array<double, 7> des_vel{0, 0, 0, 0, 0, 0, 0};
    des_vel[joint_number] = trajectory_[index_];
    for (size_t i = 0; i < 7; ++i)
      velocity_joint_handles_[i].setCommand(des_vel[i]);

    // Get robot state
    franka::RobotState robot_state = state_handle_->getRobotState();

    // Update velocity buffer
    updateDQFiltered(robot_state);

    // Calculate filter velocity
    // for (size_t i = 0; i < 7; ++i)
    //   dq_filtered_[i] = filter_params_ * dq_filtered_[i] +
    //                     (1 - filter_params_) * robot_state.dq[i];


    // Previous tau_cmd
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

    // coriolis
    std::array<double, 7> coriolis = model_handle_->getCoriolis();
    // Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Matrix<double, 7, 1> tau_cmd;

    // Calculate tau command
    for (size_t i = 0; i < 7; ++i)
      // tau_cmd(i) = coriolis[i] + kp_ * (robot_state.q_d[i] - robot_state.q[i])
      //                  + kd_ * (robot_state.dq_d[i] - /*robot_state.dq[i]*/ dq_filtered_[i]);

      tau_cmd(i) = coriolis[i] + kp_ * (robot_state.q_d[i] - robot_state.q[i])
                       + kd_ * (robot_state.dq_d[i] - getDQFiltered(i));

    tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);

    // Send torque command
    for (size_t i = 0; i < 7; ++i)
      effort_joint_handles_[i].setCommand(tau_cmd[i]);

    // Realtime publisher
    elapsed_time_ += period;
    if (rate_trigger_() && publisher_.trylock()) {
      publisher_.msg_.time = elapsed_time_.toSec();
      for (size_t i = 0; i < 7; ++i) {
        publisher_.msg_.data1[i] = robot_state.q[i];
        publisher_.msg_.data2[i] = robot_state.q_d[i];
        // publisher_.msg_.data3[i] = robot_state.dq[i];
        publisher_.msg_.data3[i] =   dq_filtered_[i];
        publisher_.msg_.data4[i] = robot_state.dq_d[i];
      }
      publisher_.unlockAndPublish();
    }

  }

  void PandaJointVelocityController::stopping(const ros::Time&) {}

  std::vector<double> PandaJointVelocityController::generateTrajectory() {
    // generate smooth velocity trajectory path
    std::vector<double> trajectory;
    double kTimeStep = 0.001;
    double kAccelerationTime = 1;       // Acceleration and Deceleration time
    double kConstantVelocityTime = 3;

    double a = 0;
    double v = 0;
    double t = 0;
    double a_max = 0.1;
    while (t < (2 * kAccelerationTime + kConstantVelocityTime)) {
      // Calculate acceleration
      if (t <= kAccelerationTime) a = a_max * pow(sin(t * M_PI / kAccelerationTime), 2);
      else if (t <= kAccelerationTime + kConstantVelocityTime)
        a = 0;
      else {
        double deceleration_time = (kAccelerationTime + kConstantVelocityTime) - t;
        a = -a_max * pow(sin(deceleration_time * M_PI / kAccelerationTime), 2);
      }

      //Calculate velocity
      v += a * kTimeStep;
      t += kTimeStep;
      trajectory.push_back(v);
    }
    return trajectory;
  }

  Eigen::Matrix<double, 7, 1> PandaJointVelocityController::saturateTorqueRate(
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

  double PandaJointVelocityController::getDQFiltered(size_t index) {
    double value = 0;
    for (size_t i = index; i < 7 * dq_filter_size_; i += 7) {
      value += dq_buffer_.get()[i];
    }
    return value / dq_filter_size_;
  }

  void PandaJointVelocityController::updateDQFiltered(const franka::RobotState& state) {
    for (size_t i = 0; i < 7; i++) {
      dq_buffer_.get()[dq_current_filter_position_ * 7 + i] = state.dq[i];
    }
    dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
  }

} // end namespace


PLUGINLIB_EXPORT_CLASS(franka_controllers::PandaJointVelocityController,
                       controller_interface::ControllerBase)
