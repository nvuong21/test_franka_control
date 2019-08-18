#include <franka_controllers/cartesian_velocity_controller.h>

#include <cmath>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>
#include <ros/ros.h>

namespace franka_controllers {
bool PandaCartesianVelocityController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
  // Read parameters in config file
  std::string arm_id;
  node_handle.getParam("arm_id", arm_id);

  std::vector<std::string> joint_names;
  node_handle.getParam("joint_names", joint_names);

  // get k_gain array
  std::vector<double> k_array;
  node_handle.getParam("k_gains", k_array);

  // Calculate k_gain and d_gain matrix
  k_gains_.setZero();
  d_gains_.setZero();
  for (size_t i = 0; i < 7; ++i){
    k_gains_(i, i) = k_array[i];
    d_gains_(i, i) = 2 * sqrt(k_array[i]) + 0.1;
  }

  // Publish rate for realtime publisher
  double publish_rate;
  node_handle.getParam("publish_rate", publish_rate);
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  // Realtime Publisher
  publisher_.init(node_handle, "data", 1);

  // Generate trajectory
  trajectory_ = generateTrajectory();
  index_ = 0;
}

void PandaCartesianVelocityController::starting(const ros::Time&) {

}

void PandaCartesianVelocityController::update(const ros::Time&, const ros::Duration& period) {
  // Get desire velocity
  index_ += 1;
  if (index_ >= trajectory_.size()) index_ = trajectory_.size() - 1;
  std::array<double, 6> velocity = trajectory_[index_];

  // Set desire velocity
  cartesian_velocity_handle_->setCommand(velocity);

  // Get robot state
  franka::RobotState robot_state = state_handle_->getRobotState();

  // Current and desired position and velocity
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_d(robot_state.q_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_d(robot_state.dq_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // Get coriolis
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  // Torque command
  Eigen::Matrix<double, 7, 1> tau_cmd;
  tau_cmd = coriolis + k_gains_ * (q_d - q) + d_gains_ * (dq_d - dq);
  tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);

  //Send command
  for (size_t i = 0; i < 7; ++i)
    joint_handles_[i].setCommand(tau_cmd(i));

  // Realtime Publisher
  elapsed_time_ += period;
  if (rate_trigger_() && publisher_.trylock()) {

    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Matrix<double, 6, 1> measured_vel;
    measured_vel = jacobian * dq;

    publisher_.msg_.time = elapsed_time_.toSec();
    for (size_t i = 0; i < 7; ++i) {
      publisher_.msg_.data1[i] = robot_state.q[i];
      publisher_.msg_.data2[i] = robot_state.q_d[i];
      // publisher_.msg_.data3[i] = robot_state.dq[i];
      publisher_.msg_.data3[i] = robot_state.dq[i];
      publisher_.msg_.data4[i] = robot_state.dq_d[i];
    }

    // for (size_t i = 0; i < 6; ++i) {
    //   publisher_.msg_.data1[i] = velocity[i];
    //   publisher_.msg_.data2[i] = measured_vel(i);
    // }


    publisher_.unlockAndPublish();
  }
}

std::vector<std::array<double, 6>> PandaCartesianVelocityController::generateTrajectory() {
  std::vector<std::array<double, 6>> trajectory;

  double kTimeStep = 0.001;
  double time_max = 10.0;
  // double v_max = 0.2;
  double freq = 2.0;
  double radius = 0.1;

  std::array<double, 6> v = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double t = 0.0;
  double angle = 0.0;
  double v_y, v_z;


  while (t <= time_max) {
    angle = 2.0 * M_PI / freq * t;
    v_y = radius * std::cos(angle) * 2.0 * M_PI / freq;
    v_z = -radius * std::sin(angle) * 2.0 * M_PI / freq;
    v[1] = v_y;
    v[2] = v_z;
    trajectory.push_back(v);
    t += kTimeStep;
  }

  return trajectory;
}

Eigen::Matrix<double, 7, 1> PandaCartesianVelocityController::saturateTorqueRate(
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

} // end namespace

PLUGINLIB_EXPORT_CLASS(franka_controllers::PandaCartesianVelocityController,
                       controller_interface::ControllerBase)
