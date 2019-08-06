// #pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <Eigen/Core>

// Header for realtime publisher
#include <franka_controllers/JointTorque.h>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

namespace franka_controllers {

class PandaJointControllerTorqueInput : public controller_interface::MultiInterfaceController<
                                          franka_hw::FrankaModelInterface,
                                          hardware_interface::EffortJointInterface,
                                          franka_hw::FrankaStateInterface> {
/* The controller subsribe to the /joint_d topic */

 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
       const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
       const Eigen::Matrix<double, 7, 1>& tau_J_d);

  // hardware_interface::PositionJointInterface* position_joint_interface_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::array<double, 7> desired_joint_position_{};
  // std::array<double, 7> target_joint_position_{};
  std::array<double, 7> sum_error_{};
  const double delta_tau_max_{1.0};

  double vel_current_{0.0};
  double vel_max_{0.1};
  double acceleration_time_{2.0};

  ros::Subscriber desired_joint_position_sub_;
  void desiredJointPositionCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

  double kp{0.1};
  double ki{0.0};
  double kd{0.0};

  // Realtime Publisher
  ros::Duration elapsed_time_;
  franka_hw::TriggerRate rate_trigger_{1.0};
  realtime_tools::RealtimePublisher<JointTorque> publisher_;
};
}
