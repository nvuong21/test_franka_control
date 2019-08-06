#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>

// Header for realtime publisher
#include <std_msgs/Float64.h>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>
// #include <franka_controllers/SetDesiredJoint.h>

namespace franka_controllers {

class PandaJointPositionController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
/* The controller subsribe to the /joint_d topic */

 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::array<double, 7> desired_joint_position_{};
  // double desired_joint_velocity_;

  double acceleration_time_{2.0};
  double vel_max_{0.05};
  double vel_current_{0.0};

  ros::Subscriber desired_joint_position_sub_;
  void desiredJointPositionCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
  // ros::ServiceServer desired_joint_position_ser_;
  // void desiredJointPositionCallback(const franka_controllers::SetDesiredJoint::Request& msg,
  //                                   franka_controllers::SetDesiredJoint::Request& res);

  // Variable for realtime publisher
  franka_hw::TriggerRate rate_trigger_{1.0};
  double joint_error_{0.0};
  realtime_tools::RealtimePublisher<std_msgs::Float64> joint_publisher_;
};
}
