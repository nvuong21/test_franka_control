#include <franka_controllers/joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_controllers {

bool PandaJointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  desired_joint_position_sub_ = node_handle.subscribe(
    "/joint_d", 20, &PandaJointPositionController::desiredJointPositionCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  // desired_joint_position_ser_ = node_handle.advertise

  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("JointPositionController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  // std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

  // Realtime Publisher
  joint_publisher_.init(node_handle, "joint_error", 1);


  return true;
}

void PandaJointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    desired_joint_position_[i] = position_joint_handles_[i].getPosition();
  }
}

void PandaJointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  if (vel_current_ < vel_max_) {
    vel_current_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
  }
  vel_current_ = std::fmin(vel_current_, vel_max_);

  std::array<double, 7> currrent_joint_position_;
  std::array<double, 7> deviation;
  std::array<double, 7> step;
  // std::array<double, 7> direction{};
  for (size_t i = 0; i < 7; ++i) {
    currrent_joint_position_[i] = position_joint_handles_[i].getPosition();
    deviation[i] = desired_joint_position_[i] - currrent_joint_position_[i];
    step[i] = vel_current_ * period.toSec() * (deviation[i] / fabs(deviation[i]));
  }

  for (size_t i = 0; i < 7; ++i) {

    if (fabs(deviation[i]) < vel_current_ * period.toSec())
      position_joint_handles_[i].setCommand(desired_joint_position_[i]);
    else
      position_joint_handles_[i].setCommand(currrent_joint_position_[i] + step[i]);
  }


  // Realtime publisher
  // if (rate_trigger_() && joint_publisher_.trylock()) {
  //   joint_error_ = 0.0;
  //   for (size_t i = 0; i < 7; ++i)
  //     joint_error_ += fabs(position_joint_handles_[i].getPosition() -
  //                          desired_joint_position_[i]);
  //   joint_publisher_.msg_.data = joint_error_;
  //   joint_publisher_.unlockAndPublish();
  // }

  ros::spinOnce();
}


void PandaJointPositionController::desiredJointPositionCallback(
      const std_msgs::Float64MultiArrayConstPtr& msg) {
  for (size_t i = 0; i < 7; ++i) {
    desired_joint_position_[i] = msg->data[i];
  }
}

}  // namespace franka_controllers

PLUGINLIB_EXPORT_CLASS(franka_controllers::PandaJointPositionController,
                       controller_interface::ControllerBase)
