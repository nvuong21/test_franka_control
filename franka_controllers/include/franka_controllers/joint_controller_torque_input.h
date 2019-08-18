#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
// #include <std_msgs/Float64MultiArray.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <Eigen/Core>
// #include <geometry_msgs/PoseStamped.h>

// for realtime publisher
#include <franka_controllers/Float64Array.h>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

namespace franka_controllers {

class PandaJointControllerTorqueInput : public controller_interface::MultiInterfaceController<
                                          franka_hw::FrankaModelInterface,
                                          hardware_interface::EffortJointInterface,
                                          franka_hw::FrankaStateInterface> {
 // PD joint position controller

 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:

  const double delta_tau_max_{1.0};
  // saturateTorqueRate ensure that the difference in command torque < delta_tau_max_
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
       const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
       const Eigen::Matrix<double, 7, 1>& tau_J_d);

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // command joint position at each timestep
  std::array<double, 7> des_pos_{};
  double des_vel_{0};
  std::vector<double> displacement_;

  // desired joint position
  std::array<double, 7> target_pos_{};
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  Eigen::Matrix<double, 7, 1> gravity_initial_;
  // std::array<double, 7> target_joint_position_{};

  double vel_max_{0.3};
  double acceleration_time_{2.0};


  // ros::Subscriber des_pos_sub_;
  // void desiredJointPositionCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
  // double filter_params_{0.005};

  // Eigen::Vector3d position_d_;
  // Eigen::Quaterniond orientation_d_;
  // Eigen::Vector3d position_d_target_;
  // Eigen::Quaterniond orientation_d_target_;
  ///

  double kp_{1.0};
  double kd_{0.0};

  // Realtime Publisher
  ros::Duration elapsed_time_;
  franka_hw::TriggerRate rate_trigger_{1.0};
  realtime_tools::RealtimePublisher<Float64Array> publisher_;
};
}
