
#include <memory>
#include <array>
#include <string>
// #include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <Eigen/Core>


#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
// #include <franka_hw/franka_cartesian_command_interface.h>   // include franka_state_interface

// Dynamci reconfigure
#include <dynamic_reconfigure/server.h>
#include <franka_controllers/impedance_paramConfig.h>

// RealtimePublisher
#include <franka_hw/trigger_rate.h>
#include <franka_controllers/Float64Array.h>
#include <realtime_tools/realtime_publisher.h>

// interactive_marker
// #include <geometry_msgs/PoseStamped.h>

namespace franka_controllers {
class TestController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::EffortJointInterface,
                                           franka_hw::FrankaStateInterface,
                                           franka_hw::FrankaModelInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:

   // Saturation
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
       const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
       const Eigen::Matrix<double, 7, 1>& tau_J_d);
  // std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  // For realtime publisher
  ros::Duration elapsed_time_;
  franka_hw::TriggerRate rate_trigger_{50.0};
  realtime_tools::RealtimePublisher<Float64Array> publisher_;

  // Dynamic reconfigure
  double filter_params_{0.01};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Affine3d transform_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  Eigen::Matrix<double, 6, 1> twist_d_;
  Eigen::Matrix<double, 6, 1> twist_target_;
  Eigen::Vector3d initial_pos_;

  // Adjoint matrix
  Eigen::Matrix<double, 6, 6> adjoint(Eigen::Affine3d transform);

  // ros::Subscriber sub_desire_pose_;
  // void desirePoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_controllers::impedance_paramConfig>>
      dynamic_reconfigure_param_;
  ros::NodeHandle dynamic_reconfigure_param_node_;
  void dynamicParamCallback(franka_controllers::impedance_paramConfig& config,
                               uint32_t level);
};

}  // namespace franka_example_controllers
