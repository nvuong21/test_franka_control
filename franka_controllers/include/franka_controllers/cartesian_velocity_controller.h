#include <vector>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <Eigen/Core>

// For Realtime publisher
#include <realtime_tools/realtime_publisher.h>
#include <franka_hw/trigger_rate.h>
#include <franka_controllers/Float64Array.h>

namespace franka_controllers {
class PandaCartesianVelocityController : public controller_interface::MultiInterfaceController<
                                          franka_hw::FrankaStateInterface,
                                          franka_hw::FrankaModelInterface,
                                          franka_hw::FrankaVelocityCartesianInterface,
                                          hardware_interface::EffortJointInterface> {
  public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

  private:
    // Saturation
    const double delta_tau_max_{1.0};
    // saturateTorqueRate ensure that the difference in command torque < delta_tau_max_
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
         const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
         const Eigen::Matrix<double, 7, 1>& tau_J_d);

    // Interface handles
    std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> cartesian_velocity_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    // Function to generate an example motion
    std::vector<std::array<double, 6>> generateTrajectory();
    std::vector<std::array<double, 6>> trajectory_;
    size_t index_;

    // PD gains
    Eigen::Matrix<double, 7, 7> k_gains_;
    Eigen::Matrix<double, 7, 7> d_gains_;

    // RealtimePublisher
    ros::Duration elapsed_time_;
    franka_hw::TriggerRate rate_trigger_{1.0};
    realtime_tools::RealtimePublisher<Float64Array> publisher_;

};
} // namespace franka_controllers
