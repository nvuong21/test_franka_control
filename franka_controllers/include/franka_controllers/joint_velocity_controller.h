// Not validate
// The controller publish joint velocity and position
// through "joint_velocity_controller/data" topic

#include <array>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

// For Realtime publisher
#include <realtime_tools/realtime_publisher.h>
#include <franka_controllers/Float64Array.h>
#include <franka_hw/trigger_rate.h>

namespace franka_controllers {
class PandaJointVelocityController : public controller_interface::MultiInterfaceController<
                                      franka_hw::FrankaModelInterface,
                                      hardware_interface::EffortJointInterface,
                                      hardware_interface::VelocityJointInterface,
                                      franka_hw::FrankaStateInterface> {
  public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

  private:
    const double delta_tau_max_{1.0};
    // saturateTorqueRate ensure that the difference in command torque < delta_tau_max_
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
         const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
         const Eigen::Matrix<double, 7, 1>& tau_J_d);

    // Handle to access to the hardware_interface
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
    std::vector<hardware_interface::JointHandle> effort_joint_handles_;

    // Function to generate an example motion
    std::vector<double> generateTrajectory();
    std::vector<double> trajectory_;
    size_t index_;

    // PD gains
    double kp_{1.0};
    double kd_{0.0};

    // Realtime publisher
    ros::Duration elapsed_time_;
    franka_hw::TriggerRate rate_trigger_{1.0};
    realtime_tools::RealtimePublisher<Float64Array> publisher_;

    // Filter velocity
    const double filter_params_{0.02};
    std::array<double, 7> dq_filtered_{};

    // Filter strategy 2
    const size_t dq_filter_size_{5};
    std::unique_ptr<double[]> dq_buffer_;
    size_t dq_current_filter_position_{0};

    double getDQFiltered(size_t index);
    void updateDQFiltered(const franka::RobotState& state);

};

} //namespace franka_controllers
