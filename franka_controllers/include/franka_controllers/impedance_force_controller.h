// not validate
#include <vector>

// Interface used
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
// #include <franka_hw::FrankaVelocityCartesianInterface>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>  // For effortInterface
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <realtime_tools/realtime_publisher.h>
#include <franka_hw/trigger_rate.h>
#include <franka_controllers/Float64Array.h>


namespace franka_controllers {

class ImpedanceForceController : public controller_interface::MultiInterfaceController<
                                        hardware_interface::EffortJointInterface,
                                        /*franka_hw::FrankaVelocityCartesianInterface,*/
                                        franka_hw::FrankaStateInterface,
                                        franka_hw::FrankaModelInterface> {
  public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time& ) override;
    void update(const ros::Time& ,const ros::Duration& period) override;

  private:
    // Ensure that d_torque < delta_tau_max_
    const double delta_tau_max_{1.0};
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);

    // Handle to available interfaces
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
    // std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;

    // Parameters
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    double k_collision_{0};

    Eigen::Matrix<double, 7, 1> tau_ext_initial_;
    Eigen::Matrix<double, 6, 1> wrench_ext_initial_;

    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;

    ros::Duration elapsed_time_;

    double vel_{0.05};
    double des_force_{5.0};

    // RealtimePublisher
    franka_hw::TriggerRate rate_trigger_{30.0};
    realtime_tools::RealtimePublisher<Float64Array> publisher_;
};
}
