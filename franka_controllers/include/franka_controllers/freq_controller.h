#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

namespace franka_controllers{
class FreqController : public controller_interface::MultiInterfaceController<
                                franka_hw::FrankaStateInterface,
                                franka_hw::FrankaModelInterface,
                                hardware_interface::EffortJointInterface> {

  public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void stopping(const ros::Time&) override;

  private:
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    static constexpr double delta_tau_max_{1.0};
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);

    // Dynamic reconfigure
    double target_frequency{0.0};
    double target_amplitude{0.0};
}
}
