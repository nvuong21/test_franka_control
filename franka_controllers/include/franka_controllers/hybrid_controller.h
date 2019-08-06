#include <memory>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <Eigen/Core>

// #include <realtime_tools/realtime_publisher.h>

namespace franka_controllers {

class PandaHybridController : public controller_interface::MultiInterfaceController<
                                  franka_hw::FrankaModelInterface,
                                  hardware_interface::EffortJointInterface,
                                  franka_hw::FrankaStateInterface
                                  /*,franka_hw::FrankaVelocityCartesianInterface*/> {

  public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

  private:
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);

    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    static constexpr double kDeltaTauMax{1.0};

    // double desired_force_{1.0};
    double kp_f_{0.0};
    double kd_v_{0.0};
    double ki_f_{0.0};

    double kp_c_{0.0};
    double kd_c_{0.0};
    double ki_c_{0.0};
    Eigen::Matrix<double, 7, 1> tau_ext_initial_;
    Eigen::Matrix<double, 7, 1> tau_error_;

    ros::Duration elapsed_time_;

    // Cartesian Interface
    // franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
    // std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;

};

}
