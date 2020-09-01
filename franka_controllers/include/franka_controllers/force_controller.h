// Not validate
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <franka_controllers/force_controller_paramConfig.h>

// Realtime Publisher
#include <realtime_tools/realtime_publisher.h>
#include <franka_hw/trigger_rate.h>
#include <franka_controllers/Float64Array.h>

namespace franka_controllers {
class PandaForceController : public controller_interface::MultiInterfaceController<
                                      franka_hw::FrankaStateInterface,
                                      franka_hw::FrankaModelInterface,
                                      hardware_interface::EffortJointInterface> {

  public:
    // Controller Implementation
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

  private:
    // Handle to communicate (read, write) with the interfaces
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    // Limit torque (the difference from 2 continuous torque < delta_tau_max_)
    //
    // Input: tau_d_calculated - the torque calculated from the control formula
    //                 tau_J_d - previous command torque
    //
    // Output: the limited tau_d_calculated
    // (the same as before if difference < delta_tau_max_)
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);
    static constexpr double delta_tau_max_{1.0};

    //Force controller
    double desired_force_{0.0};
    double k_p_{0.0};
    double k_i_{0.0};
    double k_d_{0.0};
    Eigen::Matrix<double, 6, 1> force_ext_initial_; // Initial ext force offset
    Eigen::Matrix<double, 6, 1> force_error_int_;   // Sum of force error

    // DYnamic reconfigure
    double target_k_p_{0.0};
    double target_k_i_{0.0};
    double target_k_d_{0.0};
    double target_force_{0.0};
    double filter_params_{0.005};
    void updateDynamicReconfigure();

    std::unique_ptr<dynamic_reconfigure::Server<franka_controllers::force_controller_paramConfig>>
        dynamic_server_force_controller_param_;
    ros::NodeHandle dynamic_reconfigure_force_controller_param_node_;
    void ForceControllerParamCallback(franka_controllers::force_controller_paramConfig& config,
                                 uint32_t level);

    // realtime publisher
    realtime_tools::RealtimePublisher<Float64Array> publisher_;
    franka_hw::TriggerRate rate_trigger_{30.0};
    double elapsed_time_;
};
} // End namespace
