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

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <franka_controllers/hybrid_paramConfig.h>

// realtime_publisher
#include <realtime_tools/realtime_publisher.h>
#include <franka_hw/trigger_rate.h>
#include <franka_controllers/Float64Array.h>

namespace franka_controllers {
class HybridForceController : public controller_interface::MultiInterfaceController<
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

    static constexpr double delta_tau_max_{5.0};
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);

    // Friction model
    Eigen::Matrix<double, 7, 3> friction_coeffs_;

    // Parallel hybrid control
    Eigen::Matrix<double, 6, 6> S_;

    // z Force control
    double desired_force_{0.0};
    double kp_f_{0.0};
    double ki_f_{0.0};
    double kd_f_{0.0};
    Eigen::Matrix<double, 6, 1> force_ext_initial_;
    Eigen::Matrix<double, 6, 1> force_error_;
    Eigen::Matrix<double, 6, 1> force_prev_;
    Eigen::Matrix<double, 6, 1> force_filter_;

    // Averaging Filter
    size_t filter_size_{10};
    size_t current_filter_pos_{0};
    std::unique_ptr<double[]> filter_buffer_;
    Eigen::Matrix<double, 6, 1> getFitlerValue();
    void updateFitlerValue(const franka::RobotState& state);

    // x-y Motion control
    Eigen::Affine3d transform_;
    Eigen::Matrix<double, 6, 6> stiffness_m_;
    Eigen::Matrix<double, 6, 6> damping_m_;
    Eigen::Matrix<double, 6, 1> twist_d_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    // Adjoint transform
    Eigen::Matrix<double, 6, 6> adjoint(Eigen::Affine3d transform);

    // Dynamic reconfigure
    double target_force_{0.0};
    Eigen::Matrix<double, 6, 1> target_twist_;
    Eigen::Matrix<double, 6, 6> target_stiffness_m_;
    Eigen::Matrix<double, 6, 6> target_damping_m_;
    double filter_params_{0.01};
    void updateDynamicReconfigure();

    std::unique_ptr<dynamic_reconfigure::Server<franka_controllers::hybrid_paramConfig>>
          dynamic_server_param_;
    ros::NodeHandle dynamic_server_node_;
    void hybridParamCallback(franka_controllers::hybrid_paramConfig& config,
                             uint32_t level);

    //Realtime Publisher
    double elapsed_time_{0.0};
    realtime_tools::RealtimePublisher<franka_controllers::Float64Array> publisher_;
    franka_hw::TriggerRate rate_trigger_{100.0};
};
} // end namespace
