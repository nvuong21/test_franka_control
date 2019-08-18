// Publish robot state
// Publish topic: PandaStateController/robot_state

#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>

// For realtime publisher
// #include <franka_controllers/Float64Array.h>
#include <franka_controllers/RobotState.h>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_controllers {

class PandaStateController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaStateInterface,
                                                            franka_hw::FrankaModelInterface> {

  public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time& ) override;
    void update(const ros::Time&, const ros::Duration& period) override;

  private:
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

    realtime_tools::RealtimePublisher<franka_controllers::RobotState> state_publisher_;
    franka_hw::TriggerRate trigger_publish_;
};

} // namespace
