#ifndef CALIXTO_ROS_BOT__DIFFBOT_SYSTEM_HPP_
#define CALIXTO_ROS_BOT__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "calixto-ros-bot/mcu_comms.hpp"
#include "calixto-ros-bot/wheel.hpp"

namespace calixto_ros_bot
{

class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
  struct Config
  {
    std::string left_wheel_name  = "";
    std::string right_wheel_name = "";
    float loop_rate         = 0.0;
    std::string device      = "";
    int baud_rate           = 0;
    int timeout_ms          = 0;
    int enc_counts_per_rev  = 0;
  };

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  rclcpp::Logger get_logger() const { return *logger_; }

  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  McuComms comms_;
  Config cfg_;

  Wheel wheel_left_, wheel_right_;

  std::shared_ptr<rclcpp::Logger> logger_;   // added
  rclcpp::Clock::SharedPtr clock_;           // added

  // MCU status (E-stop/bumper/CAN fault) publisher - 2026-09-30. A bare
  // internal rclcpp::Node just for this one publisher, created in
  // on_activate(); publishing needs no executor/spinning since this node
  // never subscribes to or services anything. See mcu_comms.hpp's
  // status_flags()/front_estop_active()/etc. for the data this publishes.
  rclcpp::Node::SharedPtr diagnostics_node_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace ros2_control_demo_example_2

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
