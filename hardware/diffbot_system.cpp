#include "calixto-ros-bot/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace calixto_ros_bot
{

hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.DiffBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Read config params
  cfg_.left_wheel_name  = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate        = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device           = info_.hardware_parameters["device"];
  cfg_.baud_rate        = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms       = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Setup both wheels
  wheel_left_.setup(cfg_.left_wheel_name,   cfg_.enc_counts_per_rev);
  wheel_right_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Validate joints — expect 1 command, 2 state interfaces each
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu command interfaces. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has '%s' command interface. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu state interfaces. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(wheel_left_.name, hardware_interface::HW_IF_POSITION, &wheel_left_.pos);
  state_interfaces.emplace_back(wheel_left_.name, hardware_interface::HW_IF_VELOCITY, &wheel_left_.vel);

  state_interfaces.emplace_back(wheel_right_.name, hardware_interface::HW_IF_POSITION, &wheel_right_.pos);
  state_interfaces.emplace_back(wheel_right_.name, hardware_interface::HW_IF_VELOCITY, &wheel_right_.vel);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(wheel_left_.name, hardware_interface::HW_IF_VELOCITY, &wheel_left_.cmd);
  command_interfaces.emplace_back(wheel_right_.name, hardware_interface::HW_IF_VELOCITY, &wheel_right_.cmd);

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  comms_.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  

  // for reading hardware
  if (!comms_.connected())
    return hardware_interface::return_type::ERROR;

  int left_ticks, right_ticks;
  // Binary protocol (2026-09-29): read_encoder_values() now returns false on
  // a timeout, bad sync byte, or CRC mismatch - distinguishable from a
  // genuine 0,0 reading, unlike the old ASCII protocol. On failure, skip
  // this cycle's position/velocity update entirely (keep the last known
  // values) rather than computing a bogus velocity spike from ticks that
  // were never actually read.
  if (!comms_.read_encoder_values(left_ticks, right_ticks))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000, "Telemetry read failed - keeping last known wheel state");
    return hardware_interface::return_type::OK;
  }

  // Store previous positions for velocity calculation
  double prev_left = wheel_left_.pos;
  double prev_right = wheel_right_.pos;

  // Convert ticks to radians
  wheel_left_.pos = left_ticks * wheel_left_.rads_per_count;
  wheel_right_.pos = right_ticks * wheel_right_.rads_per_count;

  // Velocity = delta position / delta time
  double dt = period.seconds();
  wheel_left_.vel = (wheel_left_.pos - prev_left) / dt;
  wheel_right_.vel = (wheel_right_.pos - prev_right) / dt;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
    return hardware_interface::return_type::ERROR;

      RCLCPP_INFO_THROTTLE(
    get_logger(), *clock_, 1000,
    "CMD: L=%.2f R=%.2f",
    wheel_left_.cmd,
    wheel_right_.cmd
  );

  comms_.set_motor_values(wheel_left_.cmd, wheel_right_.cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  calixto_ros_bot::DiffBotSystemHardware, hardware_interface::SystemInterface)
