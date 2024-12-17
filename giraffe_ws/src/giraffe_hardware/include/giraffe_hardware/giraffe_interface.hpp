#ifndef ARDUINO_INTERFACE_H
#define ARDUINO_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/macros.hpp"
#include <hardware_interface/system_interface.hpp>

#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <vector>
#include <string>
#include <memory>
#include <termios.h>

#include <sensor_msgs/msg/joint_state.hpp>

namespace arduino_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ArduinoInterface : public hardware_interface::SystemInterface
{
public:
  ArduinoInterface();
  virtual ~ArduinoInterface();

  // LifecycleNodeInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // SystemInterface
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Position command and state storage for all joints
  std::vector<double> position_commands_;
  std::vector<double> position_states_;

  // Simulated feedback parameters
  double simulation_speed_factor_ = 0.05; // How fast actual position approaches commanded position

  int SerialPort = -1;
  struct termios tty;
  int WriteToSerial(const unsigned char* buf, int nBytes);
  int ReadSerial(unsigned char* buf, int nBytes);

  // ROS interfaces
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
};

}  // namespace arduino_controller

#endif  // ARDUINO_INTERFACE_H
