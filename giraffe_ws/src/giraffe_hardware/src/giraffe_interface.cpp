#include "giraffe_hardware/giraffe_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

namespace arduino_controller
{
ArduinoInterface::ArduinoInterface() 
{
}

ArduinoInterface::~ArduinoInterface()
{
    if (SerialPort != -1)
    {
        close(SerialPort);
    }
}

CallbackReturn ArduinoInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  // We expect 6 joints based on the YAML configuration. The actual count is derived from hardware_info.
  size_t num_joints = info_.joints.size();
  position_commands_.resize(num_joints, 0.0);
  position_states_.resize(num_joints, 0.0);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // We only need position state interfaces as per configuration
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Position commands
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn ArduinoInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Activating 5DoF arm and gripper hardware interface...");

  // Initialize positions and commands
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    position_states_[i] = 0.0;
    position_commands_[i] = 0.0;
  }

  // Create a separate rclcpp Node for publishing joint states for debugging
  node_ = rclcpp::Node::make_shared("arduino_hardware_interface_node");
  state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("debug_joint_states", 10);

  RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Hardware interface activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArduinoInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (SerialPort != -1)
    {
        tcflush(SerialPort, TCIFLUSH);
        close(SerialPort);
        SerialPort = -1;
    }
    RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Hardware interface deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

int ArduinoInterface::WriteToSerial(const unsigned char* buf, int nBytes)
{
    if (SerialPort == -1)
    {
        // Serial port not set up - simulation only
        return nBytes; 
    }

    return ::write(SerialPort, const_cast<unsigned char*>(buf), nBytes);
}

int ArduinoInterface::ReadSerial(unsigned char* buf, int nBytes)
{
    if (SerialPort == -1)
    {
        // No actual hardware read - simulation
        std::fill(buf, buf + nBytes, 0);
        return nBytes;
    }

    auto t_start = std::chrono::high_resolution_clock::now();
    int n = 0;
    while(n < nBytes)
    {
        int ret = ::read(SerialPort, &buf[n], 1);
        if(ret < 0)
        {
            return ret;
        }

        n+=ret;
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        if(elapsed_time_ms > 10000)
        {
            break;
        }
    }
    return n;
}

// Write commands to the hardware (currently just simulating)
hardware_interface::return_type ArduinoInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // In a real hardware scenario, you would send position_commands_ to the controller or actuators here.
  // For now, we are just simulating.

  // RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Received new position commands:");
  // for (size_t i = 0; i < info_.joints.size(); ++i)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "  Joint '%s' command: %.3f", info_.joints[i].name.c_str(), position_commands_[i]);
  // }

  // You could send these commands to Arduino or other hardware here, if set up.
  // For debugging, we do no actual hardware write beyond logging.

  return hardware_interface::return_type::OK;
}

// Read states from the hardware (simulated feedback)
hardware_interface::return_type ArduinoInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();

  // Simulate the arm moving towards the commanded positions
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    double error = position_commands_[i] - position_states_[i];
    // Move the actual state a fraction of the way to the commanded position
    position_states_[i] = position_commands_[i];
  }

  // Publish for debugging
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    msg.name.push_back(info_.joints[i].name);
    msg.position.push_back(position_states_[i]);
    // If desired, you can add velocity or effort sim if needed
  }

  state_publisher_->publish(msg);

  // RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Updated joint states and published to 'debug_joint_states'.");

  return hardware_interface::return_type::OK;
}

}  // namespace arduino_controller

PLUGINLIB_EXPORT_CLASS(arduino_controller::ArduinoInterface, hardware_interface::SystemInterface)
