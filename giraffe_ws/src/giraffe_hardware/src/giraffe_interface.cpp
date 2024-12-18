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

namespace giraffe_controller
{
GiraffeInterface::GiraffeInterface() 
{
}

GiraffeInterface::~GiraffeInterface()
{
    if (SerialPort != -1)
    {
        close(SerialPort);
    }
}

CallbackReturn GiraffeInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  size_t num_joints = info_.joints.size();
  position_commands_.resize(num_joints, 0.0);
  position_states_.resize(num_joints, 0.0);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GiraffeInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GiraffeInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn GiraffeInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GiraffeInterface"), "Activating giraffe hardware interface...");

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    position_states_[i] = 0.0;
    position_commands_[i] = 0.0;
  }

  node_ = rclcpp::Node::make_shared("giraffe_hardware_interface_node");

  feedback_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "feedback", 10, std::bind(&GiraffeInterface::feedback_callback, this, std::placeholders::_1));

  command_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("command", 10);

  // Create an executor and spin in a separate thread
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {
    executor_->spin();
  });

  RCLCPP_INFO(rclcpp::get_logger("GiraffeInterface"), "Hardware interface activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn GiraffeInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("GiraffeInterface"), "Hardware interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// int GiraffeInterface::WriteToSerial(const unsigned char* buf, int nBytes)
// {
//     if (SerialPort == -1)
//     {
//         // Simulation mode
//         return nBytes; 
//     }

//     return ::write(SerialPort, const_cast<unsigned char*>(buf), nBytes);
// }

// int GiraffeInterface::ReadSerial(unsigned char* buf, int nBytes)
// {
//     if (SerialPort == -1)
//     {
//         // Simulation mode
//         std::fill(buf, buf + nBytes, 0);
//         return nBytes;
//     }

//     auto t_start = std::chrono::high_resolution_clock::now();
//     int n = 0;
//     while(n < nBytes)
//     {
//         int ret = ::read(SerialPort, &buf[n], 1);
//         if(ret < 0)
//         {
//             return ret;
//         }

//         n += ret;
//         auto t_end = std::chrono::high_resolution_clock::now();
//         double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
//         if(elapsed_time_ms > 10000)
//         {
//             break;
//         }
//     }
//     return n;
// }

void GiraffeInterface::feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // RCLCPP_INFO(rclcpp::get_logger("GiraffeInterface"), "Feedback received!");
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  last_feedback_msg_ = msg;
}

// In write, we publish the current commands as a JointState
hardware_interface::return_type GiraffeInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
{
  if (!command_publisher_)
    return hardware_interface::return_type::OK;

  // Publish the current command to "command" topic
  sensor_msgs::msg::JointState cmd_msg;
  cmd_msg.header.stamp = node_->now();

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    cmd_msg.name.push_back(info_.joints[i].name);
    cmd_msg.position.push_back(position_commands_[i]);
  }

  command_publisher_->publish(cmd_msg);

  return hardware_interface::return_type::OK;
}

// In read, we use the last received feedback message to update position_states_
hardware_interface::return_type GiraffeInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  sensor_msgs::msg::JointState::SharedPtr feedback_copy;

  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_copy = last_feedback_msg_;
  }

  if (!feedback_copy)
  {
    RCLCPP_INFO(rclcpp::get_logger("GiraffeInterface"), "No feedback received yet, no update");
    // No feedback received yet, no update
    return hardware_interface::return_type::OK;
  }

  // Map feedback positions to our internal joint states
  // Assuming the feedback message includes all joints in the same order as info_.joints
  // If order or joint set differs, you'd need a matching step
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // Log joint name and position
    // RCLCPP_INFO_STREAM(
    //   rclcpp::get_logger("GiraffeInterface"), "Joint " << info_.joints[i].name << " position: " << feedback_copy->position[i]
    // ); 
    // Find the corresponding joint in feedback
    auto it = std::find(feedback_copy->name.begin(), feedback_copy->name.end(), info_.joints[i].name);
    if (it != feedback_copy->name.end())
    {
      size_t idx = std::distance(feedback_copy->name.begin(), it);
      if (idx < feedback_copy->position.size())
      {
        position_states_[i] = feedback_copy->position[idx];
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace giraffe_controller

PLUGINLIB_EXPORT_CLASS(giraffe_controller::GiraffeInterface, hardware_interface::SystemInterface)
