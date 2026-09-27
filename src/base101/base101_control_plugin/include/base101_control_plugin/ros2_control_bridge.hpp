#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>

namespace base101_control_plugin {

// Bridges ros2_control command/state interfaces to the Axon 2 firmware's
// per-servo topics (link101-fw, see HARDWARE.md): one std_msgs/Float64
// publisher per joint on <cmd_topic_prefix>/servo_<servo_id>/command, and a
// single sensor_msgs/JointState subscription (state_topic) matched BY NAME —
// not by array position, so a servo dropping off the bus or the firmware's
// discovery order changing can't silently reassign one joint's state to
// another's. This only covers the arm (position joints); locomotion talks to
// the firmware directly (no ros2_control on hardware for the wheels — see
// attic/README.md for why this plugin was parked and resurrected arm-only).
class ROS2ControlBridge : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ROS2ControlBridge)

  // Lifecycle
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // I/O
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters (from the <hardware><param> block in the xacro)
  std::string state_topic_{"/link101/joint_states"};
  std::string cmd_topic_prefix_{"/link101/servos"};

  // One entry per joint this component owns.
  std::vector<std::string> joints_;
  std::unordered_map<std::string, int> servo_id_;   // joint name -> servo_id param

  // Buffers
  std::unordered_map<std::string, double> cmd_pos_;             // rad
  std::unordered_map<std::string, bool> cmd_pos_received_;      // first real command seen
  std::unordered_map<std::string, double> pos_state_;           // rad
  std::unordered_map<std::string, double> vel_state_;           // rad/s

  // Holds commands at the current position until the firmware has reported a
  // real (non-startup-default) state for every joint — same reasoning as the
  // old wheel/camera bridge this was resurrected from: publishing a stale
  // default (usually 0.0) as the first command would snap the arm there.
  bool received_all_joint_states_{false};

  // ROS
  std::shared_ptr<rclcpp::Node> node_;
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::mutex state_mtx_;
  std::unordered_map<std::string, bool> have_state_;   // joint -> seen at least once

  void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

} // namespace base101_control_plugin
