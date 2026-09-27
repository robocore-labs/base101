#include "base101_control_plugin/ros2_control_bridge.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <limits>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace base101_control_plugin {

CallbackReturn ROS2ControlBridge::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  auto it = info_.hardware_parameters.find("state_topic");
  if (it != info_.hardware_parameters.end()) state_topic_ = it->second;
  it = info_.hardware_parameters.find("cmd_topic_prefix");
  if (it != info_.hardware_parameters.end()) cmd_topic_prefix_ = it->second;

  for (const auto & j : info_.joints) {
    bool has_pos_cmd = false;
    for (const auto & ci : j.command_interfaces) {
      if (ci.name == HW_IF_POSITION) has_pos_cmd = true;
    }
    if (!has_pos_cmd) continue;

    auto id_it = j.parameters.find("servo_id");
    if (id_it == j.parameters.end()) {
      RCLCPP_FATAL(rclcpp::get_logger("base101_control_plugin"),
        "joint '%s' has a position command interface but no <param name=\"servo_id\">"
        " — this plugin addresses the Feetech bus by ID, not by array position.",
        j.name.c_str());
      return CallbackReturn::ERROR;
    }

    joints_.push_back(j.name);
    servo_id_[j.name] = std::stoi(id_it->second);
    cmd_pos_[j.name] = std::numeric_limits<double>::quiet_NaN();
    pos_state_[j.name] = 0.0;
    vel_state_[j.name] = 0.0;
    have_state_[j.name] = false;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ROS2ControlBridge::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto & name : joints_) {
    state_interfaces.emplace_back(name, HW_IF_POSITION, &pos_state_[name]);
    state_interfaces.emplace_back(name, HW_IF_VELOCITY, &vel_state_[name]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ROS2ControlBridge::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & name : joints_) {
    command_interfaces.emplace_back(name, HW_IF_POSITION, &cmd_pos_[name]);
  }
  return command_interfaces;
}

CallbackReturn ROS2ControlBridge::on_configure(const rclcpp_lifecycle::State &)
{
  node_ = std::make_shared<rclcpp::Node>("base101_control_plugin_bridge");

  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  for (const auto & name : joints_) {
    const std::string topic =
      cmd_topic_prefix_ + "/servo_" + std::to_string(servo_id_[name]) + "/command";
    cmd_pub_[name] = node_->create_publisher<std_msgs::msg::Float64>(topic, pub_qos);
  }

  auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, sub_qos,
    std::bind(&ROS2ControlBridge::state_callback, this, std::placeholders::_1));

  exec_.add_node(node_);

  RCLCPP_INFO(node_->get_logger(),
    "Configured base101_control_plugin bridge: %zu joint(s), cmd_topic_prefix=%s state_topic=%s",
    joints_.size(), cmd_topic_prefix_.c_str(), state_topic_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn ROS2ControlBridge::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(),
    "Bridge activated — holding at current position until every joint has "
    "reported at least one real state from %s", state_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ROS2ControlBridge::on_deactivate(const rclcpp_lifecycle::State &)
{
  exec_.remove_node(node_);
  state_sub_.reset();
  cmd_pub_.clear();
  node_.reset();
  return CallbackReturn::SUCCESS;
}

void ROS2ControlBridge::state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lk(state_mtx_);

  for (size_t i = 0; i < msg->name.size(); ++i) {
    const auto & name = msg->name[i];
    if (!pos_state_.count(name)) continue;  // not one of ours (or firmware hasn't
                                             // been configured with this joint's
                                             // name in its servo ID table yet)

    if (i < msg->position.size()) pos_state_[name] = msg->position[i];
    if (i < msg->velocity.size()) vel_state_[name] = msg->velocity[i];
    have_state_[name] = true;
  }
}

return_type ROS2ControlBridge::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  exec_.spin_some();
  return return_type::OK;
}

return_type ROS2ControlBridge::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  {
    std::scoped_lock<std::mutex> lk(state_mtx_);

    if (!received_all_joint_states_) {
      bool all = true;
      for (const auto & j : joints_) {
        if (!have_state_[j]) { all = false; break; }
      }
      if (all) {
        received_all_joint_states_ = true;
        RCLCPP_INFO(node_->get_logger(),
          "All joints reported state at least once — accepting controller commands");
      } else {
        // Hold whichever joints HAVE reported at their current position, so a
        // controller command written before the bridge is fully up can't
        // snap a joint to a stale/default 0.0 the instant the rest catch up.
        for (const auto & j : joints_) {
          if (have_state_[j]) cmd_pos_[j] = pos_state_[j];
        }
      }
    }
  }

  for (const auto & j : joints_) {
    const double value = cmd_pos_[j];
    if (std::isnan(value)) continue;  // no command written to this interface yet
    std_msgs::msg::Float64 out;
    out.data = value;
    cmd_pub_[j]->publish(out);
  }

  return return_type::OK;
}

} // namespace base101_control_plugin

PLUGINLIB_EXPORT_CLASS(base101_control_plugin::ROS2ControlBridge, hardware_interface::SystemInterface)
