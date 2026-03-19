#include "so_arm_100_hardware/so_arm_100_interface.hpp"
#include <chrono>
#include <cmath>
#include <errno.h>
#include <fcntl.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <iostream>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace so_arm_100_controller {
SOARM100Interface::SOARM100Interface() {}

SOARM100Interface::~SOARM100Interface() {
  if (use_serial_) {
    st3215_.end();
  }
}

CallbackReturn SOARM100Interface::on_init(
    const hardware_interface::HardwareInfo &hardware_info) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  CallbackReturn result =
      hardware_interface::SystemInterface::on_init(hardware_info);
#pragma GCC diagnostic pop
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  use_serial_ =
      hardware_info.hardware_parameters.count("use_serial")
          ? (hardware_info.hardware_parameters.at("use_serial") == "true")
          : false;

  serial_port_ = hardware_info.hardware_parameters.count("serial_port")
                     ? hardware_info.hardware_parameters.at("serial_port")
                     : "/dev/ttyUSB0";

  // Check both common parameter names for baud rate
  if (hardware_info.hardware_parameters.count("serial_baudrate")) {
    serial_baudrate_ =
        std::stoi(hardware_info.hardware_parameters.at("serial_baudrate"));
  } else if (hardware_info.hardware_parameters.count("baud_rate")) {
    serial_baudrate_ =
        std::stoi(hardware_info.hardware_parameters.at("baud_rate"));
  } else {
    serial_baudrate_ = 1000000;
  }

  RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
              "Hardware info: use_serial=%s, serial_port=%s, baudrate=%d",
              use_serial_ ? "true" : "false", serial_port_.c_str(),
              serial_baudrate_);

  size_t num_joints = info_.joints.size();
  position_commands_.resize(num_joints, 0.0);
  position_states_.resize(num_joints, 0.0);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SOARM100Interface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name,
                                  hardware_interface::HW_IF_POSITION,
                                  &position_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SOARM100Interface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &position_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn SOARM100Interface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
              "Activating so_arm_100 hardware interface...");

  if (use_serial_) {
    if (!st3215_.begin(serial_baudrate_, serial_port_.c_str())) {
      RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"),
                   "Failed to initialize motors");
      return CallbackReturn::ERROR;
    }

    st3215_.IOTimeOut = 10; // More conservative timeout for initial discovery
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      uint8_t servo_id = static_cast<uint8_t>(i + 1);

      // First ping the servo with retries
      bool ping_success = false;
      for (int retry = 0; retry < 3; ++retry) {
        if (st3215_.Ping(servo_id) != -1) {
          ping_success = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      if (!ping_success) {
        RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"),
                    "No response from servo %d after retries. Continuing in "
                    "degraded mode...",
                    servo_id);
        position_states_[i] = 0.0;
        position_commands_[i] = 0.0;
        continue;
      }

      // Set to position control mode
      if (!st3215_.Mode(servo_id, 0)) {
        RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"),
                    "Failed to set mode for servo %d. Continuing...", servo_id);
        continue;
      }

      // Read initial position and set command to match
      if (st3215_.FeedBack(servo_id) != -1) {
        int pos = st3215_.ReadPos(-1); // Use cached read
        position_states_[i] = ticks_to_radians(pos, i);
        position_commands_[i] = position_states_[i];
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
                    "Servo %d initialized at position %d", servo_id, pos);
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
                "Serial communication initialized on %s", serial_port_.c_str());
  }

  // NOTE: Node creation and Services were removed here to fix ROS2 Jazzy
  // Segmentation Faults. The hardware interface now focuses purely on
  // read/write loops.

  // Load calibration
  std::string calib_file =
      info_.hardware_parameters.count("calibration_file")
          ? info_.hardware_parameters.at("calibration_file")
          : "";

  if (!calib_file.empty()) {
    if (!load_calibration(calib_file)) {
      RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"),
                  "Failed to load calibration file: %s", calib_file.c_str());
    }
  }

  // Enable torque by default on activation
  set_torque_enable(true);

  RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
              "Hardware interface activated");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
SOARM100Interface::on_deactivate(const rclcpp_lifecycle::State &) {
  // NOTE: Removed executor/thread cleanup as they were removed to fix
  // SegFaults.

  if (use_serial_) {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      uint8_t servo_id = static_cast<uint8_t>(i + 1);
      st3215_.EnableTorque(servo_id, 0);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
              "Hardware interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

#if 0
void SOARM100Interface::feedback_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  last_feedback_msg_ = msg;
}
#endif

hardware_interface::return_type
SOARM100Interface::write(const rclcpp::Time &time,
                         const rclcpp::Duration &period) {
  (void)time;
  (void)period;
  if (use_serial_ && torque_enabled_) { // Only write if torque is enabled
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      uint8_t servo_id = static_cast<uint8_t>(i + 1);
      // Convert from radians (-π to π) to servo ticks (0-4095)
      int joint_pos_cmd = radians_to_ticks(position_commands_[i], i);

      RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"),
                   "Servo %d command: %.2f rad -> %d ticks", servo_id,
                   position_commands_[i], joint_pos_cmd);

      if (!st3215_.RegWritePosEx(servo_id, joint_pos_cmd, 4500, 255)) {
        RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"),
                    "Failed to write position to servo %d", servo_id);
      }
    }
    st3215_.RegWriteAction();
  }

  // NOTE: Command publisher removed to fix Jazzy SegFaults.

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SOARM100Interface::read(const rclcpp::Time &time,
                        const rclcpp::Duration &period) {
  (void)time;
  (void)period;
  if (use_serial_) {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      uint8_t servo_id = static_cast<uint8_t>(i + 1);

      if (st3215_.FeedBack(servo_id) != -1) {
        int raw_pos = st3215_.ReadPos(-1);
        position_states_[i] = ticks_to_radians(raw_pos, i);

        if (torque_enabled_) {
          double speed = -1 * st3215_.ReadSpeed(-1) * 2 * M_PI / 4096.0;
          double pwm = -1 * st3215_.ReadLoad(-1) / 10.0;
          double temperature = st3215_.ReadTemper(-1);
          double voltage = st3215_.ReadVoltage(-1) / 10.0;
          double current = st3215_.ReadCurrent(-1) * 6.5 / 1000.0;

          RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"),
                       "Servo %d: raw_pos=%d (%.2f rad) speed=%.2f pwm=%.2f "
                       "temp=%.1f V=%.1f I=%.3f",
                       servo_id, raw_pos, position_states_[i], speed, pwm,
                       temperature, voltage, current);
        }
      } else {
        RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"),
                    "Failed to read feedback from servo %d", servo_id);
      }
    }
  } else {
    sensor_msgs::msg::JointState::SharedPtr feedback_copy;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      feedback_copy = last_feedback_msg_;
    }

    if (feedback_copy) {
      for (size_t i = 0; i < info_.joints.size(); ++i) {
        auto it = std::find(feedback_copy->name.begin(),
                            feedback_copy->name.end(), info_.joints[i].name);
        if (it != feedback_copy->name.end()) {
          size_t idx = std::distance(feedback_copy->name.begin(), it);
          if (idx < feedback_copy->position.size()) {
            position_states_[i] =
                ticks_to_radians(feedback_copy->position[idx], i);
          }
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

void SOARM100Interface::calibrate_servo(uint8_t servo_id, int current_pos) {
  size_t idx = servo_id - 1;
  // Calculate offset from current position to expected zero
  int offset = current_pos - zero_positions_[idx];
  RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
              "Servo %d: current=%d, zero=%d, offset=%d", servo_id, current_pos,
              zero_positions_[idx], offset);
}

double SOARM100Interface::ticks_to_radians(int ticks, size_t servo_idx) {
  const std::string &joint_name = info_.joints[servo_idx].name;

  if (joint_calibration_.count(joint_name) > 0) {
    const auto &calib = joint_calibration_[joint_name];
    // Convert to normalized position first (0 to 1)
    double normalized = (double)(ticks - calib.min_ticks) / calib.range_ticks;
    // Then convert to radians (-π to π)
    return (normalized * 2.0 - 1.0) * M_PI;
  }

  // Fallback to default calibration
  return servo_directions_[servo_idx] * (ticks - zero_positions_[servo_idx]) *
         2 * M_PI / 4096.0;
}

int SOARM100Interface::radians_to_ticks(double radians, size_t servo_idx) {
  const std::string &joint_name = info_.joints[servo_idx].name;

  if (joint_calibration_.count(joint_name) > 0) {
    const auto &calib = joint_calibration_[joint_name];
    // Convert from radians (-π to π) to normalized position (0 to 1)
    double normalized = (radians / M_PI + 1.0) / 2.0;
    // Then convert to ticks
    return calib.min_ticks + (int)(normalized * calib.range_ticks);
  }

  // Fallback to default calibration
  return zero_positions_[servo_idx] +
         servo_directions_[servo_idx] * (int)(radians * 4096.0 / (2 * M_PI));
}

#if 0
void SOARM100Interface::record_current_position() {
...
void SOARM100Interface::calibration_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  record_current_position();
  response->success = true;
  response->message = last_calibration_data_;
}
#endif

void SOARM100Interface::set_torque_enable(bool enable) {
  if (use_serial_) {
    // First set all servos
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      uint8_t servo_id = static_cast<uint8_t>(i + 1);

      if (!enable) {
        // When disabling:
        // 1. Set to idle mode first
        st3215_.Mode(servo_id, 2); // Mode 2 = idle
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // 2. Disable torque
        st3215_.EnableTorque(servo_id, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // 3. Double check it's disabled
        st3215_.EnableTorque(servo_id, 0);
      } else {
        // When enabling:
        // 1. Set position mode
        st3215_.Mode(servo_id, 0); // Mode 0 = position
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // 2. Enable torque
        st3215_.EnableTorque(servo_id, 1);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Wait a bit to ensure commands are processed
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Update state after all servos are set
    torque_enabled_ = enable;

    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
                "Torque %s for all servos", enable ? "enabled" : "disabled");
  }
}

#if 0
void SOARM100Interface::torque_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  bool new_state = !torque_enabled_;

  // Set response before changing state
  response->success = true;
  response->message =
      std::string("Torque ") + (new_state ? "enabled" : "disabled");

  // Change state after setting response
  set_torque_enable(new_state);

  RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
              "Torque service called, response: %s", response->message.c_str());
}
#endif

bool SOARM100Interface::load_calibration(const std::string &filepath) {
  try {
    YAML::Node config = YAML::LoadFile(filepath);
    auto joints = config["joints"];
    if (!joints) {
      RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"),
                   "No joints section in calibration file");
      return false;
    }

    for (const auto &joint : joints) {
      std::string name = joint.first.as<std::string>();
      const auto &data = joint.second;

      if (!data["min"] || !data["center"] || !data["max"]) {
        RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"),
                     "Missing calibration data for joint %s", name.c_str());
        continue;
      }

      JointCalibration calib;
      calib.min_ticks = data["min"]["ticks"].as<int>();
      calib.center_ticks = data["center"]["ticks"].as<int>();
      calib.max_ticks = data["max"]["ticks"].as<int>();
      calib.range_ticks = calib.max_ticks - calib.min_ticks;

      joint_calibration_[name] = calib;

      RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
                  "Loaded calibration for %s: min=%d, center=%d, max=%d",
                  name.c_str(), calib.min_ticks, calib.center_ticks,
                  calib.max_ticks);
    }
    return true;
  } catch (const YAML::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"),
                 "Failed to load calibration: %s", e.what());
    return false;
  }
}

double SOARM100Interface::normalize_position(const std::string &joint_name,
                                             int ticks) {
  if (joint_calibration_.count(joint_name) == 0) {
    return 0.0;
  }

  const auto &calib = joint_calibration_[joint_name];
  double normalized = (ticks - calib.min_ticks) / calib.range_ticks;
  return std::clamp(normalized, 0.0, 1.0);
}

} // namespace so_arm_100_controller

PLUGINLIB_EXPORT_CLASS(so_arm_100_controller::SOARM100Interface,
                       hardware_interface::SystemInterface)
