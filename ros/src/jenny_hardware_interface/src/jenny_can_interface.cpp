#include "jenny_can_interface.hpp"

namespace jenny_hardware_interface {

////////////////////// on_init /////////////////////////
CallbackReturn
JennyHardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");

  // robot has 6 joints and 2 interfaces
  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);

  for (const auto &joint : info_.joints) {
    for (const auto &interface : joint.state_interfaces) {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }
  RCLCPP_INFO(logger, "Initializing Jenny Hardware Interface...");

  // config_.device = info_.hardware_parameters["device"];
  // config_.homing = info_.hardware_parameters["homing"];
  // if (info_.hardware_parameters["debug"] == "true") {
  //   config_.debug = true;
  // } else {
  //   config_.debug = false;
  // }
  config_.debug = false;
  config_.correction = true;

  // thread for receiving can responses
  stop_thread_.store(false);
  can_response_thread_ = std::thread(&JennyHardwareInterface::handleCANResponses, this);

  return CallbackReturn::SUCCESS;
}

////////////////////// on_configure /////////////////////////
CallbackReturn
JennyHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");

  // create can connection
  if (config_.debug) {RCLCPP_INFO(logger, "Connecting to Can Bus...");}
  // Initialize Sender
  try {
    if (config_.debug) {RCLCPP_INFO(logger, "Initializing CAN Sender...");}
    sender = std::make_unique<drivers::socketcan::SocketCanSender>("can0");
    if (config_.debug) {RCLCPP_INFO(logger, "CAN Sender initialized and opened successfully.");}

  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(logger, "Failed to configure SocketCAN: %s", e.what());
    sender.reset();
    return CallbackReturn::FAILURE;

  } catch (...) {
    RCLCPP_FATAL(logger, "An unexpected error occurred during SocketCAN configuration.");
    sender.reset();
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

////////////////////// on_activate /////////////////////////
CallbackReturn
JennyHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");

  for (int i = 0; i < 6; i++) {
    joint_position_[i] = 0.0;
    joint_velocities_[i] = 0.0;
  }

  // TODO: configure homing and check position before starting

  ready = true;

  return CallbackReturn::SUCCESS;
}


////////////////////// read /////////////////////////
return_type JennyHardwareInterface::read(const rclcpp::Time & /*time*/,
                                          const rclcpp::Duration &) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");
  if (ready == false) {
    return return_type::OK;
  }

  // for each joint
  for (int i = 0; i < 6; i++) {
    // get current position
    joint_position_[i] = getJointPosition(i);
  }

  // calculate speeds
  calculateMotorVelocities();

  // for each motor
  bool check;
  for (int id = 1; id < 7; id++) {
    // request position for next run
    check = requestPosition(id);
    if (!check) {
      RCLCPP_ERROR(logger, "Failed to request Position from motor with id: %d" , id);
    }
  }

  // debug:
  // printJointInfo(1);

  return return_type::OK;
}

////////////////////// write /////////////////////////
return_type JennyHardwareInterface::write(const rclcpp::Time &,
                                           const rclcpp::Duration &) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");

  // dont write if not ready yet
  if (ready == false) {
    return return_type::OK;
  }

  double diff = 0.0;

  // joint 1-4
  for (int i = 0; i < 4; i++) {
    // calculate difference in position
    diff = abs(joint_position_command_[i] - joint_position_[i]);
    double speed = abs(joint_velocities_command_[i] * MotorConstants::RADPS_TO_RPM);
    double position = joint_position_command_[i] * MotorConstants::RAD_TO_DEG;
    // joint position correction
    if (diff > 0.001 && speed < 0.00001 && config_.correction) {
      RCLCPP_INFO(logger, "Starting joint position correction on joint: %d", i);
      speed = 0.5;
    }
    setJointPosition(i, position, speed, RobotConstants::AXIS_ACCELERATION[i]);
  }
  // joint 5-6
  double speed = abs(joint_velocities_command_[4] * MotorConstants::RADPS_TO_RPM);
  double position_B = joint_position_command_[4] * MotorConstants::RAD_TO_DEG;
  double position_C = joint_position_command_[5] * MotorConstants::RAD_TO_DEG;
  double BC_position[2] = {position_B, position_C};
  diff = abs(joint_position_command_[4] - joint_position_[4]) + abs(joint_position_command_[5] - joint_position_[5]);
  // joint position correction
  if (diff > 0.001 && speed < 0.00001 && config_.correction) {
    RCLCPP_INFO(logger, "Starting joint position correction on BC joints");
    speed = 0.5;
  }
  setBCJointPosition(BC_position, speed, RobotConstants::AXIS_ACCELERATION[4]);

  return return_type::OK;
}

////////////////////// send Data /////////////////////////
bool JennyHardwareInterface::sendData(uint8_t id, std::vector<uint8_t> data_vec) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");
  // create can id
  drivers::socketcan::CanId canId = drivers::socketcan::CanId(id, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
  // creating crc
  uint16_t crc = id;
  for (size_t i = 0; i < data_vec.size(); i++) {
    crc += data_vec[i];
  }
  crc &= 0xFF;  // Keep only lower byte
  // append crc to data
  data_vec.push_back(static_cast<uint8_t>(crc));

  // try send data
  try {
    sender->send(data_vec.data(), data_vec.size(), canId);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Error sending CAN frame: %s", e.what());
    return false;
  }
  return true;
}
////////////////////// set Normal (1-4) Joint Position
bool JennyHardwareInterface::setJointPosition(uint8_t id, double position, double speed, double acceleration) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");
  if (id < 4) {
    bool check = setMotorPosition((id + 1), (position * RobotConstants::AXIS_RATIO[id]), (speed * RobotConstants::AXIS_RATIO[id]), acceleration);
    return check;
  }
  if (config_.debug) {
    RCLCPP_INFO(logger, "Setting Position: ID: %d CurrentPos; %f Position: %.3f Speed: %.3f", id, (joint_position_[id] * MotorConstants::RAD_TO_DEG * RobotConstants::AXIS_RATIO[id]), (position * RobotConstants::AXIS_RATIO[id]), (speed * RobotConstants::AXIS_RATIO[id]));
  }
  return true;
}
////////////////////// set BC (5-6) Joint Position
bool JennyHardwareInterface::setBCJointPosition(double position[2], double speed, double acceleration) {
  double motor_5_position = position[0] - position[1];
  double motor_6_position = position[0] + position[1];
  bool check1 = setMotorPosition(5, (motor_5_position * RobotConstants::AXIS_RATIO[4]), (speed * RobotConstants::AXIS_RATIO[4]), acceleration);
  bool check2 = setMotorPosition(6, (motor_6_position * RobotConstants::AXIS_RATIO[5]), (speed * RobotConstants::AXIS_RATIO[5]), acceleration);
  return (check1 && check2);
}
////////////////////// set Motor Position
bool JennyHardwareInterface::setMotorPosition(uint8_t id, double position, double speed, double acceleration) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");

  // setting up values for can message
  int32_t position_value = static_cast<int32_t>(position);
  position_value *= MotorConstants::ENCODER_STEPS;
  position_value *= RobotConstants::AXIS_SET_INVERTED[id - 1];
  position_value /= MotorConstants::DEGREES_PER_REVOLUTION;
  uint16_t speed_value = static_cast<uint16_t>(std::clamp(speed, 0.0, 3000.0));  // Default speed
  uint8_t acceleration_value = static_cast<uint8_t>(std::clamp(acceleration, 0.0, 255.0));

  // creating data without crc
  std::vector<uint8_t> data = {
    CANCommands::ABSOLUTE_POSITION,  // 0xF5
    static_cast<uint8_t>((speed_value >> 8) & 0xFF),  // Speed high byte
    static_cast<uint8_t>(speed_value & 0xFF),         // Speed low byte
    acceleration_value,                                  // Acceleration
    static_cast<uint8_t>((position_value >> 16) & 0xFF),  // Position high byte
    static_cast<uint8_t>((position_value >> 8) & 0xFF),   // Position middle byte
    static_cast<uint8_t>(position_value & 0xFF)           // Position low byte
  };

  // sending data
  bool check = sendData(id, data);
  return check;
}

////////////////////// request Position
bool JennyHardwareInterface::requestPosition(uint8_t id) { 
  std::vector<uint8_t> data = {0x31};
  bool check = sendData(id, data);
  return check;
}

////////////////////// calculate Motor Speed
void JennyHardwareInterface::calculateMotorVelocities() { 
  // calculate time difference
  current_time_ = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time_ - previous_time_);
  auto milliseconds = duration.count();

  // find out speed
  for (int i = 0; i < 6; i++) {
    motor_velocity_buffer_[i] = (motor_position_buffer_[i] - motor_previous_position_buffer_[i]) / milliseconds;
    motor_velocity_buffer_[i] *= 1000; // to seconds
    motor_velocity_buffer_[i] *= MotorConstants::DEG_TO_RAD; // to rads
  }

  // after calculations (for next time)
  for (int i = 0; i < 6; i++) {
    motor_previous_position_buffer_[i] = motor_position_buffer_[i];
  }
  previous_time_ = std::chrono::high_resolution_clock::now();
}

////////////////////// read Position
double JennyHardwareInterface::getJointPosition(uint8_t id) {
  double joint_position;
  if (id == 4) {
    joint_position = (0.5 * (motor_position_buffer_[4] - motor_position_buffer_[5]));
    joint_position = (joint_position * MotorConstants::DEG_TO_RAD);
    joint_position = (joint_position * RobotConstants::AXIS_GET_INVERTED[id]);
    joint_position = (joint_position / RobotConstants::AXIS_RATIO[id]);
  } else if (id == 5) {
    joint_position = (0.5 * (motor_position_buffer_[5] + motor_position_buffer_[4]));
    joint_position = (joint_position * MotorConstants::DEG_TO_RAD);
    joint_position = (joint_position * RobotConstants::AXIS_GET_INVERTED[id]);
    joint_position = (joint_position / RobotConstants::AXIS_RATIO[id]);
  } else {
    joint_position = (motor_position_buffer_[id] * MotorConstants::DEG_TO_RAD);
    joint_position = (joint_position * RobotConstants::AXIS_GET_INVERTED[id]);
    joint_position = (joint_position / RobotConstants::AXIS_RATIO[id]);
  }

  return joint_position;
}

////////////////////// read Speed
double JennyHardwareInterface::getJointVelocity(uint8_t id) {
  double joint_velocity;
  if (id == 4) {
    joint_velocity = (0.5 * (motor_velocity_buffer_[5] + motor_velocity_buffer_[4]));
    joint_velocity = (joint_velocity * RobotConstants::AXIS_GET_INVERTED[id]);
    joint_velocity = (joint_velocity / RobotConstants::AXIS_RATIO[id]);
  } else if (id == 5) {
    joint_velocity = (0.5 * (motor_velocity_buffer_[5] + motor_velocity_buffer_[4]));
    joint_velocity = (joint_velocity * RobotConstants::AXIS_GET_INVERTED[id]);
    joint_velocity = (joint_velocity / RobotConstants::AXIS_RATIO[id]);
  } else {
    joint_velocity = motor_velocity_buffer_[id];
    joint_velocity = (joint_velocity * RobotConstants::AXIS_GET_INVERTED[id]);
    joint_velocity = (joint_velocity / RobotConstants::AXIS_RATIO[id]);
  }
  return joint_velocity;
}

////////////////////// handle responses
void JennyHardwareInterface::handleCANResponses() {
  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");

  // Initialize Receiver
  try {
    if (config_.debug) {RCLCPP_INFO(logger, "Initializing CAN Receiver...");}
    receiver = std::make_unique<drivers::socketcan::SocketCanReceiver>("can0");
    if (config_.debug) {RCLCPP_INFO(logger, "CAN Receiver initialized and opened successfully.");}
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(logger, "Failed to configure SocketCAN: %s", e.what());
    receiver.reset();

  } catch (...) {
    RCLCPP_FATAL(logger, "An unexpected error occurred during SocketCAN configuration.");
    receiver.reset();
  }

  std::vector<uint8_t> response(8, 0); // Maximum data length for standard CAN frames/
                                       //
  while (rclcpp::ok() && !stop_thread_.load()) {
    try {
      drivers::socketcan::CanId can_id = receiver->receive(response.data(), std::chrono::milliseconds(10));
      uint8_t id = can_id.get();
      uint8_t length = can_id.length();
      
      ////////////// motor position response
      if (length == 8 && response[0] == 0x31){
        int64_t value = (static_cast<int64_t>(response[1]) << 40) |
          (static_cast<int64_t>(response[2]) << 32) |
          (static_cast<int64_t>(response[3]) << 24) |
          (static_cast<int64_t>(response[4]) << 16) |
          (static_cast<int64_t>(response[5]) << 8) |
          static_cast<int64_t>(response[6]);
        // Apply two's complement to handle signed 48-bit values
        if (value & 0x800000000000) {  // Check if the 47th bit (sign bit) is set
          value |= 0xFFFF000000000000;  // Sign-extend to 64 bits
        }
        // Convert addition value to degrees
        motor_position_buffer_[id-1] = (double)(value * MotorConstants::DEGREES_PER_REVOLUTION) / MotorConstants::ENCODER_STEPS;

      ////////////// motor velocity response (not used cause really inresponsive)
      } else if (length == 4 && response[0] == 0x32){
        int16_t value = 
          (static_cast<int16_t>(response[1]) << 8) |
          static_cast<int16_t>(response[2]);
        motor_velocity_buffer_[id-1] = (double)value;

      //////////// ignore
      } else if (length == 2) {

      //////////// ignore
      } else if (length == 8 && response[0] == 0xF5) {
        
      //////////// start/stopping
      } else if (length == 3 && response[0] == 0xF5) {
        if (config_.debug) {
          if (response[1] == 1) {
            RCLCPP_INFO(logger, "Motor %d started motion", id);
          } else if (response[1] == 2) {
            RCLCPP_INFO(logger, "Motor %d completed motion", id);
          }
        }

      //////////// unkown
      } else {
        char buffer[100];
        memset(buffer, 0, sizeof buffer);
        for (size_t i = 0; i < length; ++i) {
          std::sprintf(buffer + strlen(buffer), "%X ", response[i]);
        }
        RCLCPP_INFO(logger, "Uknown Response from id %d: %s ", id, buffer);
      }

    } catch (const drivers::socketcan::SocketCanTimeout &) {
      // No frame received within the timeout, which is expected for a zero timeout when no messages are present.
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(this->get_logger(), "Error receiving CAN frame: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Encountered unkown error when during handlePositionResponses");
    }
  }
  return;
}

void JennyHardwareInterface::printJointInfo(uint8_t id) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");
  double setPos = joint_position_command_[id];
  double actPos = getJointPosition(id);
  double posDiff = abs(actPos - setPos);
  double setVel = joint_position_command_[id];
  double actVel = getJointPosition(id);
  double velDiff = abs(actVel - setVel);
  RCLCPP_INFO(logger, "SetPos: %.4f ActPos: %.4f PosDiff: %.4f SetVel: %.4f ActVel: %.4f VelDiff: %.4f", setPos, actPos, posDiff, setVel, actVel, velDiff);
}

////////////////////// on_cleanup /////////////////////////
CallbackReturn
JennyHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");
  RCLCPP_INFO(logger, "Cleaning up ...please wait...");
  stop_thread_.store(true);

  RCLCPP_INFO(logger, "Successfully cleaned up!");

  return CallbackReturn::SUCCESS;
}

////////////////////// on_shutdown /////////////////////////
CallbackReturn
JennyHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyHardwareInterface");
  RCLCPP_INFO(logger, "Shutting down ...please wait...");
  stop_thread_.store(true);
  can_response_thread_.join();

  RCLCPP_INFO(logger, "Successfully cleaned up!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
JennyHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto &joint_name : joint_interfaces["position"]) {
    state_interfaces.emplace_back(joint_name, "position",
                                  &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name : joint_interfaces["velocity"]) {
    state_interfaces.emplace_back(joint_name, "velocity",
                                  &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
JennyHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto &joint_name : joint_interfaces["position"]) {
    command_interfaces.emplace_back(joint_name, "position",
                                    &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name : joint_interfaces["velocity"]) {
    command_interfaces.emplace_back(joint_name, "velocity",
                                    &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}

} // namespace jenny_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(jenny_hardware_interface::JennyHardwareInterface,
                       hardware_interface::SystemInterface)
