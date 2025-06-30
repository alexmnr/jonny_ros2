#include "jenny_motor_control.hpp"
#include <rclcpp/logging.hpp>

bool JennyMotorControl::init() {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  setupSender();
  setupReceiver();

  RCLCPP_INFO(logger, "X Axis: Starting Homing Procedure");
  setZero(1);

  RCLCPP_INFO(logger, "X Axis: Seeking 1. Endstop...");
  // start seeking
  setAbsoluteMotorPosition(1, 360*MotorConstants::AXIS_RATIO[0], motor_seeking_speeds[0], 200);

  // wait for endstop
  bool output;
  while(1) {
    output = readEndStop(1, 100);
    if (output) {
      stopAbsoluteMotor(1, 0);
      break;
    }
  }
  RCLCPP_INFO(logger, "X Axis: Locating 1. Endstop...");
  
  // start locating
  setRelativeMotorPosition(1, -40*MotorConstants::AXIS_RATIO[0], motor_locating_speeds[0], 200);

  // wait for endstop
  while(1) {
    output = readEndStop(1, 100);
    if (!output) {
      stopRelativeMotor(1, 0);
      break;
    }
  }

  // save location of first endstop
  double pos1 = readMotorPosition(1, 100);
  
  RCLCPP_INFO(logger, "X Axis: Seeking 2. Endstop...");

  // start seeking
  setRelativeMotorPosition(1, 40*MotorConstants::AXIS_RATIO[0], motor_seeking_speeds[0], 200);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // wait for endstop
  while(1) {
    output = readEndStop(1, 100);
    if (!output) {
      stopRelativeMotor(1, 0);
      break;
    }
  }

  RCLCPP_INFO(logger, "X Axis: Locating 2. Endstop...");

  // start locating
  setRelativeMotorPosition(1, -40*MotorConstants::AXIS_RATIO[0], motor_locating_speeds[0], 200);

  // wait for endstop
  while(1) {
    output = readEndStop(1, 100);
    if (output) {
      stopRelativeMotor(1, 0);
      break;
    }
  }

  // save location of second endstop
  double pos2 = readMotorPosition(1, 100);

  RCLCPP_INFO(logger, "X Axis: Moving to Zero Position...");

  double goal = pos1 + ((2.0/3.0) * (pos2 - pos1)) - M_PI;
  setAbsoluteMotorPosition(1, goal*(180/M_PI)*MotorConstants::AXIS_RATIO[0], 100, 20);

  RCLCPP_INFO(logger, "X Axis succesfully homed!");

  return true;
}

////////////////////// read Motor Position /////////////////////////
double JennyMotorControl::readMotorPosition(uint8_t can_id, uint16_t timeout) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  bool check = requestPosition(can_id);
  if (!check) {
    RCLCPP_ERROR(logger, "Error requesting Position for Motor: %d", can_id);
  }
  for (int i = 0; i < timeout; i++) {
    uint8_t id, length;
    std::vector<uint8_t> response(8, 0);
    tie(id, length, response) = receiveData(1);
    if (id == can_id && length == 8 && response[0] == 0x31) {
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
      double position = (double)(value * MotorConstants::DEGREES_PER_REVOLUTION) / MotorConstants::ENCODER_STEPS;
      position = (position * MotorConstants::DEG_TO_RAD);
      position = (position * MotorConstants::AXIS_GET_INVERTED[id-1]);
      position = (position / MotorConstants::AXIS_RATIO[id-1]);
      return position;
    }
  }

  return 0.0;
}

////////////////////// read Endstop /////////////////////////
bool JennyMotorControl::readEndStop(uint8_t can_id, uint16_t timeout) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  std::vector<uint8_t> data = {CANCommands::READ_IO};
  sendData(can_id, data);

  for (int i = 0; i < timeout; i++) {
    uint8_t id, length;
    std::vector<uint8_t> response(8, 0);
    tie(id, length, response) = receiveData(1);
    if (id == can_id && length == 3 && response[0] == 0x34) {
      if (response[1] == 14) {
        return true;
      } else {
        return false;
      }
    }
  }

  return false;
}

////////////////////// request Position /////////////////////////
bool JennyMotorControl::requestPosition(uint8_t can_id) { 
  std::vector<uint8_t> data = {CANCommands::READ_ENCODER};
  bool check = sendData(can_id, data);
  return check;
}

////////////////////// send Data /////////////////////////
bool JennyMotorControl::sendData(uint8_t can_id, std::vector<uint8_t> data_vec) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  // create can id
  drivers::socketcan::CanId canId = drivers::socketcan::CanId(can_id, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
  // creating crc
  uint16_t crc = can_id;
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

////////////////////// receive Data /////////////////////////
std::tuple<uint8_t, uint8_t, std::vector<uint8_t>> JennyMotorControl::receiveData(uint16_t timeout) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  std::vector<uint8_t> response(8, 0); // Maximum data length for standard CAN frames/
  uint8_t id = 0;
  uint8_t length = 0;
  try {
    drivers::socketcan::CanId can_id = receiver->receive(response.data(), std::chrono::milliseconds(timeout));
    id = can_id.get();
    length = can_id.length();
  } catch (const drivers::socketcan::SocketCanTimeout &) {
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(logger, "Error receiving CAN frame: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger, "Encountered unkown error when during handlePositionResponses");
  }
  return std::make_tuple(id, length, response);
}

////////////////////// setup Sender /////////////////////////
bool JennyMotorControl::setupSender() {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  try {
    sender = std::make_unique<drivers::socketcan::SocketCanSender>("can0");
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(logger, "Failed to configure SocketCAN: %s", e.what());
    sender.reset();
    return false;
  } catch (...) {
    RCLCPP_FATAL(logger, "An unexpected error occurred during SocketCAN configuration.");
    sender.reset();
    return false;
  }
  return true;
}

////////////////////// setup Receiver /////////////////////////
bool JennyMotorControl::setupReceiver() {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  // Initialize Receiver
  try {
    receiver = std::make_unique<drivers::socketcan::SocketCanReceiver>("can0", false);
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(logger, "Failed to configure SocketCAN: %s", e.what());
    receiver.reset();
    return false;
  } catch (...) {
    RCLCPP_FATAL(logger, "An unexpected error occurred during SocketCAN configuration.");
    receiver.reset();
    return false;
  }
  return true;
}

////////////////////// set Absolute Motor Position /////////////////////////
bool JennyMotorControl::setAbsoluteMotorPosition(uint8_t can_id, double position, double speed, double acceleration) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");

  // setting up values for can message
  int32_t position_value = static_cast<int32_t>(position);
  position_value *= MotorConstants::ENCODER_STEPS;
  position_value *= MotorConstants::AXIS_SET_INVERTED[can_id - 1];
  position_value /= MotorConstants::DEGREES_PER_REVOLUTION;
  uint16_t speed_value = static_cast<uint16_t>(std::clamp(speed, 0.0, 3000.0));
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
  bool check = sendData(can_id, data);
  return check;
}

////////////////////// set Relative Motor Position /////////////////////////
bool JennyMotorControl::setRelativeMotorPosition(uint8_t can_id, double position, double speed, double acceleration) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");

  // setting up values for can message
  int32_t position_value = static_cast<int32_t>(position);
  position_value *= MotorConstants::ENCODER_STEPS;
  position_value *= MotorConstants::AXIS_SET_INVERTED[can_id - 1];
  position_value /= MotorConstants::DEGREES_PER_REVOLUTION;
  uint16_t speed_value = static_cast<uint16_t>(std::clamp(speed, 0.0, 3000.0));
  uint8_t acceleration_value = static_cast<uint8_t>(std::clamp(acceleration, 0.0, 255.0));

  // creating data without crc
  std::vector<uint8_t> data = {
    CANCommands::RELATIVE_POSITION,  // 0xF5
    static_cast<uint8_t>((speed_value >> 8) & 0xFF),  // Speed high byte
    static_cast<uint8_t>(speed_value & 0xFF),         // Speed low byte
    acceleration_value,                                  // Acceleration
    static_cast<uint8_t>((position_value >> 16) & 0xFF),  // Position high byte
    static_cast<uint8_t>((position_value >> 8) & 0xFF),   // Position middle byte
    static_cast<uint8_t>(position_value & 0xFF)           // Position low byte
  };

  // sending data
  bool check = sendData(can_id, data);
  return check;
}

////////////////////// set Motor Velocity /////////////////////////
bool JennyMotorControl::setMotorVelocity(uint8_t can_id, double speed, double acceleration) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");

  // setting up values for can message
  uint8_t direction = speed >= 0 ? 0x00 : 0x80;
  uint16_t speed_value = static_cast<uint16_t>(std::clamp(abs(speed), 0.0, 3000.0));
  uint8_t acceleration_value = static_cast<uint8_t>(std::clamp(acceleration, 0.0, 255.0));

  // creating data without crc
  std::vector<uint8_t> data = {
    CANCommands::SPEED_CONTROL,  // 0xF6
    static_cast<uint8_t>(direction | ((speed_value >> 8) & 0x0F)),  // Direction + high nibble of speed
    static_cast<uint8_t>(speed_value & 0xFF),  // Low byte of speed
    static_cast<uint8_t>(acceleration_value),                                  // Acceleration
  };

  // sending data
  bool check = sendData(can_id, data);
  return check;
}

////////////////////// set Zero /////////////////////////
bool JennyMotorControl::setZero(uint8_t can_id) {
  std::vector<uint8_t> data = {CANCommands::SET_ZERO_POSITION};
  bool check = sendData(can_id, data);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return check;
}

////////////////////// stop Motor in Absolute Mode /////////////////////////
bool JennyMotorControl::stopAbsoluteMotor(uint8_t can_id, double acceleration) {
  return setAbsoluteMotorPosition(can_id, 0, 0, acceleration);
}

////////////////////// stop Motor in Relative Mode /////////////////////////
bool JennyMotorControl::stopRelativeMotor(uint8_t can_id, double acceleration) {
  return setRelativeMotorPosition(can_id, 0, 0, acceleration);
}

int main() {
  JennyMotorControl motor_control = JennyMotorControl();
  motor_control.init();
  return 0;
}
