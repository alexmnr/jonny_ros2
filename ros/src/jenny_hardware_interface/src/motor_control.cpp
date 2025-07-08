#include "jenny_motor_control.hpp"
#include <rclcpp/logging.hpp>

////////////////////// request Status /////////////////////////
bool JennyMotorControl::requestStatus(uint8_t can_id) { 
  std::vector<uint8_t> data = {CANCommands::QUERY_MOTOR};
  bool check = sendData(can_id, data);
  return check;
}

////////////////////// read Motor Position /////////////////////////
uint8_t JennyMotorControl::readStatus(uint8_t can_id, uint16_t timeout) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  bool check = requestStatus(can_id);
  if (!check) {
    RCLCPP_ERROR(logger, "Error requesting Status for Motor: %d", can_id);
  }
  // wait for response
  for (int i = 0; i < timeout; i++) {
    uint8_t id, length;
    std::vector<uint8_t> response(8, 0);
    tie(id, length, response) = receiveData(1);
    if (id == can_id && length == 3 && response[0] == 0xF1) {
      return response[1];
    }
  }

  return 0;
}

////////////////////// request Position /////////////////////////
bool JennyMotorControl::requestPosition(uint8_t can_id) { 
  std::vector<uint8_t> data = {CANCommands::READ_ENCODER};
  bool check = sendData(can_id, data);
  return check;
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

////////////////////// set Absolute Motor Position /////////////////////////
bool JennyMotorControl::setAbsoluteMotorPosition(uint8_t can_id, double position, double speed, double acceleration) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");

  // setting up values for can message
  int32_t position_value = static_cast<int32_t>(position);
  position_value *= MotorConstants::ENCODER_STEPS;
  position_value *= MotorConstants::AXIS_SET_INVERTED[can_id - 1];
  position_value /= MotorConstants::DEGREES_PER_REVOLUTION;
  uint16_t speed_value = static_cast<uint16_t>(std::clamp(speed*MotorConstants::DEGPS_TO_RPM, 0.0, 3000.0));
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
  uint16_t speed_value = static_cast<uint16_t>(std::clamp(speed*MotorConstants::DEGPS_TO_RPM, 0.0, 3000.0));
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
  uint16_t speed_value = static_cast<uint16_t>(std::clamp(abs(speed*MotorConstants::DEGPS_TO_RPM), 0.0, 3000.0));
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
