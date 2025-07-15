#include "jonny_motor_control.hpp"
#include <rclcpp/logging.hpp>

////////////////////// set Normal (1-4) Joint Position (Relative)
bool JonnyMotorControl::setRelativeXYZAJointPosition(uint8_t id, double position, double speed, double acceleration) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyJointInterface");
  if (id > 3) {
    RCLCPP_ERROR(logger, "Wrong Joint for XYZA Control!");
    return false;
  }
  bool check = setRelativeMotorPosition(id + 1, position*RobotConstants::AXIS_RATIO[id]*RobotConstants::AXIS_SET_INVERTED[id], speed*RobotConstants::AXIS_RATIO[id], acceleration);
  return check;
}

////////////////////// set Normal (1-4) Joint Position (Absolute)
bool JonnyMotorControl::setAbsoluteXYZAJointPosition(uint8_t id, double position, double speed, double acceleration) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyJointInterface");
  if (id > 3) {
    RCLCPP_ERROR(logger, "Wrong Joint for XYZA Control!");
    return false;
  }
  bool check = setAbsoluteMotorPosition(id + 1, position*RobotConstants::AXIS_RATIO[id]*RobotConstants::AXIS_SET_INVERTED[id], speed*RobotConstants::AXIS_RATIO[id], acceleration);
  return check;
}

////////////////////// set BC (5-6) Joint Position (Relative)
bool JonnyMotorControl::setRelativeBCJointPosition(double position[2], double speed, double acceleration) {
  double motor_5_position = (position[0] - position[1]) * RobotConstants::AXIS_SET_INVERTED[4];
  double motor_6_position = (position[0] + position[1]) * RobotConstants::AXIS_SET_INVERTED[5];
  double motor_5_speed = speed;
  double motor_6_speed = speed;
  bool check1 = setRelativeMotorPosition(5, (motor_5_position * RobotConstants::AXIS_RATIO[4]), (motor_5_speed * RobotConstants::AXIS_RATIO[4]), acceleration);
  bool check2 = setRelativeMotorPosition(6, (motor_6_position * RobotConstants::AXIS_RATIO[5]), (motor_6_speed * RobotConstants::AXIS_RATIO[5]), acceleration);
  return (check1 && check2);
}

////////////////////// set BC (5-6) Joint Position (Absolute)
bool JonnyMotorControl::setAbsoluteBCJointPosition(double position[2], double speed, double acceleration) {
  double motor_5_position = (position[0] - position[1]) * RobotConstants::AXIS_SET_INVERTED[4];
  double motor_6_position = (position[0] + position[1]) * RobotConstants::AXIS_SET_INVERTED[5];
  double motor_5_speed = speed;
  double motor_6_speed = speed;
  bool check1 = setAbsoluteMotorPosition(5, (motor_5_position * RobotConstants::AXIS_RATIO[4]), (motor_5_speed * RobotConstants::AXIS_RATIO[4]), acceleration);
  bool check2 = setAbsoluteMotorPosition(6, (motor_6_position * RobotConstants::AXIS_RATIO[5]), (motor_6_speed * RobotConstants::AXIS_RATIO[5]), acceleration);
  return (check1 && check2);
}

