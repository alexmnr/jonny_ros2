#include "jonny_robot_control.hpp"
#include <rclcpp/logging.hpp>

////////////////////// set Normal (1-4) Joint Position (Relative)
bool JonnyRobotControl::setRelativeXYZAJointPosition(uint8_t id, double position, double speed, double acceleration) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyJointInterface");
  if (id > 3) {
    RCLCPP_ERROR(logger, "Wrong Joint for XYZA Control!");
    return false;
  }
  bool check = setRelativeMotorPosition(id + 1, position*RobotConstants::AXIS_RATIO[id], speed*RobotConstants::AXIS_RATIO[id], acceleration);
  return check;
}

////////////////////// set Normal (1-4) Joint Position (Absolute)
bool JonnyRobotControl::setAbsoluteXYZAJointPosition(uint8_t id, double position, double speed, double acceleration) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyJointInterface");
  if (id > 3) {
    RCLCPP_ERROR(logger, "Wrong Joint for XYZA Control!");
    return false;
  }
  bool check = setAbsoluteMotorPosition(id + 1, position*RobotConstants::AXIS_RATIO[id], speed*RobotConstants::AXIS_RATIO[id], acceleration);
  return check;
}

////////////////////// set BC (5-6) Joint Position (Relative)
bool JonnyRobotControl::setRelativeBCJointPosition(double position[2], double speed, double acceleration) {
  double motor_5_position = (position[0] - position[1]);
  double motor_6_position = (position[0] + position[1]);
  double motor_5_speed;
  double motor_6_speed;
  double motor_5_acceleration;
  double motor_6_acceleration;
  if (abs(motor_5_position) > abs(motor_6_position)) {
    motor_5_speed = speed;
    motor_6_speed = speed * (motor_6_position / motor_5_position);
    motor_5_acceleration = acceleration;
    motor_6_acceleration = acceleration * (motor_6_position / motor_5_position);
  } else if (abs(motor_6_position) > abs(motor_5_position)) {
    motor_6_speed = speed;
    motor_5_speed = speed * (motor_5_position / motor_6_position);
    motor_6_acceleration = acceleration;
    motor_5_acceleration = acceleration * (motor_5_position / motor_6_position);
  } else {
    motor_5_speed = speed;
    motor_6_speed = speed;
    motor_5_acceleration = acceleration;
    motor_6_acceleration = acceleration;
  }
  bool check1 = setRelativeMotorPosition(5, (motor_5_position * RobotConstants::AXIS_RATIO[4]), (motor_5_speed * RobotConstants::AXIS_RATIO[4]), motor_5_acceleration);
  bool check2 = setRelativeMotorPosition(6, (motor_6_position * RobotConstants::AXIS_RATIO[5]), (motor_6_speed * RobotConstants::AXIS_RATIO[5]), motor_6_acceleration);
  return (check1 && check2);
}

////////////////////// set BC (5-6) Joint Position (Absolute)
bool JonnyRobotControl::setAbsoluteBCJointPosition(double position[2], double speed, double acceleration) {
  double current_motor_5_position = getMotorPosition(5, 100);
  double current_motor_6_position = getMotorPosition(6, 100);
  double motor_5_position = (position[0] - position[1]) * RobotConstants::AXIS_RATIO[4];
  double motor_6_position = (position[0] + position[1]) * RobotConstants::AXIS_RATIO[5];
  double motor_5_diff = abs(motor_5_position - current_motor_5_position);
  double motor_6_diff = abs(motor_6_position - current_motor_6_position);
  double motor_5_speed;
  double motor_6_speed;
  double motor_5_acceleration;
  double motor_6_acceleration;
  if (motor_5_diff > motor_6_diff) {
    motor_5_speed = speed;
    motor_6_speed = speed * (motor_6_diff / motor_5_diff);
    motor_5_acceleration = acceleration;
    motor_6_acceleration = acceleration * (motor_6_diff / motor_5_diff);
  } else if (motor_6_diff > motor_5_diff) {
    motor_6_speed = speed;
    motor_5_speed = speed * (motor_5_diff / motor_6_diff);
    motor_6_acceleration = acceleration;
    motor_5_acceleration = acceleration * (motor_5_diff / motor_6_diff);
  } else {
    motor_5_speed = speed;
    motor_6_speed = speed;
    motor_5_acceleration = acceleration;
    motor_6_acceleration = acceleration;
  }
  bool check1 = setAbsoluteMotorPosition(5, motor_5_position, (motor_5_speed * RobotConstants::AXIS_RATIO[4]), motor_5_acceleration);
  bool check2 = setAbsoluteMotorPosition(6, motor_6_position, (motor_6_speed * RobotConstants::AXIS_RATIO[5]), motor_6_acceleration);
  return (check1 && check2);
}

////////////////////// get Joint Position
double JonnyRobotControl::getJointPosition(uint8_t id, uint16_t timeout) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyJointInterface");
  if (id < 4) {
    double motor_position = getMotorPosition(id+1, timeout);
    double joint_position = motor_position / RobotConstants::AXIS_RATIO[id];
    return joint_position;
  } else if (id == 4) {
    double motor_position_5 = getMotorPosition(5, timeout);
    double motor_position_6 = getMotorPosition(6, timeout);
    double joint_position = (0.5 * (motor_position_6 + motor_position_5));
    joint_position /= RobotConstants::AXIS_RATIO[4];
    return joint_position;
  } else if (id == 5) {
    double motor_position_5 = getMotorPosition(5, timeout);
    double motor_position_6 = getMotorPosition(6, timeout);
    double joint_position = (0.5 * (motor_position_6 - motor_position_5));
    joint_position /= RobotConstants::AXIS_RATIO[4];
    return joint_position;
  } else {
    RCLCPP_ERROR(logger, "Wrong Joint ID!");
    return 0.0;
  }
}

