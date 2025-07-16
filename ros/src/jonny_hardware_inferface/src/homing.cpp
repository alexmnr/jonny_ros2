#include "jonny_robot_control.hpp"
#include <rclcpp/logging.hpp>

////////////////////// Homing BC Axis /////////////////////////
void JonnyRobotControl::homeBCAxis(){
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHomingControl");
  RCLCPP_INFO(logger, "B Axis: Starting Homing Procedure");
  setZero(5);
  setZero(6);
  bool output;
  uint8_t status;
  double max_position;
  double max_speed;

  // seek
  RCLCPP_INFO(logger, "B Axis: Seeking Endstop...");
  // move till endstop triggered
  bool safety = false;
  for (int i = 0; i < 2; i++) {
    max_position = 180*RobotConstants::AXIS_RATIO[5];
    max_speed = 20*RobotConstants::AXIS_RATIO[5];
    setRelativeMotorPosition(5, -max_position, max_speed, 200);
    setRelativeMotorPosition(6, max_position, max_speed, 200);
    while(1) {
      status = getStatus(5, 100);
      output = readEndStop(6, 100);
      if (output) {
        stopRelativeMotor(5, 0);
        stopRelativeMotor(6, 0);
        break;
      }
      if (status == 1) {
        RCLCPP_ERROR(logger, "Could not find BC Endstop");
        return;
      }
    }
    if (safety) {
      break;
    }
    // check if the correct magnet was detected
    max_position = 90*RobotConstants::AXIS_RATIO[5];
    max_speed = 20*RobotConstants::AXIS_RATIO[5];
    setRelativeMotorPosition(5, -max_position, max_speed, 200);
    setRelativeMotorPosition(6, -max_position, max_speed, 200);
    waitTillStopped(5);
    waitTillStopped(6);
    output = readEndStop(6, 100);
    if (output) {
      break;
    } else {
      safety = true;
    }
  }
  // move back
  max_position = -10*RobotConstants::AXIS_RATIO[5];
  max_speed = 40*RobotConstants::AXIS_RATIO[5];
  setRelativeMotorPosition(5, -max_position, max_speed, 200);
  setRelativeMotorPosition(6, max_position, max_speed, 200);
  waitTillStopped(5);
  waitTillStopped(6);
  // Locate
  RCLCPP_INFO(logger, "B Axis: Locating Endstop...");
  max_position = 15*RobotConstants::AXIS_RATIO[5];
  max_speed = 4*RobotConstants::AXIS_RATIO[5];
  setRelativeMotorPosition(5, -max_position, max_speed, 200);
  setRelativeMotorPosition(6, max_position, max_speed, 200);
  while(1) {
    status = getStatus(5, 100);
    output = readEndStop(6, 100);
    if (output) {
      stopRelativeMotor(5, 0);
      stopRelativeMotor(6, 0);
      break;
    }
    if (status == 1) {
      RCLCPP_ERROR(logger, "Could not find BC Endstop");
      return;
    }
  }
  waitTillStopped(5);
  waitTillStopped(6);
  double pos1 = getMotorPosition(5, 100);
  // approach from different reaction
  max_position = 40*RobotConstants::AXIS_RATIO[5];
  max_speed = 20*RobotConstants::AXIS_RATIO[5];
  setRelativeMotorPosition(5, -max_position, max_speed, 200);
  setRelativeMotorPosition(6, max_position, max_speed, 200);
  while(1) {
    status = getStatus(5, 100);
    output = readEndStop(6, 100);
    if (!output) {
      stopRelativeMotor(5, 0);
      stopRelativeMotor(6, 0);
      break;
    }
    if (status == 1) {
      RCLCPP_ERROR(logger, "Could not find BC Endstop");
      return;
    }
  }
  waitTillStopped(5);
  waitTillStopped(6);
  // Locate
  RCLCPP_INFO(logger, "B Axis: Locating Endstop...");
  max_position = -10*RobotConstants::AXIS_RATIO[5];
  max_speed = 4*RobotConstants::AXIS_RATIO[5];
  setRelativeMotorPosition(5, -max_position, max_speed, 200);
  setRelativeMotorPosition(6, max_position, max_speed, 200);
  while(1) {
    status = getStatus(5, 100);
    output = readEndStop(6, 100);
    if (output) {
      stopRelativeMotor(5, 0);
      stopRelativeMotor(6, 0);
      break;
    }
    if (status == 1) {
      RCLCPP_ERROR(logger, "Could not find BC Endstop");
      return;
    }
  }
  waitTillStopped(5);
  waitTillStopped(6);
  double pos2 = getMotorPosition(5, 100);

  // move
  double goal_position = abs((pos1 - pos2) / 2) + (RobotConstants::AXIS_ZERO_POSITION[4] * MotorConstants::DEG_TO_RAD);
  goal_position = goal_position * MotorConstants::RAD_TO_DEG * RobotConstants::AXIS_RATIO[5];
  double goal_speed = 30 * RobotConstants::AXIS_RATIO[5];
  setRelativeMotorPosition(5, goal_position, goal_speed, 200);
  setRelativeMotorPosition(6, -goal_position, goal_speed, 200);
  waitTillStopped(5);
  waitTillStopped(6);
  RCLCPP_INFO(logger, "B Axis succesfully homed!");
  setZero(5);
  setZero(6);

  // C Axis Homing
  RCLCPP_INFO(logger, "C Axis: Starting Homing Procedure");
  // seek
  RCLCPP_INFO(logger, "C Axis: Seeking Endstop...");
  max_position = 360*RobotConstants::AXIS_RATIO[5];
  max_speed = 20*RobotConstants::AXIS_RATIO[5];
  setRelativeMotorPosition(5, max_position, max_speed, 200);
  setRelativeMotorPosition(6, max_position, max_speed, 200);
  while(1) {
    status = getStatus(5, 100);
    output = readEndStop(6, 100);
    if (output) {
      stopRelativeMotor(5, 0);
      stopRelativeMotor(6, 0);
      break;
    }
    if (status == 1) {
      RCLCPP_ERROR(logger, "Could not find BC Endstop");
      return;
    }
  }
  // Move back
  max_position = -10*RobotConstants::AXIS_RATIO[5];
  max_speed = 40*RobotConstants::AXIS_RATIO[5];
  setRelativeMotorPosition(5, max_position, max_speed, 200);
  setRelativeMotorPosition(6, max_position, max_speed, 200);
  waitTillStopped(5);
  waitTillStopped(6);
  // Locate
  RCLCPP_INFO(logger, "C Axis: Locating Endstop...");
  max_position = 15*RobotConstants::AXIS_RATIO[5];
  max_speed = 4*RobotConstants::AXIS_RATIO[5];
  setRelativeMotorPosition(5, max_position, max_speed, 200);
  setRelativeMotorPosition(6, max_position, max_speed, 200);
  while(1) {
    status = getStatus(5, 100);
    output = readEndStop(6, 100);
    if (output) {
      stopRelativeMotor(5, 0);
      stopRelativeMotor(6, 0);
      break;
    }
    if (status == 1) {
      RCLCPP_ERROR(logger, "Could not find BC Endstop");
      return;
    }
  }
  pos1 = getMotorPosition(5, 100);
  // moving to other side
  max_position = 90*RobotConstants::AXIS_RATIO[5];
  max_speed = 20*RobotConstants::AXIS_RATIO[5];
  setRelativeMotorPosition(5, max_position, max_speed, 200);
  setRelativeMotorPosition(6, max_position, max_speed, 200);
  while(1) {
    status = getStatus(5, 100);
    output = readEndStop(6, 100);
    if (!output) {
      stopRelativeMotor(5, 0);
      stopRelativeMotor(6, 0);
      break;
    }
    if (status == 1) {
      RCLCPP_ERROR(logger, "Could not find BC Endstop");
      return;
    }
  }
  // Locate
  RCLCPP_INFO(logger, "C Axis: Locating Endstop...");
  max_position = -20*RobotConstants::AXIS_RATIO[5];
  max_speed = 4*RobotConstants::AXIS_RATIO[5];
  setRelativeMotorPosition(5, max_position, max_speed, 200);
  setRelativeMotorPosition(6, max_position, max_speed, 200);
  while(1) {
    status = getStatus(5, 100);
    output = readEndStop(6, 100);
    if (output) {
      stopRelativeMotor(5, 0);
      stopRelativeMotor(6, 0);
      break;
    }
    if (status == 1) {
      RCLCPP_ERROR(logger, "Could not find BC Endstop");
      return;
    }
  }
  pos2 = getMotorPosition(5, 100);
  // move
  goal_position = - abs((pos2 - pos1) / 2) - (RobotConstants::AXIS_ZERO_POSITION[5] * MotorConstants::DEG_TO_RAD);
  goal_position = goal_position * MotorConstants::RAD_TO_DEG * RobotConstants::AXIS_RATIO[5];
  goal_speed = 30 * RobotConstants::AXIS_RATIO[5];
  setRelativeMotorPosition(5, goal_position, goal_speed, 200);
  setRelativeMotorPosition(6, goal_position, goal_speed, 200);
  waitTillStopped(5);
  waitTillStopped(6);
  RCLCPP_INFO(logger, "C Axis succesfully homed!");
  setZero(5);
  setZero(6);
}

////////////////////// Homing A Axis /////////////////////////
void JonnyRobotControl::homeAAxis(){
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHomingControl");
  int id = 3;
  int can_id = id + 1;
  RCLCPP_INFO(logger, "A Axis: Starting Homing Procedure");
  setZero(can_id);
  bool check;

  // seek
  if (readEndStop(can_id, 100) == false) {
    // start seeking (skip if already in endstop zone)
    RCLCPP_INFO(logger, "A Axis: Seeking 1. Endstop...");
    check = moveTillEndstop(can_id, 5, 180, 15, 100);
    if (check == false) {
      RCLCPP_ERROR(logger, "Failed to seek for endstop on A Axis");
    }
  }
  // locate
  RCLCPP_INFO(logger, "A Axis: Locating 1. Endstop...");
  check = moveTillEndstop(can_id, 5, -10, 0.5, 0);
  if (check == false) {
    RCLCPP_ERROR(logger, "Failed to locate for endstop on A Axis");
  }
  double pos1 = getMotorPosition(can_id, 100);

  // seek
  RCLCPP_INFO(logger, "A Axis: Seeking 2. Endstop...");
  check = moveTillEndstop(can_id, 5, -180, 15, 100);
  if (check == false) {
    RCLCPP_ERROR(logger, "Failed to seek for endstop on A Axis");
  }
  // locate
  RCLCPP_INFO(logger, "A Axis: Locating 2. Endstop...");
  check = moveTillEndstop(can_id, 5, 10, 0.5, 0);
  if (check == false) {
    RCLCPP_ERROR(logger, "Failed to locate for endstop on A Axis");
  }
  double pos2 = getMotorPosition(can_id, 100);
  
  // // move
  RCLCPP_INFO(logger, "A Axis: Moving to Zero Position...");
  // RCLCPP_INFO(logger, "Pos1: %f Pos2: %f", pos1, pos2);
  double goal_position = pos2 + ((1.0/2.0) * (pos1 - pos2)) + (RobotConstants::AXIS_ZERO_POSITION[id] * MotorConstants::DEG_TO_RAD);
  goal_position = goal_position * MotorConstants::RAD_TO_DEG * RobotConstants::AXIS_RATIO[id];
  double goal_speed = 25 * RobotConstants::AXIS_RATIO[id];
  setAbsoluteMotorPosition(can_id, goal_position, goal_speed, 20);
  waitTillStopped(can_id);
  setZero(can_id);
  RCLCPP_INFO(logger, "A Axis succesfully homed!");
}

////////////////////// Homing Z Axis /////////////////////////
void JonnyRobotControl::homeZAxis(){
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHomingControl");
  int id = 2;
  int can_id = id + 1;
  RCLCPP_INFO(logger, "Z Axis: Starting Homing Procedure");
  setZero(can_id);
  bool check;

  // seek
  if (readEndStop(can_id, 100) == false) {
    // start seeking (skip if already in endstop zone)
    RCLCPP_INFO(logger, "Z Axis: Seeking Endstop...");
    check = moveTillEndstop(can_id, can_id, -180, 4, 100);
    if (check == false) {
      RCLCPP_ERROR(logger, "Failed to seek for endstop on Z Axis");
    }
  }
  // locate
  RCLCPP_INFO(logger, "Z Axis: Locating Endstop...");
  check = moveTillEndstop(can_id, can_id, 20, 1, 0);
  if (check == false) {
    RCLCPP_ERROR(logger, "Failed to locate for endstop on Z Axis");
  }
  check = moveTillEndstop(can_id, can_id, -5, 0.5, 0);
  if (check == false) {
    RCLCPP_ERROR(logger, "Failed to locate for endstop on Z Axis");
  }
  // move
  RCLCPP_INFO(logger, "Z Axis: Moving to Zero Position...");
  double goal_position = - getMotorPosition(can_id, 100) + (RobotConstants::AXIS_ZERO_POSITION[id] * MotorConstants::DEG_TO_RAD);
  goal_position = goal_position * MotorConstants::RAD_TO_DEG * RobotConstants::AXIS_RATIO[id];
  double goal_speed = 12 * RobotConstants::AXIS_RATIO[id];

  setAbsoluteMotorPosition(can_id, goal_position, goal_speed, 100);
  waitTillStopped(can_id);
  setZero(can_id);
  RCLCPP_INFO(logger, "Z Axis succesfully homed!");
}

////////////////////// Homing Y Axis /////////////////////////
void JonnyRobotControl::homeYAxis(){
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHomingControl");
  int id = 1;
  int can_id = id + 1;
  RCLCPP_INFO(logger, "Y Axis: Starting Homing Procedure");
  setZero(can_id);
  bool check;
  // seek
  if (readEndStop(can_id, 100) == false) {
    // start seeking (skip if already in endstop zone)
    RCLCPP_INFO(logger, "Y Axis: Seeking Endstop...");
    check = moveTillEndstop(can_id, can_id, -180, 4, 100);
    if (check == false) {
      RCLCPP_ERROR(logger, "Failed to seek for endstop on Y Axis");
    }
  }
  // locate
  RCLCPP_INFO(logger, "Y Axis: Locating Endstop...");
  check = moveTillEndstop(can_id, can_id, 20, 1, 0);
  if (check == false) {
    RCLCPP_ERROR(logger, "Failed to locate for endstop on Y Axis");
  }
  check = moveTillEndstop(can_id, can_id, -5, 0.5, 0);
  if (check == false) {
    RCLCPP_ERROR(logger, "Failed to locate for endstop on Y Axis");
  }
  // move
  RCLCPP_INFO(logger, "Y Axis: Moving to Zero Position...");
  double goal_position = getMotorPosition(can_id, 100) - (RobotConstants::AXIS_ZERO_POSITION[id] * MotorConstants::DEG_TO_RAD);
  goal_position = goal_position * MotorConstants::RAD_TO_DEG * RobotConstants::AXIS_RATIO[id];
  double goal_speed = 12 * RobotConstants::AXIS_RATIO[id];

  setAbsoluteMotorPosition(can_id, goal_position, goal_speed, 100);
  waitTillStopped(can_id);
  setZero(can_id);
  RCLCPP_INFO(logger, "Y Axis succesfully homed!");
}

////////////////////// Homing X Axis /////////////////////////
void JonnyRobotControl::homeXAxis(){
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHomingControl");
  int id = 0;
  int can_id = id + 1;
  RCLCPP_INFO(logger, "X Axis: Starting Homing Procedure");
  setZero(can_id);
  bool check;

  // seek
  if (readEndStop(can_id, 100) == false) {
    // start seeking (skip if already in endstop zone)
    RCLCPP_INFO(logger, "X Axis: Seeking 1. Endstop...");
    check = moveTillEndstop(can_id, can_id, 360, 8, 50);
    if (check == false) {
      RCLCPP_ERROR(logger, "Failed to seek for endstop on X Axis");
    }
  }
  // locate
  RCLCPP_INFO(logger, "X Axis: Locating 1. Endstop...");
  check = moveTillEndstop(can_id, can_id, -30, 2, 0);
  if (check == false) {
    RCLCPP_ERROR(logger, "Failed to locate for endstop on X Axis");
  }
  check = moveTillEndstop(can_id, can_id, 20, 1, 0);
  if (check == false) {
    RCLCPP_ERROR(logger, "Failed to locate for endstop on X Axis");
  }
  double pos1 = getMotorPosition(1, 100);

  // seek 2nd
  RCLCPP_INFO(logger, "X Axis: Seeking 2. Endstop...");
  check = moveTillEndstop(can_id, can_id, 50, 8, 50);
  if (check == false) {
    RCLCPP_ERROR(logger, "Failed to seek for endstop on X Axis");
  }
  // locate
  RCLCPP_INFO(logger, "X Axis: Locating 2. Endstop...");
  check = moveTillEndstop(can_id, can_id, -30, 1, 0);
  if (check == false) {
    RCLCPP_ERROR(logger, "Failed to locate for endstop on X Axis");
  }
  double pos2 = getMotorPosition(1, 100);

  // move
  RCLCPP_INFO(logger, "X Axis: Moving to Zero Position...");
  double goal_position = pos1 + ((1.0/2.0) * (pos2 - pos1)) + (RobotConstants::AXIS_ZERO_POSITION[id] * MotorConstants::DEG_TO_RAD);
  goal_position = goal_position * MotorConstants::RAD_TO_DEG * RobotConstants::AXIS_RATIO[id];
  double goal_speed = 25 * RobotConstants::AXIS_RATIO[id];
  setAbsoluteMotorPosition(can_id, goal_position, goal_speed, 20);
  waitTillStopped(can_id);
  setZero(can_id);
  RCLCPP_INFO(logger, "X Axis succesfully homed!");
}

////////////////////// moveTillEndstop /////////////////////////
bool JonnyRobotControl::moveTillEndstop(uint8_t motor_id, uint8_t endstop_id, double limit, double speed, double acceleration) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHomingControl");

  double max_position = limit*RobotConstants::AXIS_RATIO[motor_id-1];
  double max_speed = speed*RobotConstants::AXIS_RATIO[motor_id-1];

  bool base_reading = readEndStop(endstop_id, 100);

  // start seeking
  setRelativeMotorPosition(motor_id, max_position, max_speed, acceleration);

  // wait for endstop or to reach limit position
  bool output;
  uint8_t status;
  while(1) {
    status = getStatus(motor_id, 100);
    output = readEndStop(endstop_id, 100);
    if (output != base_reading) {
      stopRelativeMotor(motor_id, 0);
      return true;
    }
    if (status == 1) {
      RCLCPP_INFO(logger, "Could not find Endstop");
      return false;
    }
  }

  return false;
}

////////////////////// read Endstop /////////////////////////
bool JonnyRobotControl::readEndStop(uint8_t can_id, uint16_t timeout) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyRobotControl");
  std::vector<uint8_t> data = {CANCommands::READ_IO};
  sendData(can_id, data);

  for (int i = 0; i < timeout; i++) {
    uint8_t id, length;
    std::vector<uint8_t> response(8, 0);
    tie(id, length, response) = receiveData(1);
    if (id == can_id && length == 3 && response[0] == 0x34) {
      if (response[1] == 14 || response[1] == 0) {
        return true;
      } else {
        return false;
      }
    }
  }

  return false;
}
