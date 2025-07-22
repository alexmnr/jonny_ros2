#include "jonny_robot_control.hpp"
#include <rclcpp/logging.hpp>

bool JonnyRobotControl::init() {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyRobotControl");
  setupSender();
  setupReceiver();


  // setAbsoluteXYZAJointPosition(0, 0, 20, 100);
  // setAbsoluteXYZAJointPosition(1, 0, 20, 100);
  // setAbsoluteXYZAJointPosition(2, 0, 20, 100);
  // setAbsoluteXYZAJointPosition(3, 0, 20, 100);
  // double BC_position[2] = {0, 0};
  // setAbsoluteBCJointPosition(BC_position, 40, 100);
  
  // setAbsoluteXYZAJointPosition(0, -90, 10, 100);
  // setAbsoluteXYZAJointPosition(1, -30, 20, 100);
  // setAbsoluteXYZAJointPosition(2, -100, 20, 100);
  // setAbsoluteXYZAJointPosition(3, -45, 20, 100);
  // double BC_position[2] = {0, 0};
  // setAbsoluteBCJointPosition(BC_position, 40, 100);

  return true;
}

bool JonnyRobotControl::check() {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyRobotControl");
  // request status of all motors
  uint8_t status;
  bool fail = false;
  for (int can_id = 1; can_id < 7; can_id++) {
    status = getStatus(can_id, 100);
    if (status == 0) {
      RCLCPP_ERROR(logger, "Failed to request Status for motor with id: %d", can_id);
      fail = true;
    } else if (status == 1) {
      RCLCPP_DEBUG(logger, "Motor with id: %d is ready", can_id);
    } else {
      RCLCPP_ERROR(logger, "Motor with id: %d is reachable but not ready!", can_id);
      fail = true;
    }
  }

  return !fail;
}

bool JonnyRobotControl::moveToZero() {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyRobotControl");
  RCLCPP_INFO(logger, "Zeroing all Motors!");
  for (int i = 1; i < 7; i++) {
    setAbsoluteMotorPosition(i, 0, 15*RobotConstants::AXIS_RATIO[i-1], 20);
  }
  uint8_t status;
  bool done = false;
  while (1) {
    // log position
    RCLCPP_INFO(logger, "1: %5.0f -> 0 2: %5.0f -> 0 3: %5.0f -> 0 4: %5.0f -> 0 5.0: %5.0f -> 0 6: %5.0f -> 0", 
        abs(std::round(getMotorPosition(1, 100))), 
        abs(std::round(getMotorPosition(2, 100))), 
        abs(std::round(getMotorPosition(3, 100))), 
        abs(std::round(getMotorPosition(4, 100))), 
        abs(std::round(getMotorPosition(5, 100))), 
        abs(std::round(getMotorPosition(6, 100))));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    done = true;
    for (int a = 1; a < 7; a++) {
      status = getStatus(a, 100);
      if (status != 1) {
        done = false;
      }
    }
    if (done) {
      RCLCPP_INFO(logger, "Done!");
      return true;
    }
  }
  return true;

}

void JonnyRobotControl::homeAll() {
  homeXAxis();
  homeYAxis();
  homeZAxis();
  homeAAxis();
  homeBCAxis();
  // std::thread tX(&JonnyRobotControl::homeXAxis, this);
  // std::thread tY(&JonnyRobotControl::homeYAxis, this);
  // std::thread tZ(&JonnyRobotControl::homeZAxis, this);
  // std::thread tA(&JonnyRobotControl::homeAAxis, this);
  // std::thread tBC(&JonnyRobotControl::homeBCAxis, this);
  // tX.join();
  // tY.join();
  // tZ.join();
  // tA.join();
  // tBC.join();
}

int main() {
  JonnyRobotControl robot = JonnyRobotControl();
  robot.init();
  // robot.homeAll();
  // robot.moveToZero();
  return 0;
}
