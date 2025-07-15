#include "jonny_robot_control.hpp"
#include <rclcpp/logging.hpp>
#include <thread>

bool JonnyRobotControl::init() {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyRobotControl");
  setupSender();
  setupReceiver();

  // std::thread tX(&JonnyRobotControl::homeXAxis, this);
  // std::thread tY(&JonnyRobotControl::homeYAxis, this);
  // std::thread tZ(&JonnyRobotControl::homeZAxis, this);
  // std::thread tA(&JonnyRobotControl::homeAAxis, this);
  // std::thread tBC(&JonnyRobotControl::homeBCAxis, this);
  // homeXAxis();
  // homeYAxis();
  // homeZAxis();
  // homeAAxis();
  // homeBCAxis();
  // tX.join();
  // tY.join();
  // tZ.join();
  // tA.join();
  // tBC.join();
  setAbsoluteXYZAJointPosition(1, -30, 20, 100);
  setAbsoluteXYZAJointPosition(2, -100, 20, 100);

  return true;
}

int main() {
  JonnyRobotControl motor_control = JonnyRobotControl();
  motor_control.init();
  return 0;
}
