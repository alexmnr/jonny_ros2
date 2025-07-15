#include "jonny_robot_control.hpp"
#include <rclcpp/logging.hpp>

bool JonnyRobotControl::init() {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyRobotControl");
  setupSender();
  setupReceiver();

  homeXAxis();
  homeYAxis();
  homeZAxis();
  homeAAxis();
  homeBCAxis();

  return true;
}

int main() {
  JonnyRobotControl motor_control = JonnyRobotControl();
  motor_control.init();
  return 0;
}
