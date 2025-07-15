#include "jonny_motor_control.hpp"
#include <rclcpp/logging.hpp>

bool JonnyMotorControl::init() {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyMotorControl");
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
  JonnyMotorControl motor_control = JonnyMotorControl();
  motor_control.init();
  return 0;
}
