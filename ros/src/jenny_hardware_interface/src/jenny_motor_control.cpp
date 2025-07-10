#include "jenny_motor_control.hpp"
#include <rclcpp/logging.hpp>

bool JennyMotorControl::init() {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  setupSender();
  setupReceiver();

  homeXAxis();
  homeYAxis();
  homeZAxis();
  homeAAxis();

  return true;
}

int main() {
  JennyMotorControl motor_control = JennyMotorControl();
  motor_control.init();
  return 0;
}
