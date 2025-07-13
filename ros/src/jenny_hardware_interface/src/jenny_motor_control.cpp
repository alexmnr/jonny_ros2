#include "jenny_motor_control.hpp"
#include <rclcpp/logging.hpp>

bool JennyMotorControl::init() {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  setupSender();
  setupReceiver();

  // homeXAxis();
  // homeYAxis();
  // homeZAxis();
  // homeAAxis();
  // double BC_position[2] = {0, 20};
  // setRelativeBCJointPosition(BC_position, 50, 200);
  // waitTillStopped(5);
  homeBCAxis();

  return true;
}

int main() {
  JennyMotorControl motor_control = JennyMotorControl();
  motor_control.init();
  return 0;
}
