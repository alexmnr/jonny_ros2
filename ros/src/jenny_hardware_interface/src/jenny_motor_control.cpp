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
  // setRelativeMotorPosition(6, -5000, 2000, 200);
  // double BC_position[2] = {0, 0};
  // setAbsoluteBCJointPosition(BC_position, 72, 200);
  setMotorVelocity(6, 000, 5);

  return true;
}

int main() {
  JennyMotorControl motor_control = JennyMotorControl();
  motor_control.init();
  return 0;
}
