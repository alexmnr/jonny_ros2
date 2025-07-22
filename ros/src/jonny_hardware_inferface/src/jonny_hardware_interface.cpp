#include "jonny_hardware_interface.hpp"
#include <rclcpp/logging.hpp>

namespace jonny_hardware_interface {

////////////////////// on_init /////////////////////////
CallbackReturn
JonnyHardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");
  node_ = rclcpp::Node::make_shared("JonnyHardwareInterfaceLogger");

  // robot has 6 joints and 2 interfaces
  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);

  for (const auto &joint : info_.joints) {
    for (const auto &interface : joint.state_interfaces) {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }
  RCLCPP_INFO(logger, "Initializing Jonny Hardware Interface...");

  // set up config
  config_.debug = false;
  config_.homing = false;

  // set up motor data
  auto now = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 6; i++) {
    motor_stats[i].id = i;
    motor_stats[i].ready = true;
    motor_stats[i].position = 0.0;
    motor_stats[i].previous_position = 0.0;
    motor_stats[i].velocity = 0.0;
    motor_stats[i].previous_time = now;
  }

  JonnyRobotControl robot = JonnyRobotControl();

  // Create status publisher
  pub_ = node_->create_publisher<jonny_interfaces::msg::HardwareStatus>("/hardware_status", 10);

  return CallbackReturn::SUCCESS;
}

////////////////////// on_configure /////////////////////////
CallbackReturn
JonnyHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");

  robot.init();

  return CallbackReturn::SUCCESS;
}

////////////////////// on_activate /////////////////////////
CallbackReturn
JonnyHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");

  for (int i = 0; i < 6; i++) {
    joint_position_[i] = 0.0;
    joint_velocities_[i] = 0.0;
  }

  bool check = robot.check();
  if (!check) {
    RCLCPP_ERROR(logger, "Connected Robot failed check!");
    return CallbackReturn::FAILURE;
  }

  if (config_.homing) {
    RCLCPP_INFO(logger, "Homing Robot!");
    robot.homeAll();
  }

  robot.moveToZero();

  RCLCPP_INFO(logger, "Initialization finished!");
  ready = true;

  return CallbackReturn::SUCCESS;
}


////////////////////// read /////////////////////////
return_type JonnyHardwareInterface::read(const rclcpp::Time & /*time*/,
                                          const rclcpp::Duration &) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");
  if (ready == false) {
    return return_type::OK;
  }

  // get current position for each joint
  for (int i = 0; i < 6; i++) {
    joint_position_[i] = getJointPosition(i) * JonnyRobotControl::MotorConstants::DEG_TO_RAD;
  }

  // publish Hardware Info Topic
  publishHardwareInfo();

  return return_type::OK;
}

////////////////////// write /////////////////////////
return_type JonnyHardwareInterface::write(const rclcpp::Time &,
                                           const rclcpp::Duration &) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");

  // dont write if not ready yet
  if (ready == false) {
    return return_type::OK;
  }

  // calculate each motors speed
  for (int i = 0; i < 6; i++) {
    // find time difference
    current_loop_time_ = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_loop_time_ - motor_stats[i].previous_time);
    double milliseconds = duration.count();
    // calculate speed
    motor_stats[i].position = robot.getMotorPosition(i+1, 100);
    double buffer;
    buffer = (motor_stats[i].position - motor_stats[i].previous_position) / milliseconds;
    buffer *= 1000; // to seconds
    motor_stats[i].velocity = buffer;
    // after calculations (for next time)
    motor_stats[i].previous_position = motor_stats[i].position;
    motor_stats[i].previous_time = current_loop_time_;
  }

  // find time difference
  current_loop_time_ = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_loop_time_ - previous_loop_time_);
  double milliseconds = duration.count();
  // joint 1-4
  for (int i = 0; i < 4; i++) {
    double position = joint_position_command_[i] * JonnyRobotControl::MotorConstants::RAD_TO_DEG;
    double position_diff = abs(position - getJointPosition(i));
    double speed = position_diff * (100 / milliseconds);
    robot.setAbsoluteXYZAJointPosition(i, position, speed, 0);
  }
  // joint 5-6
  double position_5 = joint_position_command_[4] * JonnyRobotControl::MotorConstants::RAD_TO_DEG;
  double position_diff_5 = abs(position_5 - getJointPosition(4));
  double speed_5 = position_diff_5 * (100 / milliseconds);
  double position_6 = joint_position_command_[5] * JonnyRobotControl::MotorConstants::RAD_TO_DEG;
  double position_diff_6 = abs(position_6 - getJointPosition(5));
  double speed_6 = position_diff_6 * (100 / milliseconds);
  double BC_position[2] = {position_5, position_6};
  if (speed_5 > speed_6) {
    robot.setAbsoluteBCJointPosition(BC_position, speed_5, 0);
  } else {
    robot.setAbsoluteBCJointPosition(BC_position, speed_6, 0);
  }

  // save for next run
  previous_loop_time_ = current_loop_time_;
  
  return return_type::OK;
}

////////////////////// get Joint Position
double JonnyHardwareInterface::getJointPosition(uint8_t id) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyJointInterface");
  if (id < 4) {
    double motor_position = motor_stats[id].position;
    double joint_position = motor_position / JonnyRobotControl::RobotConstants::AXIS_RATIO[id];
    return joint_position;
  } else if (id == 4) {
    double motor_position_5 = motor_stats[4].position;
    double motor_position_6 = motor_stats[5].position;
    double joint_position = (0.5 * (motor_position_6 + motor_position_5));
    joint_position /= JonnyRobotControl::RobotConstants::AXIS_RATIO[4];
    return joint_position;
  } else if (id == 5) {
    double motor_position_5 = motor_stats[4].position;
    double motor_position_6 = motor_stats[5].position;
    double joint_position = (0.5 * (motor_position_6 - motor_position_5));
    joint_position /= JonnyRobotControl::RobotConstants::AXIS_RATIO[4];
    return joint_position;
  } else {
    RCLCPP_ERROR(logger, "Wrong Joint ID!");
    return 0.0;
  }
}
////////////////////// calculate Joint Velocity
double JonnyHardwareInterface::getJointVelocity(uint8_t id) {
  double joint_velocity;
  if (id == 4) {
    joint_velocity = (0.5 * (motor_stats[5].velocity + motor_stats[4].velocity));
    joint_velocity = (joint_velocity / JonnyRobotControl::RobotConstants::AXIS_RATIO[id]);
  } else if (id == 5) {
    joint_velocity = (0.5 * (motor_stats[5].velocity - motor_stats[4].velocity));
    joint_velocity = (joint_velocity / JonnyRobotControl::RobotConstants::AXIS_RATIO[id]);
  } else {
    joint_velocity = motor_stats[id].velocity;
    joint_velocity = (joint_velocity / JonnyRobotControl::RobotConstants::AXIS_RATIO[id]);
  }
  return joint_velocity;
}


////////////////////// print Joint Info
void JonnyHardwareInterface::printJointInfo(uint8_t id) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");
  double setPos = joint_position_command_[id];
  double actPos = getJointPosition(id);
  double posDiff = abs(actPos - setPos);
  double setVel = joint_velocities_command_[id];
  double actVel = getJointVelocity(id);
  double velDiff = abs(actVel - setVel);
  RCLCPP_INFO(logger, "SetPos: %.4f ActPos: %.4f PosDiff: %.4f SetVel: %.4f ActVel: %.4f VelDiff: %.4f", setPos, actPos, posDiff, setVel, actVel, velDiff);
}

////////////////////// publish Hardware Info
void JonnyHardwareInterface::publishHardwareInfo() {
  auto msg = jonny_interfaces::msg::HardwareStatus();
  msg.stamp = node_->get_clock()->now();
  msg.status = "Jup";
  for (int i = 0; i < 6; i++) {
    auto joint = jonny_interfaces::msg::JointStatus();
    joint.id = i;
    joint.set_position = std::round(joint_position_command_[i] * JonnyRobotControl::MotorConstants::RAD_TO_DEG * 100) / 100;
    joint.actual_position = std::round(getJointPosition(i) * 100) / 100;
    joint.set_velocity = std::round(joint_velocities_command_[i] * JonnyRobotControl::MotorConstants::RAD_TO_DEG * 100) / 100;
    joint.actual_velocity = std::round(getJointVelocity(i) * 100) / 100;
    msg.joints.push_back(joint);
  }
  pub_->publish(msg);
}

////////////////////// on_cleanup /////////////////////////
CallbackReturn
JonnyHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");
  RCLCPP_INFO(logger, "Cleaning up ...please wait...");
  RCLCPP_INFO(logger, "Successfully cleaned up!");
  return CallbackReturn::SUCCESS;
}

////////////////////// on_shutdown /////////////////////////
CallbackReturn
JonnyHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");
  RCLCPP_INFO(logger, "Shutting down ...please wait...");
  RCLCPP_INFO(logger, "Successfully shuttted down!");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
JonnyHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto &joint_name : joint_interfaces["position"]) {
    state_interfaces.emplace_back(joint_name, "position",
                                  &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name : joint_interfaces["velocity"]) {
    state_interfaces.emplace_back(joint_name, "velocity",
                                  &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
JonnyHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto &joint_name : joint_interfaces["position"]) {
    command_interfaces.emplace_back(joint_name, "position",
                                    &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name : joint_interfaces["velocity"]) {
    command_interfaces.emplace_back(joint_name, "velocity",
                                    &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}

} // namespace jonny_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(jonny_hardware_interface::JonnyHardwareInterface,
                       hardware_interface::SystemInterface)
