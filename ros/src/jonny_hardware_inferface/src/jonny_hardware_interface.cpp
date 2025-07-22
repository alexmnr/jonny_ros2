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

  // // thread for receiving can responses
  // stop_thread_.store(false);
  // can_response_thread_ = std::thread(&JonnyHardwareInterface::handleCANResponses, this);

  // // Create status publisher
  // pub_ = node_->create_publisher<jonny_interfaces::msg::HardwareStatus>("/hardware_status", 10);

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
  // for (int i = 0; i < 6; i++) {
  //   joint_position_[i] = getJointPosition(i);
  // }

  joint_position_[0] = robot.getJointPosition(0, 100) * JonnyRobotControl::MotorConstants::DEG_TO_RAD;
  joint_position_[1] = robot.getJointPosition(1, 100) * JonnyRobotControl::MotorConstants::DEG_TO_RAD;
  joint_position_[2] = robot.getJointPosition(2, 100) * JonnyRobotControl::MotorConstants::DEG_TO_RAD;
  joint_position_[3] = robot.getJointPosition(3, 100) * JonnyRobotControl::MotorConstants::DEG_TO_RAD;
  joint_position_[4] = robot.getJointPosition(4, 100) * JonnyRobotControl::MotorConstants::DEG_TO_RAD;
  joint_position_[5] = robot.getJointPosition(5, 100) * JonnyRobotControl::MotorConstants::DEG_TO_RAD;
  // joint_position_[0] = joint_position_command_[0];
  // joint_position_[1] = joint_position_command_[1];
  // joint_position_[2] = joint_position_command_[2];
  // joint_position_[3] = joint_position_command_[3];
  // joint_position_[4] = joint_position_command_[4];
  // joint_position_[5] = joint_position_command_[5];

  // publish Hardware Info Topic
  // publishHardwareInfo();

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
    double position_diff = abs(position - robot.getJointPosition(i, 100));
    double speed = position_diff * (100 / milliseconds);
    robot.setAbsoluteXYZAJointPosition(i, position, speed, 0);
  }
  // joint 5-6
  double position_5 = joint_position_command_[4] * JonnyRobotControl::MotorConstants::RAD_TO_DEG;
  double position_diff_5 = abs(position_5 - robot.getJointPosition(4, 100));
  double speed_5 = position_diff_5 * (100 / milliseconds);
  double position_6 = joint_position_command_[5] * JonnyRobotControl::MotorConstants::RAD_TO_DEG;
  double position_diff_6 = abs(position_6 - robot.getJointPosition(5, 100));
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

// ////////////////////// handle responses
// void JonnyHardwareInterface::handleCANResponses() {
//   rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");
//   // Initialize Receiver
//   try {
//     if (config_.debug) {RCLCPP_INFO(logger, "Initializing CAN Receiver...");}
//     receiver = std::make_unique<drivers::socketcan::SocketCanReceiver>("can0");
//     if (config_.debug) {RCLCPP_INFO(logger, "CAN Receiver initialized and opened successfully.");}
//   } catch (const std::runtime_error & e) {
//     RCLCPP_FATAL(logger, "Failed to configure SocketCAN: %s", e.what());
//     receiver.reset();
//     return;
//   } catch (...) {
//     RCLCPP_FATAL(logger, "An unexpected error occurred during SocketCAN configuration.");
//     receiver.reset();
//     return;
//   }

//   std::vector<uint8_t> response(8, 0); // Maximum data length for standard CAN frames/
//                                        //
//   while (rclcpp::ok() && !stop_thread_.load()) {
//     try {
//       drivers::socketcan::CanId can_id = receiver->receive(response.data(), std::chrono::milliseconds(10));
//       uint8_t id = can_id.get();
//       uint8_t length = can_id.length();
      
//       ////////////// motor position response
//       if (length == 8 && response[0] == 0x31){
//         int64_t value = (static_cast<int64_t>(response[1]) << 40) |
//           (static_cast<int64_t>(response[2]) << 32) |
//           (static_cast<int64_t>(response[3]) << 24) |
//           (static_cast<int64_t>(response[4]) << 16) |
//           (static_cast<int64_t>(response[5]) << 8) |
//           static_cast<int64_t>(response[6]);
//         // Apply two's complement to handle signed 48-bit values
//         if (value & 0x800000000000) {  // Check if the 47th bit (sign bit) is set
//           value |= 0xFFFF000000000000;  // Sign-extend to 64 bits
//         }
//         // Convert addition value to degrees
//         motor_stats[id-1].position = (double)(value * MotorConstants::DEGREES_PER_REVOLUTION) / MotorConstants::ENCODER_STEPS;

//         // find time difference
//         current_thread_time_ = std::chrono::high_resolution_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_thread_time_ - motor_stats[id-1].previous_time);
//         auto milliseconds = duration.count();

//         // calculate speed
//         double buffer;
//         buffer = (motor_stats[id-1].position - motor_stats[id-1].previous_position) / milliseconds;
//         buffer *= 1000; // to seconds
//         buffer *= MotorConstants::DEG_TO_RAD; // to rads
//         motor_stats[id-1].velocity = buffer;
//         // after calculations (for next time)
//         motor_stats[id-1].previous_position = motor_stats[id-1].position;
//         motor_stats[id-1].previous_time = current_thread_time_;

//       //////////// ignore
//       } else if (length == 2) {

//       //////////// ignore
//       } else if (length == 8 && response[0] == 0xF5) {
        
//       //////////// motor status
//       } else if (length == 3 && response[0] == 0xF1) {
//         if (response[1] == 1) {
//           motor_stats[id - 1].ready = true;
//         }
//       //////////// start/stopping
//       } else if (length == 3 && response[0] == 0xF5) {
//         if (config_.debug) {
//           if (response[1] == 1) {
//             RCLCPP_INFO(logger, "Motor %d started motion", id);
//           } else if (response[1] == 2) {
//             RCLCPP_INFO(logger, "Motor %d completed motion", id);
//           }
//         }

//       //////////// unkown
//       } else {
//         char buffer[100];
//         memset(buffer, 0, sizeof buffer);
//         for (size_t i = 0; i < length; ++i) {
//           std::sprintf(buffer + strlen(buffer), "%X ", response[i]);
//         }
//         RCLCPP_INFO(logger, "Uknown Response from id %d: %s ", id, buffer);
//       }

//     } catch (const drivers::socketcan::SocketCanTimeout &) {
//       // No frame received within the timeout, which is expected for a zero timeout when no messages are present.
//     } catch (const std::runtime_error & e) {
//       RCLCPP_ERROR(logger, "Error receiving CAN frame: %s", e.what());
//     } catch (...) {
//       RCLCPP_ERROR(logger, "Encountered unkown error when during handlePositionResponses");
//     }
//   }
//   return;
// }

// ////////////////////// print Joint Info
// void JonnyHardwareInterface::printJointInfo(uint8_t id) {
//   rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");
//   double setPos = joint_position_command_[id];
//   double actPos = getJointPosition(id);
//   double posDiff = abs(actPos - setPos);
//   double setVel = joint_velocities_command_[id];
//   double actVel = getJointVelocity(id);
//   double velDiff = abs(actVel - setVel);
//   RCLCPP_INFO(logger, "SetPos: %.4f ActPos: %.4f PosDiff: %.4f SetVel: %.4f ActVel: %.4f VelDiff: %.4f", setPos, actPos, posDiff, setVel, actVel, velDiff);
// }

// ////////////////////// publish Hardware Info
// void JonnyHardwareInterface::publishHardwareInfo() {
//   auto msg = jonny_interfaces::msg::HardwareStatus();
//   msg.stamp = node_->get_clock()->now();
//   msg.status = "Jup";
//   for (int i = 0; i < 6; i++) {
//     auto joint = jonny_interfaces::msg::JointStatus();
//     joint.id = i;
//     joint.set_position = joint_position_command_[i];
//     joint.actual_position = getJointPosition(i);
//     joint.set_velocity = joint_velocities_command_[i];
//     joint.actual_velocity = getJointVelocity(i);
//     msg.joints.push_back(joint);
//   }
//   pub_->publish(msg);
// }

////////////////////// on_cleanup /////////////////////////
CallbackReturn
JonnyHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");
  RCLCPP_INFO(logger, "Cleaning up ...please wait...");
  // stop_thread_.store(true);
  // if (can_response_thread_.joinable()) {
  //   can_response_thread_.join();
  // }

  RCLCPP_INFO(logger, "Successfully cleaned up!");

  return CallbackReturn::SUCCESS;
}

////////////////////// on_shutdown /////////////////////////
CallbackReturn
JonnyHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &) {
  rclcpp::Logger logger = rclcpp::get_logger("JonnyHardwareInterface");
  RCLCPP_INFO(logger, "Shutting down ...please wait...");
  // stop_thread_.store(true);
  // can_response_thread_.join();

  RCLCPP_INFO(logger, "Successfully cleaned up!");

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
