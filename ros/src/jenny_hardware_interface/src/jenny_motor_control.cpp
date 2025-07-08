#include "jenny_motor_control.hpp"
#include <rclcpp/logging.hpp>

bool JennyMotorControl::init() {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  setupSender();
  setupReceiver();

  homeXAxis();
  homeYAxis();
  homeZAxis();

  return true;
}

////////////////////// send Data /////////////////////////
bool JennyMotorControl::sendData(uint8_t can_id, std::vector<uint8_t> data_vec) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  // create can id
  drivers::socketcan::CanId canId = drivers::socketcan::CanId(can_id, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame);
  // creating crc
  uint16_t crc = can_id;
  for (size_t i = 0; i < data_vec.size(); i++) {
    crc += data_vec[i];
  }
  crc &= 0xFF;  // Keep only lower byte
  // append crc to data
  data_vec.push_back(static_cast<uint8_t>(crc));

  // try send data
  try {
    sender->send(data_vec.data(), data_vec.size(), canId);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Error sending CAN frame: %s", e.what());
    return false;
  }
  return true;
}

////////////////////// receive Data /////////////////////////
std::tuple<uint8_t, uint8_t, std::vector<uint8_t>> JennyMotorControl::receiveData(uint16_t timeout) {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  std::vector<uint8_t> response(8, 0); // Maximum data length for standard CAN frames/
  uint8_t id = 0;
  uint8_t length = 0;
  try {
    drivers::socketcan::CanId can_id = receiver->receive(response.data(), std::chrono::milliseconds(timeout));
    id = can_id.get();
    length = can_id.length();
  } catch (const drivers::socketcan::SocketCanTimeout &) {
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(logger, "Error receiving CAN frame: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger, "Encountered unkown error when during handlePositionResponses");
  }
  return std::make_tuple(id, length, response);
}

////////////////////// setup Sender /////////////////////////
bool JennyMotorControl::setupSender() {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  try {
    sender = std::make_unique<drivers::socketcan::SocketCanSender>("can0");
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(logger, "Failed to configure SocketCAN: %s", e.what());
    sender.reset();
    return false;
  } catch (...) {
    RCLCPP_FATAL(logger, "An unexpected error occurred during SocketCAN configuration.");
    sender.reset();
    return false;
  }
  return true;
}

////////////////////// setup Receiver /////////////////////////
bool JennyMotorControl::setupReceiver() {
  rclcpp::Logger logger = rclcpp::get_logger("JennyMotorControl");
  // Initialize Receiver
  try {
    receiver = std::make_unique<drivers::socketcan::SocketCanReceiver>("can0", false);
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(logger, "Failed to configure SocketCAN: %s", e.what());
    receiver.reset();
    return false;
  } catch (...) {
    RCLCPP_FATAL(logger, "An unexpected error occurred during SocketCAN configuration.");
    receiver.reset();
    return false;
  }
  return true;
}


int main() {
  JennyMotorControl motor_control = JennyMotorControl();
  motor_control.init();
  return 0;
}
