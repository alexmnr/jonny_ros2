#include <rclcpp/logging.hpp>
#include <math.h>
#include <tuple>
#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"

class JennyMotorControl {
  public:
    bool init();

  private:
    // functions
    bool setupSender();
    bool setupReceiver();

    // End Stop
    bool readEndStop(uint8_t can_id, uint16_t timeout);

    // Motor Control by Position
    bool setAbsoluteMotorPosition(uint8_t can_id, double position, double speed, double acceleration);
    bool setRelativeMotorPosition(uint8_t can_id, double position, double speed, double acceleration);
    bool stopAbsoluteMotor(uint8_t can_id, double acceleration);
    bool stopRelativeMotor(uint8_t can_id, double acceleration);

    // Get Position
    bool requestPosition(uint8_t can_id);
    double readMotorPosition(uint8_t can_id, uint16_t timeout);
    
    // Motor Control by velocity
    bool setMotorVelocity(uint8_t can_id, double speed, double acceleration);

    // Other
    bool setZero(uint8_t can_id);
    bool requestStatus(uint8_t can_id);
    uint8_t readStatus(uint8_t can_id, uint16_t timeout);

    // Homing
    void homeXAxis();
    void homeYAxis();
    void homeZAxis();

    bool moveTillEndstop(uint8_t id, double limit, double speed, double acceleration);

    // can function
    bool sendData(uint8_t can_id, std::vector<uint8_t> data_vec);
    std::tuple<uint8_t, uint8_t, std::vector<uint8_t>> receiveData(uint16_t timeout);

    // motor constants
    const std::array<double, 6> motor_seeking_speeds = {30, 1, 1, 1, 1, 1};
    const std::array<double, 6> motor_locating_speeds = {5, 20, 20, 1, 1, 1};
    const std::array<double, 6> motor_home_location = {-180, -92, 119, 1, 1, 1};

    // can stuff
    std::unique_ptr<drivers::socketcan::SocketCanSender> sender;
    std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver;

  public: 
    struct CANCommands {
      // Query commands
      static constexpr uint8_t QUERY_MOTOR = 0xF1; /**< Query motor command. */
      // Read commands
      static constexpr uint8_t READ_ENCODER = 0x31; /**< Read encoder value (addition). */
      static constexpr uint8_t READ_VELOCITY = 0x32; /**< Read velocity command. */
      static constexpr uint8_t READ_IO = 0x34; /**< Read IO command. */

      // Set/Control commands
      static constexpr uint8_t CALIBRATE = 0x80; /**< Calibrate command. */
      static constexpr uint8_t SET_WORKING_MODE = 0x82; /**< Set working mode command. */
      static constexpr uint8_t SET_CURRENT = 0x83; /**< Set current command. */
      static constexpr uint8_t SET_SUBDIVISIONS = 0x84; /**< Set subdivisions command. */
      static constexpr uint8_t SET_HOME_PARAMS = 0x90; /**< Set home parameters command. */
      static constexpr uint8_t GO_HOME = 0x91; /**< Go home command. */
      static constexpr uint8_t SET_ZERO_POSITION = 0x92; /**< Set zero command. */

      // Motion commands
      static constexpr uint8_t ENABLE_MOTOR = 0xF3; /**< Enable motor command. */
      static constexpr uint8_t RELATIVE_POSITION = 0xF4; /**< Relative position command. */
      static constexpr uint8_t ABSOLUTE_POSITION = 0xF5; /**< Absolute position command. */
      static constexpr uint8_t SPEED_CONTROL = 0xF6; /**< Speed control command. */
      static constexpr uint8_t EMERGENCY_STOP = 0xF7; /**< Emergency stop command. */
    };
    struct MotorConstants {
      // Encoder constants
      static constexpr double ENCODER_STEPS = 16384.0;  // 0x4000 steps per revolution
      static constexpr double DEGREES_PER_REVOLUTION = 360.0;
      static constexpr double RADIANS_PER_REVOLUTION = 2.0 * M_PI;

      // Motor speed limits (RPM)
      static constexpr double MAX_RPM_OPEN = 400.0;    // OPEN mode max speed
      static constexpr double MAX_RPM_CLOSE = 1500.0;  // CLOSE mode max speed
      static constexpr double MAX_RPM_vFOC = 3000.0;   // vFOC mode max speed

      // Conversion factors
      static constexpr double RPM_TO_RADPS = M_PI / 30.0;     // RPM to rad/s (π/30)
      static constexpr double RADPS_TO_RPM = 30.0 / M_PI;     // rad/s to RPM (30/π)
      static constexpr double DEGPS_TO_RPM = 60.0 / 360.0;     // rad/s to RPM (30/π)
      static constexpr double DEG_TO_RAD = M_PI / 180.0;      // degrees to radians
      static constexpr double RAD_TO_DEG = 180.0 / M_PI;      // radians to degrees
      
      // reversed or not
      static constexpr double AXIS_SET_INVERTED[6] = {1, -1, 1, -1, -1, 1};
      static constexpr double AXIS_GET_INVERTED[6] = {1, -1, -1, -1, -1, 1};
      static constexpr double AXIS_RATIO[6] = {14, 150, 150, 45, 45, 45};
    };

};
