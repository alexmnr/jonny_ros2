#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/utils/logger.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace moveit_servo;
TwistCommand target_twist = { "link_6", { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

void topic_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  target_twist.velocities[0] = msg->twist.linear.y;
  target_twist.velocities[1] = msg->twist.linear.x;
  target_twist.velocities[2] = msg->twist.linear.z;
  target_twist.velocities[3] = msg->twist.angular.x;
  target_twist.velocities[4] = msg->twist.angular.z;
  target_twist.velocities[5] = msg->twist.angular.y;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // The servo object expects to get a ROS node.
  const rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("moveit_servo_node");
  moveit::setNodeLoggerName(node->get_name());

  // Get the servo parameters.
  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();


  // The publisher to send trajectory message to the robot controller.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
      node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                         rclcpp::SystemDefaultsQoS());
  // Create the servo object
  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(node, servo_params);
  Servo servo = Servo(node, servo_param_listener, planning_scene_monitor);

  // Wait for some time, so that the planning scene is loaded in rviz.
  // This is just for convenience, should not be used for sync in real application.
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Get the robot state and joint model group info.
  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);

  // Set the command type for servo.
  servo.setCommandType(CommandType::TWIST);

  auto subscription = node->create_subscription<geometry_msgs::msg::TwistStamped>("/twist_cmds", 10, topic_callback);

  // Frequency at which commands will be sent to the robot controller.
  rclcpp::WallRate rate(1.0 / servo_params.publish_period);

  // create command queue to build trajectory message and add current robot state
  std::deque<KinematicState> joint_cmd_rolling_window;
  KinematicState current_state = servo.getCurrentRobotState(true /* wait for updated state */);
  updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, node->now());

  RCLCPP_INFO_STREAM(node->get_logger(), servo.getStatusMessage());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok())
  {
    KinematicState joint_state = servo.getNextJointState(robot_state, target_twist);
    const StatusCode status = servo.getStatus();
    if (status != StatusCode::INVALID)
    {
      updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, node->now());
      if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
      {
        trajectory_outgoing_cmd_pub->publish(msg.value());
      }
      if (!joint_cmd_rolling_window.empty())
      {
        robot_state->setJointGroupPositions(joint_model_group, joint_cmd_rolling_window.back().positions);
        robot_state->setJointGroupVelocities(joint_model_group, joint_cmd_rolling_window.back().velocities);
      }
    }
    exec.spin_once();
    rate.sleep();
  }
  rclcpp::shutdown();
}

