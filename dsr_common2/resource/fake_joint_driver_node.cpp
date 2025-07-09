/**
 * @file fake_joint_driver_node.cpp
 * @author Ryosuke Tajima
 * @copyright 2016, 2017, Tokyo Opensource Robotics Kyokai Association
 * @license http://www.apache.org/licenses/LICENSE-2.0 Apache-2.0
 *
 * Device driver node to fake loopback joints.
 */
#include <controller_manager/controller_manager.hpp>
#include <fake_joint_driver/fake_joint_driver.h>
#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("fake_joint_driver");
static constexpr double SPIN_RATE = 200;  // Hz

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

/**
 * @brief Main function
 */
int main(int argc, char** argv)
{
  // Init ROS node
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = rclcpp::Node::make_shared("fake_joint_driver_node");

  auto controller_name = node->declare_parameter("controller_name", std::string());
  if (controller_name.empty())
  {
    RCLCPP_ERROR(LOGGER, "Can't have empty controller name -- please set the parameter `controller_name`");
    return -1;
  }

  // Create hardware interface
  auto robot = std::make_shared<FakeJointDriver>(node);
  // Connect to controller manager
  controller_manager::ControllerManager cm(robot, executor);

  auto controller1 = cm.load_controller("fake_joint_state_controller","joint_state_controller/JointStateController");
  auto controller2 = cm.load_controller(controller_name, "joint_trajectory_controller/JointTrajectoryController");

  // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
  auto future_handle = std::async(std::launch::async, spin, executor);
  /**
  // we can either configure each controller individually through its services
  // or we use the controller manager to configure every loaded controller
  if (cm.configure() != controller_interface::return_type::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "at least one controller failed to configure");
    return -1;
  }
  RCLCPP_INFO(LOGGER, "Successfully configured all controllers");

  // and activate all controller
  if (cm.activate() != controller_interface::return_type::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "At least one controller failed to activate");
    return -1;
  }
  RCLCPP_INFO(LOGGER, "Successfully activated all controllers");
  **/
  controller1->get_lifecycle_node()->configure();
  controller1->get_lifecycle_node()->activate();
  controller2->get_lifecycle_node()->configure();
  controller2->get_lifecycle_node()->activate();
  // Set spin rate
  rclcpp::Rate rate(SPIN_RATE);
  while (rclcpp::ok())
  {
    robot->update();
    cm.update();
    rate.sleep();
  }

  return 0;
}
