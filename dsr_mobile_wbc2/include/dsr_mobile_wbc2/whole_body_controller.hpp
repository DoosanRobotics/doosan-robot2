#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"

namespace dsr_mobile_wbc2
{

class WholeBodyController : public controller_interface::ControllerInterface
{
public:
  WholeBodyController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJT = rclcpp_action::ServerGoalHandle<FollowJT>;

  bool enable_base_wbc_ {false};
  std::string base_cmd_topic_;
  std::string base_cmd_msg_type_ {"auto"};
  std::string forward_action_ns_;
  double max_linear_vel_ {0.5};
  double max_angular_vel_ {1.0};
  double base_linear_sign_ {1.0};
  double base_angular_sign_ {1.0};
  std::string base_multi_dof_joint_name_ {"base_planar_joint"};
  std::string base_x_joint_name_ {"base_x"};
  std::string base_y_joint_name_ {"base_y"};
  std::string base_yaw_joint_name_ {"base_yaw"};
  std::string arm_joint_state_topic_ {"/joint_states"};
  std::string whole_joint_state_topic_ {"/joint_states_whole"};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr base_cmd_stamped_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr whole_joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_state_sub_;
  rclcpp_action::Server<FollowJT>::SharedPtr action_server_;
  rclcpp_action::Client<FollowJT>::SharedPtr dual_arm_action_client_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJT::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowJT> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleFollowJT> goal_handle);
  void execute(const std::shared_ptr<GoalHandleFollowJT> goal_handle);
  void publish_base_velocity_profile(const trajectory_msgs::msg::JointTrajectory & whole_traj);
  void publish_base_velocity_profile(
    const trajectory_msgs::msg::MultiDOFJointTrajectory & whole_traj);
  void publish_base_joint_state(double x, double y, double yaw);
  FollowJT::Goal build_arm_goal(const FollowJT::Goal & whole_goal, bool & has_base_joint) const;

  bool active_ {false};
  std::mutex base_state_mutex_;
  std::mutex arm_state_mutex_;
  sensor_msgs::msg::JointState arm_joint_state_msg_;
  bool arm_joint_state_initialized_ {false};
  bool base_state_initialized_ {false};
  double base_state_x_ {0.0};
  double base_state_y_ {0.0};
  double base_state_yaw_ {0.0};
  double idle_base_state_publish_period_sec_ {0.02};  // keep-alive timer tick
  double base_state_keepalive_duration_sec_ {1.0};  // keep publishing briefly after updates
  double joint_state_time_offset_sec_ {0.02};  // offset joint state stamp to reduce TF future extrapolation
  rclcpp::Time last_idle_base_state_publish_time_;
  rclcpp::Time base_state_keepalive_deadline_;
  rclcpp::TimerBase::SharedPtr idle_base_state_timer_;
};

}  // namespace dsr_mobile_wbc2
