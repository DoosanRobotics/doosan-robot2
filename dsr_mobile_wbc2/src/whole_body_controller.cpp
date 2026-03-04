#include "dsr_mobile_wbc2/whole_body_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <functional>
#include <future>
#include <limits>
#include <thread>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace
{
constexpr const char * kLoggerName = "dsr_mobile_wbc2";
using namespace std::chrono_literals;

std::string normalize_type_string(std::string v)
{
  v.erase(
    std::remove_if(
      v.begin(), v.end(),
      [](unsigned char c) {return std::isspace(c) != 0;}),
    v.end());
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return v;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalize_angle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}
}  // namespace

namespace dsr_mobile_wbc2
{

WholeBodyController::WholeBodyController() : controller_interface::ControllerInterface() {}

controller_interface::InterfaceConfiguration
WholeBodyController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
WholeBodyController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn WholeBodyController::on_init()
{
  try {
    auto_declare<bool>("enable_base_wbc", false);
    auto_declare<std::string>("base_cmd_topic", "~/base_cmd_vel");
    auto_declare<std::string>("base_cmd_msg_type", "auto");
    auto_declare<std::string>(
      "forward_action_ns", "dual_dsr_moveit_controller/follow_joint_trajectory");
    auto_declare<double>("max_linear_vel", 0.5);
    auto_declare<double>("max_angular_vel", 1.0);
    auto_declare<double>("base_linear_sign", 1.0);
    auto_declare<double>("base_angular_sign", 1.0);
    auto_declare<std::string>("base_multi_dof_joint_name", "base_planar_joint");
    auto_declare<std::string>("base_x_joint_name", "base_x");
    auto_declare<std::string>("base_y_joint_name", "base_y");
    auto_declare<std::string>("base_yaw_joint_name", "base_yaw");
    auto_declare<std::string>("arm_joint_state_topic", "/joint_states");
    auto_declare<std::string>("whole_joint_state_topic", "/joint_states_whole");
    auto_declare<double>("idle_base_state_publish_period_sec", 0.02);
    auto_declare<double>("base_state_keepalive_duration_sec", 1.0);
    auto_declare<double>("joint_state_time_offset_sec", 0.02);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WholeBodyController::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    enable_base_wbc_ = get_node()->get_parameter("enable_base_wbc").as_bool();
    base_cmd_topic_ = get_node()->get_parameter("base_cmd_topic").as_string();
    base_cmd_msg_type_ = get_node()->get_parameter("base_cmd_msg_type").as_string();
    base_cmd_msg_type_ = normalize_type_string(base_cmd_msg_type_);
    forward_action_ns_ = get_node()->get_parameter("forward_action_ns").as_string();
    max_linear_vel_ = get_node()->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = get_node()->get_parameter("max_angular_vel").as_double();
    base_linear_sign_ = get_node()->get_parameter("base_linear_sign").as_double();
    base_angular_sign_ = get_node()->get_parameter("base_angular_sign").as_double();
    base_multi_dof_joint_name_ =
      get_node()->get_parameter("base_multi_dof_joint_name").as_string();
    base_x_joint_name_ = get_node()->get_parameter("base_x_joint_name").as_string();
    base_y_joint_name_ = get_node()->get_parameter("base_y_joint_name").as_string();
    base_yaw_joint_name_ = get_node()->get_parameter("base_yaw_joint_name").as_string();
    arm_joint_state_topic_ = get_node()->get_parameter("arm_joint_state_topic").as_string();
    whole_joint_state_topic_ = get_node()->get_parameter("whole_joint_state_topic").as_string();
    idle_base_state_publish_period_sec_ =
      get_node()->get_parameter("idle_base_state_publish_period_sec").as_double();
    base_state_keepalive_duration_sec_ =
      get_node()->get_parameter("base_state_keepalive_duration_sec").as_double();
    joint_state_time_offset_sec_ =
      get_node()->get_parameter("joint_state_time_offset_sec").as_double();
    if (idle_base_state_publish_period_sec_ < 0.0) {
      idle_base_state_publish_period_sec_ = 0.0;
    }
    if (base_state_keepalive_duration_sec_ < 0.0) {
      base_state_keepalive_duration_sec_ = 0.0;
    }
    if (joint_state_time_offset_sec_ < 0.0) {
      joint_state_time_offset_sec_ = 0.0;
    }

    if (enable_base_wbc_) {
      base_cmd_pub_.reset();
      base_cmd_stamped_pub_.reset();
      if (base_cmd_msg_type_ == "twist_stamped" || base_cmd_msg_type_ == "stamped") {
        base_cmd_stamped_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
          base_cmd_topic_, rclcpp::SystemDefaultsQoS());
      } else if (base_cmd_msg_type_ == "twist" || base_cmd_msg_type_ == "unstamped") {
        base_cmd_pub_ = get_node()->create_publisher<geometry_msgs::msg::Twist>(
          base_cmd_topic_, rclcpp::SystemDefaultsQoS());
      } else {
        try {
          base_cmd_stamped_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
            base_cmd_topic_, rclcpp::SystemDefaultsQoS());
          base_cmd_msg_type_ = "twist_stamped";
        } catch (const std::exception &) {
          base_cmd_pub_ = get_node()->create_publisher<geometry_msgs::msg::Twist>(
            base_cmd_topic_, rclcpp::SystemDefaultsQoS());
          base_cmd_msg_type_ = "twist";
        }
      }
    } else {
      base_cmd_pub_.reset();
      base_cmd_stamped_pub_.reset();
    }
    auto whole_joint_state_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    auto arm_joint_state_qos = rclcpp::SensorDataQoS();
    whole_joint_state_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      whole_joint_state_topic_, whole_joint_state_qos);
    arm_joint_state_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      arm_joint_state_topic_, arm_joint_state_qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg) {
          return;
        }
        {
          std::lock_guard<std::mutex> lk(arm_state_mutex_);
          arm_joint_state_msg_ = *msg;
          arm_joint_state_initialized_ = true;
        }
      });

    dual_arm_action_client_ = rclcpp_action::create_client<FollowJT>(
      get_node(), forward_action_ns_);

    // Use private namespace to avoid action name collisions in controller_manager process.
    action_server_ = rclcpp_action::create_server<FollowJT>(
      get_node(),
      "~/follow_joint_trajectory",
      std::bind(&WholeBodyController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WholeBodyController::handle_cancel, this, std::placeholders::_1),
      std::bind(&WholeBodyController::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Configured whole-body controller. enable_base_wbc=%s, base_cmd_topic='%s', base_cmd_msg_type='%s', forward_action_ns='%s'",
      enable_base_wbc_ ? "true" : "false",
      base_cmd_topic_.c_str(),
      base_cmd_msg_type_.c_str(),
      forward_action_ns_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to configure whole-body controller: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WholeBodyController::on_activate(const rclcpp_lifecycle::State &)
{
  active_ = true;
  {
    std::lock_guard<std::mutex> lk(base_state_mutex_);
    // Keep base joint state stream alive from startup so downstream TF/state
    // consumers always receive a consistent odom->base chain.
    base_state_initialized_ = true;
  }
  last_idle_base_state_publish_time_ = get_node()->now();
  base_state_keepalive_deadline_ = last_idle_base_state_publish_time_;
  if (idle_base_state_publish_period_sec_ > 0.0) {
    const auto period = std::chrono::duration<double>(idle_base_state_publish_period_sec_);
    idle_base_state_timer_ = get_node()->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        if (!active_ || !whole_joint_state_pub_) {
          return;
        }
        const auto now = get_node()->now();
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        bool initialized = false;
        {
          std::lock_guard<std::mutex> lk(base_state_mutex_);
          initialized = base_state_initialized_;
          x = base_state_x_;
          y = base_state_y_;
          yaw = base_state_yaw_;
        }
        if (initialized) {
          sensor_msgs::msg::JointState arm_js;
          bool arm_initialized = false;
          {
            std::lock_guard<std::mutex> lk(arm_state_mutex_);
            arm_initialized = arm_joint_state_initialized_;
            if (arm_initialized) {
              arm_js = arm_joint_state_msg_;
            }
          }
          sensor_msgs::msg::JointState js;
          js.header.stamp = now + rclcpp::Duration::from_seconds(joint_state_time_offset_sec_);
          js.header.frame_id.clear();
          if (!arm_initialized) {
            return;
          }
          const size_t n = std::min(arm_js.name.size(), arm_js.position.size());
          js.name.reserve(n + 3);
          js.position.reserve(n + 3);
          for (size_t i = 0; i < n; ++i) {
            const auto & name = arm_js.name[i];
            if (name == base_x_joint_name_ || name == base_y_joint_name_ || name == base_yaw_joint_name_) {
              continue;
            }
            js.name.push_back(name);
            js.position.push_back(arm_js.position[i]);
          }
          js.name.push_back(base_x_joint_name_);
          js.name.push_back(base_y_joint_name_);
          js.name.push_back(base_yaw_joint_name_);
          js.position.push_back(x);
          js.position.push_back(y);
          js.position.push_back(yaw);
          js.velocity.clear();
          js.effort.clear();
          whole_joint_state_pub_->publish(js);
        }
      });
  } else {
    idle_base_state_timer_.reset();
  }
  RCLCPP_INFO(get_node()->get_logger(), "Whole-body controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WholeBodyController::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  idle_base_state_timer_.reset();
  RCLCPP_INFO(get_node()->get_logger(), "Whole-body controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
WholeBodyController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse WholeBodyController::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const FollowJT::Goal> goal)
{
  if (!active_) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejecting goal: controller is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->trajectory.joint_names.empty() || goal->trajectory.points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejecting goal: empty joint_names or points");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WholeBodyController::handle_cancel(
  const std::shared_ptr<GoalHandleFollowJT>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WholeBodyController::handle_accepted(const std::shared_ptr<GoalHandleFollowJT> goal_handle)
{
  std::thread{std::bind(&WholeBodyController::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void WholeBodyController::execute(const std::shared_ptr<GoalHandleFollowJT> goal_handle)
{
  auto result = std::make_shared<FollowJT::Result>();
  const auto whole_goal = *(goal_handle->get_goal());
  bool has_base_joint = false;
  auto arm_goal = build_arm_goal(whole_goal, has_base_joint);
  const bool has_multi_dof_base =
    !whole_goal.multi_dof_trajectory.joint_names.empty() &&
    !whole_goal.multi_dof_trajectory.points.empty();
  const bool run_base_profile =
    enable_base_wbc_ && (has_base_joint || has_multi_dof_base) &&
    (static_cast<bool>(base_cmd_pub_) || static_cast<bool>(base_cmd_stamped_pub_));

  std::future<void> base_future;
  if (run_base_profile) {
    base_future = std::async(
      std::launch::async,
      [this, whole_goal, has_base_joint]() {
        if (has_base_joint) {
          publish_base_velocity_profile(whole_goal.trajectory);
        } else {
          publish_base_velocity_profile(whole_goal.multi_dof_trajectory);
        }
      });
  }

  if (arm_goal.trajectory.joint_names.empty()) {
    if (run_base_profile) {
      base_future.wait();
    }
    result->error_code = FollowJT::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    return;
  }

  if (!dual_arm_action_client_ || !dual_arm_action_client_->wait_for_action_server(2s)) {
    result->error_code = FollowJT::Result::INVALID_GOAL;
    result->error_string = "dual arm action server not available";
    if (run_base_profile) {
      base_future.wait();
    }
    goal_handle->abort(result);
    return;
  }

  auto forward_goal_future = dual_arm_action_client_->async_send_goal(arm_goal);
  if (forward_goal_future.wait_for(5s) != std::future_status::ready) {
    result->error_code = FollowJT::Result::INVALID_GOAL;
    result->error_string = "timeout sending goal to dual arm controller";
    if (run_base_profile) {
      base_future.wait();
    }
    goal_handle->abort(result);
    return;
  }

  auto forward_goal_handle = forward_goal_future.get();
  if (!forward_goal_handle) {
    result->error_code = FollowJT::Result::INVALID_GOAL;
    result->error_string = "dual arm controller rejected goal";
    if (run_base_profile) {
      base_future.wait();
    }
    goal_handle->abort(result);
    return;
  }

  bool cancel_sent = false;
  auto forward_result_future = dual_arm_action_client_->async_get_result(forward_goal_handle);
  while (forward_result_future.wait_for(50ms) != std::future_status::ready) {
    if (goal_handle->is_canceling() && !cancel_sent) {
      dual_arm_action_client_->async_cancel_goal(forward_goal_handle);
      cancel_sent = true;
    }
  }

  const auto wrapped_result = forward_result_future.get();
  if (run_base_profile) {
    base_future.wait();
  }

  if (wrapped_result.result) {
    result->error_code = wrapped_result.result->error_code;
    result->error_string = wrapped_result.result->error_string;
  }

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      goal_handle->succeed(result);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      goal_handle->canceled(result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
    default:
      goal_handle->abort(result);
      break;
  }
}

void WholeBodyController::publish_base_velocity_profile(
  const trajectory_msgs::msg::JointTrajectory & whole_traj)
{
  if ((!base_cmd_pub_ && !base_cmd_stamped_pub_) ||
      whole_traj.points.empty() ||
      whole_traj.joint_names.empty())
  {
    return;
  }

  auto find_index = [&](const std::string & name) {
    for (size_t i = 0; i < whole_traj.joint_names.size(); ++i) {
      if (whole_traj.joint_names[i] == name) {
        return i;
      }
    }
    return std::numeric_limits<size_t>::max();
  };

  const size_t idx_base_x = find_index(base_x_joint_name_);
  const size_t idx_base_y = find_index(base_y_joint_name_);
  const size_t idx_base_yaw = find_index(base_yaw_joint_name_);

  if (idx_base_x == std::numeric_limits<size_t>::max() &&
      idx_base_y == std::numeric_limits<size_t>::max() &&
      idx_base_yaw == std::numeric_limits<size_t>::max()) {
    return;
  }

  auto read_position = [](const trajectory_msgs::msg::JointTrajectoryPoint & p, size_t idx) {
    if (idx == std::numeric_limits<size_t>::max() || idx >= p.positions.size()) {
      return 0.0;
    }
    return p.positions[idx];
  };

  auto point_time = [](const trajectory_msgs::msg::JointTrajectoryPoint & p) {
    return static_cast<double>(p.time_from_start.sec) +
           static_cast<double>(p.time_from_start.nanosec) * 1e-9;
  };

  for (size_t i = 1; i < whole_traj.points.size(); ++i) {
    const auto & prev = whole_traj.points[i - 1];
    const auto & cur = whole_traj.points[i];

    const double dt = point_time(cur) - point_time(prev);
    if (dt <= 1e-3) {
      continue;
    }

    const double yaw_prev = read_position(prev, idx_base_yaw);
    const double yaw_cur = read_position(cur, idx_base_yaw);
    const double vx_world =
      (read_position(cur, idx_base_x) - read_position(prev, idx_base_x)) / dt;
    const double vy_world =
      (read_position(cur, idx_base_y) - read_position(prev, idx_base_y)) / dt;
    const double wz_world = normalize_angle(yaw_cur - yaw_prev) / dt;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(
      base_linear_sign_ * (std::cos(yaw_prev) * vx_world + std::sin(yaw_prev) * vy_world),
      -std::abs(max_linear_vel_),
      std::abs(max_linear_vel_));
    // diff_drive_controller is non-holonomic: ignore lateral command.
    cmd.linear.y = 0.0;
    cmd.angular.z = std::clamp(
      base_angular_sign_ * wz_world,
      -std::abs(max_angular_vel_),
      std::abs(max_angular_vel_));

    const auto end_tp =
      std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(dt));
    const double x_prev = read_position(prev, idx_base_x);
    const double y_prev = read_position(prev, idx_base_y);
    const double yaw_prev_abs = read_position(prev, idx_base_yaw);
    const double x_cur = read_position(cur, idx_base_x);
    const double y_cur = read_position(cur, idx_base_y);
    const double yaw_cur_abs = read_position(cur, idx_base_yaw);
    publish_base_joint_state(x_prev, y_prev, yaw_prev_abs);
    while (std::chrono::steady_clock::now() < end_tp) {
      if (base_cmd_stamped_pub_) {
        geometry_msgs::msg::TwistStamped stamped;
        stamped.header.stamp = get_node()->now();
        stamped.twist = cmd;
        base_cmd_stamped_pub_->publish(stamped);
      } else if (base_cmd_pub_) {
        base_cmd_pub_->publish(cmd);
      }
      std::this_thread::sleep_for(50ms);
    }
    publish_base_joint_state(x_cur, y_cur, yaw_cur_abs);
  }

  geometry_msgs::msg::Twist stop;
  if (!whole_traj.points.empty()) {
    const auto & last = whole_traj.points.back();
    const auto read_position_last = [](const trajectory_msgs::msg::JointTrajectoryPoint & p, size_t idx) {
      if (idx == std::numeric_limits<size_t>::max() || idx >= p.positions.size()) {
        return 0.0;
      }
      return p.positions[idx];
    };
    publish_base_joint_state(
      read_position_last(last, idx_base_x),
      read_position_last(last, idx_base_y),
      read_position_last(last, idx_base_yaw));
  }
  if (base_cmd_stamped_pub_) {
    geometry_msgs::msg::TwistStamped stamped_stop;
    stamped_stop.header.stamp = get_node()->now();
    stamped_stop.twist = stop;
    base_cmd_stamped_pub_->publish(stamped_stop);
  } else if (base_cmd_pub_) {
    base_cmd_pub_->publish(stop);
  }
}

void WholeBodyController::publish_base_velocity_profile(
  const trajectory_msgs::msg::MultiDOFJointTrajectory & whole_traj)
{
  if ((!base_cmd_pub_ && !base_cmd_stamped_pub_) ||
      whole_traj.points.empty() ||
      whole_traj.joint_names.empty())
  {
    return;
  }

  size_t base_joint_idx = std::numeric_limits<size_t>::max();
  if (!base_multi_dof_joint_name_.empty()) {
    for (size_t i = 0; i < whole_traj.joint_names.size(); ++i) {
      if (whole_traj.joint_names[i] == base_multi_dof_joint_name_) {
        base_joint_idx = i;
        break;
      }
    }
  }
  if (base_joint_idx == std::numeric_limits<size_t>::max()) {
    base_joint_idx = 0;
  }

  auto point_time = [](const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint & p) {
    return static_cast<double>(p.time_from_start.sec) +
           static_cast<double>(p.time_from_start.nanosec) * 1e-9;
  };

  for (size_t i = 1; i < whole_traj.points.size(); ++i) {
    const auto & prev = whole_traj.points[i - 1];
    const auto & cur = whole_traj.points[i];
    if (base_joint_idx >= prev.transforms.size() || base_joint_idx >= cur.transforms.size()) {
      continue;
    }

    const double dt = point_time(cur) - point_time(prev);
    if (dt <= 1e-3) {
      continue;
    }

    const auto & prev_tf = prev.transforms[base_joint_idx];
    const auto & cur_tf = cur.transforms[base_joint_idx];
    const double yaw_prev = yaw_from_quaternion(prev_tf.rotation);
    const double yaw_cur = yaw_from_quaternion(cur_tf.rotation);

    const double vx_world = (cur_tf.translation.x - prev_tf.translation.x) / dt;
    const double vy_world = (cur_tf.translation.y - prev_tf.translation.y) / dt;
    const double wz = normalize_angle(yaw_cur - yaw_prev) / dt;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(
      base_linear_sign_ * (std::cos(yaw_prev) * vx_world + std::sin(yaw_prev) * vy_world),
      -std::abs(max_linear_vel_), std::abs(max_linear_vel_));
    // diff_drive_controller is non-holonomic: ignore lateral command.
    cmd.linear.y = 0.0;
    cmd.angular.z = std::clamp(
      base_angular_sign_ * wz, -std::abs(max_angular_vel_), std::abs(max_angular_vel_));

    const auto end_tp =
      std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(dt));
    publish_base_joint_state(prev_tf.translation.x, prev_tf.translation.y, yaw_prev);
    while (std::chrono::steady_clock::now() < end_tp) {
      if (base_cmd_stamped_pub_) {
        geometry_msgs::msg::TwistStamped stamped;
        stamped.header.stamp = get_node()->now();
        stamped.twist = cmd;
        base_cmd_stamped_pub_->publish(stamped);
      } else if (base_cmd_pub_) {
        base_cmd_pub_->publish(cmd);
      }
      std::this_thread::sleep_for(50ms);
    }
    publish_base_joint_state(cur_tf.translation.x, cur_tf.translation.y, yaw_cur);
  }

  geometry_msgs::msg::Twist stop;
  if (!whole_traj.points.empty() &&
    base_joint_idx < whole_traj.points.back().transforms.size())
  {
    const auto & tf = whole_traj.points.back().transforms[base_joint_idx];
    publish_base_joint_state(tf.translation.x, tf.translation.y, yaw_from_quaternion(tf.rotation));
  }
  if (base_cmd_stamped_pub_) {
    geometry_msgs::msg::TwistStamped stamped_stop;
    stamped_stop.header.stamp = get_node()->now();
    stamped_stop.twist = stop;
    base_cmd_stamped_pub_->publish(stamped_stop);
  } else if (base_cmd_pub_) {
    base_cmd_pub_->publish(stop);
  }
}

WholeBodyController::FollowJT::Goal WholeBodyController::build_arm_goal(
  const FollowJT::Goal & whole_goal,
  bool & has_base_joint) const
{
  FollowJT::Goal arm_goal;
  has_base_joint = false;

  const auto & whole_traj = whole_goal.trajectory;
  if (whole_traj.joint_names.empty() || whole_traj.points.empty()) {
    return arm_goal;
  }

  std::vector<size_t> arm_indices;
  arm_indices.reserve(whole_traj.joint_names.size());
  for (size_t i = 0; i < whole_traj.joint_names.size(); ++i) {
    const auto & jn = whole_traj.joint_names[i];
    const bool is_base_joint =
      (jn == base_x_joint_name_ || jn == base_y_joint_name_ || jn == base_yaw_joint_name_);
    if (is_base_joint) {
      has_base_joint = true;
      continue;
    }
    arm_indices.push_back(i);
    arm_goal.trajectory.joint_names.push_back(jn);
  }

  arm_goal.trajectory.header = whole_traj.header;
  arm_goal.trajectory.points.reserve(whole_traj.points.size());
  for (const auto & src : whole_traj.points) {
    trajectory_msgs::msg::JointTrajectoryPoint dst;
    dst.time_from_start = src.time_from_start;
    dst.positions.reserve(arm_indices.size());
    dst.velocities.reserve(arm_indices.size());
    dst.accelerations.reserve(arm_indices.size());
    dst.effort.reserve(arm_indices.size());

    for (const auto idx : arm_indices) {
      if (idx < src.positions.size()) {
        dst.positions.push_back(src.positions[idx]);
      }
      if (idx < src.velocities.size()) {
        dst.velocities.push_back(src.velocities[idx]);
      }
      if (idx < src.accelerations.size()) {
        dst.accelerations.push_back(src.accelerations[idx]);
      }
      if (idx < src.effort.size()) {
        dst.effort.push_back(src.effort[idx]);
      }
    }
    arm_goal.trajectory.points.push_back(std::move(dst));
  }

  arm_goal.path_tolerance = whole_goal.path_tolerance;
  arm_goal.goal_tolerance = whole_goal.goal_tolerance;
  arm_goal.goal_time_tolerance = whole_goal.goal_time_tolerance;
  return arm_goal;
}

void WholeBodyController::publish_base_joint_state(double x, double y, double yaw)
{
  if (!whole_joint_state_pub_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(base_state_mutex_);
    base_state_x_ = x;
    base_state_y_ = y;
    base_state_yaw_ = yaw;
    base_state_initialized_ = true;
  }
  base_state_keepalive_deadline_ =
    get_node()->now() + rclcpp::Duration::from_seconds(base_state_keepalive_duration_sec_);
  sensor_msgs::msg::JointState js;
  js.header.stamp = get_node()->now() + rclcpp::Duration::from_seconds(joint_state_time_offset_sec_);
  js.header.frame_id.clear();
  {
    std::lock_guard<std::mutex> lk(arm_state_mutex_);
    if (!arm_joint_state_initialized_) {
      return;
    }
    const size_t n = std::min(arm_joint_state_msg_.name.size(), arm_joint_state_msg_.position.size());
    js.name.reserve(n + 3);
    js.position.reserve(n + 3);
    for (size_t i = 0; i < n; ++i) {
      const auto & name = arm_joint_state_msg_.name[i];
      if (name == base_x_joint_name_ || name == base_y_joint_name_ || name == base_yaw_joint_name_) {
        continue;
      }
      js.name.push_back(name);
      js.position.push_back(arm_joint_state_msg_.position[i]);
    }
  }
  js.name.push_back(base_x_joint_name_);
  js.name.push_back(base_y_joint_name_);
  js.name.push_back(base_yaw_joint_name_);
  js.position.push_back(x);
  js.position.push_back(y);
  js.position.push_back(yaw);
  js.velocity.clear();
  js.effort.clear();
  whole_joint_state_pub_->publish(js);
}

}  // namespace dsr_mobile_wbc2

PLUGINLIB_EXPORT_CLASS(
  dsr_mobile_wbc2::WholeBodyController,
  controller_interface::ControllerInterface)
