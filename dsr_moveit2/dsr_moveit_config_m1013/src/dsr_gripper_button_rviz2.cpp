#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>

#include <QPushButton>
#include <QHBoxLayout>
#include <QTimer>

#include <pluginlib/class_list_macros.hpp>

namespace dsr_moveit_config_m1013
{

class GripperControllerPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

  explicit GripperControllerPanel(QWidget* parent = nullptr)
    : rviz_common::Panel(parent)
  {
    auto* layout = new QHBoxLayout(this);
    open_button_ = new QPushButton("Open", this);
    close_button_ = new QPushButton("Close", this);
    layout->addWidget(open_button_);
    layout->addWidget(close_button_);
    setLayout(layout);

    // Initially disable buttons until action server is found
    open_button_->setEnabled(false);
    close_button_->setEnabled(false);

    connect(open_button_, SIGNAL(clicked()), this, SLOT(onOpen()));
    connect(close_button_, SIGNAL(clicked()), this, SLOT(onClose()));
  }

  void onInitialize() override
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    action_client_ = rclcpp_action::create_client<GripperCommand>(
      node_, "/gripper_position_controller/gripper_cmd");

    // Use a timer to periodically check for the action server
    server_check_timer_ = new QTimer(this);
    connect(server_check_timer_, &QTimer::timeout, [this]() {
      if (action_client_->action_server_is_ready()) {
        open_button_->setEnabled(true);
        close_button_->setEnabled(true);
        open_button_->setText("Open");
        close_button_->setText("Close");
        server_check_timer_->stop();
      } else {
        open_button_->setText("Waiting...");
        close_button_->setText("Waiting...");
      }
    });
    server_check_timer_->start(500);
  }

private Q_SLOTS:
  void onOpen()
  {
    send_goal(0.0);
  }

  void onClose()
  {
    send_goal(0.8);
  }

private:
  void send_goal(double position)
  {
    if (!action_client_ || !action_client_->action_server_is_ready()) {
      RCLCPP_ERROR(node_->get_logger(), "Action client not available.");
      return;
    }

    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = 50.0;

    RCLCPP_INFO(node_->get_logger(), "Sending gripper goal: %f", position);
    action_client_->async_send_goal(goal_msg);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<GripperCommand>::SharedPtr action_client_;
  
  QPushButton* open_button_{nullptr};
  QPushButton* close_button_{nullptr};
  QTimer* server_check_timer_{nullptr};
};

}

PLUGINLIB_EXPORT_CLASS(dsr_moveit_config_m1013::GripperControllerPanel, rviz_common::Panel)

#include "dsr_gripper_button_rviz2.moc"