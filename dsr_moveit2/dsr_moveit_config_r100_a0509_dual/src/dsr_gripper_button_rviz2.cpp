#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>

#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QTimer>

#include <pluginlib/class_list_macros.hpp>

namespace dsr_moveit_config_r100_a0509_dual
{

class GripperControllerPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  // ros2_controllers 의 parallel_gripper_action_controller 는 ParallelGripperCommand 액션을 제공한다
  using GripperCommand = control_msgs::action::ParallelGripperCommand;
  using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

  explicit GripperControllerPanel(QWidget* parent = nullptr)
    : rviz_common::Panel(parent)
  {
    auto* vlayout = new QVBoxLayout(this);

    auto* name_row = new QHBoxLayout();
    name_row->addWidget(new QLabel("Action Name:", this));
    action_name_edit_ = new QLineEdit(this);
    action_name_edit_->setPlaceholderText("(auto-scan)");
    name_row->addWidget(action_name_edit_);
    vlayout->addLayout(name_row);

    auto* btn_row = new QHBoxLayout();
    open_button_ = new QPushButton("Open", this);
    close_button_ = new QPushButton("Close", this);
    btn_row->addWidget(open_button_);
    btn_row->addWidget(close_button_);
    vlayout->addLayout(btn_row);

    setLayout(vlayout);

    // Initially disable buttons until action server is found
    open_button_->setEnabled(false);
    close_button_->setEnabled(false);

    connect(open_button_, &QPushButton::clicked, this, &GripperControllerPanel::onOpen);
    connect(close_button_, &QPushButton::clicked, this, &GripperControllerPanel::onClose);
    connect(action_name_edit_, &QLineEdit::editingFinished,
      this, &GripperControllerPanel::onActionNameChanged);
  }

  void onInitialize() override
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    server_check_timer_ = new QTimer(this);
    connect(server_check_timer_, &QTimer::timeout, this, &GripperControllerPanel::checkActionServer);
    server_check_timer_->start(500);
  }

  void load(const rviz_common::Config & config) override
  {
    rviz_common::Panel::load(config);
    QString val;
    if (config.mapGetString("Action Name", &val)) {
      action_name_edit_->setText(val);
      action_name_ = val;
    }
  }

  void save(rviz_common::Config config) const override
  {
    rviz_common::Panel::save(config);
    config.mapSetValue("Action Name", action_name_edit_->text());
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

  void onActionNameChanged()
  {
    action_client_.reset();
    action_name_ = action_name_edit_->text();
    open_button_->setEnabled(false);
    close_button_->setEnabled(false);
    if (!server_check_timer_->isActive()) {
      server_check_timer_->start(500);
    }
  }

  void checkActionServer()
  {
    if (!action_client_) {
      const std::string configured = action_name_.toStdString();
      if (!configured.empty()) {
        // Action Name 이 설정된 경우 정확히 그 이름으로 바인딩한다
        try {
          action_client_ = rclcpp_action::create_client<GripperCommand>(node_, configured);
        } catch (const std::exception & e) {
          // 잘못된 이름은 rclcpp 가 예외로 거부하므로 RViz 를 종료시키지 않고 패널에 표시한다
          RCLCPP_ERROR(node_->get_logger(), "Invalid gripper Action Name '%s': %s", configured.c_str(), e.what());
          open_button_->setText("Invalid name");
          close_button_->setText("Invalid name");
          server_check_timer_->stop();
          return;
        }
        RCLCPP_INFO(node_->get_logger(), "Gripper action bound: %s", configured.c_str());
      } else {
        // 액션은 <이름>/_action/status 토픽을 만들므로 토픽 목록에서 정확한 접미사로 찾는다
        const std::string suffix = "/gripper_position_controller/gripper_cmd/_action/status";
        const std::string ctrl_path = "/gripper_position_controller/gripper_cmd";
        const std::string status_tail = "/_action/status";
        const std::string node_ns = node_->get_namespace();

        std::vector<std::string> candidates;
        for (const auto & entry : node_->get_topic_names_and_types()) {
          const std::string & topic = entry.first;
          if (topic.size() >= suffix.size() &&
              topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0) {
            candidates.push_back(topic.substr(0, topic.size() - status_tail.size()));
          }
        }

        if (candidates.size() == 1) {
          action_client_ = rclcpp_action::create_client<GripperCommand>(node_, candidates[0]);
          RCLCPP_INFO(node_->get_logger(), "Gripper action found: %s", candidates[0].c_str());
        } else if (candidates.size() > 1) {
          // 네임스페이스가 노드 네임스페이스와 일치하는 액션을 선택한다
          for (const auto & c : candidates) {
            if (c.size() > ctrl_path.size() &&
                c.compare(c.size() - ctrl_path.size(), ctrl_path.size(), ctrl_path) == 0 &&
                c.substr(0, c.size() - ctrl_path.size()) == node_ns) {
              action_client_ = rclcpp_action::create_client<GripperCommand>(node_, c);
              RCLCPP_INFO(node_->get_logger(), "Gripper action found: %s", c.c_str());
              break;
            }
          }
          if (!action_client_) {
            RCLCPP_WARN(node_->get_logger(),
              "Multiple gripper actions found; none matches node namespace '%s'. "
              "Set 'Action Name' in panel config to select one.",
              node_ns.c_str());
          }
        }
      }
    }

    if (action_client_ && action_client_->action_server_is_ready()) {
      open_button_->setEnabled(true);
      close_button_->setEnabled(true);
      open_button_->setText("Open");
      close_button_->setText("Close");
      server_check_timer_->stop();
    } else {
      open_button_->setText("Waiting...");
      close_button_->setText("Waiting...");
    }
  }

private:
  void send_goal(double position)
  {
    if (!action_client_ || !action_client_->action_server_is_ready()) {
      RCLCPP_ERROR(node_->get_logger(), "Action client not available.");
      return;
    }

    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = {position};

    rclcpp_action::Client<GripperCommand>::SendGoalOptions opts;
    opts.goal_response_callback =
      [this](const GoalHandleGripperCommand::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(node_->get_logger(), "Gripper goal rejected by server.");
        }
      };
    opts.result_callback =
      [this](const GoalHandleGripperCommand::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(),
              "Gripper SUCCEEDED: reached_goal=%s stalled=%s",
              result.result->reached_goal ? "true" : "false",
              result.result->stalled ? "true" : "false");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(node_->get_logger(),
              "Gripper ABORTED: reached_goal=%s stalled=%s",
              result.result->reached_goal ? "true" : "false",
              result.result->stalled ? "true" : "false");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(),
              "Gripper CANCELED: reached_goal=%s stalled=%s",
              result.result->reached_goal ? "true" : "false",
              result.result->stalled ? "true" : "false");
            break;
          default:
            RCLCPP_WARN(node_->get_logger(), "Gripper goal ended with unknown result code.");
            break;
        }
      };

    RCLCPP_INFO(node_->get_logger(), "Sending gripper goal: %f", position);
    action_client_->async_send_goal(goal_msg, opts);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<GripperCommand>::SharedPtr action_client_;

  QPushButton* open_button_{nullptr};
  QPushButton* close_button_{nullptr};
  QLineEdit* action_name_edit_{nullptr};
  QTimer* server_check_timer_{nullptr};
  QString action_name_;
};

}  // namespace dsr_moveit_config_r100_a0509_dual

PLUGINLIB_EXPORT_CLASS(dsr_moveit_config_r100_a0509_dual::GripperControllerPanel, rviz_common::Panel)

#include "dsr_gripper_button_rviz2.moc"
