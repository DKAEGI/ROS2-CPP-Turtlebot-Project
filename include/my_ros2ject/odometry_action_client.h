#ifndef ODOMETRY_ACTION_CLIENT_H
#define ODOMETRY_ACTION_CLIENT_H

#include "my_ros2ject/odometry_action_client.h"
#include <functional>
#include <memory>
#include <thread>

#include "my_ros2ject/action/detail/odom_record__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_ros2ject/action/odom_record.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

class OdometryActionClient : public rclcpp::Node {
public:
  using OdomRecord = my_ros2ject::action::OdomRecord;
  using GoalHandleOdomRecord = rclcpp_action::ClientGoalHandle<OdomRecord>;

  explicit OdometryActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("odometry_action_client", node_options), goal_done_(false) {

    this->client_ptr_ = rclcpp_action::create_client<OdomRecord>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "record_odom");

    this->action_client_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&OdometryActionClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    this->action_client_timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = OdomRecord::Goal();

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<OdomRecord>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&OdometryActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&OdometryActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&OdometryActionClient::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<OdomRecord>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr action_client_timer_;
  bool goal_done_;

  void goal_response_callback(
      std::shared_future<GoalHandleOdomRecord::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(),
                   "Goal was rejected by the action server");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server");
  }

  void feedback_callback(
      GoalHandleOdomRecord::SharedPtr,
      const std::shared_ptr<const OdomRecord::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(),
                "Feedback received: Total current distance = %f",
                feedback->current_total);
  }

  void result_callback(const GoalHandleOdomRecord::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "OdomRecord received:");
    for (const auto &point : result.result->list_of_odoms) {
      RCLCPP_INFO(this->get_logger(), "Point: (%f, %f, %f)", point.x, point.y,
                  point.z);
    }
  }
}; // class OdometryActionClient


#endif // ODOMETRY_ACTION_CLIENT_H
