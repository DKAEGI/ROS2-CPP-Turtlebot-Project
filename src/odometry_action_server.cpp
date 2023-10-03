#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "my_ros2ject/action/odom_record.hpp"

class Odometry_Action_Server : public rclcpp::Node
{
public:
  using OdomRecord = my_ros2ject::action::OdomRecord;
  using GoalHandleOdomRecord = rclcpp_action::ServerGoalHandle<OdomRecord>;

  explicit Odometry_Action_Server(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("odom_record_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<OdomRecord>(
      this,
      "record_odom",
      std::bind(&Odometry_Action_Server::handle_goal, this, _1),
      std::bind(&Odometry_Action_Server::handle_cancel, this, _1),
      std::bind(&Odometry_Action_Server::handle_accepted, this, _1));

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&Odometry_Action_Server::odom_callback, this, _1));

    publisher_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    publisher_fb = this->create_publisher<std_msgs::msg::Float32>("current_total", 10);

  }

private:
  rclcpp_action::Server<OdomRecord>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_fb;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_result;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  geometry_msgs::msg::Twist vel_;
  std_msgs::msg::Float32 fb_info;
  float start_pos_x = 0;
  float start_pos_y = 0;
  float current_pos_x = 0;
  float current_pos_y = 0;
  float current_rot_z = 0;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->current_pos_x = msg->pose.pose.position.x;
    this->current_pos_y = msg->pose.pose.position.y;
    this->current_rot_z = msg->pose.pose.orientation.z;
  }

  // Goal response
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  // Cancel response
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleOdomRecord> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleOdomRecord> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Odometry_Action_Server::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleOdomRecord> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<OdomRecord::Feedback>();
    auto result = std::make_shared<OdomRecord::Result>();
    std::vector<geometry_msgs::msg::Point32> total_list_of_odoms; // Create a vector to hold Point32 messages
    geometry_msgs::msg::Point32 current_list_of_odoms;
    rclcpp::Rate loop_rate(1);
    float distance_travelled = 0;
    this->start_pos_x = this->current_pos_x;
    this->start_pos_y = this->current_pos_y;

    while (distance_travelled < 5.5 && rclcpp::ok()) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->list_of_odoms = total_list_of_odoms;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // add x, y, rot z to list_of_odoms
      current_list_of_odoms.x = current_pos_x;
      current_list_of_odoms.y = current_pos_y;
      current_list_of_odoms.z = current_rot_z;
      total_list_of_odoms.push_back(current_list_of_odoms);

      // get distance traveled and send feedback
      distance_travelled += get_distance_travelled();
      feedback->current_total = distance_travelled;
      this->start_pos_x = this->current_pos_x;
      this->start_pos_y = this->current_pos_y;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      // publish feedback to current_total topic
      fb_info.data = distance_travelled;
      publisher_fb->publish(fb_info);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->list_of_odoms = total_list_of_odoms;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  // Calculate traveled distance in 1s
  float get_distance_travelled() {
    float distance_travelled =
        sqrt(pow(this->current_pos_x - this->start_pos_x, 2) +
             pow(this->current_pos_y - this->start_pos_y, 2));
    return distance_travelled;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<Odometry_Action_Server>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
