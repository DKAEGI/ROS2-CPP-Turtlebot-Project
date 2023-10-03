#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "my_ros2ject/srv/find_wall.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "my_ros2ject/action/odom_record.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "my_ros2ject/action/detail/odom_record__struct.hpp"
#include "std_msgs/msg/float32.hpp"

#include "my_ros2ject/odometry_action_client.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
using FindWall = my_ros2ject::srv::FindWall;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono;      // nanoseconds, system_clock, seconds

// SIMULATION
const int LASER_DATA_SIZE = 360;
const int LASER_FRONT = 0;
const int LASER_RIGHT = 270;
const int LASER_LEFT = 90;
const int LASER_BACK = 180;
const int SIGN = -1; // Direction of Laserbeam
const int THRESHOLD = 10;

// REAL ROBOT
// const int LASER_DATA_SIZE = 720;
// const int LASER_FRONT = 360;
// const int LASER_RIGHT = 180;
// const int LASER_LEFT = 540;
// const int LASER_BACK = 0;
// const int SIGN = 1; // Direction of Laserbeam
// const int THRESHOLD = 10;

class Follow_Wall : public rclcpp::Node {
public:
  Follow_Wall() : Node("follow_wall") {
    // Create subscriber for laserscan
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Follow_Wall::laserscan_callback, this, _1));

    // Create subscriber to check the current action feedback
    subscriber_fb = this->create_subscription<std_msgs::msg::Float32>(
        "current_total", 10, std::bind(&Follow_Wall::action_fb_callback, this, _1));

    // Create a publisher to the "cmd_vel" topic with a queue size of 10
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create a timer to publish the velocity
    timer_ = this->create_wall_timer(
        500ms, std::bind(&Follow_Wall::timer_callback, this));

    // Initialize laserdata
    for (int i = 0; i < LASER_DATA_SIZE; i++) {
      laserdata_[i] = 10;
    }
    right_laser = 100.0;
    front_laser = 100.0;
    
    // Start action fb info
    fb_info = 0.0;
    
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store the laser data in the laserdata_ array
    std::copy(msg->ranges.begin(), msg->ranges.end(), laserdata_.begin());

    // Loop through the ranges of right laser
    right_laser = 100.0;
    for (int i = LASER_RIGHT - THRESHOLD; i < LASER_RIGHT + THRESHOLD; ++i) {
      right_laser = std::min(right_laser, static_cast<double>(msg->ranges[i]));
    }

    // Loop through the ranges of front laser
    front_laser = 100.0;
    for (int i = LASER_FRONT; i < LASER_FRONT + THRESHOLD; ++i) {
      front_laser = std::min(front_laser, static_cast<double>(msg->ranges[i]));
    }
  }

  void action_fb_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    // Get the data from the action feedback
    fb_info = msg->data;
  }

  void timer_callback() {
    // Constants for the P-controller
    double kp = 1.0;

    // Desired distance from the right laserscan
    double desiredDistance = 0.25;

    // Calculate the error (the difference between the desired and current
    // distances)
    double distanceError = desiredDistance - right_laser;

    // Proportional control for angular velocity
    double angularP = kp * distanceError;

    // Set velocity for following the wall
    vel_.linear.x = 0.1;
    vel_.angular.z = angularP;
    if (front_laser < 0.5) {
		vel_.linear.x = 0.1;
		vel_.angular.z = 0.4;
    } else if (fb_info > 5.5) {
        // Action finished --> Stop robot and finish node
        vel_.linear.x = 0.0;
		vel_.angular.z = 0.0;
        publisher_->publish(vel_);
        sleep_for(1s);
        rclcpp::shutdown();
    }

    // RCLCPP_INFO(this->get_logger(), "[FRONT] = '%lf'", front_laser);
    // RCLCPP_INFO(this->get_logger(), "[RIGHT] = '%lf'", right_laser);
    // RCLCPP_INFO(this->get_logger(), "[LINEAR SPEED] = '%lf'", vel_.linear.x);
    // RCLCPP_INFO(this->get_logger(), "[ANGULAR SPEED] = '%lf'", vel_.angular.z);

    publisher_->publish(vel_);

    // RCLCPP_INFO(this->get_logger(), "Following the wall");
  }

  // Declare the private member variables
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist vel_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_fb;
  float fb_info;
  // Create laserdata_ float array
  std::array<float, LASER_DATA_SIZE> laserdata_;
  double right_laser;
  double front_laser;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Call Service to find the nearest Wall
  std::shared_ptr<rclcpp::Node> node_client =
      rclcpp::Node::make_shared("service_client");
  rclcpp::Client<FindWall>::SharedPtr client =
      node_client->create_client<FindWall>("find_wall");

  auto request = std::make_shared<FindWall::Request>();

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result_future = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node_client, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->wallfound == true) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned success");
    } else if (result->wallfound == false) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned false");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service /find_wall");
  }
  // Service finished

  // Create an instance of the FollowWallNode class
  auto follow_wall_node = std::make_shared<Follow_Wall>();

  // Create an instance of the OdometryActionClientNode class
  auto odometry_action_client_node = std::make_shared<OdometryActionClient>();

  // Create MultiThreadedExecutor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(follow_wall_node);
  executor.add_node(odometry_action_client_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}