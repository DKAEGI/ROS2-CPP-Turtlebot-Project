#include "geometry_msgs/msg/twist.hpp"
#include "my_ros2ject/srv/find_wall.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

using FindWall = my_ros2ject::srv::FindWall;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono;      // nanoseconds, system_clock, seconds

// SIMULATION
const int LASER_DATA_SIZE = 360; 
const int LASER_FRONT = 0;       
const int LASER_RIGHT = 270;    
const int LASER_LEFT = 90;       
const int LASER_BACK = 180;     
const int SIGN = -1; // Direction of Laserbeam

// REAL ROBOT
// const int LASER_DATA_SIZE = 720; 
// const int LASER_FRONT = 360;      
// const int LASER_RIGHT = 180;  
// const int LASER_LEFT = 540;       
// const int LASER_BACK = 0;      
// const int SIGN = 1; // Direction of Laserbeam

class Find_Wall : public rclcpp::Node {
public:
  Find_Wall() : Node("find_wall") {
    // Initialize callback groups
    subscriber_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    service_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create subscription using the callback group
    auto subscriber_qos =
        rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = subscriber_cb_group_;
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", subscriber_qos,
        std::bind(&Find_Wall::laserscan_callback, this, _1),
        subscription_options);

    // Create a publisher to the "cmd_vel" topic with a queue size of 10
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create a service named "find_wall" with a callback function
    srv_ = this->create_service<FindWall>(
        "find_wall", std::bind(&Find_Wall::moving_callback, this, _1, _2),
        rmw_qos_profile_services_default, service_cb_group_);

    // Initialize laserdata
    for (int i = 0; i < LASER_DATA_SIZE; i++) {
      laserdata_[i] = 10;
    }
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // laserscan callback triggered
    // RCLCPP_INFO(this->get_logger(),
    //             "Laserscan callback triggered. Number of ranges: %zu",
    //             msg->ranges.size());

    // // Log the front laser value
    // RCLCPP_INFO(this->get_logger(), "Front Laser Value is : '%f'",
    //             msg->ranges[LASER_FRONT]);

    // Store the laser data in the laserdata_ array
    std::copy(msg->ranges.begin(), msg->ranges.end(), laserdata_.begin());
  }

  // Declare member variables
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Service<FindWall>::SharedPtr srv_;
  geometry_msgs::msg::Twist vel_;
  rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  // Create laserdata_ float array
  std::array<float, LASER_DATA_SIZE> laserdata_;

  void moving_callback(const std::shared_ptr<FindWall::Request> request,
                       const std::shared_ptr<FindWall::Response> response) {

    RCLCPP_INFO(this->get_logger(), "server_callback started with request",
                request.get());

    while (laserdata_[0] == 10) { // Wait until laserdata gets realistic values
      RCLCPP_INFO(this->get_logger(), "laser data not ready");
      sleep_for(1s);
    }

    // Stop the robot
    vel_.linear.x = 0;
    vel_.angular.z = 0;
    publisher_->publish(vel_);
    sleep_for(1s);
    // Log Laser values
    RCLCPP_INFO(this->get_logger(), "Front Laser Value is : '%f'",
                laserdata_[LASER_FRONT]);
    RCLCPP_INFO(this->get_logger(), "Right Laser Value is : '%f'",
                laserdata_[LASER_RIGHT]);
    RCLCPP_INFO(this->get_logger(), "Left Laser Value is : '%f'",
                laserdata_[LASER_LEFT]);
    RCLCPP_INFO(this->get_logger(), "Back Laser Value is : '%f'",
                laserdata_[LASER_BACK]);

    // Initialize values
    double min_laser = 100000;
    int min_index = 0;

    // Find the minimum laser value and its index
    for (int i = 0; i < LASER_DATA_SIZE; i++) {
      if (laserdata_[i] < min_laser) {
        min_laser = laserdata_[i];
        min_index = i;
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "Smallest laserbeam is : '%d' ; and has the value: '%lf'",
                min_index, min_laser);

    // Calculate the desired angle (in degrees) for the front value to align
    // with the minimum value
    double desired_angle_deg =
        SIGN * (LASER_FRONT - min_index) * 360 / LASER_DATA_SIZE;

    // Calculate the time needed for rotation to achieve the desired angle
    double angular_vel = 0.2;
    double rot_angle_rad = desired_angle_deg * 0.01745; // Convert to radians
    double rot_time = rot_angle_rad / angular_vel;
    // rotate left or right
    if (rot_angle_rad < 0) {
      angular_vel = -angular_vel;
      rot_time = -rot_time;
    }
    RCLCPP_INFO(this->get_logger(), "rot time is : '%f'", rot_time);
    RCLCPP_INFO(this->get_logger(), "desired angle is : '%f'",
                desired_angle_deg);
    RCLCPP_INFO(this->get_logger(), "rot angle rad is : '%f'", rot_angle_rad);
    RCLCPP_INFO(this->get_logger(), "angular vel is : '%f'", angular_vel);

    // Rotate the robot
    vel_.angular.z = angular_vel;
    publisher_->publish(vel_);
    sleep_for(milliseconds(static_cast<int>(rot_time * 1000)));
    vel_.angular.z = 0;
    publisher_->publish(vel_);
    RCLCPP_INFO(this->get_logger(), "robot has rotated to the wall");
    RCLCPP_INFO(this->get_logger(), "Move to the wall");

    // Go to the wall
    while (laserdata_[LASER_FRONT] > 0.3) {
      RCLCPP_INFO(this->get_logger(), "Front Laser Value is : '%f'",
                  laserdata_[LASER_FRONT]);
      vel_.linear.x = 0.1;
      publisher_->publish(vel_);
      sleep_for(500ms);
    }

    // Rotate to the left
    vel_.linear.x = 0;
    publisher_->publish(vel_);
    sleep_for(1s);
    rot_angle_rad = 100 * 0.01745; // Convert to radians, turn 100 degrees
    rot_time = rot_angle_rad / abs(angular_vel);
    vel_.angular.z = abs(angular_vel);
    RCLCPP_INFO(this->get_logger(), "rot time is : '%f'", rot_time);
    RCLCPP_INFO(this->get_logger(), "rot angle rad is : '%f'", rot_angle_rad);
    publisher_->publish(vel_);
    sleep_for(milliseconds(static_cast<int>(rot_time * 1000)));
    vel_.angular.z = 0;
    publisher_->publish(vel_);
    RCLCPP_INFO(this->get_logger(), "robot has rotated to the left");
    sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "Right Laser Value is : '%f'",
                laserdata_[LASER_RIGHT]);

    if (laserdata_[LASER_RIGHT] < 0.35 && laserdata_[LASER_RIGHT] > 0.2) {
      // Set the response success variable to true
      response->wallfound = true;
    } else {
      response->wallfound = false;
    }
    
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create an instance of the Find_Wall class
  auto node = std::make_shared<Find_Wall>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}