#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class BaristaChase : public rclcpp::Node {
public:
  BaristaChase()
      : Node("barista_chase_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_), min_safe_distance_(0.36), is_chasing_(true),
        error_distance_(0.0), error_yaw_(0.0) {

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);
    tf_timer_ = this->create_wall_timer(
        100ms, std::bind(&BaristaChase::listen_to_tf, this));
    control_timer_ = this->create_wall_timer(
        100ms, std::bind(&BaristaChase::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Robot chase node initialized.");

    // Controller gains
    kp_distance_ = 0.8;
    kp_yaw_ = 1.5;
    kd_distance_ = 0.5;
    kd_yaw_ = 0.7;

    prev_error_distance_ = 0.0;
    prev_error_yaw_ = 0.0;
  }

private:
  void listen_to_tf() {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
          "rick/base_link", "morty/base_link", tf2::TimePointZero);
      double dx = transform_stamped.transform.translation.x;
      double dy = transform_stamped.transform.translation.y;
      error_distance_ = std::sqrt(dx * dx + dy * dy);
      error_yaw_ = std::atan2(dy, dx);

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  void control_loop() {
    geometry_msgs::msg::Twist cmd_vel;

    if (error_distance_ > min_safe_distance_) {
      // PD control
      double delta_error_distance = error_distance_ - prev_error_distance_;
      double delta_error_yaw = error_yaw_ - prev_error_yaw_;

      double linear_vel =
          kp_distance_ * error_distance_ + kd_distance_ * delta_error_distance;
      double angular_vel = kp_yaw_ * error_yaw_ + kd_yaw_ * delta_error_yaw;

      if (angular_vel > 1.) {
        angular_vel = 1.;
      } else if (angular_vel < -1.) {
        angular_vel = -1.;
      }

      cmd_vel.linear.x = linear_vel;
      cmd_vel.angular.z = angular_vel;
      is_chasing_ = true;

    } else {
      // Stop the chase
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;

      if (is_chasing_) {
        RCLCPP_INFO(this->get_logger(), "Too close to Morty! Stopping.");
      }
      is_chasing_ = false;
    }

    // Save the current errors for the next iteration
    prev_error_distance_ = error_distance_;
    prev_error_yaw_ = error_yaw_;

    cmd_vel_pub_->publish(cmd_vel);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double kp_distance_;
  double kp_yaw_;
  double kd_distance_;
  double kd_yaw_;
  double min_safe_distance_;
  bool is_chasing_;
  double error_distance_;
  double error_yaw_;
  double prev_error_distance_;
  double prev_error_yaw_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaristaChase>());
  rclcpp::shutdown();
  return 0;
}
