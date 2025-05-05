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
        tf_listener_(tf_buffer_), min_safe_distance_(0.36),
        max_angular_vel_(2.5), error_distance_(0.0), error_yaw_(0.0),
        is_chasing_(true), transform_available_(false) {

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);
    runner_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "morty/cmd_vel", 10,
        std::bind(&BaristaChase::runner_cmd_callback, this,
                  std::placeholders::_1));
    tf_timer_ = this->create_wall_timer(
        100ms, std::bind(&BaristaChase::listen_to_tf, this));
    control_timer_ = this->create_wall_timer(
        100ms, std::bind(&BaristaChase::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Robot chase node initialized.");

    // Runner robot velocities
    morty_linear_x_ = 0.0;
    morty_angular_z_ = 0.0;

    // Controller gains
    kp_distance_ = 0.4;
    kd_distance_ = 0.9;
    kp_yaw_ = 1.4;
    kd_yaw_ = 0.8;
    ki_distance_ = .1;

    prev_error_distance_ = 0.0;
    prev_error_yaw_ = 0.0;
    integral_error_distance_ = 0.0;
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
      transform_available_ = true;

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      transform_available_ = false;
    }
  }
  void runner_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    morty_linear_x_ = msg->linear.x;
    morty_angular_z_ = msg->angular.z;
  }

  void control_loop() {
    if (!transform_available_) {
      return;
    }

    geometry_msgs::msg::Twist cmd_vel;

    if (error_distance_ > min_safe_distance_) {

      double delta_error_distance = error_distance_ - prev_error_distance_;
      double delta_error_yaw = error_yaw_ - prev_error_yaw_;

      // Integrate errors over time (100ms) if chasing, not moving in circle
      if (std::abs(morty_angular_z_) < 0.05) {
        if (error_distance_ > 0.8) {
          const double max_integral_distance = 20.0;
          integral_error_distance_ += error_distance_ * 0.1;
          integral_error_distance_ =
              std::clamp(integral_error_distance_, -max_integral_distance,
                         max_integral_distance);
        } else {
          integral_error_distance_ *= .95;
        }
      } else {
        integral_error_distance_ = 0.0;
      }

      // PID controller
      double linear_vel = kp_distance_ * error_distance_ +
                          ki_distance_ * integral_error_distance_ +
                          kd_distance_ * delta_error_distance;
      double angular_vel = kp_yaw_ * error_yaw_ + kd_yaw_ * delta_error_yaw;

      // Morty stopped, reduce linear velocity by extra
      if (std::abs(morty_linear_x_) < 0.05 && linear_vel > 1.5) {
        linear_vel *= 0.7; // Slow down by 30%
      }

      // Limit angular vel
      angular_vel =
          std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

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

    cmd_vel_pub_->publish(cmd_vel);

    // Save the current errors for the next iteration
    prev_error_distance_ = error_distance_;
    prev_error_yaw_ = error_yaw_;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr runner_cmd_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double kp_distance_, kp_yaw_;
  double kd_distance_, kd_yaw_;
  double ki_distance_;
  double min_safe_distance_, max_angular_vel_;
  double morty_linear_x_, morty_angular_z_;
  double error_distance_, error_yaw_;
  double prev_error_distance_, prev_error_yaw_, integral_error_distance_;
  bool is_chasing_, transform_available_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaristaChase>());
  rclcpp::shutdown();
  return 0;
}
