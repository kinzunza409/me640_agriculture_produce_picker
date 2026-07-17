#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace kinova_pose_goal
{

using SteadyClock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;

std::string clean_frame(std::string frame)
{
  const auto first = frame.find_first_not_of(" /\t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = frame.find_last_not_of(" \t\r\n");
  return frame.substr(first, last - first + 1);
}

class FakeTargetFrameNode : public rclcpp::Node
{
public:
  FakeTargetFrameNode()
  : Node("fake_target_frame")
  {
    parent_frame_ = clean_frame(declare_parameter<std::string>("parent_frame", "base_link"));
    child_frame_ = clean_frame(
      declare_parameter<std::string>("child_frame", "fake_target_frame"));
    ee_frame_ = clean_frame(
      declare_parameter<std::string>("ee_frame", "end_effector_link"));
    target_topic_ = declare_parameter<std::string>("target_topic", "/kinova/target_pose");
    update_rate_ = std::max(1.0, declare_parameter<double>("update_rate", 50.0));
    travel_distance_ = std::max(0.0, declare_parameter<double>("travel_distance", 0.1));
    leg_duration_ = std::max(0.1, declare_parameter<double>("leg_duration", 10.0));
    publish_initial_target_ = declare_parameter<bool>("publish_initial_target", true);

    axis_x_ = declare_parameter<double>("axis_x", 1.0);
    axis_y_ = declare_parameter<double>("axis_y", 0.0);
    axis_z_ = declare_parameter<double>("axis_z", 0.0);
    const double axis_norm = std::sqrt(axis_x_ * axis_x_ + axis_y_ * axis_y_ + axis_z_ * axis_z_);
    if (parent_frame_.empty() || child_frame_.empty() || ee_frame_.empty() || axis_norm < 1.0e-9) {
      throw std::runtime_error("fake target frames and motion axis must be valid");
    }
    axis_x_ /= axis_norm;
    axis_y_ /= axis_norm;
    axis_z_ /= axis_norm;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    target_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      target_topic_, rclcpp::QoS(1).reliable().transient_local());
    movement_service_ = create_service<std_srvs::srv::SetBool>(
      "~/set_moving",
      std::bind(
        &FakeTargetFrameNode::movement_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_),
      std::bind(&FakeTargetFrameNode::update, this));
    last_update_ = SteadyClock::now();

    RCLCPP_INFO(
      get_logger(),
      "Fake target '%s' starts stationary in '%s'; %.3f m cosine round trip over %.1f s; "
      "call ~/set_moving to start",
      child_frame_.c_str(), parent_frame_.c_str(), travel_distance_, 2.0 * leg_duration_);
  }

private:
  void movement_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (request->data && publish_initial_target_ && !target_published_) {
      response->success = false;
      response->message = "initial target has not been captured yet";
      return;
    }

    if (request->data && elapsed_motion_ >= 2.0 * leg_duration_) {
      elapsed_motion_ = 0.0;
    }
    moving_ = request->data;
    last_update_ = SteadyClock::now();
    response->success = true;
    response->message = moving_ ? "fake target motion started" : "fake target motion paused";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void update()
  {
    const auto current_time = SteadyClock::now();
    const double dt = std::chrono::duration<double>(current_time - last_update_).count();
    last_update_ = current_time;
    if (moving_) {
      elapsed_motion_ = std::min(elapsed_motion_ + dt, 2.0 * leg_duration_);
      if (elapsed_motion_ >= 2.0 * leg_duration_) {
        moving_ = false;
        RCLCPP_INFO(get_logger(), "Fake target completed one round trip and stopped");
      }
    }

    const double offset = motion_offset(elapsed_motion_);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;
    transform.transform.translation.x = axis_x_ * offset;
    transform.transform.translation.y = axis_y_ * offset;
    transform.transform.translation.z = axis_z_ * offset;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(transform);

    if (publish_initial_target_ && !target_published_) {
      publish_current_ee_as_target();
    }
  }

  double motion_offset(double elapsed) const
  {
    if (elapsed <= leg_duration_) {
      return 0.5 * travel_distance_ * (1.0 - std::cos(kPi * elapsed / leg_duration_));
    }
    const double return_time = std::min(elapsed - leg_duration_, leg_duration_);
    return 0.5 * travel_distance_ * (1.0 + std::cos(kPi * return_time / leg_duration_));
  }

  void publish_current_ee_as_target()
  {
    if (target_publisher_->get_subscription_count() == 0) {
      return;
    }
    try {
      const auto current_ee = tf_buffer_->lookupTransform(
        child_frame_, ee_frame_, tf2::TimePointZero, tf2::durationFromSec(0.0));
      geometry_msgs::msg::PoseStamped target;
      target.header.stamp = now();
      target.header.frame_id = child_frame_;
      target.pose.position.x = current_ee.transform.translation.x;
      target.pose.position.y = current_ee.transform.translation.y;
      target.pose.position.z = current_ee.transform.translation.z;
      target.pose.orientation = current_ee.transform.rotation;
      target_publisher_->publish(target);
      target_published_ = true;
      RCLCPP_INFO(
        get_logger(), "Published one initial target preserving current '%s' pose in '%s'",
        ee_frame_.c_str(), child_frame_.c_str());
    } catch (const tf2::TransformException & error) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting to capture initial target: %s", error.what());
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr movement_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string parent_frame_;
  std::string child_frame_;
  std::string ee_frame_;
  std::string target_topic_;
  double update_rate_;
  double travel_distance_;
  double leg_duration_;
  double axis_x_;
  double axis_y_;
  double axis_z_;
  double elapsed_motion_{0.0};
  bool publish_initial_target_{true};
  bool target_published_{false};
  bool moving_{false};
  SteadyClock::time_point last_update_;
};

}  // namespace kinova_pose_goal

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<kinova_pose_goal::FakeTargetFrameNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fake_target_frame"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
