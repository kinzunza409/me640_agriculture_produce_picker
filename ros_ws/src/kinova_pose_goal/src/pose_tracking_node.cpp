#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace kinova_pose_goal
{

struct Quaternion
{
  double x;
  double y;
  double z;
  double w;
};

bool finite(double value)
{
  return std::isfinite(value);
}

Quaternion normalized(Quaternion q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 1.0e-12) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return q;
}

Quaternion multiply(const Quaternion & a, const Quaternion & b)
{
  return {
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

void clamp_vector(double & x, double & y, double & z, double maximum)
{
  const double norm = std::sqrt(x * x + y * y + z * z);
  if (norm > maximum && norm > 0.0) {
    const double scale = maximum / norm;
    x *= scale;
    y *= scale;
    z *= scale;
  }
}

std::string normalized_frame(std::string frame)
{
  const auto first = frame.find_first_not_of(" /\t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = frame.find_last_not_of(" \t\r\n");
  return frame.substr(first, last - first + 1);
}

class PoseTrackingNode : public rclcpp::Node
{
public:
  PoseTrackingNode()
  : Node("kinova_pose_tracker")
  {
    target_topic_ = declare_parameter<std::string>("target_topic", "/kinova/target_pose");
    twist_topic_ = declare_parameter<std::string>(
      "twist_command_topic", "/servo_node/delta_twist_cmds");
    planning_frame_ = normalized_frame(
      declare_parameter<std::string>("planning_frame", "base_link"));
    ee_frame_ = normalized_frame(declare_parameter<std::string>("ee_frame", "end_effector_link"));
    control_rate_ = std::max(1.0, declare_parameter<double>("control_rate", 50.0));
    tf_timeout_ = std::max(0.0, declare_parameter<double>("tf_timeout", 0.05));
    linear_gain_ = std::max(0.0, declare_parameter<double>("linear_gain", 1.0));
    angular_gain_ = std::max(0.0, declare_parameter<double>("angular_gain", 1.0));
    max_linear_speed_ = std::max(
      0.0, declare_parameter<double>("max_linear_speed", 0.03));
    max_angular_speed_ = std::max(
      0.0, declare_parameter<double>("max_angular_speed", 0.10));
    position_tolerance_ = std::max(
      0.0, declare_parameter<double>("position_tolerance", 0.005));
    orientation_tolerance_ = std::max(
      0.0, declare_parameter<double>("orientation_tolerance", 0.03));
    enabled_ = declare_parameter<bool>("start_enabled", false);

    if (planning_frame_.empty() || ee_frame_.empty()) {
      throw std::runtime_error("planning_frame and ee_frame must not be empty");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    twist_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, 10);
    target_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&PoseTrackingNode::target_callback, this, std::placeholders::_1));
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "~/set_enabled",
      std::bind(
        &PoseTrackingNode::enable_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_),
      std::bind(&PoseTrackingNode::control_step, this));

    RCLCPP_INFO(
      get_logger(),
      "Tracking '%s' with '%s' in '%s'; commands -> '%s'; enabled=%s",
      target_topic_.c_str(), ee_frame_.c_str(), planning_frame_.c_str(),
      twist_topic_.c_str(), enabled_ ? "true" : "false");
  }

private:
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    auto target = *message;
    target.header.frame_id = normalized_frame(target.header.frame_id);
    auto & p = target.pose.position;
    auto & q = target.pose.orientation;
    if (target.header.frame_id.empty() || !finite(p.x) || !finite(p.y) || !finite(p.z) ||
      !finite(q.x) || !finite(q.y) || !finite(q.z) || !finite(q.w))
    {
      RCLCPP_ERROR(get_logger(), "Rejected invalid pose target");
      return;
    }

    const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (q_norm < 1.0e-9) {
      RCLCPP_ERROR(get_logger(), "Rejected pose target with a zero-length quaternion");
      return;
    }
    q.x /= q_norm;
    q.y /= q_norm;
    q.z /= q_norm;
    q.w /= q_norm;

    target_ = target;
    RCLCPP_INFO(get_logger(), "Accepted target in frame '%s'", target.header.frame_id.c_str());
  }

  void enable_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    enabled_ = request->data;
    if (!enabled_) {
      publish_zero_twist();
    }
    response->success = true;
    response->message = enabled_ ? "pose tracking enabled" : "pose tracking disabled";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void control_step()
  {
    if (!enabled_ || !target_.has_value()) {
      return;
    }

    auto current_target = target_.value();
    current_target.header.stamp.sec = 0;
    current_target.header.stamp.nanosec = 0;

    geometry_msgs::msg::PoseStamped target_in_planning_frame;
    geometry_msgs::msg::TransformStamped current_ee;
    try {
      target_in_planning_frame = tf_buffer_->transform(
        current_target, planning_frame_, tf2::durationFromSec(tf_timeout_));
      current_ee = tf_buffer_->lookupTransform(
        planning_frame_, ee_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Tracking paused: TF unavailable: %s", error.what());
      return;
    }

    const auto & target_position = target_in_planning_frame.pose.position;
    const auto & current_position = current_ee.transform.translation;
    double vx = linear_gain_ * (target_position.x - current_position.x);
    double vy = linear_gain_ * (target_position.y - current_position.y);
    double vz = linear_gain_ * (target_position.z - current_position.z);
    const double position_error = std::sqrt(vx * vx + vy * vy + vz * vz) /
      std::max(linear_gain_, 1.0e-12);
    if (position_error <= position_tolerance_) {
      vx = 0.0;
      vy = 0.0;
      vz = 0.0;
    }
    clamp_vector(vx, vy, vz, max_linear_speed_);

    const auto & tq = target_in_planning_frame.pose.orientation;
    const auto & cq = current_ee.transform.rotation;
    Quaternion target_q = normalized({tq.x, tq.y, tq.z, tq.w});
    const Quaternion current_inverse = normalized({-cq.x, -cq.y, -cq.z, cq.w});
    Quaternion error_q = normalized(multiply(target_q, current_inverse));
    if (error_q.w < 0.0) {
      error_q.x = -error_q.x;
      error_q.y = -error_q.y;
      error_q.z = -error_q.z;
      error_q.w = -error_q.w;
    }

    const double vector_norm = std::sqrt(
      error_q.x * error_q.x + error_q.y * error_q.y + error_q.z * error_q.z);
    double angle = 0.0;
    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;
    if (vector_norm > 1.0e-12) {
      angle = 2.0 * std::atan2(vector_norm, std::clamp(error_q.w, -1.0, 1.0));
      if (angle > orientation_tolerance_) {
        const double scale = angular_gain_ * angle / vector_norm;
        wx = scale * error_q.x;
        wy = scale * error_q.y;
        wz = scale * error_q.z;
      }
    }
    clamp_vector(wx, wy, wz, max_angular_speed_);

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = planning_frame_;
    command.twist.linear.x = vx;
    command.twist.linear.y = vy;
    command.twist.linear.z = vz;
    command.twist.angular.x = wx;
    command.twist.angular.y = wy;
    command.twist.angular.z = wz;
    twist_publisher_->publish(command);
  }

  void publish_zero_twist()
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = planning_frame_;
    twist_publisher_->publish(command);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<geometry_msgs::msg::PoseStamped> target_;
  std::string target_topic_;
  std::string twist_topic_;
  std::string planning_frame_;
  std::string ee_frame_;
  double control_rate_;
  double tf_timeout_;
  double linear_gain_;
  double angular_gain_;
  double max_linear_speed_;
  double max_angular_speed_;
  double position_tolerance_;
  double orientation_tolerance_;
  bool enabled_;
};

}  // namespace kinova_pose_goal

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<kinova_pose_goal::PoseTrackingNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("kinova_pose_tracker"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
