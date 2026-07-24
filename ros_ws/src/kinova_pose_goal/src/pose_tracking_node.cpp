#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace kinova_pose_goal
{

using SteadyClock = std::chrono::steady_clock;

bool finite(double value)
{
  return std::isfinite(value);
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

double seconds_since(const SteadyClock::time_point & time)
{
  return std::chrono::duration<double>(SteadyClock::now() - time).count();
}

double translation_jump(
  const geometry_msgs::msg::TransformStamped & previous,
  const geometry_msgs::msg::TransformStamped & current)
{
  const double dx = current.transform.translation.x - previous.transform.translation.x;
  const double dy = current.transform.translation.y - previous.transform.translation.y;
  const double dz = current.transform.translation.z - previous.transform.translation.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double rotation_jump(
  const geometry_msgs::msg::TransformStamped & previous,
  const geometry_msgs::msg::TransformStamped & current)
{
  const auto & a = previous.transform.rotation;
  const auto & b = current.transform.rotation;
  const double a_norm = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
  const double b_norm = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w);
  if (a_norm < 1.0e-12 || b_norm < 1.0e-12) {
    return std::numeric_limits<double>::infinity();
  }
  const double dot = std::abs(
    (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w) / (a_norm * b_norm));
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

bool finite_transform(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & t = transform.transform.translation;
  const auto & q = transform.transform.rotation;
  return finite(t.x) && finite(t.y) && finite(t.z) && finite(q.x) && finite(q.y) &&
         finite(q.z) && finite(q.w);
}

class PoseTrackingNode : public rclcpp::Node
{
public:
  PoseTrackingNode()
  : Node("kinova_pose_tracker")
  {
    target_topic_ = declare_parameter<std::string>("target_topic", "/kinova/target_pose");
    pose_command_topic_ = declare_parameter<std::string>(
      "pose_command_topic", "/servo_node/pose_target_cmds");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    servo_status_topic_ = declare_parameter<std::string>(
      "servo_status_topic", "/servo_node/status");
    servo_pause_service_ = declare_parameter<std::string>(
      "servo_pause_service", "/servo_node/pause_servo");
    planning_frame_ = normalized_frame(
      declare_parameter<std::string>("planning_frame", "base_link"));

    republish_rate_ = std::max(
      1.0, declare_parameter<double>("target_republish_rate", 50.0));
    tf_timeout_ = std::max(0.0, declare_parameter<double>("tf_timeout", 0.2));
    max_translation_jump_ = std::max(
      0.0, declare_parameter<double>("max_translation_jump", 0.02));
    max_rotation_jump_ = std::max(
      0.0, declare_parameter<double>("max_rotation_jump", 0.05));
    valid_samples_to_recover_ = std::max<int64_t>(
      1, declare_parameter<int64_t>("valid_samples_to_recover", 5));
    joint_state_timeout_ = std::max(
      0.0, declare_parameter<double>("joint_state_timeout", 0.5));
    servo_status_timeout_ = std::max(
      0.0, declare_parameter<double>("servo_status_timeout", 0.5));
    servo_status_grace_period_ = std::max(
      0.0, declare_parameter<double>("servo_status_grace_period", 1.0));
    servo_pause_delay_ = std::max(
      0.0, declare_parameter<double>("servo_pause_delay", 0.3));

    if (planning_frame_.empty()) {
      throw std::runtime_error("planning_frame must not be empty");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_command_topic_, rclcpp::SystemDefaultsQoS());
    target_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&PoseTrackingNode::target_callback, this, std::placeholders::_1));
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PoseTrackingNode::joint_state_callback, this, std::placeholders::_1));
    servo_status_subscription_ = create_subscription<moveit_msgs::msg::ServoStatus>(
      servo_status_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&PoseTrackingNode::servo_status_callback, this, std::placeholders::_1));
    pause_client_ = create_client<std_srvs::srv::SetBool>(servo_pause_service_);
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "~/set_enabled",
      std::bind(
        &PoseTrackingNode::enable_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / republish_rate_),
      std::bind(&PoseTrackingNode::control_step, this));

    RCLCPP_INFO(
      get_logger(),
      "Target keeper ready: '%s' -> '%s' at %.1f Hz; planning frame '%s'; enabled=false",
      target_topic_.c_str(), pose_command_topic_.c_str(), republish_rate_,
      planning_frame_.c_str());
  }

private:
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    auto target = *message;
    target.header.frame_id = normalized_frame(target.header.frame_id);
    auto & p = target.pose.position;
    auto & q = target.pose.orientation;

    const bool values_are_finite = finite(p.x) && finite(p.y) && finite(p.z) &&
      finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w);
    const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (target.header.frame_id.empty() || !values_are_finite || q_norm < 1.0e-9) {
      RCLCPP_ERROR(get_logger(), "Rejected invalid pose target");
      if (enabled_) {
        latch_fault("invalid PoseStamped received while tracking");
      }
      return;
    }

    q.x /= q_norm;
    q.y /= q_norm;
    q.z /= q_norm;
    q.w /= q_norm;

    if (enabled_ && target_.has_value() &&
      target_->header.frame_id != target.header.frame_id)
    {
      latch_fault(
        "target frame changed from '" + target_->header.frame_id + "' to '" +
        target.header.frame_id + "' while tracking");
    }

    target_ = target;
    last_target_transform_.reset();
    valid_tf_samples_ = 0;
    RCLCPP_INFO(
      get_logger(), "Stored target in frame '%s'", target.header.frame_id.c_str());
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (!message->name.empty() && message->name.size() == message->position.size()) {
      joint_state_received_ = true;
      last_joint_state_receipt_ = SteadyClock::now();
    }
  }

  void servo_status_callback(const moveit_msgs::msg::ServoStatus::SharedPtr message)
  {
    servo_status_received_ = true;
    last_servo_status_receipt_ = SteadyClock::now();
    last_servo_status_code_ = message->code;
    last_servo_status_message_ = message->message;

    if (enabled_ && is_hard_servo_status(message->code)) {
      latch_fault("MoveIt Servo: " + message->message);
    } else if (enabled_ && message->code != moveit_msgs::msg::ServoStatus::NO_WARNING) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "MoveIt Servo: %s", message->message.c_str());
    }
  }

  void enable_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (!request->data) {
      enabled_ = false;
      schedule_pause();
      response->success = true;
      response->message = "pose tracking disabled; smooth halt requested";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    if (enabled_) {
      response->success = true;
      response->message = "pose tracking already enabled";
      return;
    }
    if (!target_.has_value()) {
      reject_enable(response, "no valid pose target has been received");
      return;
    }
    if (!joint_state_is_fresh()) {
      reject_enable(response, "joint state is unavailable or stale");
      return;
    }
    if (!pause_client_->service_is_ready() || pause_request_in_flight_) {
      reject_enable(response, "MoveIt Servo pause service is not ready");
      return;
    }
    if (!sample_target_transform(false)) {
      reject_enable(response, "target TF is invalid: " + last_tf_error_);
      return;
    }

    const int64_t required_samples = fault_latched_ ? valid_samples_to_recover_ : 1;
    if (valid_tf_samples_ < required_samples) {
      reject_enable(
        response, "waiting for " + std::to_string(required_samples - valid_tf_samples_) +
        " additional valid TF sample(s)");
      return;
    }

    if (fault_latched_) {
      RCLCPP_WARN(
        get_logger(), "Clearing latched fault after explicit enable: %s",
        fault_reason_.c_str());
    }
    fault_latched_ = false;
    fault_reason_.clear();
    pause_pending_ = false;
    // A paused Servo node does not publish status. Ignore any pre-fault status and
    // require a fresh sample after unpausing, within the configured grace period.
    servo_status_received_ = false;
    last_servo_status_message_.clear();
    enabled_ = true;
    enabled_at_ = SteadyClock::now();
    send_pause_request(false);

    response->success = true;
    response->message = "pose tracking enabled";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void reject_enable(
    const std_srvs::srv::SetBool::Response::SharedPtr & response,
    const std::string & reason)
  {
    response->success = false;
    response->message = "cannot enable pose tracking: " + reason;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }

  void control_step()
  {
    process_pending_pause();

    if (!target_.has_value()) {
      return;
    }
    if (!sample_target_transform(enabled_)) {
      return;
    }
    if (!enabled_) {
      return;
    }
    if (!joint_state_is_fresh()) {
      latch_fault("joint state became unavailable or stale");
      return;
    }
    if (seconds_since(enabled_at_) > servo_status_grace_period_ &&
      (!servo_status_received_ || seconds_since(last_servo_status_receipt_) > servo_status_timeout_))
    {
      latch_fault("MoveIt Servo status became unavailable or stale");
      return;
    }
    if (servo_status_received_ && is_hard_servo_status(last_servo_status_code_)) {
      latch_fault("MoveIt Servo: " + last_servo_status_message_);
      return;
    }

    geometry_msgs::msg::PoseStamped command;
    try {
      tf2::doTransform(*target_, command, *last_target_transform_);
    } catch (const tf2::TransformException & error) {
      latch_fault(std::string("failed to transform target pose: ") + error.what());
      return;
    }
    command.header.stamp = now();
    command.header.frame_id = planning_frame_;
    pose_publisher_->publish(command);
  }

  bool sample_target_transform(bool fault_on_failure)
  {
    geometry_msgs::msg::TransformStamped transform;
    const std::string & target_frame = target_->header.frame_id;

    if (target_frame == planning_frame_) {
      transform.header.stamp = now();
      transform.header.frame_id = planning_frame_;
      transform.child_frame_id = target_frame;
      transform.transform.rotation.w = 1.0;
    } else {
      try {
        transform = tf_buffer_->lookupTransform(
          planning_frame_, target_frame, tf2::TimePointZero,
          tf2::durationFromSec(std::min(tf_timeout_, 0.02)));
      } catch (const tf2::TransformException & error) {
        return handle_invalid_tf(
          "TF " + planning_frame_ + " <- " + target_frame + " unavailable: " + error.what(),
          fault_on_failure);
      }
    }

    if (!finite_transform(transform)) {
      return handle_invalid_tf("target transform contains NaN/Inf", fault_on_failure);
    }

    const bool has_timestamp =
      transform.header.stamp.sec != 0 || transform.header.stamp.nanosec != 0;
    const rclcpp::Time current_time = now();
    // ROS time zero means the simulation clock has not been initialized yet.
    if (has_timestamp && current_time.nanoseconds() != 0) {
      const rclcpp::Time stamp(transform.header.stamp, get_clock()->get_clock_type());
      const double age = (current_time - stamp).seconds();
      if (age < -0.05 || age > tf_timeout_) {
        return handle_invalid_tf(
          "target TF age " + std::to_string(age) + " s exceeds limit " +
          std::to_string(tf_timeout_) + " s", fault_on_failure);
      }
    }

    if (last_target_transform_.has_value()) {
      const double linear_jump = translation_jump(*last_target_transform_, transform);
      const double angular_jump = rotation_jump(*last_target_transform_, transform);
      if (linear_jump > max_translation_jump_ || angular_jump > max_rotation_jump_) {
        last_target_transform_ = transform;
        valid_tf_samples_ = 1;
        const std::string reason =
          "target TF discontinuity: translation=" + std::to_string(linear_jump) +
          " m, rotation=" + std::to_string(angular_jump) + " rad, frames '" +
          planning_frame_ + "' <- '" + target_frame + "'";
        last_tf_error_ = reason;
        if (fault_on_failure) {
          latch_fault(reason);
        } else {
          RCLCPP_WARN(get_logger(), "%s", reason.c_str());
        }
        return false;
      }
    }

    last_target_transform_ = transform;
    valid_tf_samples_ = std::min(valid_tf_samples_ + 1, valid_samples_to_recover_);
    last_tf_error_.clear();
    return true;
  }

  bool handle_invalid_tf(const std::string & reason, bool fault_on_failure)
  {
    last_tf_error_ = reason;
    last_target_transform_.reset();
    valid_tf_samples_ = 0;
    if (fault_on_failure) {
      latch_fault(reason);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason.c_str());
    }
    return false;
  }

  bool joint_state_is_fresh() const
  {
    return joint_state_received_ && seconds_since(last_joint_state_receipt_) <= joint_state_timeout_;
  }

  bool is_hard_servo_status(int8_t status) const
  {
    return status == moveit_msgs::msg::ServoStatus::INVALID ||
           status == moveit_msgs::msg::ServoStatus::HALT_FOR_SINGULARITY ||
           status == moveit_msgs::msg::ServoStatus::HALT_FOR_COLLISION ||
           status == moveit_msgs::msg::ServoStatus::JOINT_BOUND;
  }

  void latch_fault(const std::string & reason)
  {
    if (fault_latched_) {
      return;
    }
    fault_latched_ = true;
    fault_reason_ = reason;
    enabled_ = false;
    schedule_pause();
    RCLCPP_ERROR(get_logger(), "Tracking fault latched: %s", reason.c_str());
  }

  void schedule_pause()
  {
    pause_pending_ = true;
    pause_deadline_ = SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(servo_pause_delay_));
  }

  void process_pending_pause()
  {
    if (!pause_pending_ || pause_request_in_flight_ || SteadyClock::now() < pause_deadline_) {
      return;
    }
    if (!pause_client_->service_is_ready()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot pause MoveIt Servo: service '%s' unavailable",
        servo_pause_service_.c_str());
      return;
    }
    pause_pending_ = false;
    send_pause_request(true);
  }

  void send_pause_request(bool pause)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = pause;
    pause_request_in_flight_ = true;
    pause_client_->async_send_request(
      request,
      [this, pause](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        pause_request_in_flight_ = false;
        try {
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_ERROR(
              get_logger(), "MoveIt Servo pause request failed: %s", response->message.c_str());
            if (pause) {
              schedule_pause();
            } else {
              latch_fault("MoveIt Servo rejected unpause request");
            }
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "MoveIt Servo pause service failed: %s", error.what());
          if (pause) {
            schedule_pause();
          } else {
            latch_fault("MoveIt Servo unpause service call failed");
          }
        }
      });
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr servo_status_subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pause_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<geometry_msgs::msg::PoseStamped> target_;
  std::optional<geometry_msgs::msg::TransformStamped> last_target_transform_;
  std::string target_topic_;
  std::string pose_command_topic_;
  std::string joint_state_topic_;
  std::string servo_status_topic_;
  std::string servo_pause_service_;
  std::string planning_frame_;
  std::string fault_reason_;
  std::string last_tf_error_;
  std::string last_servo_status_message_;
  double republish_rate_;
  double tf_timeout_;
  double max_translation_jump_;
  double max_rotation_jump_;
  double joint_state_timeout_;
  double servo_status_timeout_;
  double servo_status_grace_period_;
  double servo_pause_delay_;
  int64_t valid_samples_to_recover_;
  int64_t valid_tf_samples_{0};
  int8_t last_servo_status_code_{moveit_msgs::msg::ServoStatus::NO_WARNING};
  bool enabled_{false};
  bool fault_latched_{false};
  bool joint_state_received_{false};
  bool servo_status_received_{false};
  bool pause_pending_{false};
  bool pause_request_in_flight_{false};
  SteadyClock::time_point enabled_at_{SteadyClock::now()};
  SteadyClock::time_point last_joint_state_receipt_{SteadyClock::now()};
  SteadyClock::time_point last_servo_status_receipt_{SteadyClock::now()};
  SteadyClock::time_point pause_deadline_{SteadyClock::now()};
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
