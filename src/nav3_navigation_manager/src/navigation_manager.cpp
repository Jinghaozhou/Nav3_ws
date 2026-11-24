#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pct_planner_interfaces/srv/plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <scan_planner_msgs/msg/planner_status.hpp>
#include <scan_planner_msgs/msg/execution_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "nav3_navigation_manager/action/navigate_to_pose.hpp"

namespace {
using Clock = std::chrono::steady_clock;
using Action = nav3_navigation_manager::action::NavigateToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
using Plan = pct_planner_interfaces::srv::Plan;
using Status = scan_planner_msgs::msg::PlannerStatus;
using Execution = scan_planner_msgs::msg::ExecutionState;

bool finitePoint(const geometry_msgs::msg::Point &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}
}  // namespace

class NavigationManager : public rclcpp::Node {
 public:
  NavigationManager() : Node("nav3_navigation_manager") {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    action_name_ = declare_parameter<std::string>("action_name", "/nav3/navigate_to_pose");
    const auto permission_topic = declare_parameter<std::string>(
        "localization_permission_topic", "/localization/navigation_allowed");
    const auto status_topic = declare_parameter<std::string>(
        "scan_status_topic", "/scan_planner/status");
    const auto execution_topic = declare_parameter<std::string>(
        "scan_execution_topic", "/scan_planner/execution_state");
    const auto service_name = declare_parameter<std::string>("pct_plan_service", "/pct/plan");
    const auto path_topic = declare_parameter<std::string>("managed_path_topic", "/nav3/managed_path");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/nav3/navigation_state");
    const auto heartbeat_topic = declare_parameter<std::string>("task_heartbeat_topic", "/nav3/task_heartbeat");
    bypass_checks_ = declare_parameter<bool>("bypass_checks", false);
    bypass_max_task_duration_ = positive("bypass_max_task_duration_sec", 45.0);
    start_z_override_enabled_ = declare_parameter<bool>("start_z_override_enabled", false);
    start_z_override_m_ = declare_parameter<double>("start_z_override_m", 0.0);
    if (start_z_override_enabled_ && !std::isfinite(start_z_override_m_)) {
      throw std::invalid_argument("start_z_override_m must be finite when enabled");
    }
    permission_timeout_ = positive("permission_timeout_sec", 1.0);
    status_timeout_ = positive("scan_status_timeout_sec", 1.0);
    execution_timeout_ = positive("scan_execution_timeout_sec", 1.0);
    execution_loss_timeout_ = positive("execution_loss_timeout_sec", 2.0);
    tf_max_age_ = positive("tf_max_age_sec", 0.75);
    scan_ack_timeout_ = positive("scan_ack_timeout_sec", 5.0);
    planning_timeout_ = positive("planning_timeout_sec", 120.0);
    max_task_duration_ = positive("max_task_duration_sec", 600.0);
    endpoint_tolerance_ = positive("path_endpoint_tolerance_m", 0.5);
    max_start_motion_ = positive("max_start_motion_m", 0.5);
    if (map_frame_.empty() || base_frame_.empty() || map_frame_ == base_frame_ ||
        action_name_.empty() || permission_topic.empty() || status_topic.empty() || execution_topic.empty() ||
        service_name.empty() || path_topic.empty() || state_topic.empty()) {
      throw std::invalid_argument("navigation manager requires nonempty, distinct frames and interface names");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    pct_client_ = create_client<Plan>(service_name);
    managed_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        path_topic, rclcpp::QoS(1).reliable().transient_local());
    state_pub_ = create_publisher<std_msgs::msg::String>(
        state_topic, rclcpp::QoS(1).reliable().transient_local());
    heartbeat_pub_ = create_publisher<std_msgs::msg::Bool>(heartbeat_topic,rclcpp::QoS(1).reliable());
    permission_sub_ = create_subscription<std_msgs::msg::Bool>(
        permission_topic, rclcpp::QoS(1).reliable(),
        [this](std_msgs::msg::Bool::ConstSharedPtr message) {
          permission_ = message->data;
          permission_received_ = Clock::now();
        });
    status_sub_ = create_subscription<Status>(
        status_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](Status::ConstSharedPtr message) { onStatus(*message); });
    execution_sub_ = create_subscription<Execution>(
        execution_topic, rclcpp::QoS(10).reliable(),
        [this](Execution::ConstSharedPtr message) {
          execution_ = *message;
          execution_received_ = Clock::now();
          if (active_ && scan_path_id_ && message->path_id == *scan_path_id_ && message->tracking) {
            controller_has_tracked_ = true;
            last_tracking_ = Clock::now();
          }
        });

    // A new manager session revokes any path retained from a previous task.
    publishEmptyPath();
    action_server_ = rclcpp_action::create_server<Action>(
        this, action_name_,
        std::bind(&NavigationManager::onGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavigationManager::onCancel, this, std::placeholders::_1),
        std::bind(&NavigationManager::onAccepted, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Navigation manager ready: action=%s path=%s", action_name_.c_str(), path_topic.c_str());
    if (bypass_checks_) {
      RCLCPP_WARN(get_logger(), "BYPASS_CHECKS enabled: readiness, TF age, path endpoint/start, and execution fault checks are skipped; task limit %.1f s",
          bypass_max_task_duration_);
    }
  }

 private:
  enum class Phase { Idle, Planning, AwaitScan, Executing };

  double positive(const std::string &name, double fallback) {
    const double value = declare_parameter<double>(name, fallback);
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument(name + " must be positive and finite");
    return value;
  }

  bool fresh(const std::optional<Clock::time_point> &received, double limit) const {
    return received && std::chrono::duration<double>(Clock::now() - *received).count() <= limit;
  }

  geometry_msgs::msg::PoseStamped currentPose() const {
    const auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    const double age = (now() - rclcpp::Time(transform.header.stamp)).seconds();
    if (!bypass_checks_ && (!std::isfinite(age) || age < 0.0 || age > tf_max_age_))
      throw std::runtime_error("map to base_link TF is stale or future dated");
    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    if (!finitePoint(pose.pose.position)) throw std::runtime_error("robot pose is not finite");
    return pose;
  }

  std::string readinessProblem() const {
    if (bypass_checks_) return {};
    if (!permission_ || !fresh(permission_received_, permission_timeout_))
      return "localization permission is absent, false or expired";
    if (!status_ || !fresh(status_received_, status_timeout_))
      return "SCAN status is absent or expired";
    if (!execution_ || !fresh(execution_received_, execution_timeout_))
      return "SCAN controller execution state is absent or expired";
    if (!pct_client_->service_is_ready()) return "PCT planning service is unavailable";
    try {
      (void)currentPose();
    } catch (const std::exception &error) {
      return error.what();
    }
    return {};
  }

  rclcpp_action::GoalResponse onGoal(
      const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
    if (reserved_ || active_) {
      RCLCPP_WARN(get_logger(), "Rejected goal: another navigation task is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->target.header.frame_id != map_frame_ ||
        !finitePoint(goal->target.pose.position)) {
      RCLCPP_WARN(get_logger(), "Rejected goal: target must have finite XYZ in %s", map_frame_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto problem = readinessProblem();
    if (!problem.empty()) {
      RCLCPP_WARN(get_logger(), "Rejected goal: %s", problem.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandle> goal_handle) {
    if (goal_handle != active_) return rclcpp_action::CancelResponse::REJECT;
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
    reserved_ = false;
    active_ = goal_handle;
    std_msgs::msg::Bool heartbeat;
    heartbeat.data=true;
    heartbeat_pub_->publish(heartbeat);
    target_ = goal_handle->get_goal()->target;
    phase_ = Phase::Planning;
    task_started_ = phase_started_ = Clock::now();
    cancel_requested_ = false;
    try {
      auto request = std::make_shared<Plan::Request>();
      request->start = currentPose();
      if (start_z_override_enabled_) {
        RCLCPP_WARN(get_logger(), "PCT start height override: TF z=%.3f -> request z=%.3f",
            request->start.pose.position.z, start_z_override_m_);
        request->start.pose.position.z = start_z_override_m_;
      }
      requested_start_ = request->start;
      request->goal = target_;
      pct_client_->async_send_request(request,
          [this, goal_handle](rclcpp::Client<Plan>::SharedFuture future) {
            if (active_ != goal_handle || phase_ != Phase::Planning || cancel_requested_) return;
            try {
              onPlan(future.get());
            } catch (const std::exception &error) {
              abortTask(std::string("PCT request failed: ") + error.what());
            }
          });
      RCLCPP_INFO(get_logger(), "Planning to XYZ=(%.3f, %.3f, %.3f)",
          target_.pose.position.x, target_.pose.position.y, target_.pose.position.z);
    } catch (const std::exception &error) {
      abortTask(std::string("cannot request PCT plan: ") + error.what());
    }
  }

  void onPlan(const std::shared_ptr<Plan::Response> &response) {
    const auto problem = readinessProblem();
    if (!problem.empty()) {
      abortTask("navigation readiness lost during planning: " + problem);
      return;
    }
    if (!response->success) {
      abortTask("PCT planning failed: " + response->message);
      return;
    }
    auto path = response->path;
    if (path.header.frame_id != map_frame_ || path.poses.empty() || path.poses.size() > 200000) {
      abortTask("PCT returned an empty, oversized or wrong-frame path");
      return;
    }
    for (const auto &pose : path.poses) {
      if ((pose.header.frame_id != "" && pose.header.frame_id != map_frame_) ||
          !finitePoint(pose.pose.position)) {
        abortTask("PCT returned a nonfinite or wrong-frame path pose");
        return;
      }
    }
    if (!bypass_checks_ &&
        distance(path.poses.back().pose.position, target_.pose.position) > endpoint_tolerance_) {
      abortTask("PCT path endpoint differs too far from the requested goal");
      return;
    }
    if (!bypass_checks_) {
      const auto current = currentPose();
      if (distance(current.pose.position, requested_start_.pose.position) > max_start_motion_ ||
          distance(path.poses.front().pose.position, current.pose.position) > endpoint_tolerance_) {
        abortTask("robot moved during planning or PCT path start is inconsistent");
        return;
      }
      // PCT snaps endpoints to its elevation/grid representation. The action
      // still promises the requested physical XYZ; connect the last short
      // segment and let SCAN collision-check it before execution.
      if (distance(path.poses.back().pose.position,target_.pose.position)>1e-6) {
        if(path.poses.size()>=200000) {abortTask("PCT path has no capacity for the requested endpoint");return;}
        path.poses.push_back(target_);
      }
    }
    path.header.stamp = now();
    baseline_path_id_ = status_ ? status_->path_id : 0;
    path_sent_stamp_ = path.header.stamp;
    path_endpoint_ = path.poses.back().pose.position;
    scan_path_id_.reset();
    controller_has_tracked_ = false;
    last_tracking_.reset();
    phase_ = Phase::AwaitScan;
    phase_started_ = Clock::now();
    managed_path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "Published managed path with %zu poses", path.poses.size());
  }

  void onStatus(const Status &message) {
    status_ = message;
    status_received_ = Clock::now();
    if (!active_ || cancel_requested_ ||
        (phase_ != Phase::AwaitScan && phase_ != Phase::Executing)) return;
    if (message.header.frame_id != map_frame_ ||
        rclcpp::Time(message.header.stamp) < rclcpp::Time(path_sent_stamp_)) return;
    if (phase_ == Phase::AwaitScan) {
      if (message.path_id == baseline_path_id_) return;
      if (!bypass_checks_ && distance(message.goal, path_endpoint_) > endpoint_tolerance_) return;
      scan_path_id_ = message.path_id;
      phase_ = Phase::Executing;
      phase_started_ = Clock::now();
      controller_has_tracked_ = false;
      last_tracking_.reset();
      RCLCPP_INFO(get_logger(), "SCAN accepted managed path id=%llu",
          static_cast<unsigned long long>(*scan_path_id_));
    }
    if (message.path_id != *scan_path_id_) {
      if (!bypass_checks_) abortTask("SCAN replaced the active path unexpectedly");
      return;
    }
    if (message.state == Status::GOAL_REACHED) {
      finishTask(true, "SCAN reports goal reached");
    } else if (!bypass_checks_ && (message.state == Status::LOCAL_PLAN_FAILED ||
               message.state == Status::GLOBAL_PATH_INVALID ||
               message.state == Status::LOCALIZATION_UNAVAILABLE ||
               message.state == Status::DATA_STALE || message.state == Status::TF_CHANGED)) {
      abortTask("SCAN stopped: " + message.reason);
    }
  }

  void publishEmptyPath() {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = map_frame_;
    empty.header.stamp = now();
    managed_path_pub_->publish(empty);
  }

  void finishTask(bool success, const std::string &message, bool canceled = false) {
    if (!active_) return;
    publishEmptyPath();
    auto result = std::make_shared<Action::Result>();
    result->success = success;
    result->message = message;
    auto finished = std::move(active_);
    phase_ = Phase::Idle;
    scan_path_id_.reset();
    controller_has_tracked_ = false;
    last_tracking_.reset();
    cancel_requested_ = false;
    if (canceled) finished->canceled(result);
    else if (success) finished->succeed(result);
    else finished->abort(result);
    RCLCPP_INFO(get_logger(), "Navigation task ended: %s", message.c_str());
  }

  void abortTask(const std::string &message) { finishTask(false, message); }

  void tick() {
    std_msgs::msg::Bool heartbeat;
    heartbeat.data=static_cast<bool>(active_);
    heartbeat_pub_->publish(heartbeat);
    if (!active_) {
      const auto problem = readinessProblem();
      publishState(problem.empty() ? "READY" : "WAITING: " + problem);
      return;
    }
    if (cancel_requested_) {
      finishTask(false, "navigation canceled", true);
      return;
    }
    const auto problem = readinessProblem();
    if (!problem.empty()) {
      abortTask("navigation readiness lost: " + problem);
      return;
    }
    const auto elapsed = std::chrono::duration<double>(Clock::now() - phase_started_).count();
    const auto total = std::chrono::duration<double>(Clock::now() - task_started_).count();
    if (total > (bypass_checks_ ? bypass_max_task_duration_ : max_task_duration_)) {
      abortTask("navigation task time limit exceeded");
      return;
    }
    if (!bypass_checks_ && phase_ == Phase::Planning && elapsed > planning_timeout_) {
      abortTask("PCT planning timed out");
      return;
    }
    if (!bypass_checks_ && phase_ == Phase::AwaitScan && elapsed > scan_ack_timeout_) {
      abortTask("SCAN did not acknowledge the managed path");
      return;
    }
    if (!bypass_checks_ && phase_ == Phase::Executing && status_ &&
        (status_->state == Status::WAITING_INPUT || status_->state == Status::WAITING_PATH) &&
        elapsed > scan_ack_timeout_) {
      abortTask("SCAN did not start executing the managed path");
      return;
    }
    if (!bypass_checks_ && phase_ == Phase::Executing && !controller_has_tracked_ && elapsed > scan_ack_timeout_) {
      abortTask("SCAN controller did not track the active path");
      return;
    }
    if (!bypass_checks_ && phase_ == Phase::Executing && controller_has_tracked_ &&
        !fresh(last_tracking_, execution_loss_timeout_)) {
      abortTask("SCAN controller stopped tracking the active path");
      return;
    }
    const auto phase = phase_ == Phase::Planning ? "PLANNING" :
                       phase_ == Phase::AwaitScan ? "WAITING_SCAN" : "EXECUTING";
    publishState(phase);
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->phase = phase;
    feedback->detail = execution_ && phase_ == Phase::Executing ? execution_->reason :
                       (status_ ? status_->reason : "");
    feedback->distance_remaining = -1.0;
    try {
      feedback->current_pose = currentPose();
      feedback->distance_remaining = distance(feedback->current_pose.pose.position, target_.pose.position);
    } catch (const std::exception &) {
      // Readiness check above already handles stale or missing TF.
    }
    active_->publish_feedback(feedback);
  }

  void publishState(const std::string &value) {
    if (last_state_ == value) return;
    std_msgs::msg::String message;
    message.data = value;
    state_pub_->publish(message);
    last_state_ = value;
  }

  std::string map_frame_, base_frame_, action_name_;
  double permission_timeout_, status_timeout_, execution_timeout_, execution_loss_timeout_, tf_max_age_;
  double scan_ack_timeout_, planning_timeout_, max_task_duration_, endpoint_tolerance_;
  double max_start_motion_;
  double bypass_max_task_duration_;
  bool bypass_checks_ = false;
  bool start_z_override_enabled_ = false;
  double start_z_override_m_ = 0.0;
  rclcpp::Client<Plan>::SharedPtr pct_client_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr managed_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permission_sub_;
  rclcpp::Subscription<Status>::SharedPtr status_sub_;
  rclcpp::Subscription<Execution>::SharedPtr execution_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::optional<Clock::time_point> permission_received_, status_received_, execution_received_, last_tracking_;
  bool permission_ = false, reserved_ = false, cancel_requested_ = false;
  std::optional<Status> status_;
  std::optional<Execution> execution_;
  bool controller_has_tracked_ = false;
  std::shared_ptr<GoalHandle> active_;
  geometry_msgs::msg::PoseStamped target_;
  geometry_msgs::msg::PoseStamped requested_start_;
  geometry_msgs::msg::Point path_endpoint_;
  builtin_interfaces::msg::Time path_sent_stamp_;
  uint64_t baseline_path_id_ = 0;
  std::optional<uint64_t> scan_path_id_;
  Phase phase_ = Phase::Idle;
  Clock::time_point task_started_, phase_started_;
  std::string last_state_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
