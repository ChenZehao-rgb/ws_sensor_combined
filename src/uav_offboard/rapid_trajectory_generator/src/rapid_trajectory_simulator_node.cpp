#include "RapidTrajectoryGenerator.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <traj_offboard/srv/set_target.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator;
using namespace std::chrono_literals;

namespace
{
std::string toString(RapidTrajectoryGenerator::InputFeasibilityResult result)
{
  switch (result)
  {
    case RapidTrajectoryGenerator::InputFeasible:
      return "feasible";
    case RapidTrajectoryGenerator::InputIndeterminable:
      return "indeterminable";
    case RapidTrajectoryGenerator::InputInfeasibleThrustHigh:
      return "infeasible: thrust too high";
    case RapidTrajectoryGenerator::InputInfeasibleThrustLow:
      return "infeasible: thrust too low";
    case RapidTrajectoryGenerator::InputInfeasibleRates:
      return "infeasible: body rates too high";
    default:
      return "unknown";
  }
}

static inline void quat2RPY(const geometry_msgs::msg::Quaternion & quat, double & roll,
                            double & pitch, double & yaw)
{
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}
}  // namespace

class RapidTrajectorySimulator : public rclcpp::Node
{
public:
  RapidTrajectorySimulator()
  : rclcpp::Node("rapid_trajectory_simulator"),
    gravity_(0.0, 0.0, -9.81),
    sample_period_(1.0 / 20.0),
    cruise_speed_(1.5),
    min_through_speed_(0.5),
    max_through_speed_(3.0)
  {
    // Initialize offboard state and PX4 interfaces
    offboard_state_.data = "WAITINGFORCOMMAND";
    offboard_ctrl_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    traj_setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_cmd_pub_ =
      this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    // State subscriptions from PX4
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_profile.depth = 5;
    auto qos =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

    vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      std::bind(&RapidTrajectorySimulator::VehicleLocalPositionCallback, this, std::placeholders::_1));
    vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", qos,
      std::bind(&RapidTrajectorySimulator::VehicleAttitudeCallback, this, std::placeholders::_1));
    vehicle_imu_sub_ = this->create_subscription<px4_msgs::msg::VehicleImu>(
      "/fmu/out/vehicle_imu", qos,
      std::bind(&RapidTrajectorySimulator::VehicleImuCallback, this, std::placeholders::_1));
    vehicle_home_position_sub_ = this->create_subscription<px4_msgs::msg::HomePosition>(
      "/fmu/out/home_position", qos,
      std::bind(&RapidTrajectorySimulator::VehicleHomePositionCallback, this, std::placeholders::_1));
    offboard_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/uav_offboard_fsm/offboard_state", 10,
      std::bind(&RapidTrajectorySimulator::OffboardStateCallback, this, std::placeholders::_1));

    // Service to accept external target commands (from FSM)
    set_target_srv_ = this->create_service<traj_offboard::srv::SetTarget>(
      "online_traj_generator/set_target",
      std::bind(&RapidTrajectorySimulator::handle_set_target, this, std::placeholders::_1,
                std::placeholders::_2));

    // Debug publisher for planned states
    current_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/rapid_traj/current_state", 10);

    // Timer driving offboard control and trajectory publishing
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(sample_period_),
      std::bind(&RapidTrajectorySimulator::controlLoopOnTimer, this));
  }

private:
  struct StateSample
  {
    ::Vec3 position{0.0, 0.0, 0.0};
    ::Vec3 velocity{0.0, 0.0, 0.0};
    ::Vec3 acceleration{0.0, 0.0, 0.0};
  };

  // Offboard state holders
  geometry_msgs::msg::PoseStamped uav_pose_;
  sensor_msgs::msg::Imu uav_imu_;
  px4_msgs::msg::HomePosition uav_home_position_;
  std_msgs::msg::String offboard_state_;
  px4_msgs::msg::TrajectorySetpoint target_pose_{};
  bool has_target_{false};
  bool has_home_{false};
  uint64_t offboard_setpoint_counter_{0U};
  px4_msgs::msg::TrajectorySetpoint last_cmd_{};
  rclcpp::Time last_cmd_time_{};

  // Takeoff/flight state
  enum class FlightState
  {
    WAITINGFORCOMMAND,
    TAKEOFF,
    TRAJECTORY_FOLLOWING
  };
  FlightState flight_state_{FlightState::WAITINGFORCOMMAND};
  bool takeoff_complete_{false};
  static constexpr float TAKEOFF_HEIGHT = 5.0f;
  static constexpr float POSITION_TOLERANCE = 0.2f;

  // Trajectory planner state
  StateSample current_state_{};
  sensor_msgs::msg::JointState current_point_pub_;
  const ::Vec3 gravity_;
  const double sample_period_;
  const double cruise_speed_;
  const double min_through_speed_;
  const double max_through_speed_;
  std::optional<RapidTrajectoryGenerator> active_traj_;
  rclcpp::Time segment_start_time_{};
  double segment_duration_{0.0};
  std::deque<::Vec3> target_queue_;
  double commanded_yaw_{0.0};

  // ROS interfaces
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleImu>::SharedPtr vehicle_imu_sub_;
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr vehicle_home_position_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr offboard_state_sub_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr current_state_pub_;

  rclcpp::Service<traj_offboard::srv::SetTarget>::SharedPtr set_target_srv_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  // PX4 state callbacks
  void VehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    uav_pose_.header.stamp = this->now();
    uav_pose_.header.frame_id = "map";  // ENU frame
    uav_pose_.pose.position.x = msg->y - uav_home_position_.y;
    uav_pose_.pose.position.y = msg->x - uav_home_position_.x;
    uav_pose_.pose.position.z = -msg->z + uav_home_position_.z;
  }

  void VehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    uav_pose_.pose.orientation.x = msg->q[1];
    uav_pose_.pose.orientation.y = msg->q[0];
    uav_pose_.pose.orientation.z = -msg->q[2];
    uav_pose_.pose.orientation.w = msg->q[3];
  }

  void VehicleImuCallback(const px4_msgs::msg::VehicleImu::SharedPtr msg)
  {
    uav_imu_.header.stamp = this->now();
    uav_imu_.header.frame_id = "base_link";  // NED frame
    uav_imu_.angular_velocity.x = msg->delta_velocity[1];
    uav_imu_.angular_velocity.y = msg->delta_velocity[0];
    uav_imu_.angular_velocity.z = -msg->delta_velocity[2];
    uav_imu_.linear_acceleration.x = msg->delta_angle[1];
    uav_imu_.linear_acceleration.y = msg->delta_angle[0];
    uav_imu_.linear_acceleration.z = -msg->delta_angle[2];
  }

  void VehicleHomePositionCallback(const px4_msgs::msg::HomePosition::SharedPtr msg)
  {
    uav_home_position_.timestamp = msg->timestamp;
    uav_home_position_.x = msg->x;
    uav_home_position_.y = msg->y;
    uav_home_position_.z = msg->z;
    has_home_ = true;
    RCLCPP_INFO(this->get_logger(), "Received home position: [%.2f, %.2f, %.2f]", uav_home_position_.x,
                uav_home_position_.y, uav_home_position_.z);
  }

  void OffboardStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    offboard_state_ = *msg;
  }

  // Offboard helpers
  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_ctrl_mode_pub_->publish(msg);
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_cmd_pub_->publish(msg);
  }

  void handle_set_target(const traj_offboard::srv::SetTarget::Request::SharedPtr request,
                         traj_offboard::srv::SetTarget::Response::SharedPtr response)
  {
    target_pose_ = request->target;
    has_target_ = true;
    commanded_yaw_ = target_pose_.yaw;
    enqueueTarget(::Vec3(target_pose_.position[0], target_pose_.position[1], target_pose_.position[2]));
    response->success = true;
    RCLCPP_INFO(get_logger(), "Received new target: (%.2f, %.2f, %.2f) yaw=%.2f",
                target_pose_.position[0], target_pose_.position[1], target_pose_.position[2],
                commanded_yaw_);
  }

  px4_msgs::msg::TrajectorySetpoint convertENUToNED(
    const px4_msgs::msg::TrajectorySetpoint & enu_setpoint) const
  {
    px4_msgs::msg::TrajectorySetpoint ned_setpoint = enu_setpoint;

    ned_setpoint.position[0] = enu_setpoint.position[1] + uav_home_position_.x;
    ned_setpoint.position[1] = enu_setpoint.position[0] + uav_home_position_.y;
    ned_setpoint.position[2] = -enu_setpoint.position[2] + uav_home_position_.z;

    ned_setpoint.velocity[0] = enu_setpoint.velocity[1];
    ned_setpoint.velocity[1] = enu_setpoint.velocity[0];
    ned_setpoint.velocity[2] = -enu_setpoint.velocity[2];

    ned_setpoint.acceleration[0] = enu_setpoint.acceleration[1];
    ned_setpoint.acceleration[1] = enu_setpoint.acceleration[0];
    ned_setpoint.acceleration[2] = -enu_setpoint.acceleration[2];

    ned_setpoint.jerk[0] = enu_setpoint.jerk[1];
    ned_setpoint.jerk[1] = enu_setpoint.jerk[0];
    ned_setpoint.jerk[2] = -enu_setpoint.jerk[2];

    static constexpr float HALF_PI = 1.57079632679f;
    const float yaw_ned = HALF_PI - enu_setpoint.yaw;
    ned_setpoint.yaw = std::atan2(std::sin(yaw_ned), std::cos(yaw_ned));
    ned_setpoint.yawspeed = -enu_setpoint.yawspeed;

    return ned_setpoint;
  }

  px4_msgs::msg::TrajectorySetpoint makePositionHoldSetpoint(float x, float y, float z,
                                                             float yaw) const
  {
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.position[0] = x;
    setpoint.position[1] = y;
    setpoint.position[2] = z;
    setpoint.yaw = yaw;
    return setpoint;
  }

  px4_msgs::msg::TrajectorySetpoint publishConvertedSetpoint(
    px4_msgs::msg::TrajectorySetpoint enu_setpoint)
  {
    enu_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    auto ned_setpoint = convertENUToNED(enu_setpoint);
    traj_setpoint_pub_->publish(ned_setpoint);
    return enu_setpoint;
  }

  bool isArrivedAtPosition(px4_msgs::msg::TrajectorySetpoint setpoint, float tolerance)
  {
    float pos_error_x = std::abs(uav_pose_.pose.position.x - setpoint.position[0]);
    float pos_error_y = std::abs(uav_pose_.pose.position.y - setpoint.position[1]);
    float pos_error_z = std::abs(uav_pose_.pose.position.z - setpoint.position[2]);

    return pos_error_x < tolerance && pos_error_y < tolerance && pos_error_z < tolerance;
  }

  // Trajectory planner helpers
  void enqueueTarget(const ::Vec3 & target)
  {
    target_queue_.push_back(target);
  }

  void tryStartNextSegment(const rclcpp::Time & now)
  {
    if (target_queue_.empty())
    {
      if (!active_traj_)
      {
        has_target_ = false;
      }
      return;
    }

    const ::Vec3 target = target_queue_.front();
    const std::optional<::Vec3> next_target =
      target_queue_.size() >= 2 ? std::make_optional(target_queue_[1]) : std::nullopt;
    const std::optional<::Vec3> after_next =
      target_queue_.size() >= 3 ? std::make_optional(target_queue_[2]) : std::nullopt;

    ::Vec3 x0 = current_state_.position;
    ::Vec3 v0 = current_state_.velocity;
    ::Vec3 a0 = current_state_.acceleration;

    if (active_traj_)
    {
      const double elapsed = std::max(0.0, (now - segment_start_time_).seconds());
      const double t = std::min(elapsed, segment_duration_);
      x0 = active_traj_->GetPosition(t);
      v0 = active_traj_->GetVelocity(t);
      a0 = active_traj_->GetAcceleration(t);
    }
    else
    {
      // Use the latest measured pose as start when no active segment
      x0 = ::Vec3(uav_pose_.pose.position.x, uav_pose_.pose.position.y, uav_pose_.pose.position.z);
    }

    active_traj_.emplace(x0, v0, a0, gravity_);
    active_traj_->SetGoalPosition(target);
    active_traj_->SetGoalVelocity(computeThroughVelocity(x0, target, next_target, after_next, v0));
    active_traj_->SetGoalAcceleration(::Vec3(0.0, 0.0, 0.0));

    const double tf = computeSegmentTime(target, x0);
    active_traj_->Generate(tf);

    const auto feasibility = active_traj_->CheckInputFeasibility(0.5, 15.0, 10.0, 0.05);
    RCLCPP_INFO(get_logger(),
                "New segment -> target(%.2f, %.2f, %.2f), tf=%.2f, feasibility=%s", target.x,
                target.y, target.z, tf, toString(feasibility).c_str());

    segment_start_time_ = now;
    segment_duration_ = tf;
    target_queue_.pop_front();
  }

  double computeSegmentTime(const ::Vec3 & target, const ::Vec3 & start) const
  {
    const double distance = (target - start).GetNorm2();
    constexpr double min_time = 0.4;
    constexpr double max_time = 5.0;
    return std::clamp(distance / std::max(0.1, cruise_speed_), min_time, max_time);
  }

  ::Vec3 computeThroughVelocity(const ::Vec3 & start, const ::Vec3 & target,
                                const std::optional<::Vec3> & next_target,
                                const std::optional<::Vec3> & after_next,
                                const ::Vec3 & entry_velocity) const
  {
    if (!next_target)
    {
      return ::Vec3(0.0, 0.0, 0.0);
    }

    ::Vec3 in_dir = target - start;
    double in_norm = in_dir.GetNorm2();
    if (in_norm > 1e-3)
    {
      in_dir = (1.0 / in_norm) * in_dir;
    }

    const ::Vec3 dir = *next_target - target;
    double dist = dir.GetNorm2();
    if (dist < 1e-3)
    {
      return ::Vec3(0.0, 0.0, 0.0);
    }

    ::Vec3 out_dir = (1.0 / dist) * dir;
    if (after_next)
    {
      const ::Vec3 next_leg = *after_next - *next_target;
      const double next_leg_norm = next_leg.GetNorm2();
      if (next_leg_norm > 1e-3)
      {
        out_dir = (0.5 * out_dir) + (0.5 * ((1.0 / next_leg_norm) * next_leg));
      }
    }

    ::Vec3 blended = in_dir.GetNorm2() > 1e-3 ? in_dir + out_dir : out_dir;
    const double blended_norm = blended.GetNorm2();
    if (blended_norm < 1e-3)
    {
      blended = out_dir;
    }
    else
    {
      blended = (1.0 / blended_norm) * blended;
    }

    const double entry_speed = entry_velocity.GetNorm2();
    const double target_speed =
      std::clamp(entry_speed > 1e-3 ? entry_speed : cruise_speed_, min_through_speed_,
                 max_through_speed_);

    return blended * target_speed;
  }

  void updateTrajectorySample(const rclcpp::Time & now)
  {
    if (!active_traj_)
    {
      tryStartNextSegment(now);
      if (!active_traj_)
      {
        return;
      }
    }

    double elapsed = std::max(0.0, (now - segment_start_time_).seconds());
    if (elapsed >= segment_duration_)
    {
      tryStartNextSegment(now);
      if (!active_traj_)
      {
        return;
      }
      elapsed = std::max(0.0, (now - segment_start_time_).seconds());
    }

    const double t = std::min(elapsed, segment_duration_);
    current_state_.position = active_traj_->GetPosition(t);
    current_state_.velocity = active_traj_->GetVelocity(t);
    current_state_.acceleration = active_traj_->GetAcceleration(t);

    current_point_pub_.header.stamp = now;
    current_point_pub_.name = {"x",  "y",  "z",  "vx", "vy",
                               "vz", "ax", "ay", "az"};
    current_point_pub_.position = {current_state_.position.x, current_state_.position.y,
                                   current_state_.position.z};
    current_point_pub_.velocity = {current_state_.velocity.x, current_state_.velocity.y,
                                   current_state_.velocity.z};
    current_point_pub_.effort = {current_state_.acceleration.x, current_state_.acceleration.y,
                                 current_state_.acceleration.z};
    current_state_pub_->publish(current_point_pub_);
  }

  void publish_trajectory_setpoint()
  {
    const auto now = this->now();
    updateTrajectorySample(now);

    if (!active_traj_)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 3000,
                           "No active trajectory, holding position");
      auto hold_setpoint =
        makePositionHoldSetpoint(uav_pose_.pose.position.x, uav_pose_.pose.position.y,
                                 uav_pose_.pose.position.z, commanded_yaw_);
      last_cmd_ = publishConvertedSetpoint(hold_setpoint);
      last_cmd_time_ = now;
      return;
    }

    px4_msgs::msg::TrajectorySetpoint enu_setpoint{};
    enu_setpoint.position[0] = current_state_.position.x;
    enu_setpoint.position[1] = current_state_.position.y;
    enu_setpoint.position[2] = current_state_.position.z;
    enu_setpoint.velocity[0] = current_state_.velocity.x;
    enu_setpoint.velocity[1] = current_state_.velocity.y;
    enu_setpoint.velocity[2] = current_state_.velocity.z;
    enu_setpoint.acceleration[0] = current_state_.acceleration.x;
    enu_setpoint.acceleration[1] = current_state_.acceleration.y;
    enu_setpoint.acceleration[2] = current_state_.acceleration.z;
    enu_setpoint.yaw = commanded_yaw_;
    enu_setpoint.yawspeed = 0.0f;

    last_cmd_ = publishConvertedSetpoint(enu_setpoint);
    last_cmd_time_ = now;
  }

  void publish_takeoff_setpoint(px4_msgs::msg::TrajectorySetpoint takeoff_setpoint)
  {
    takeoff_setpoint.yaw = commanded_yaw_;
    last_cmd_ = publishConvertedSetpoint(takeoff_setpoint);
    last_cmd_time_ = this->now();
  }

  void controlLoopOnTimer()
  {
    publish_offboard_control_mode();

    switch (flight_state_)
    {
      case FlightState::WAITINGFORCOMMAND:
      {
        if (offboard_state_.data == "TAKEOFF")
        {
          flight_state_ = FlightState::TAKEOFF;
          RCLCPP_INFO(get_logger(), "Received TAKEOFF command, initiating takeoff sequence...");
        }
        else
        {
          RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 3000,
                               "Waiting for TAKEOFF command...");
        }
        break;
      }
      case FlightState::TAKEOFF:
      {
        if (offboard_setpoint_counter_ == 10)
        {
          publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
          publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                                  1.0f);
          RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                               "Offboard & arm commands sent");
        }
        if (offboard_setpoint_counter_ < 11)
        {
          ++offboard_setpoint_counter_;
        }

        auto HoverSetpoint = makePositionHoldSetpoint(0.0f, 0.0f, TAKEOFF_HEIGHT, commanded_yaw_);
        publish_takeoff_setpoint(HoverSetpoint);

        if (isArrivedAtPosition(HoverSetpoint, POSITION_TOLERANCE))
        {
          takeoff_complete_ = true;
          flight_state_ = FlightState::TRAJECTORY_FOLLOWING;
          RCLCPP_INFO(get_logger(),
                      "Takeoff complete! Hovering at (%.2f, %.2f, %.2f)",
                      uav_pose_.pose.position.x, uav_pose_.pose.position.y,
                      uav_pose_.pose.position.z);
        }
        else
        {
          RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "Taking off...");
        }
        break;
      }
      case FlightState::TRAJECTORY_FOLLOWING:
      {
        if (!has_target_ && target_queue_.empty() && !active_traj_)
        {
          RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 3000,
                               "Waiting for first target...");
          auto HoverSetpoint =
            makePositionHoldSetpoint(0.0f, 0.0f, TAKEOFF_HEIGHT, commanded_yaw_);
          publish_takeoff_setpoint(HoverSetpoint);
          break;
        }
        publish_trajectory_setpoint();
        break;
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RapidTrajectorySimulator>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
