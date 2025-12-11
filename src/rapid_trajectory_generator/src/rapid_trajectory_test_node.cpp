#include "RapidTrajectoryGenerator.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

using RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator;

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
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  using namespace std::chrono_literals;

  class RapidTrajectoryStreamer : public rclcpp::Node
  {
  public:
    RapidTrajectoryStreamer()
    : rclcpp::Node("rapid_trajectory_test_node"),
      gravity_(0.0, 0.0, -9.81),
      sample_period_(1.0 / 20.0),  // 20 Hz output of pos/vel/acc
      cruise_speed_(1.5),          // nominal pass-through speed
      min_through_speed_(0.5),
      max_through_speed_(3.0)
    {
      sample_timer_ = create_wall_timer(
        std::chrono::duration<double>(sample_period_),
        std::bind(&RapidTrajectoryStreamer::onSampleTimer, this));

      // 示例：预填充多个目标点，确保起步时就有前瞻，避免第一段刹停
      enqueueTarget(::Vec3(1.0, 0.0, 0.5));
      enqueueTarget(::Vec3(1.5, 1.5, 1.0));
      enqueueTarget(::Vec3(0.5, 2.0, 1.5));
      enqueueTarget(::Vec3(0.0, 0.0, 1.0));
    }

  private:
    struct StateSample
    {
      ::Vec3 position{0.0, 0.0, 0.0};
      ::Vec3 velocity{0.0, 0.0, 0.0};
      ::Vec3 acceleration{0.0, 0.0, 0.0};
    };

    void enqueueTarget(const ::Vec3 & target)
    {
      target_queue_.push_back(target);
    }

    void onSampleTimer()
    {
      const auto now = this->now();
      if (!active_traj_.has_value())
      {
        tryStartNextSegment(now);
      }

      if (!active_traj_.has_value())
      {
        return;
      }

      const double elapsed = std::max(0.0, (now - segment_start_time_).seconds());
      const double t = std::min(elapsed, segment_duration_);

      current_state_.position = active_traj_->GetPosition(t);
      current_state_.velocity = active_traj_->GetVelocity(t);
      current_state_.acceleration = active_traj_->GetAcceleration(t);

      RCLCPP_INFO(
        get_logger(),
        "t=%.2f pos=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f, %.2f) acc=(%.2f, %.2f, %.2f)",
        t,
        current_state_.position.x,
        current_state_.position.y,
        current_state_.position.z,
        current_state_.velocity.x,
        current_state_.velocity.y,
        current_state_.velocity.z,
        current_state_.acceleration.x,
        current_state_.acceleration.y,
        current_state_.acceleration.z);

      // 仅当规划段自然结束时才切换到下一个目标，保证不中途抢占
      if (elapsed >= segment_duration_)
      {
        tryStartNextSegment(now);
      }
    }

    void tryStartNextSegment(const rclcpp::Time & now)
    {
      if (target_queue_.empty())
      {
        return;
      }

      const bool first_segment = !active_traj_.has_value();
      if (first_segment && target_queue_.size() < 2)
      {
        // 没有前瞻就起步会导致末端速度被迫减到 0，等待更多点
        return;
      }

      const ::Vec3 target = target_queue_.front();
      const std::optional<::Vec3> next_target =
        target_queue_.size() >= 2 ? std::make_optional(target_queue_[1]) : std::nullopt;
      const std::optional<::Vec3> after_next =
        target_queue_.size() >= 3 ? std::make_optional(target_queue_[2]) : std::nullopt;

      startNewSegment(target, next_target, after_next, now);
      target_queue_.pop_front();
    }

    void startNewSegment(
      const ::Vec3 & target,
      const std::optional<::Vec3> & next_target,
      const std::optional<::Vec3> & after_next,
      const rclcpp::Time & now)
    {
      // 取当前时刻的实际状态作为起点，保证连续性
      ::Vec3 x0 = current_state_.position;
      ::Vec3 v0 = current_state_.velocity;
      ::Vec3 a0 = current_state_.acceleration;
      if (active_traj_.has_value())
      {
        const double elapsed = std::max(0.0, (now - segment_start_time_).seconds());
        const double t = std::min(elapsed, segment_duration_);
        x0 = active_traj_->GetPosition(t);
        v0 = active_traj_->GetVelocity(t);
        a0 = active_traj_->GetAcceleration(t);
      }

      active_traj_.emplace(x0, v0, a0, gravity_);
      active_traj_->SetGoalPosition(target);
      active_traj_->SetGoalVelocity(computeThroughVelocity(x0, target, next_target, after_next, v0));
      active_traj_->SetGoalAcceleration(::Vec3(0.0, 0.0, 0.0));

      const double tf = computeSegmentTime(target, x0);
      active_traj_->Generate(tf);

      const auto feasibility = active_traj_->CheckInputFeasibility(0.5, 15.0, 10.0, 0.05);
      RCLCPP_INFO(
        get_logger(),
        "New segment -> target(%.2f, %.2f, %.2f), tf=%.2f, feasibility=%s",
        target.x,
        target.y,
        target.z,
        tf,
        toString(feasibility).c_str());

      segment_start_time_ = now;
      segment_duration_ = tf;
    }

    double computeSegmentTime(const ::Vec3 & target, const ::Vec3 & start) const
    {
      const double distance = (target - start).GetNorm2();
      constexpr double min_time = 0.4;
      constexpr double max_time = 5.0;
      return std::clamp(distance / std::max(0.1, cruise_speed_), min_time, max_time);
    }

    ::Vec3 computeThroughVelocity(
      const ::Vec3 & start,
      const ::Vec3 & target,
      const std::optional<::Vec3> & next_target,
      const std::optional<::Vec3> & after_next,
      const ::Vec3 & entry_velocity) const
    {
      if (!next_target)
      {
        // 最后一个点减速停下
        return ::Vec3(0.0, 0.0, 0.0);
      }

      // 入射方向与出射方向做方向平滑，减少尖角减速
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
        // 提前看一个点，降低连续两个拐角的突变
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
      const double target_speed = std::clamp(
        entry_speed > 1e-3 ? entry_speed : cruise_speed_,
        min_through_speed_,
        max_through_speed_);

      return blended * target_speed;
    }

    StateSample current_state_;
    const ::Vec3 gravity_;
    const double sample_period_;
    const double cruise_speed_;
    const double min_through_speed_;
    const double max_through_speed_;
    std::optional<RapidTrajectoryGenerator> active_traj_;
    rclcpp::Time segment_start_time_{};
    double segment_duration_{0.0};
    std::deque<::Vec3> target_queue_;
    rclcpp::TimerBase::SharedPtr sample_timer_;
  };

  rclcpp::spin(std::make_shared<RapidTrajectoryStreamer>());
  rclcpp::shutdown();
  return 0;
}
