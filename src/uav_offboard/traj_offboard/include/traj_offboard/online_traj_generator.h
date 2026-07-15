#ifndef TRAJ_GENERATOR_H
#define TRAJ_GENERATOR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>

#if __has_include(<ruckig/ruckig.hpp>)
#  include <ruckig/ruckig.hpp>
#  define TRAJ_OFFBOARD_HAVE_RUCKIG 1
#else
#  define TRAJ_OFFBOARD_HAVE_RUCKIG 0
#endif

#define YAW_VEL_LIMIT 0.2
#define YAW_ACC_LIMIT 0.2
#define YAW_JERK_LIMIT 0.2

namespace traj_generator {

// Problem dimensionality (x, y, z, yaw)
constexpr int STATE_NUM = 4;
constexpr double Ts = 0.05;  // s
constexpr double POS_ERROR_TOLERATE = 0.05;
constexpr double YAW_ERROR_TOLERATE = 0.05;

const std::vector<double> VEL_LIMIT = {0.2, 0.2, 0.1, YAW_VEL_LIMIT};
const std::vector<double> ACC_LIMIT = {1.0, 1.0, 1.0, YAW_ACC_LIMIT};
const std::vector<double> JERK_LIMIT = {0.8, 0.8, 0.8, YAW_JERK_LIMIT};

class TrajGenerator {
#if TRAJ_OFFBOARD_HAVE_RUCKIG
  template <class VectorT>
  static std::string formatVector(const VectorT &values) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4) << "[";
    for (std::size_t id = 0; id < STATE_NUM; id++) {
      if (id > 0) {
        stream << ", ";
      }
      stream << values[id];
    }
    stream << "]";
    return stream.str();
  }

  static const char *resultName(ruckig::Result res) {
    switch (res) {
      case ruckig::Result::Error:
        return "Error";
      case ruckig::Result::ErrorInvalidInput:
        return "ErrorInvalidInput";
      case ruckig::Result::ErrorTrajectoryDuration:
        return "ErrorTrajectoryDuration";
      case ruckig::Result::ErrorPositionalLimits:
        return "ErrorPositionalLimits";
      case ruckig::Result::ErrorExecutionTimeCalculation:
        return "ErrorExecutionTimeCalculation";
      case ruckig::Result::ErrorSynchronizationCalculation:
        return "ErrorSynchronizationCalculation";
      default:
        return "Unknown";
    }
  }
#endif

public:
  TrajGenerator() {
    state_.name = {"pos_x", "pos_y", "pos_z", "yaw"};
    state_.position = {0.0, 0.0, 0.0, 0.0};
    state_.velocity = {0.0, 0.0, 0.0, 0.0};
    state_.effort = {0.0, 0.0, 0.0, 0.0};

    command_.name = {"pos_x", "pos_y", "pos_z", "yaw"};
    command_.position = {0.0, 0.0, 0.0, 0.0};
    command_.velocity = {0.0, 0.0, 0.0, 0.0};
    command_.effort = {0.0, 0.0, 0.0, 0.0};

    targ_.name = {"pos_x", "pos_y", "pos_z", "yaw"};
    targ_.position = {0.0, 0.0, 0.0, 0.0};
    targ_.velocity = {0.0, 0.0, 0.0, 0.0};
    targ_.effort = {0.0, 0.0, 0.0, 0.0};

    initRuckig();
#if TRAJ_OFFBOARD_HAVE_RUCKIG
    RCLCPP_INFO(rclcpp::get_logger("traj_offboard"), "Ruckig OTG enabled (STATE_NUM=%d, Ts=%.3f s)", STATE_NUM, Ts);
#else
    RCLCPP_WARN(rclcpp::get_logger("traj_offboard"), "Ruckig headers not found; using simple fallback trajectory generator");
#endif
  }

  virtual ~TrajGenerator() = default;

  void initRuckig() {
#if TRAJ_OFFBOARD_HAVE_RUCKIG
    for (std::size_t id = 0; id < STATE_NUM; id++) {
      ruckigInput_.max_velocity[id] = VEL_LIMIT[id];
      ruckigInput_.max_acceleration[id] = ACC_LIMIT[id];
      ruckigInput_.max_jerk[id] = JERK_LIMIT[id];
    }

    for (std::size_t id = 0; id < STATE_NUM; id++) {
      ruckigInput_.current_position[id] = 0.0;
      ruckigInput_.current_velocity[id] = 0.0;
      ruckigInput_.current_acceleration[id] = 0.0;
    }

    for (std::size_t id = 0; id < STATE_NUM; id++) {
      ruckigInput_.target_position[id] = 0.0;
      ruckigInput_.target_velocity[id] = 0.0;
      ruckigInput_.target_acceleration[id] = 0.0;
    }
#endif
  }

  bool trajGenerate() {
#if TRAJ_OFFBOARD_HAVE_RUCKIG
    ruckig::Result res = ruckigOtg_.update(ruckigInput_, ruckigOutput_);
    if (res == ruckig::Result::Working || res == ruckig::Result::Finished) {
      for (std::size_t id = 0; id < STATE_NUM; id++) {
        command_.position[id] = ruckigOutput_.new_position[id];
        command_.velocity[id] = ruckigOutput_.new_velocity[id];
        command_.effort[id] = ruckigOutput_.new_acceleration[id];
      }
      ruckigOutput_.pass_to_input(ruckigInput_);
      return true;
    } else {
      auto logger = rclcpp::get_logger("traj_offboard");
      RCLCPP_ERROR(logger,
                   "TrajGenerator::trajGenerate: ruckig res is '%s' | current_p=%s current_v=%s current_a=%s target_p=%s target_v=%s target_a=%s",
                   resultName(res),
                   formatVector(ruckigInput_.current_position).c_str(),
                   formatVector(ruckigInput_.current_velocity).c_str(),
                   formatVector(ruckigInput_.current_acceleration).c_str(),
                   formatVector(ruckigInput_.target_position).c_str(),
                   formatVector(ruckigInput_.target_velocity).c_str(),
                   formatVector(ruckigInput_.target_acceleration).c_str());
      return false;
    }
#else
    // Fallback: simple first-order step towards target obeying velocity limits
    const double dt = Ts;
    for (std::size_t id = 0; id < STATE_NUM; id++) {
      const double cur = state_.position[id];
      const double tar = targ_.position[id];
      const double max_step = VEL_LIMIT[id] * dt;
      double diff = tar - cur;
      const double step = std::clamp(diff, -max_step, max_step);
      command_.position[id] = cur + step;
      command_.velocity[id] = step / dt;
      command_.effort[id] = 0.0;
    }
    return true;
#endif
  }

  // States, Commands, Targets (for debugging/telemetry)
  sensor_msgs::msg::JointState state_;
  sensor_msgs::msg::JointState command_;
  sensor_msgs::msg::JointState targ_;

protected:
#if TRAJ_OFFBOARD_HAVE_RUCKIG
  ruckig::Ruckig<STATE_NUM> ruckigOtg_{Ts};
  ruckig::InputParameter<STATE_NUM> ruckigInput_;
  ruckig::OutputParameter<STATE_NUM> ruckigOutput_;
#endif
};

}  // namespace traj_generator

#endif
