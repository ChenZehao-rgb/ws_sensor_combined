// ROS 2 + PX4 compatible online trajectory generator node

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <traj_offboard/srv/get_trajectory_setpoint.hpp>

#include <algorithm>
#include <cmath>

#include <traj_offboard/online_traj_generator.h>

// ANSI color helpers for terminal log highlights (green = node ready / first trajectory).
#define LOG_COLOR_GREEN "\033[1;32m"
#define LOG_COLOR_RESET "\033[0m"

using traj_generator::TrajGenerator;
using traj_generator::STATE_NUM;

class OnlineTrajGenerator : public rclcpp::Node, public TrajGenerator {
public:
	OnlineTrajGenerator()
		: rclcpp::Node("online_traj_generator"), TrajGenerator() {
		// Debug/telemetry publishers
		ruckig_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
			"/online_traj_generator/ruckig_state", 10);
		ruckig_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
			"/online_traj_generator/ruckig_command", 10);
		ruckig_targ_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
			"/online_traj_generator/ruckig_targ", 10);
		// Service to get current trajectory setpoints
		get_traj_setpoints_srv_ = this->create_service<traj_offboard::srv::GetTrajectorySetpoint>(
			"/online_traj_generator/get_trajectory_setpoints", std::bind(&OnlineTrajGenerator::handleGetTrajSetpoints, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(),
                LOG_COLOR_GREEN "Online trajectory generator ready | service=/online_traj_generator/get_trajectory_setpoints debug_topics=/online_traj_generator/ruckig_*" LOG_COLOR_RESET);
	}

private:
  // State sources
  px4_msgs::msg::TrajectorySetpoint current_state_, traj_target_;
  px4_msgs::msg::TrajectorySetpoint last_command_state_;
  // ROS 2 interfaces

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ruckig_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ruckig_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ruckig_targ_pub_;

  rclcpp::Service<traj_offboard::srv::GetTrajectorySetpoint>::SharedPtr get_traj_setpoints_srv_;
  bool isFirstTraj_{true};
  bool hasLastCommand_{false};

#if TRAJ_OFFBOARD_HAVE_RUCKIG
  void seedRuckigCurrentFromState() {
    for (std::size_t id = 0; id < STATE_NUM; id++) {
      ruckigInput_.current_position[id] = state_.position[id];
      ruckigInput_.current_velocity[id] = state_.velocity[id];
      ruckigInput_.current_acceleration[id] = state_.effort[id];
    }
  }
#endif

  void updateTrajGeneratorState() {
    state_.position[0] = current_state_.position[0];
    state_.position[1] = current_state_.position[1];
    state_.position[2] = current_state_.position[2];
    state_.position[3] = current_state_.yaw;

    state_.velocity[0] = current_state_.velocity[0];
    state_.velocity[1] = current_state_.velocity[1];
    state_.velocity[2] = current_state_.velocity[2];
    state_.velocity[3] = current_state_.yawspeed;

    state_.effort[0] = current_state_.acceleration[0];
    state_.effort[1] = current_state_.acceleration[1];
    state_.effort[2] = current_state_.acceleration[2];
    state_.effort[3] = 0.0;
  }

  void updateTrajGeneratorTarg() {
    targ_.position[0] = traj_target_.position[0];
    targ_.position[1] = traj_target_.position[1];
    targ_.position[2] = traj_target_.position[2];
    // Ruckig 把 position[3] 当作普通标量插值，不会自动选最短弧。这里把目标 yaw 解包到
    // current yaw 的 ±π 邻域，强制走最短旋转方向，避免跨越 ±π 时走 >180° 长路径。
    const double yaw_delta = traj_target_.yaw - current_state_.yaw;
    const double yaw_wrapped = std::atan2(std::sin(yaw_delta), std::cos(yaw_delta));
    targ_.position[3] = current_state_.yaw + yaw_wrapped;

    targ_.velocity[0] = traj_target_.velocity[0];
    targ_.velocity[1] = traj_target_.velocity[1];
    targ_.velocity[2] = traj_target_.velocity[2];
    targ_.velocity[3] = traj_target_.yawspeed;

    targ_.effort[0] = traj_target_.acceleration[0];
    targ_.effort[1] = traj_target_.acceleration[1];
    targ_.effort[2] = traj_target_.acceleration[2];
    targ_.effort[3] = 0.0;
  }

  void updateTrajectorySetpointResponse(px4_msgs::msg::TrajectorySetpoint &traj_setpoint) {
    traj_setpoint.position[0] = command_.position[0];
    traj_setpoint.position[1] = command_.position[1];
    traj_setpoint.position[2] = command_.position[2];
    traj_setpoint.yaw = command_.position[3];

    traj_setpoint.velocity[0] = command_.velocity[0];
    traj_setpoint.velocity[1] = command_.velocity[1];
    traj_setpoint.velocity[2] = command_.velocity[2];
    traj_setpoint.yawspeed = command_.velocity[3];

    traj_setpoint.acceleration[0] = command_.effort[0];
    traj_setpoint.acceleration[1] = command_.effort[1];
    traj_setpoint.acceleration[2] = command_.effort[2];
  }

  void handleGetTrajSetpoints(const traj_offboard::srv::GetTrajectorySetpoint::Request::SharedPtr request,
								 traj_offboard::srv::GetTrajectorySetpoint::Response::SharedPtr response) {
    const bool seed_current_from_request =
        isFirstTraj_ || !hasLastCommand_ || request->reset_state;
    if (seed_current_from_request) {
      current_state_ = request->current_state;
      if (isFirstTraj_) {
        isFirstTraj_ = false;
        RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "Trajectory generator active | first request received" LOG_COLOR_RESET);
      } else {
        RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "Trajectory generator re-seeded from current state | reset_state requested" LOG_COLOR_RESET);
      }
    } else {
      current_state_ = last_command_state_;
    }
    traj_target_ = request->target;

    updateTrajGeneratorState();

#if TRAJ_OFFBOARD_HAVE_RUCKIG
    if (seed_current_from_request) {
      seedRuckigCurrentFromState();
    }
#endif

    if (request->update_target) {
      updateTrajGeneratorTarg();
#if TRAJ_OFFBOARD_HAVE_RUCKIG
      // 应用本段平动速度上限覆盖：分量 >0 时替换默认 VEL_LIMIT；否则恢复默认。yaw 始终保持默认。
      using traj_generator::VEL_LIMIT;
      for (std::size_t id = 0; id < 3; id++) {
        const double override_v = request->max_velocity_xyz[id];
        ruckigInput_.max_velocity[id] = (override_v > 0.0) ? override_v : VEL_LIMIT[id];
      }
      ruckigInput_.max_velocity[3] = VEL_LIMIT[3];

      bool target_kinematics_clamped = false;
      for (std::size_t id = 0; id < STATE_NUM; id++) {
        ruckigInput_.target_position[id] = targ_.position[id];
        const double raw_velocity =
            std::isfinite(targ_.velocity[id]) ? targ_.velocity[id] : 0.0;
        const double raw_acceleration =
            std::isfinite(targ_.effort[id]) ? targ_.effort[id] : 0.0;
        const double vel_limit = std::max(0.0, 0.99 * ruckigInput_.max_velocity[id]);
        const double acc_limit = std::max(0.0, 0.99 * ruckigInput_.max_acceleration[id]);
        const double target_velocity =
            std::clamp(raw_velocity, -vel_limit, vel_limit);
        const double target_acceleration =
            std::clamp(raw_acceleration, -acc_limit, acc_limit);
        target_kinematics_clamped =
            target_kinematics_clamped ||
            std::abs(target_velocity - raw_velocity) > 1e-9 ||
            std::abs(target_acceleration - raw_acceleration) > 1e-9 ||
            !std::isfinite(targ_.velocity[id]) ||
            !std::isfinite(targ_.effort[id]);
        ruckigInput_.target_velocity[id] = target_velocity;
        ruckigInput_.target_acceleration[id] = target_acceleration;
      }
      if (target_kinematics_clamped) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Ruckig target feed-forward clamped to current velocity/acceleration limits");
      }
#endif
    }
    if (!trajGenerate()) {
      response->success = false;
      RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 2000, "trajGenerate failed");
      return;
    }

    // Stamp and publish debug joint states
    const auto stamp = this->now();
    state_.header.stamp = stamp;
    targ_.header.stamp = stamp;
    command_.header.stamp = stamp;
    ruckig_state_pub_->publish(state_);
    ruckig_targ_pub_->publish(targ_);
    ruckig_command_pub_->publish(command_);

    updateTrajectorySetpointResponse(response->trajectory_setpoint);
    last_command_state_ = response->trajectory_setpoint;
    hasLastCommand_ = true;
    response->success = true;
    return;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OnlineTrajGenerator>());
  rclcpp::shutdown();
  return 0;
}
