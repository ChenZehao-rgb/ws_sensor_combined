// ROS 2 + PX4 compatible online trajectory generator node

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <traj_offboard/srv/get_trajectory_setpoint.hpp>

#include <cmath>

#include <traj_offboard/online_traj_generator.h>

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
  }

  void updateTrajGeneratorTarg() {
    targ_.position[0] = traj_target_.position[0];
    targ_.position[1] = traj_target_.position[1];
    targ_.position[2] = traj_target_.position[2];
    targ_.position[3] = traj_target_.yaw;

    targ_.velocity[0] = traj_target_.velocity[0];
    targ_.velocity[1] = traj_target_.velocity[1];
    targ_.velocity[2] = traj_target_.velocity[2];
    targ_.velocity[3] = traj_target_.yawspeed;

    targ_.effort[0] = traj_target_.acceleration[0];
    targ_.effort[1] = traj_target_.acceleration[1];
    targ_.effort[2] = traj_target_.acceleration[2];
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
    if (isFirstTraj_ || !hasLastCommand_) {
      current_state_ = request->current_state;
      isFirstTraj_ = false;
      RCLCPP_INFO(get_logger(), "OnlineTrajGenerator: first trajectory request received");
    } else {
      current_state_ = last_command_state_;
    }
    traj_target_ = request->target;

    updateTrajGeneratorState();

#if TRAJ_OFFBOARD_HAVE_RUCKIG
    for (std::size_t id = 0; id < STATE_NUM; id++) {
      ruckigInput_.current_position[id] = state_.position[id];
      // ruckigInput_.current_velocity[id] = state_.velocity[id];
      // ruckigInput_.current_acceleration[id] = state_.effort[id];
    }
#endif

    if (request->update_target) {
      updateTrajGeneratorTarg();
#if TRAJ_OFFBOARD_HAVE_RUCKIG
      for (std::size_t id = 0; id < STATE_NUM; id++) {
        ruckigInput_.target_position[id] = targ_.position[id];
        ruckigInput_.target_velocity[id] = targ_.velocity[id];
        ruckigInput_.target_acceleration[id] = targ_.effort[id];
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
