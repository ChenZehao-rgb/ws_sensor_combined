#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <traj_offboard/srv/set_target.hpp>

using namespace std::chrono_literals;

class UavOffboardFsm : public rclcpp::Node {
  public:
    UavOffboardFsm() : rclcpp::Node("uav_offboard_fsm") {
        // Control timer: pair OffboardControlMode with a setpoint
        timer_ = this->create_wall_timer(50ms, std::bind(&UavOffboardFsm::controlLoopOnTimer, this));
        offboard_state_pub_ = this->create_publisher<std_msgs::msg::String>("/uav_offboard_fsm/offboard_state", 10);

		set_target_client_ = this->create_client<traj_offboard::srv::SetTarget>("online_traj_generator/set_target");

		control_command_sub_ = this->create_subscription<std_msgs::msg::String>(
			"/uav_offboard_fsm/control_command", 10,
			std::bind(&UavOffboardFsm::handleControlCommand, this, std::placeholders::_1));

		ruckig_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"/online_traj_generator/ruckig_state", 10,
			std::bind(&UavOffboardFsm::handleRuckigState, this, std::placeholders::_1));
		current_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"/rapid_traj/current_state", 10,
			std::bind(&UavOffboardFsm::handleRuckigState, this, std::placeholders::_1));

		trajectory_waypoints_ = {
			{0.0, 0.0, 5.0, 0.0},
			{5.0, 0.0, 5.0, 0.0},
			{5.0, 5.0, 6.0, 1.57079632679},
			{0.0, 5.0, 6.0, 3.14159265359},
			{0.0, 0.0, 5.0, -1.57079632679}
		};
    }

  private:
	struct Waypoint {
		double x;
		double y;
		double z;
		double yaw;
	};

	static constexpr double kPositionTolerance = 0.2;
	static constexpr double kYawTolerance = 0.1;

    
    // Takeoff sequence state management
    enum class ControlState {
            SELFCHECK,
            TAKEOFF,
            GOTOPOINT,
			TRAJECTORY_TRACKING,
			CIRCLE_TRACKING
    };
    std::atomic<ControlState> control_state_{ControlState::SELFCHECK};
    ControlState previous_state_{ControlState::SELFCHECK};

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr offboard_state_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_command_sub_;
    rclcpp::Client<traj_offboard::srv::SetTarget>::SharedPtr set_target_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ruckig_state_sub_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_state_sub_;

	std::vector<Waypoint> trajectory_waypoints_;
	std::optional<sensor_msgs::msg::JointState> latest_state_;
	mutable std::mutex latest_state_mutex_;
    std::size_t current_waypoint_index_{0};
    bool waypoint_command_in_flight_{false};
    rclcpp::Time last_waypoint_sent_time_{};

	std::vector<Waypoint> circle_waypoints_;
	std::size_t current_circle_index_{0};
	bool circle_command_in_flight_{false};
	rclcpp::Time last_circle_sent_time_{};

	double circle_center_x_{0.0};
	double circle_center_y_{0.0};
	double circle_radius_{5.0};
	double circle_height_{5.0};
	int circle_points_{24};

    rclcpp::TimerBase::SharedPtr timer_;
    void controlLoopOnTimer();
	void publish_offboard_state();
	void set_target_once();
	void handleTrajectoryTracking(bool state_changed);
	void handleCircleTracking(bool state_changed);
	void resetTrajectoryTracking();
	void resetCircleTracking();
	bool isWaypointReached(const Waypoint &waypoint, const sensor_msgs::msg::JointState &state);
	void sendWaypointTarget(const Waypoint &waypoint);
	void sendCircleTarget(const Waypoint &waypoint, std::size_t waypoint_index);
	static double wrapAngle(double angle);
	static std::string controlStateToString(ControlState state);
	static std::optional<ControlState> controlStateFromString(const std::string &state);
	void handleControlCommand(const std_msgs::msg::String::SharedPtr msg);
	void handleRuckigState(const sensor_msgs::msg::JointState::SharedPtr msg);
};

void UavOffboardFsm::publish_offboard_state()
{
	auto msg = std_msgs::msg::String();
	msg.data = "TAKEOFF";
	offboard_state_pub_->publish(msg);
}
void UavOffboardFsm::set_target_once()
{
	if (!set_target_client_->service_is_ready()) {
		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Set target service not available…");
		return;
	}
	auto request = std::make_shared<traj_offboard::srv::SetTarget::Request>();
	request->target.position = {10.0, 10.0, 8.0}; // Example target position
	request->target.yaw = 0.0;

	auto result = set_target_client_->async_send_request(request, [this](rclcpp::Client<traj_offboard::srv::SetTarget>::SharedFuture resp_fut) {
		try {
			auto resp = resp_fut.get();
			if (!resp->success) {
				RCLCPP_ERROR(this->get_logger(), "Failed to send target to trajectory generator.");
			}
		} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
		}
	});
}
void UavOffboardFsm::controlLoopOnTimer() 
{
	const auto current_state = control_state_.load();
	const bool state_changed = current_state != previous_state_;
	switch (current_state)
	{
		case ControlState::SELFCHECK:
		{
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "State: SELFCHECK");
			// Perform self-checks here
			// If self-checks pass, transition to TAKEOFF
			break;
		}
		case ControlState::TAKEOFF:
		{
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "State: TAKEOFF");
			// Send takeoff command to UAV
			publish_offboard_state();
			// If takeoff is successful, transition to GOTOPOINT
			
			break;
		}
		case ControlState::GOTOPOINT:
		{
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "State: GOTOPOINT");
			// Send command to go to a specific point
			set_target_once();
			// After reaching the point, you might want to transition back or to another state
			// For this example, we'll just stay in GOTOPOINT
			break;
		}
		case ControlState::TRAJECTORY_TRACKING:
		{
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "State: TRAJECTORY_TRACKING");
			handleTrajectoryTracking(state_changed);
			break;
		}
		case ControlState::CIRCLE_TRACKING:
		{
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "State: CIRCLE_TRACKING");
			handleCircleTracking(state_changed);
			break;
		}
		default:
			break;
	}
	previous_state_ = current_state;
}

void UavOffboardFsm::handleTrajectoryTracking(bool state_changed)
{
	if (state_changed) {
		resetTrajectoryTracking();
	}

	if (trajectory_waypoints_.empty()) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No waypoints configured for trajectory tracking");
		return;
	}

	std::optional<sensor_msgs::msg::JointState> state_copy;
	{
		std::lock_guard<std::mutex> lock(latest_state_mutex_);
		state_copy = latest_state_;
	}

	if (!state_copy) {
		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for state feedback before sending trajectory");
		return;
	}

	if (current_waypoint_index_ >= trajectory_waypoints_.size()) {
		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Trajectory completed; holding last waypoint");
		return;
	}

	if (!waypoint_command_in_flight_) {
		sendWaypointTarget(trajectory_waypoints_[current_waypoint_index_]);
		return;
	}

	if (!isWaypointReached(trajectory_waypoints_[current_waypoint_index_], *state_copy)) {
		return;
	}

	RCLCPP_INFO(this->get_logger(),
			"Waypoint %zu reached (%.2f, %.2f, %.2f, %.2f rad)",
			current_waypoint_index_,
			trajectory_waypoints_[current_waypoint_index_].x,
			trajectory_waypoints_[current_waypoint_index_].y,
			trajectory_waypoints_[current_waypoint_index_].z,
			trajectory_waypoints_[current_waypoint_index_].yaw);
	current_waypoint_index_++;
	waypoint_command_in_flight_ = false;
	if (current_waypoint_index_ >= trajectory_waypoints_.size()) {
		RCLCPP_INFO(this->get_logger(), "All trajectory waypoints completed");
	}
}

void UavOffboardFsm::handleCircleTracking(bool state_changed)
{
	if (state_changed) {
		resetCircleTracking();
	}

	if (circle_waypoints_.empty()) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Circle waypoints not ready");
		return;
	}

	std::optional<sensor_msgs::msg::JointState> state_copy;
	{
		std::lock_guard<std::mutex> lock(latest_state_mutex_);
		state_copy = latest_state_;
	}

	// if (!state_copy) {
	// 	RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for state feedback before circle tracking");
	// 	return;
	// }

	if (current_circle_index_ >= circle_waypoints_.size()) {
		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Circle completed; holding last point");
		return;
	}

	if (!circle_command_in_flight_) {
		sendCircleTarget(circle_waypoints_[current_circle_index_], current_circle_index_);
		return;
	}

	if (!isWaypointReached(circle_waypoints_[current_circle_index_], *state_copy)) {
		return;
	}

	RCLCPP_INFO(this->get_logger(),
		"Circle waypoint %zu reached (%.2f, %.2f, %.2f, %.2f rad)",
		current_circle_index_,
		circle_waypoints_[current_circle_index_].x,
		circle_waypoints_[current_circle_index_].y,
		circle_waypoints_[current_circle_index_].z,
		circle_waypoints_[current_circle_index_].yaw);
	current_circle_index_++;
	circle_command_in_flight_ = false;
	if (current_circle_index_ >= circle_waypoints_.size()) {
		RCLCPP_INFO(this->get_logger(), "Circle trajectory completed");
	}
}

void UavOffboardFsm::resetTrajectoryTracking()
{
	current_waypoint_index_ = 0;
	waypoint_command_in_flight_ = false;
	last_waypoint_sent_time_ = this->now();
}

void UavOffboardFsm::resetCircleTracking()
{
	circle_waypoints_.clear();
	current_circle_index_ = 0;
	circle_command_in_flight_ = false;
	last_circle_sent_time_ = this->now();

	constexpr double kPi = 3.14159265358979323846;
	constexpr double kHalfPi = kPi / 2.0;
	for (int i = 0; i < circle_points_; ++i) {
		const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(circle_points_);
		const double x = circle_center_x_ + circle_radius_ * std::cos(angle);
		const double y = circle_center_y_ + circle_radius_ * std::sin(angle);
		const double yaw = wrapAngle(angle + kHalfPi);
		circle_waypoints_.push_back({x, y, circle_height_, yaw});
	}
	if (!circle_waypoints_.empty()) {
		circle_waypoints_.push_back(circle_waypoints_.front());
	}
}

bool UavOffboardFsm::isWaypointReached(const Waypoint &waypoint, const sensor_msgs::msg::JointState &state)
{
	if (state.position.size() < 4) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
			"Received joint state with insufficient position entries (size=%zu)", state.position.size());
		return false;
	}

	const double dx = waypoint.x - state.position[0];
	const double dy = waypoint.y - state.position[1];
	const double dz = waypoint.z - state.position[2];
	const double yaw_error = wrapAngle(waypoint.yaw - state.position[3]);

	return std::abs(dx) < kPositionTolerance &&
	       std::abs(dy) < kPositionTolerance &&
	       std::abs(dz) < kPositionTolerance &&
	       std::abs(yaw_error) < kYawTolerance;
}

void UavOffboardFsm::sendWaypointTarget(const Waypoint &waypoint)
{
	if (!set_target_client_->service_is_ready()) {
		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Set target service not available…");
		return;
	}

	auto request = std::make_shared<traj_offboard::srv::SetTarget::Request>();
	request->target.position = {
		static_cast<float>(waypoint.x),
		static_cast<float>(waypoint.y),
		static_cast<float>(waypoint.z)
	};
	request->target.velocity = {0.0f, 0.0f, 0.0f};
	request->target.acceleration = {0.0f, 0.0f, 0.0f};
	request->target.yaw = static_cast<float>(waypoint.yaw);
	request->target.yawspeed = 0.0f;

	const auto waypoint_index = current_waypoint_index_;
	waypoint_command_in_flight_ = true;
	last_waypoint_sent_time_ = this->now();

	set_target_client_->async_send_request(
		request,
		[this, waypoint_index](rclcpp::Client<traj_offboard::srv::SetTarget>::SharedFuture resp_fut) {
			try {
				auto resp = resp_fut.get();
				if (!resp->success) {
					RCLCPP_ERROR(this->get_logger(), "Failed to send waypoint %zu to trajectory generator", waypoint_index);
					waypoint_command_in_flight_ = false;
				}
				else {
					RCLCPP_INFO(this->get_logger(), "Waypoint %zu dispatched to trajectory generator", waypoint_index);
				}
			} catch (const std::exception &e) {
				RCLCPP_ERROR(this->get_logger(), "Service call failed for waypoint %zu: %s", waypoint_index, e.what());
				waypoint_command_in_flight_ = false;
			}
		});
}

void UavOffboardFsm::sendCircleTarget(const Waypoint &waypoint, std::size_t waypoint_index)
{
	if (!set_target_client_->service_is_ready()) {
		RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Set target service not available…");
		return;
	}

	auto request = std::make_shared<traj_offboard::srv::SetTarget::Request>();
	request->target.position = {
		static_cast<float>(waypoint.x),
		static_cast<float>(waypoint.y),
		static_cast<float>(waypoint.z)
	};
	request->target.velocity = {0.0f, 0.0f, 0.0f};
	request->target.acceleration = {0.0f, 0.0f, 0.0f};
	request->target.yaw = static_cast<float>(waypoint.yaw);
	request->target.yawspeed = 0.0f;

	circle_command_in_flight_ = true;
	last_circle_sent_time_ = this->now();

	set_target_client_->async_send_request(
		request,
		[this, waypoint_index](rclcpp::Client<traj_offboard::srv::SetTarget>::SharedFuture resp_fut) {
			try {
				auto resp = resp_fut.get();
				if (!resp->success) {
					RCLCPP_ERROR(this->get_logger(), "Failed to send circle waypoint %zu to trajectory generator", waypoint_index);
					circle_command_in_flight_ = false;
				}
				else {
					RCLCPP_INFO(this->get_logger(), "Circle waypoint %zu dispatched to trajectory generator", waypoint_index);
				}
			} catch (const std::exception &e) {
				RCLCPP_ERROR(this->get_logger(), "Service call failed for circle waypoint %zu: %s", waypoint_index, e.what());
				circle_command_in_flight_ = false;
			}
		});
}

double UavOffboardFsm::wrapAngle(double angle)
{
	return std::atan2(std::sin(angle), std::cos(angle));
}

std::string UavOffboardFsm::controlStateToString(ControlState state)
{
	switch (state)
	{
		case ControlState::SELFCHECK:
			return "SELFCHECK";
		case ControlState::TAKEOFF:
			return "TAKEOFF";
		case ControlState::GOTOPOINT:
			return "GOTOPOINT";
		case ControlState::TRAJECTORY_TRACKING:
			return "TRAJECTORY_TRACKING";
		case ControlState::CIRCLE_TRACKING:
			return "CIRCLE_TRACKING";
	}
	return "UNKNOWN";
}

std::optional<UavOffboardFsm::ControlState> UavOffboardFsm::controlStateFromString(const std::string &state)
{
	std::string upper = state;
	std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) {
		return static_cast<char>(std::toupper(c));
	});
	if (upper == "SELFCHECK" || upper == "S") {
		return ControlState::SELFCHECK;
	}
	if (upper == "TAKEOFF" || upper == "T") {
		return ControlState::TAKEOFF;
	}
	if (upper == "GOTOPOINT" || upper == "G") {
		return ControlState::GOTOPOINT;
	}
	if (upper == "TRAJECTORY_TRACKING" || upper == "R") {
		return ControlState::TRAJECTORY_TRACKING;
	}
	if (upper == "CIRCLE_TRACKING" || upper == "CIRCLE" || upper == "C") {
		return ControlState::CIRCLE_TRACKING;
	}
	return std::nullopt;
}

void UavOffboardFsm::handleControlCommand(const std_msgs::msg::String::SharedPtr msg)
{
	const auto maybe_state = controlStateFromString(msg->data);
	if (!maybe_state) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
			"Unknown control command received: %s", msg->data.c_str());
		return;
	}
	const auto new_state = *maybe_state;
	const auto current_state = control_state_.load();
	if (new_state == current_state) {
		return;
	}
	control_state_.store(new_state);
	RCLCPP_INFO(this->get_logger(), "Control state set via command: %s",
		controlStateToString(new_state).c_str());
}

void UavOffboardFsm::handleRuckigState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(latest_state_mutex_);
	latest_state_ = *msg;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UavOffboardFsm>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), /*num_threads=*/2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
