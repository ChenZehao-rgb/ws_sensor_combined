#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <functional>
#include <optional>
#include <std_msgs/msg/string.hpp>
#include <string>

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
    }

  private:
    
    // Takeoff sequence state management
    enum class ControlState {
            SELFCHECK,
            TAKEOFF,
            GOTOPOINT,
    };
    std::atomic<ControlState> control_state_{ControlState::SELFCHECK};

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr offboard_state_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_command_sub_;
    rclcpp::Client<traj_offboard::srv::SetTarget>::SharedPtr set_target_client_;

    rclcpp::TimerBase::SharedPtr timer_;
    void controlLoopOnTimer();
	void publish_offboard_state();
	void set_target_once();
	static std::string controlStateToString(ControlState state);
	static std::optional<ControlState> controlStateFromString(const std::string &state);
	void handleControlCommand(const std_msgs::msg::String::SharedPtr msg);
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
			if (resp->success) {
				RCLCPP_INFO(this->get_logger(), "Successfully sent target to trajectory generator.");
			} else {
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
		default:
			break;
	}
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UavOffboardFsm>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), /*num_threads=*/2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
