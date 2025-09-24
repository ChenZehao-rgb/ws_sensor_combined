// Offboard control bridge that:
// - Subscribes to UAV state topics (pose/twist/imu)
// - Publishes PX4 OffboardControlMode, VehicleCommand, TrajectorySetpoint
// - Offers a service to set target position/yaw for OnlineTrajGenerator

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <traj_offboard/srv/get_trajectory_setpoint.hpp>
#include <traj_offboard/srv/set_target.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

using namespace std::chrono_literals;

static inline void quat2RPY(const geometry_msgs::msg::Quaternion &quat, double &roll,
                            double &pitch, double &yaw) {
	tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf2::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
}

class OffboardControlBridge : public rclcpp::Node {
  public:
    OffboardControlBridge() : rclcpp::Node("offboard_control_bridge") {
        // PX4 pubs
        offboard_ctrl_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        traj_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // UAV state subscriptions from px4_msgs (stored for reference)
        // QoS profile setting for PX4 compatibility - match PX4's exact settings
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos_profile.depth = 5;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos, std::bind(&OffboardControlBridge::VehicleLocalPositionCallback, this, std::placeholders::_1));
        vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos, std::bind(&OffboardControlBridge::VehicleAttitudeCallback, this, std::placeholders::_1));
        vehicle_imu_sub_ = this->create_subscription<px4_msgs::msg::VehicleImu>(
            "/fmu/out/vehicle_imu", qos, std::bind(&OffboardControlBridge::VehicleImuCallback, this, std::placeholders::_1));
        // Service to set target for online trajectory generator
        set_target_srv_ = this->create_service<traj_offboard::srv::SetTarget>(
            "online_traj_generator/set_target", std::bind(&OffboardControlBridge::handle_set_target, this, std::placeholders::_1, std::placeholders::_2));
        // Client to get current trajectory cmd from online trajectory generator
        client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        get_traj_setpoint_client_ = this->create_client<traj_offboard::srv::GetTrajectorySetpoint>(
            "online_traj_generator/get_trajectory_setpoints", rmw_qos_profile_services_default, client_callback_group_);

        // Control timer: pair OffboardControlMode with a setpoint
        timer_ = this->create_wall_timer(50ms, std::bind(&OffboardControlBridge::controlLoopOnTimer, this));
    }

  private:
    // State holders
    geometry_msgs::msg::PoseStamped uav_pose_;
    geometry_msgs::msg::TwistStamped uav_twist_;
    sensor_msgs::msg::Imu uav_imu_;
    px4_msgs::msg::TrajectorySetpoint target_pose_;
    bool update_target_{true}, has_target_{false};
    bool pending_request_{false};
    
    // Takeoff sequence state management
    enum class FlightState {
        TAKEOFF,
        TRAJECTORY_FOLLOWING
    };
    FlightState flight_state_{FlightState::TAKEOFF};
    bool takeoff_complete_{false};
    static constexpr float TAKEOFF_HEIGHT = 5.0f;
    static constexpr float POSITION_TOLERANCE = 0.2f;

    // ROS interfaces
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleImu>::SharedPtr vehicle_imu_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

    rclcpp::Service<traj_offboard::srv::SetTarget>::SharedPtr set_target_srv_;
    rclcpp::Client<traj_offboard::srv::GetTrajectorySetpoint>::SharedPtr get_traj_setpoint_client_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;

    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t offboard_setpoint_counter_{0U};
    px4_msgs::msg::TrajectorySetpoint last_cmd_{};
    rclcpp::Time last_cmd_time_{};

    void VehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
	void VehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
	void VehicleImuCallback(const px4_msgs::msg::VehicleImu::SharedPtr msg);
	void publish_offboard_control_mode();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    void handle_set_target(const traj_offboard::srv::SetTarget::Request::SharedPtr request,
                           traj_offboard::srv::SetTarget::Response::SharedPtr response);
    void publish_trajectory_setpoint();
    px4_msgs::msg::TrajectorySetpoint convertENUToNED(const px4_msgs::msg::TrajectorySetpoint &enu_setpoint) const;
    px4_msgs::msg::TrajectorySetpoint makePositionHoldSetpoint(float x, float y, float z, float yaw) const;
    px4_msgs::msg::TrajectorySetpoint publishConvertedSetpoint(px4_msgs::msg::TrajectorySetpoint enu_setpoint);
    bool isArrivedAtPosition(px4_msgs::msg::TrajectorySetpoint setpoint, float tolerance);
    void controlLoopOnTimer();
    void publish_takeoff_setpoint(px4_msgs::msg::TrajectorySetpoint takeoff_setpoint);
};

void OffboardControlBridge::VehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    uav_pose_.header.stamp = this->now();
    uav_pose_.header.frame_id = "map"; // ENU frame
    // PX4 NED to ROS ENU frame
    uav_pose_.pose.position.x = msg->y;
    uav_pose_.pose.position.y = msg->x;
    uav_pose_.pose.position.z = -msg->z;
}
void OffboardControlBridge::VehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    // PX4 NED to ROS ENU frame
	uav_pose_.pose.orientation.x = msg->q[1];
	uav_pose_.pose.orientation.y = msg->q[0];
	uav_pose_.pose.orientation.z = -msg->q[2];
	uav_pose_.pose.orientation.w = msg->q[3];
}
void OffboardControlBridge::VehicleImuCallback(const px4_msgs::msg::VehicleImu::SharedPtr msg) {
	uav_imu_.header.stamp = this->now();
	uav_imu_.header.frame_id = "base_link"; // NED frame
	// PX4 NED to ROS ENU frame
	uav_imu_.angular_velocity.x = msg->delta_velocity[1];
	uav_imu_.angular_velocity.y = msg->delta_velocity[0];
	uav_imu_.angular_velocity.z = -msg->delta_velocity[2];
	uav_imu_.linear_acceleration.x = msg->delta_angle[1];
	uav_imu_.linear_acceleration.y = msg->delta_angle[0];
	uav_imu_.linear_acceleration.z = -msg->delta_angle[2];
}
void OffboardControlBridge::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_ctrl_mode_pub_->publish(msg);
}

void OffboardControlBridge::handle_set_target(const traj_offboard::srv::SetTarget::Request::SharedPtr request,
                                              traj_offboard::srv::SetTarget::Response::SharedPtr response) {
    target_pose_ = request->target;
    response->success = true;
    update_target_ = true;
    has_target_ = true;
}

void OffboardControlBridge::publish_vehicle_command(uint16_t command, float param1, float param2) {
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

px4_msgs::msg::TrajectorySetpoint OffboardControlBridge::convertENUToNED(const px4_msgs::msg::TrajectorySetpoint &enu_setpoint) const {
    px4_msgs::msg::TrajectorySetpoint ned_setpoint = enu_setpoint;

    // Position
    ned_setpoint.position[0] = enu_setpoint.position[1];
    ned_setpoint.position[1] = enu_setpoint.position[0];
    ned_setpoint.position[2] = -enu_setpoint.position[2];

    // Velocity
    ned_setpoint.velocity[0] = enu_setpoint.velocity[1];
    ned_setpoint.velocity[1] = enu_setpoint.velocity[0];
    ned_setpoint.velocity[2] = -enu_setpoint.velocity[2];

    // Acceleration
    ned_setpoint.acceleration[0] = enu_setpoint.acceleration[1];
    ned_setpoint.acceleration[1] = enu_setpoint.acceleration[0];
    ned_setpoint.acceleration[2] = -enu_setpoint.acceleration[2];

    // Jerk
    ned_setpoint.jerk[0] = enu_setpoint.jerk[1];
    ned_setpoint.jerk[1] = enu_setpoint.jerk[0];
    ned_setpoint.jerk[2] = -enu_setpoint.jerk[2];

    static constexpr float HALF_PI = 1.57079632679f;
    const float yaw_ned = HALF_PI - enu_setpoint.yaw;
    ned_setpoint.yaw = std::atan2(std::sin(yaw_ned), std::cos(yaw_ned));
    ned_setpoint.yawspeed = -enu_setpoint.yawspeed;

    return ned_setpoint;
}
px4_msgs::msg::TrajectorySetpoint OffboardControlBridge::makePositionHoldSetpoint(float x, float y, float z, float yaw) const {
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.position[0] = x;
    setpoint.position[1] = y;
    setpoint.position[2] = z;
    // setpoint.velocity[0] = 0.0f;
    // setpoint.velocity[1] = 0.0f;
    // setpoint.velocity[2] = 0.0f;
    // setpoint.acceleration[0] = 0.0f;
    // setpoint.acceleration[1] = 0.0f;
    // setpoint.acceleration[2] = 0.0f;
    // setpoint.jerk[0] = 0.0f;
    // setpoint.jerk[1] = 0.0f;
    // setpoint.jerk[2] = 0.0f;
    setpoint.yaw = yaw;
    // setpoint.yawspeed = 0.0f;
    return setpoint;
}

px4_msgs::msg::TrajectorySetpoint OffboardControlBridge::publishConvertedSetpoint(px4_msgs::msg::TrajectorySetpoint enu_setpoint) {
    enu_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    auto ned_setpoint = convertENUToNED(enu_setpoint);
    traj_setpoint_pub_->publish(ned_setpoint);
    return enu_setpoint;
}

bool OffboardControlBridge::isArrivedAtPosition(px4_msgs::msg::TrajectorySetpoint setpoint, float tolerance) {
    float pos_error_x = std::abs(uav_pose_.pose.position.x - setpoint.position[0]);
    float pos_error_y = std::abs(uav_pose_.pose.position.y - setpoint.position[1]);
    float pos_error_z = std::abs(uav_pose_.pose.position.z - setpoint.position[2]);
    
    if (pos_error_x < tolerance && pos_error_y < tolerance && pos_error_z < tolerance) 
        return true;
    else
        return false;
}

void OffboardControlBridge::publish_trajectory_setpoint() {
    px4_msgs::msg::TrajectorySetpoint current_state;
    while (!get_traj_setpoint_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "traj generator service not available…");
        return;
    }
    if(pending_request_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Previous request still pending, skipping this cycle");
        return;
    }
    current_state.position[0] = uav_pose_.pose.position.x;
    current_state.position[1] = uav_pose_.pose.position.y;
    current_state.position[2] = uav_pose_.pose.position.z;
    double roll, pitch, yaw;
    quat2RPY(uav_pose_.pose.orientation, roll, pitch, yaw);
    current_state.yaw = yaw;

    auto request = std::make_shared<traj_offboard::srv::GetTrajectorySetpoint::Request>();
    request->current_state = current_state;
    request->target = target_pose_;
    if(update_target_) {
        request->update_target = true;
    } else {
        request->update_target = false;
    }
    pending_request_ = true;

    auto result = get_traj_setpoint_client_->async_send_request(request, [this](rclcpp::Client<traj_offboard::srv::GetTrajectorySetpoint>::SharedFuture resp_fut) {
        pending_request_ = false;
        try {
            auto resp = resp_fut.get();
            last_cmd_ = publishConvertedSetpoint(resp->trajectory_setpoint);
            last_cmd_time_ = this->now();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "get_trajectory_setpoint failed: %s", e.what());
            // hold last command
            last_cmd_ = publishConvertedSetpoint(last_cmd_);
            last_cmd_time_ = this->now();
        }
    });
    update_target_ = false; // reset after sending to traj generator
}

void OffboardControlBridge::publish_takeoff_setpoint(px4_msgs::msg::TrajectorySetpoint takeoff_setpoint) {
    px4_msgs::msg::TrajectorySetpoint current_state;
    while (!get_traj_setpoint_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "traj generator service not available…");
        return;
    }
    if(pending_request_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Previous request still pending, skipping this cycle");
        return;
    }
    current_state.position[0] = uav_pose_.pose.position.x;
    current_state.position[1] = uav_pose_.pose.position.y;
    current_state.position[2] = uav_pose_.pose.position.z;
    double roll, pitch, yaw;
    quat2RPY(uav_pose_.pose.orientation, roll, pitch, yaw);
    current_state.yaw = yaw;

    auto request = std::make_shared<traj_offboard::srv::GetTrajectorySetpoint::Request>();
    request->current_state = current_state;
    request->target = takeoff_setpoint;
    request->update_target = true;
    pending_request_ = true;

    auto result = get_traj_setpoint_client_->async_send_request(request, [this](rclcpp::Client<traj_offboard::srv::GetTrajectorySetpoint>::SharedFuture resp_fut) {
        pending_request_ = false;
        try {
            auto resp = resp_fut.get();
            last_cmd_ = publishConvertedSetpoint(resp->trajectory_setpoint);
            last_cmd_time_ = this->now();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "get_trajectory_setpoint failed: %s", e.what());
            // hold last command
            last_cmd_ = publishConvertedSetpoint(last_cmd_);
            last_cmd_time_ = this->now();
        }
    });
    update_target_ = false; // reset after sending to traj generator
}
void OffboardControlBridge::controlLoopOnTimer() {
    if (offboard_setpoint_counter_ == 10) {
        // Switch to offboard mode and arm after sending initial setpoints
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000, "Offboard & arm commands sent");
    }

    publish_offboard_control_mode();

    if (offboard_setpoint_counter_ < 11) {
        ++offboard_setpoint_counter_;
    }

    switch (flight_state_) {
        case FlightState::TAKEOFF: {
            auto HoverSetpoint = makePositionHoldSetpoint(0.0f, 0.0f, TAKEOFF_HEIGHT, 0.0f);
            // last_cmd_ = publishConvertedSetpoint(HoverSetpoint);
            publish_takeoff_setpoint(HoverSetpoint);
            if (isArrivedAtPosition(HoverSetpoint, POSITION_TOLERANCE)) {
                takeoff_complete_ = true;
                flight_state_ = FlightState::TRAJECTORY_FOLLOWING;
                RCLCPP_INFO(get_logger(), "Takeoff complete! Hovering at (%.2f, %.2f, %.2f)", uav_pose_.pose.position.x, uav_pose_.pose.position.y, uav_pose_.pose.position.z);
            }
            else{
                // 5 secondly log takeoff progress, 1s, 2s, 3s, 4s
                RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "Taking off... "); 
            }
            break;
        }
        case FlightState::TRAJECTORY_FOLLOWING: {
            if(!has_target_) {
                RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 3000, "Waiting for first target...");
                last_cmd_ = publishConvertedSetpoint(last_cmd_);
                last_cmd_time_ = this->now();
                break;
            }
            publish_trajectory_setpoint();
            break;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControlBridge>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), /*num_threads=*/2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
