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
        get_traj_setpoint_client_ = this->create_client<traj_offboard::srv::GetTrajectorySetpoint>(
            "online_traj_generator/get_trajectory_setpoints");
        auto onTimer = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                // Switch to offboard mode and arm after sending initial setpoints
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
                RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000, "Offboard & arm commands sent");
            }

            publish_offboard_control_mode();
            
            // Publish trajectory setpoint from online generator
            publish_trajectory_setpoint();

            if (offboard_setpoint_counter_ < 11) {
                ++offboard_setpoint_counter_;
            }
        };

        // Control timer: pair OffboardControlMode with a setpoint
        timer_ = this->create_wall_timer(10ms, onTimer);
    }

  private:
    // State holders
    geometry_msgs::msg::PoseStamped uav_pose_;
    geometry_msgs::msg::TwistStamped uav_twist_;
    sensor_msgs::msg::Imu uav_imu_;
    px4_msgs::msg::TrajectorySetpoint target_pose_;
    bool have_target_{false};
    
    // Takeoff sequence state management
    enum class FlightState {
        TAKEOFF,
        HOVERING,
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
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_ctrl_mode_pub_->publish(msg);
}

void OffboardControlBridge::handle_set_target(const traj_offboard::srv::SetTarget::Request::SharedPtr request,
                                              traj_offboard::srv::SetTarget::Response::SharedPtr response) {
    target_pose_ = request->target;
    have_target_ = true;
    response->success = true;
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
    setpoint.velocity[0] = 0.0f;
    setpoint.velocity[1] = 0.0f;
    setpoint.velocity[2] = 0.0f;
    setpoint.acceleration[0] = 0.0f;
    setpoint.acceleration[1] = 0.0f;
    setpoint.acceleration[2] = 0.0f;
    setpoint.jerk[0] = 0.0f;
    setpoint.jerk[1] = 0.0f;
    setpoint.jerk[2] = 0.0f;
    setpoint.yaw = yaw;
    setpoint.yawspeed = 0.0f;
    return setpoint;
}

px4_msgs::msg::TrajectorySetpoint OffboardControlBridge::publishConvertedSetpoint(px4_msgs::msg::TrajectorySetpoint enu_setpoint) {
    enu_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    auto ned_setpoint = convertENUToNED(enu_setpoint);
    traj_setpoint_pub_->publish(ned_setpoint);
    return ned_setpoint;
}



void OffboardControlBridge::publish_trajectory_setpoint() {
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    
    switch (flight_state_) {
        case FlightState::TAKEOFF: {
            // Send takeoff command to (0, 0, 5)
            setpoint.position[0] = 0.0f;  // x = 0 (East in ENU)
            setpoint.position[1] = 0.0f;  // y = 0 (North in ENU)
            setpoint.position[2] = TAKEOFF_HEIGHT;  // z = 5 (Up in ENU)
            setpoint.velocity[0] = 0.0f;
            setpoint.velocity[1] = 0.0f;
            setpoint.velocity[2] = 0.0f;
            setpoint.acceleration[0] = 0.0f;
            setpoint.acceleration[1] = 0.0f;
            setpoint.acceleration[2] = 0.0f;
            setpoint.yaw = 0.0f;  // Face North
            setpoint.yawspeed = 0.0f;
            setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            
            // Check if takeoff is complete (within tolerance of target position)
            float pos_error_x = std::abs(uav_pose_.pose.position.x - 0.0f);
            float pos_error_y = std::abs(uav_pose_.pose.position.y - 0.0f);
            float pos_error_z = std::abs(uav_pose_.pose.position.z - TAKEOFF_HEIGHT);
            
            if (pos_error_x < POSITION_TOLERANCE && 
                pos_error_y < POSITION_TOLERANCE && 
                pos_error_z < POSITION_TOLERANCE) {
                takeoff_complete_ = true;
                flight_state_ = FlightState::HOVERING;
                RCLCPP_INFO(get_logger(), "Takeoff complete! Hovering at (0,0,%.1f) and waiting for trajectory commands", TAKEOFF_HEIGHT);
            }
            else{
                // 5 secondly log takeoff progress, 1s, 2s, 3s, 4s
                RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "Taking off... ");
                
            }
            traj_setpoint_pub_->publish(convertENUToNED(setpoint));
            break;
        }
        
        case FlightState::HOVERING: {
            // Check if we have a new target from the trajectory generator
            if (get_traj_setpoint_client_ && get_traj_setpoint_client_->wait_for_service(std::chrono::milliseconds(1))) {
                auto request = std::make_shared<traj_offboard::srv::GetTrajectorySetpoint::Request>();
                
                // Fill current state from vehicle position/attitude, send as enu frame
                request->current_state.position[0] = static_cast<float>(uav_pose_.pose.position.x);
                request->current_state.position[1] = static_cast<float>(uav_pose_.pose.position.y);
                request->current_state.position[2] = static_cast<float>(uav_pose_.pose.position.z);
                
                double roll, pitch, yaw;
                quat2RPY(uav_pose_.pose.orientation, roll, pitch, yaw);
                request->current_state.yaw = static_cast<float>(yaw);
                
                // Set target if we have one
                if (have_target_) {
                    request->target = target_pose_;
                    request->update_target = true;
                    flight_state_ = FlightState::TRAJECTORY_FOLLOWING;
                    RCLCPP_INFO(get_logger(), "New target received! Switching to trajectory following mode");
                } else {
                    request->update_target = false;
                }
                
                auto future = get_traj_setpoint_client_->async_send_request(request);
                if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                    auto response = future.get();
                    if (response->success) {
                        // Publish the trajectory setpoint from the service
                        auto traj_setpoint = convertENUToNED(response->trajectory_setpoint);
                        traj_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                        traj_setpoint_pub_->publish(traj_setpoint);
                        
                        // Store for reference
                        last_cmd_ = traj_setpoint;
                        last_cmd_time_ = this->now();
                        
                        // Reset target flag after successful update
                        if (have_target_) {
                            have_target_ = false;
                        }
                    } else {
                        // Hover at takeoff position if service fails
                        setpoint.position[0] = 0.0f;
                        setpoint.position[1] = 0.0f;
                        setpoint.position[2] = TAKEOFF_HEIGHT;
                        setpoint.velocity[0] = 0.0f;
                        setpoint.velocity[1] = 0.0f;
                        setpoint.velocity[2] = 0.0f;
                        setpoint.acceleration[0] = 0.0f;
                        setpoint.acceleration[1] = 0.0f;
                        setpoint.acceleration[2] = 0.0f;
                        setpoint.yaw = 0.0f;
                        setpoint.yawspeed = 0.0f;
                        setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                        traj_setpoint_pub_->publish(convertENUToNED(setpoint));
                    }
                } else {
                    // Hover at takeoff position if service not ready
                    setpoint.position[0] = 0.0f;
                    setpoint.position[1] = 0.0f;
                    setpoint.position[2] = TAKEOFF_HEIGHT;
                    setpoint.velocity[0] = 0.0f;
                    setpoint.velocity[1] = 0.0f;
                    setpoint.velocity[2] = 0.0f;
                    setpoint.acceleration[0] = 0.0f;
                    setpoint.acceleration[1] = 0.0f;
                    setpoint.acceleration[2] = 0.0f;
                    setpoint.yaw = 0.0f;
                    setpoint.yawspeed = 0.0f;
                    setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                    traj_setpoint_pub_->publish(convertENUToNED(setpoint));
                }
            } else {
                // Hover at takeoff position if service not available
                setpoint.position[0] = 0.0f;
                setpoint.position[1] = 0.0f;
                setpoint.position[2] = -TAKEOFF_HEIGHT;
                setpoint.velocity[0] = 0.0f;
                setpoint.velocity[1] = 0.0f;
                setpoint.velocity[2] = 0.0f;
                setpoint.acceleration[0] = 0.0f;
                setpoint.acceleration[1] = 0.0f;
                setpoint.acceleration[2] = 0.0f;
                setpoint.yaw = 0.0f;
                setpoint.yawspeed = 0.0f;
                setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                traj_setpoint_pub_->publish(convertENUToNED(setpoint));
            }
            break;
        }
        
        case FlightState::TRAJECTORY_FOLLOWING: {
            // Normal trajectory following mode
            if (get_traj_setpoint_client_ && get_traj_setpoint_client_->wait_for_service(std::chrono::milliseconds(1))) {
                auto request = std::make_shared<traj_offboard::srv::GetTrajectorySetpoint::Request>();
                
                // Fill current state from vehicle position/attitude
                request->current_state.position[0] = static_cast<float>(uav_pose_.pose.position.x);
                request->current_state.position[1] = static_cast<float>(uav_pose_.pose.position.y);
                request->current_state.position[2] = static_cast<float>(uav_pose_.pose.position.z);
                
                double roll, pitch, yaw;
                quat2RPY(uav_pose_.pose.orientation, roll, pitch, yaw);
                request->current_state.yaw = static_cast<float>(yaw);
                
                // Set target if we have one
                if (have_target_) {
                    request->target = target_pose_;
                    request->update_target = true;
                } else {
                    request->update_target = false;
                }
                
                auto future = get_traj_setpoint_client_->async_send_request(request);
                if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                    auto response = future.get();
                    if (response->success) {
                        // Publish the trajectory setpoint from the service
                        auto traj_setpoint = convertENUToNED(response->trajectory_setpoint);
                        traj_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                        traj_setpoint_pub_->publish(traj_setpoint);
                        
                        // Store for reference
                        last_cmd_ = traj_setpoint;
                        last_cmd_time_ = this->now();
                        
                        // Reset target flag after successful update
                        if (have_target_) {
                            have_target_ = false;
                        }
                    } else {
                        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, 
                            "Failed to get trajectory setpoint from online generator");
                    }
                }
            } else {
                // Fallback: publish last known command or hover command
                if ((this->now() - last_cmd_time_).seconds() < 1.0 && last_cmd_.timestamp > 0) {
                    // Republish last command with updated timestamp
                    auto traj_setpoint = last_cmd_;
                    traj_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                    traj_setpoint_pub_->publish(traj_setpoint);
                } else {
                    // Publish hover command at current position
                    setpoint.position[0] = static_cast<float>(uav_pose_.pose.position.x);
                    setpoint.position[1] = static_cast<float>(uav_pose_.pose.position.y);
                    setpoint.position[2] = static_cast<float>(uav_pose_.pose.position.z);
                    setpoint.velocity[0] = 0.0f;
                    setpoint.velocity[1] = 0.0f;
                    setpoint.velocity[2] = 0.0f;
                    setpoint.acceleration[0] = 0.0f;
                    setpoint.acceleration[1] = 0.0f;
                    setpoint.acceleration[2] = 0.0f;
                    double roll, pitch, yaw;
                    quat2RPY(uav_pose_.pose.orientation, roll, pitch, yaw);
                    setpoint.yaw = static_cast<float>(yaw);
                    setpoint.yawspeed = 0.0f;
                    setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                    traj_setpoint_pub_->publish(convertENUToNED(setpoint));
                }
            }
            break;
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControlBridge>());
    rclcpp::shutdown();
    return 0;
}
