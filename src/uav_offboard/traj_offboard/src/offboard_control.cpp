// Offboard control bridge that:
// - Subscribes to UAV state topics (pose/twist/imu)
// - Publishes PX4 OffboardControlMode, VehicleCommand, TrajectorySetpoint
// - Offers a service to set target position/yaw for OnlineTrajGenerator

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/home_position.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <traj_offboard/srv/get_trajectory_setpoint.hpp>
#include <traj_offboard/srv/set_target.hpp>
#include <traj_offboard/msg/traj_complete_flag.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cinttypes>
#include <cstdint>
#include <cmath>
#include <mutex>
#include <optional>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>

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
        csv_waypoints_ = loadCsvWaypoints(waypoints_csv_path_);
        if (!csv_waypoints_.empty()) {
            csv_transit_first_setpoint_ = getCsvFirstSetpoint();
            takeoff_setpoint_ = makeCsvSetpoint(csv_waypoints_.front());
            RCLCPP_INFO(get_logger(),
                        "CSV waypoints loaded | count=%zu path=%s first=(%.2f, %.2f, %.2f, yaw %.2f) velocity=(%.2f, %.2f, %.2f)",
                        csv_waypoints_.size(), waypoints_csv_path_.c_str(),
                        csv_waypoints_.front().x, csv_waypoints_.front().y,
                        csv_waypoints_.front().z, csv_waypoints_.front().yaw,
                        csv_waypoints_.front().vx, csv_waypoints_.front().vy,
                        csv_waypoints_.front().vz);
        } else {
            takeoff_setpoint_ = makePositionHoldSetpoint(0.0f, 0.0f, 5.0f, static_cast<float>(csv_default_yaw_));
            RCLCPP_WARN(get_logger(),
                        "CSV waypoints unavailable | path=%s; using fallback takeoff target=(0.00, 0.00, 5.00)",
                        waypoints_csv_path_.c_str());
        }
        // Initialize offboard_state_ data
        offboard_state_.data = "WAITINGFORCOMMAND";
        uav_pose_.pose.orientation.w = 1.0;
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
        vehicle_home_position_sub_ = this->create_subscription<px4_msgs::msg::HomePosition>(
            "/fmu/out/home_position", qos, std::bind(&OffboardControlBridge::VehicleHomePositionCallback, this, std::placeholders::_1));
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", qos, std::bind(&OffboardControlBridge::VehicleStatusCallback, this, std::placeholders::_1));
        offboard_state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/uav_offboard_fsm/offboard_state", 10, std::bind(&OffboardControlBridge::OffboardStateCallback, this, std::placeholders::_1));
        // Service to set target for online trajectory generator
        set_target_srv_ = this->create_service<traj_offboard::srv::SetTarget>(
            "online_traj_generator/set_target", std::bind(&OffboardControlBridge::handle_set_target, this, std::placeholders::_1, std::placeholders::_2));
        // Client to get current trajectory cmd from online trajectory generator
        client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        get_traj_setpoint_client_ = this->create_client<traj_offboard::srv::GetTrajectorySetpoint>(
            "online_traj_generator/get_trajectory_setpoints", rmw_qos_profile_services_default, client_callback_group_);
        
        traj_completed_flag_pub_ = this->create_publisher<traj_offboard::msg::TrajCompleteFlag>("/traj_offboard/traj_complete_flag", 10);
        // Control timer: pair OffboardControlMode with a setpoint
        timer_ = this->create_wall_timer(20ms, std::bind(&OffboardControlBridge::controlLoopOnTimer, this));
        RCLCPP_INFO(get_logger(),
                    "Offboard bridge ready | use_takeoff_on_ground=%s fsm_state=/uav_offboard_fsm/offboard_state set_target=online_traj_generator/set_target traj_service=/online_traj_generator/get_trajectory_setpoints",
                    use_takeoff_on_ground_ ? "true" : "false");
    }

  private:
    struct CsvWaypoint {
            double x{0.0};
            double y{0.0};
            double z{0.0};
            double yaw{0.0};
            double vx{0.0};
            double vy{0.0};
            double vz{0.0};
    };
    // State holders
    geometry_msgs::msg::PoseStamped uav_pose_;
    geometry_msgs::msg::TwistStamped uav_twist_;
    sensor_msgs::msg::Imu uav_imu_;
    px4_msgs::msg::HomePosition uav_home_position_;
    px4_msgs::msg::VehicleStatus vehicle_status_;
    std_msgs::msg::String offboard_state_;
    px4_msgs::msg::TrajectorySetpoint target_pose_;
    px4_msgs::msg::TrajectorySetpoint takeoff_setpoint_{}, csv_transit_first_setpoint_{};
    bool has_target_{false};
    bool pending_request_{false};
    uint64_t target_generation_{0};
    uint64_t forwarded_target_generation_{0};
    std::mutex bridge_mutex_;
    bool use_takeoff_on_ground_{false};
    bool has_local_position_{false};
    bool has_vehicle_status_{false};
    bool px4_offboard_active_{false};
    bool manual_hover_setpoint_valid_{false};
    px4_msgs::msg::TrajectorySetpoint manual_hover_setpoint_{};
    double latest_heading_yaw_enu_{0.0};
    bool has_heading_yaw_{false};

    std::string waypoints_csv_path_{"/home/sia/ws_sensor_combined/src/uav_offboard/waypoints.csv"};
    double csv_stream_rate_hz_{50.0};
    double csv_default_yaw_{0.0};
    std::vector<CsvWaypoint> csv_waypoints_;
    std::size_t csv_transit_index_{0};
    bool csv_transit_complete_logged_{false};
    bool csv_transit_started_{false};

    traj_offboard::msg::TrajCompleteFlag traj_complete_flag_{};
    
    // Takeoff sequence state management
    enum class FlightState {
        WAITINGFORCOMMAND,
        TAKEOFF,
        TRAJECTORY_FOLLOWING
    };
    FlightState flight_state_{FlightState::WAITINGFORCOMMAND};
    bool takeoff_complete_{false};
    static constexpr float POSITION_TOLERANCE = 0.2f;

    // ROS interfaces
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleImu>::SharedPtr vehicle_imu_sub_;
    rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr vehicle_home_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr offboard_state_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

    rclcpp::Service<traj_offboard::srv::SetTarget>::SharedPtr set_target_srv_;
    rclcpp::Client<traj_offboard::srv::GetTrajectorySetpoint>::SharedPtr get_traj_setpoint_client_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;

    rclcpp::Publisher<traj_offboard::msg::TrajCompleteFlag>::SharedPtr traj_completed_flag_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t offboard_setpoint_counter_{0U};
    px4_msgs::msg::TrajectorySetpoint last_cmd_{};
    rclcpp::Time last_cmd_time_{};

    void VehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void VehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void VehicleImuCallback(const px4_msgs::msg::VehicleImu::SharedPtr msg);
    void VehicleHomePositionCallback(const px4_msgs::msg::HomePosition::SharedPtr msg);
    void VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void OffboardStateCallback(const std_msgs::msg::String::SharedPtr msg);
	void publish_offboard_control_mode_pva();
    void publish_offboard_control_mode_pv();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
    void handle_set_target(const traj_offboard::srv::SetTarget::Request::SharedPtr request,
                           traj_offboard::srv::SetTarget::Response::SharedPtr response);
    void publish_trajectory_setpoint();
    void publish_csv_transit_setpoint();
    void reset_csv_transit();
    static double wrapAngle(double angle);
    double getCurrentYawEnu() const;
    px4_msgs::msg::TrajectorySetpoint convertENUToNED(const px4_msgs::msg::TrajectorySetpoint &enu_setpoint) const;
    px4_msgs::msg::TrajectorySetpoint makePositionHoldSetpoint(float x, float y, float z, float yaw) const;
    px4_msgs::msg::TrajectorySetpoint makeCurrentLocalPositionHoldSetpoint() const;
    px4_msgs::msg::TrajectorySetpoint getCsvFirstSetpoint() const;
    px4_msgs::msg::TrajectorySetpoint makeCsvSetpoint(const CsvWaypoint & waypoint) const;
    px4_msgs::msg::TrajectorySetpoint publishConvertedSetpoint(px4_msgs::msg::TrajectorySetpoint enu_setpoint);
    void publishManualHoverSetpoint();
    void captureManualHoverSetpoint();
    bool isArrivedAtPosition(px4_msgs::msg::TrajectorySetpoint setpoint, float tolerance);
    void controlLoopOnTimer();
    void publish_takeoff_setpoint(px4_msgs::msg::TrajectorySetpoint takeoff_setpoint);
    std::vector<CsvWaypoint> loadCsvWaypoints(const std::string & path) const;
    std::optional<CsvWaypoint> parseCsvWaypointLine(const std::string & line,
                                                    std::size_t line_number) const;
};

// Wraps an angle to the range [-pi, pi)
double OffboardControlBridge::wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double OffboardControlBridge::getCurrentYawEnu() const {
    if (has_heading_yaw_) {
        return latest_heading_yaw_enu_;
    }

    double roll, pitch, yaw;
    quat2RPY(uav_pose_.pose.orientation, roll, pitch, yaw);
    return yaw;
}

void OffboardControlBridge::VehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    if (!msg->xy_valid || !msg->z_valid) {
        has_local_position_ = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "PX4 local position invalid | xy_valid=%s z_valid=%s",
                             msg->xy_valid ? "true" : "false",
                             msg->z_valid ? "true" : "false");
        return;
    }

    has_local_position_ = true;
    uav_pose_.header.stamp = this->now();
    uav_pose_.header.frame_id = "map"; // ENU frame
    // PX4 NED to ROS ENU frame
    uav_pose_.pose.position.x = msg->y - uav_home_position_.y;
    uav_pose_.pose.position.y = msg->x - uav_home_position_.x;
    uav_pose_.pose.position.z = -msg->z + uav_home_position_.z;

    static constexpr double HALF_PI = 1.5707963267948966;
    if (std::isfinite(msg->heading)) {
        // PX4 heading is clockwise from North, we want counterclockwise from East, so ENU yaw = 90deg - heading
        latest_heading_yaw_enu_ = wrapAngle(HALF_PI - static_cast<double>(msg->heading));
        has_heading_yaw_ = true;
    }
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
void OffboardControlBridge::VehicleHomePositionCallback(const px4_msgs::msg::HomePosition::SharedPtr msg) {
    uav_home_position_.timestamp = msg->timestamp;
    uav_home_position_.x = msg->x;
    uav_home_position_.y = msg->y;
    uav_home_position_.z = msg->z;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "PX4 home position | ned=(%.2f, %.2f, %.2f)",
                         uav_home_position_.x, uav_home_position_.y, uav_home_position_.z);
}

void OffboardControlBridge::VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    const bool was_offboard = px4_offboard_active_;
    vehicle_status_ = *msg;
    has_vehicle_status_ = true;
    px4_offboard_active_ =
        msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    if (use_takeoff_on_ground_) {
        return;
    }
    // 之前不是 OFFBOARD，现在是 OFFBOARD。switch moment
    if (!was_offboard && px4_offboard_active_) {
        captureManualHoverSetpoint();
        return;
    }

    if (was_offboard && !px4_offboard_active_) {
        manual_hover_setpoint_valid_ = false;
        RCLCPP_INFO(get_logger(), "PX4 OFFBOARD exited | manual hover setpoint cleared");
    }
}

void OffboardControlBridge::OffboardStateCallback(const std_msgs::msg::String::SharedPtr msg) {
    const auto previous_state = offboard_state_.data;
    offboard_state_ = *msg;
    if (previous_state != "TRANSIT_TO_AREA" && offboard_state_.data == "TRANSIT_TO_AREA") {
        reset_csv_transit();
        RCLCPP_INFO(get_logger(), "CSV transit start | count=%zu rate=%.2fHz",
                    csv_waypoints_.size(), csv_stream_rate_hz_);
    }
}
void OffboardControlBridge::publish_offboard_control_mode_pva() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = true;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_ctrl_mode_pub_->publish(msg);
}

void OffboardControlBridge::publish_offboard_control_mode_pv() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_ctrl_mode_pub_->publish(msg);
}

void OffboardControlBridge::handle_set_target(const traj_offboard::srv::SetTarget::Request::SharedPtr request,
                                              traj_offboard::srv::SetTarget::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(bridge_mutex_);
    target_pose_ = request->target;
    ++target_generation_;
    response->success = true;
    has_target_ = true;
    RCLCPP_INFO(get_logger(),
                "Target received | generation=%" PRIu64 " enu=(%.2f, %.2f, %.2f, yaw %.2f)",
                target_generation_,
                target_pose_.position[0], target_pose_.position[1],
                target_pose_.position[2], target_pose_.yaw);
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
    ned_setpoint.position[0] = enu_setpoint.position[1] + uav_home_position_.x;
    ned_setpoint.position[1] = enu_setpoint.position[0] + uav_home_position_.y;
    ned_setpoint.position[2] = -enu_setpoint.position[2] + uav_home_position_.z;

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

px4_msgs::msg::TrajectorySetpoint OffboardControlBridge::makeCurrentLocalPositionHoldSetpoint() const {
    return makePositionHoldSetpoint(
        static_cast<float>(uav_pose_.pose.position.x),
        static_cast<float>(uav_pose_.pose.position.y),
        static_cast<float>(uav_pose_.pose.position.z),
        static_cast<float>(getCurrentYawEnu()));
}

void OffboardControlBridge::publishManualHoverSetpoint() {
    if (!manual_hover_setpoint_valid_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Manual hover unavailable | waiting for PX4 OFFBOARD switch and valid local position");
        return;
    }

    last_cmd_ = publishConvertedSetpoint(manual_hover_setpoint_);
    last_cmd_time_ = this->now();
}

void OffboardControlBridge::captureManualHoverSetpoint() {
    if (!has_local_position_) {
        RCLCPP_WARN(get_logger(),
                    "PX4 OFFBOARD entered manually but local position is not valid; cannot capture hover setpoint yet");
        return;
    }

    manual_hover_setpoint_ = makeCurrentLocalPositionHoldSetpoint();
    manual_hover_setpoint_valid_ = true;
    RCLCPP_INFO(get_logger(),
                "PX4 OFFBOARD entered manually | holding latest local position enu=(%.2f, %.2f, %.2f, yaw %.2f)",
                manual_hover_setpoint_.position[0],
                manual_hover_setpoint_.position[1],
                manual_hover_setpoint_.position[2],
                manual_hover_setpoint_.yaw);
    publish_offboard_control_mode_pva();
    publishManualHoverSetpoint();
}

px4_msgs::msg::TrajectorySetpoint OffboardControlBridge::getCsvFirstSetpoint() const {
    // csv waypoints are NED relative to local origin. Convert to ENU and correct to Zero.
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.position[0] = csv_waypoints_.front().x;
    setpoint.position[1] = csv_waypoints_.front().y;
    setpoint.position[2] = csv_waypoints_.front().z;
    return setpoint;
}

px4_msgs::msg::TrajectorySetpoint OffboardControlBridge::makeCsvSetpoint(const CsvWaypoint & waypoint) const {
    // csv waypoints are NED relative to local origin. Convert to ENU and correct to Zero.
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.position[0] = static_cast<float>(waypoint.y) - csv_transit_first_setpoint_.position[1];
    setpoint.position[1] = static_cast<float>(waypoint.x) - csv_transit_first_setpoint_.position[0];
    setpoint.position[2] = static_cast<float>(waypoint.z) - 35;
    setpoint.velocity[0] = static_cast<float>(waypoint.vx);
    setpoint.velocity[1] = static_cast<float>(waypoint.vy);
    setpoint.velocity[2] = static_cast<float>(waypoint.vz);
    setpoint.yaw = static_cast<float>(waypoint.yaw);
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
    px4_msgs::msg::TrajectorySetpoint target_pose;
    uint64_t sent_generation = 0;
    bool sent_target_update = false;

    while (!get_traj_setpoint_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Trajectory service unavailable | waiting for /online_traj_generator/get_trajectory_setpoints");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        if(pending_request_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Trajectory request pending | skipping until previous response returns");
            return;
        }
        target_pose = target_pose_;
        sent_generation = target_generation_;
        sent_target_update = sent_generation != forwarded_target_generation_;
        pending_request_ = true;
    }

    current_state.position[0] = uav_pose_.pose.position.x;
    current_state.position[1] = uav_pose_.pose.position.y;
    current_state.position[2] = uav_pose_.pose.position.z;
    current_state.yaw = static_cast<float>(getCurrentYawEnu());

    auto request = std::make_shared<traj_offboard::srv::GetTrajectorySetpoint::Request>();
    request->current_state = current_state;
    request->target = target_pose;
    request->update_target = sent_target_update;

    auto result = get_traj_setpoint_client_->async_send_request(request, [this, sent_generation, sent_target_update, target_pose](rclcpp::Client<traj_offboard::srv::GetTrajectorySetpoint>::SharedFuture resp_fut) {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        try {
            auto resp = resp_fut.get();
            pending_request_ = false;
            if (!resp->success) {
                RCLCPP_ERROR(this->get_logger(),
                             "Trajectory service rejected request | generation=%" PRIu64 " update_target=%s",
                             sent_generation, sent_target_update ? "true" : "false");
                return;
            }
            if (sent_target_update && forwarded_target_generation_ < sent_generation) {
                forwarded_target_generation_ = sent_generation;
                RCLCPP_INFO(this->get_logger(),
                            "Target forwarded to trajectory generator | generation=%" PRIu64 " target=(%.2f, %.2f, %.2f, yaw %.2f)",
                            sent_generation, target_pose.position[0], target_pose.position[1],
                            target_pose.position[2], target_pose.yaw);
            }
            last_cmd_ = publishConvertedSetpoint(resp->trajectory_setpoint);
            last_cmd_time_ = this->now();
        } catch (const std::exception& e) {
            pending_request_ = false;
            RCLCPP_ERROR(this->get_logger(), "Trajectory service call failed | error=%s", e.what());
            // hold last command
            last_cmd_ = publishConvertedSetpoint(last_cmd_);
            last_cmd_time_ = this->now();
        }
    });
}

void OffboardControlBridge::publish_csv_transit_setpoint() {
    if (csv_waypoints_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "CSV transit unavailable | no loaded waypoints");
        return;
    }

    csv_transit_started_ = true;
    const std::size_t publish_index =
        std::min(csv_transit_index_, csv_waypoints_.size() - 1);
    last_cmd_ = publishConvertedSetpoint(makeCsvSetpoint(csv_waypoints_[publish_index]));
    last_cmd_time_ = this->now();

    if (csv_transit_index_ + 1 < csv_waypoints_.size()) {
        ++csv_transit_index_;
        return;
    }

    csv_transit_index_ = csv_waypoints_.size();
    if (!csv_transit_complete_logged_) {
        csv_transit_complete_logged_ = true;
        traj_complete_flag_.trajectory_completed = true;
        traj_complete_flag_.traj_last_setpoint.position[0] = last_cmd_.position[0];
        traj_complete_flag_.traj_last_setpoint.position[1] = last_cmd_.position[1];
        traj_complete_flag_.traj_last_setpoint.position[2] = last_cmd_.position[2];
        traj_complete_flag_.traj_last_setpoint.yaw = last_cmd_.yaw;
        traj_completed_flag_pub_->publish(traj_complete_flag_);
        RCLCPP_INFO(get_logger(), "CSV transit complete | count=%zu", csv_waypoints_.size());
    }
}

void OffboardControlBridge::reset_csv_transit() {
    csv_transit_index_ = 0;
    csv_transit_complete_logged_ = false;
    csv_transit_started_ = false;
}

void OffboardControlBridge::publish_takeoff_setpoint(px4_msgs::msg::TrajectorySetpoint takeoff_setpoint) {
    px4_msgs::msg::TrajectorySetpoint current_state;
    while (!get_traj_setpoint_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Trajectory service unavailable | waiting for /online_traj_generator/get_trajectory_setpoints");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        if(pending_request_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Trajectory request pending | skipping until previous response returns");
            return;
        }
        pending_request_ = true;
    }

    current_state.position[0] = uav_pose_.pose.position.x;
    current_state.position[1] = uav_pose_.pose.position.y;
    current_state.position[2] = uav_pose_.pose.position.z;
    current_state.yaw = static_cast<float>(getCurrentYawEnu());

    auto request = std::make_shared<traj_offboard::srv::GetTrajectorySetpoint::Request>();
    request->current_state = current_state;
    request->target = takeoff_setpoint;
    request->update_target = true;

    auto result = get_traj_setpoint_client_->async_send_request(request, [this](rclcpp::Client<traj_offboard::srv::GetTrajectorySetpoint>::SharedFuture resp_fut) {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        try {
            auto resp = resp_fut.get();
            pending_request_ = false;
            if (!resp->success) {
                RCLCPP_ERROR(this->get_logger(), "Takeoff trajectory service rejected request");
                return;
            }
            last_cmd_ = publishConvertedSetpoint(resp->trajectory_setpoint);
            last_cmd_time_ = this->now();
        } catch (const std::exception& e) {
            pending_request_ = false;
            RCLCPP_ERROR(this->get_logger(), "Trajectory service call failed | error=%s", e.what());
            // hold last command
            last_cmd_ = publishConvertedSetpoint(last_cmd_);
            last_cmd_time_ = this->now();
        }
    });
}
void OffboardControlBridge::controlLoopOnTimer() {
    switch (flight_state_) {
        case FlightState::WAITINGFORCOMMAND: {
            publish_offboard_control_mode_pva();
            if (use_takeoff_on_ground_) {
                if(offboard_state_.data == "UAV_START") {
                    flight_state_ = FlightState::TAKEOFF;
                    RCLCPP_INFO(get_logger(), "Bridge state -> TAKEOFF | trigger=%s arming and publishing takeoff setpoints",
                                offboard_state_.data.c_str());
                } else {
                    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                         "Bridge waiting | expected FSM state UAV_START on /uav_offboard_fsm/offboard_state");
                }
                break;
            }
            // no received status
            if (!has_vehicle_status_) {
                if (has_local_position_) {
                    last_cmd_ = publishConvertedSetpoint(makeCurrentLocalPositionHoldSetpoint());
                    last_cmd_time_ = this->now();
                }
                RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                     "Manual takeoff mode | waiting for PX4 status on /fmu/out/vehicle_status");
                break;
            }
            // no offboard switched
            if (!px4_offboard_active_) {
                if (has_local_position_) {
                    last_cmd_ = publishConvertedSetpoint(makeCurrentLocalPositionHoldSetpoint());
                    last_cmd_time_ = this->now();
                }
                RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                     "Manual takeoff mode | waiting for pilot to switch PX4 to OFFBOARD");
                break;
            }
            // if fsm switched state
            if(offboard_state_.data == "UAV_START") {
                flight_state_ = FlightState::TAKEOFF;
                RCLCPP_INFO(get_logger(), "Bridge state -> TAKEOFF | trigger=%s manual offboard active, publishing takeoff setpoints",
                            offboard_state_.data.c_str());
            } else { // else, manual_hover_setpoint_ is built, and pub every circle
                if (!manual_hover_setpoint_valid_ && has_local_position_) {
                    captureManualHoverSetpoint();
                }
                publishManualHoverSetpoint();
                RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                     "Manual offboard active | hovering until FSM state UAV_START");
            }
            break;
        }
        case FlightState::TAKEOFF: {
            publish_offboard_control_mode_pva();
            if (use_takeoff_on_ground_ && offboard_setpoint_counter_ == 10) {
                // Switch to offboard mode and arm after sending initial setpoints
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
                RCLCPP_INFO(get_logger(), "PX4 command sent | mode=OFFBOARD arm=true");
            }
            if (use_takeoff_on_ground_ && offboard_setpoint_counter_ < 11) {
                ++offboard_setpoint_counter_;
            }
            publish_takeoff_setpoint(takeoff_setpoint_);
            if (isArrivedAtPosition(takeoff_setpoint_, POSITION_TOLERANCE)) {
                takeoff_complete_ = true;
                traj_complete_flag_.take_off_completed = true;
                traj_complete_flag_.trajectory_completed = false;
                traj_complete_flag_.traj_first_setpoint.position[0] = last_cmd_.position[0];
                traj_complete_flag_.traj_first_setpoint.position[1] = last_cmd_.position[1];
                traj_complete_flag_.traj_first_setpoint.position[2] = last_cmd_.position[2];
                traj_complete_flag_.traj_first_setpoint.yaw = last_cmd_.yaw;
                traj_completed_flag_pub_->publish(traj_complete_flag_);
                flight_state_ = FlightState::TRAJECTORY_FOLLOWING;
                RCLCPP_INFO(get_logger(), "Bridge state -> TRAJECTORY_FOLLOWING | takeoff complete pos=(%.2f, %.2f, %.2f)", uav_pose_.pose.position.x, uav_pose_.pose.position.y, uav_pose_.pose.position.z);
            }
            else{
                RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 3000,
                                     "Takeoff progress | pos=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f)",
                                     uav_pose_.pose.position.x, uav_pose_.pose.position.y,
                                     uav_pose_.pose.position.z,
                                     takeoff_setpoint_.position[0],
                                     takeoff_setpoint_.position[1],
                                     takeoff_setpoint_.position[2]);
            }
            break;
        }
        case FlightState::TRAJECTORY_FOLLOWING: {
            std::lock_guard<std::mutex> lock(bridge_mutex_);
            if (offboard_state_.data == "TRANSIT_TO_AREA" && !csv_waypoints_.empty() && !csv_transit_complete_logged_) {
                publish_offboard_control_mode_pv();
                publish_csv_transit_setpoint();
                break;
            }
            if(!has_target_) {
                publish_offboard_control_mode_pva();
                if (last_cmd_time_.nanoseconds() != 0) {
                    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                         "Trajectory following | holding setpoint");
                    last_cmd_ = publishConvertedSetpoint(last_cmd_);
                    last_cmd_time_ = this->now();
                } else {
                    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
                                         "Trajectory following | waiting for first target from FSM");
                    publish_takeoff_setpoint(takeoff_setpoint_);
                }
                break;
            }
            publish_offboard_control_mode_pva();
            publish_trajectory_setpoint();
            break;
        }
    }
};

std::vector<OffboardControlBridge::CsvWaypoint>
OffboardControlBridge::loadCsvWaypoints(const std::string & path) const {
    std::vector<CsvWaypoint> waypoints;
    std::ifstream file(path);
    if (!file.is_open()) {
        return waypoints;
    }

    std::string line;
    std::size_t line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        const auto waypoint = parseCsvWaypointLine(line, line_number);
        if (waypoint) {
            waypoints.push_back(*waypoint);
        }
    }
    return waypoints;
}

std::optional<OffboardControlBridge::CsvWaypoint>
OffboardControlBridge::parseCsvWaypointLine(const std::string & line,
                                            std::size_t line_number) const {
    if (line.empty()) {
        return std::nullopt;
    }

    std::vector<std::string> columns;
    std::stringstream stream(line);
    std::string token;
    while (std::getline(stream, token, ',')) {
        columns.push_back(token);
    }

    if (columns.empty() || columns.front() == "index") {
        return std::nullopt;
    }
    if (columns.size() < 7) {
        RCLCPP_WARN(get_logger(), "CSV waypoint ignored | path=%s line=%zu reason=expected_7_columns",
                    waypoints_csv_path_.c_str(), line_number);
        return std::nullopt;
    }

    try {
        return CsvWaypoint{
            std::stod(columns[1]),
            std::stod(columns[2]),
            std::stod(columns[3]),
            csv_default_yaw_,
            std::stod(columns[4]),
            std::stod(columns[5]),
            std::stod(columns[6])};
    } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "CSV waypoint ignored | path=%s line=%zu error=%s",
                    waypoints_csv_path_.c_str(), line_number, e.what());
        return std::nullopt;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControlBridge>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), /*num_threads=*/2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
