#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/string.hpp>
#include <traj_offboard/msg/bridge_feedback.hpp>
#include <traj_offboard/srv/get_trajectory_setpoint.hpp>
#include <traj_offboard/srv/set_target.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

using namespace std::chrono_literals;

namespace {

bool finiteArray(const std::array<float, 3> & values)
{
    return std::all_of(values.begin(), values.end(), [](float value) {
        return std::isfinite(value);
    });
}

int positiveInt(long value)
{
    return static_cast<int>(std::max(1L, value));
}

}  // namespace

class OffboardControlBridge : public rclcpp::Node {
  public:
    OffboardControlBridge() : rclcpp::Node("offboard_control_bridge")
    {
        offboard_state_topic_ =
            declare_parameter<std::string>("offboard_state_topic", "/uav_offboard_fsm/offboard_state");
        bridge_feedback_topic_ =
            declare_parameter<std::string>("bridge_feedback_topic", "/uav_offboard_fsm/bridge_feedback");
        set_target_service_ =
            declare_parameter<std::string>("set_target_service", "online_traj_generator/set_target");
        get_traj_service_ = declare_parameter<std::string>(
            "get_trajectory_setpoint_service", "online_traj_generator/get_trajectory_setpoints");
        feedback_timeout_s_ = declare_parameter<double>("feedback_timeout_s", 1.0);
        distance_sensor_timeout_s_ = declare_parameter<double>("distance_sensor_timeout_s", 1.0);
        imu_timeout_s_ = declare_parameter<double>("imu_timeout_s", 1.0);
        takeoff_height_ = declare_parameter<double>("takeoff_height", 5.0);
        position_tolerance_ = declare_parameter<double>("position_tolerance", 0.2);
        yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.15);
        distance_sensor_min_signal_quality_ =
            declare_parameter<int>("distance_sensor_min_signal_quality", 1);
        offboard_warmup_setpoints_ =
            positiveInt(declare_parameter<int>("offboard_warmup_setpoints", 20));

        offboard_ctrl_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        traj_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        bridge_feedback_pub_ =
            create_publisher<traj_offboard::msg::BridgeFeedback>(bridge_feedback_topic_, 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos_profile.depth = 5;
        auto sensor_qos =
            rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
                        qos_profile);

        vehicle_local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", sensor_qos,
            std::bind(&OffboardControlBridge::handleVehicleLocalPosition, this, std::placeholders::_1));
        vehicle_attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", sensor_qos,
            std::bind(&OffboardControlBridge::handleVehicleAttitude, this, std::placeholders::_1));
        vehicle_imu_sub_ = create_subscription<px4_msgs::msg::VehicleImu>(
            "/fmu/out/vehicle_imu", sensor_qos,
            std::bind(&OffboardControlBridge::handleVehicleImu, this, std::placeholders::_1));
        vehicle_home_position_sub_ = create_subscription<px4_msgs::msg::HomePosition>(
            "/fmu/out/home_position", sensor_qos,
            std::bind(&OffboardControlBridge::handleHomePosition, this, std::placeholders::_1));
        distance_sensor_sub_ = create_subscription<px4_msgs::msg::DistanceSensor>(
            "/fmu/out/distance_sensor", sensor_qos,
            std::bind(&OffboardControlBridge::handleDistanceSensor, this, std::placeholders::_1));
        offboard_state_sub_ = create_subscription<std_msgs::msg::String>(
            offboard_state_topic_, 10,
            std::bind(&OffboardControlBridge::handleOffboardState, this, std::placeholders::_1));

        set_target_srv_ = create_service<traj_offboard::srv::SetTarget>(
            set_target_service_,
            std::bind(&OffboardControlBridge::handleSetTarget, this, std::placeholders::_1,
                      std::placeholders::_2));
        client_callback_group_ =
            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        get_traj_setpoint_client_ =
            create_client<traj_offboard::srv::GetTrajectorySetpoint>(
                get_traj_service_, rmw_qos_profile_services_default, client_callback_group_);

        offboard_state_.data = "SELF_CHECK";
        timer_ = create_wall_timer(50ms, std::bind(&OffboardControlBridge::controlLoopOnTimer, this));

        RCLCPP_INFO(get_logger(),
                    "Bridge ready | offboard_state=%s feedback=%s set_target=%s traj_service=%s",
                    offboard_state_topic_.c_str(), bridge_feedback_topic_.c_str(),
                    set_target_service_.c_str(), get_traj_service_.c_str());
    }

  private:
    struct Pose {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double yaw{0.0};
    };

    enum class FlightState {
        WAITING_FOR_UAV_START,
        TAKEOFF,
        OFFBOARD_ACTIVE
    };

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleImu>::SharedPtr vehicle_imu_sub_;
    rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr vehicle_home_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distance_sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr offboard_state_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
    rclcpp::Publisher<traj_offboard::msg::BridgeFeedback>::SharedPtr bridge_feedback_pub_;

    rclcpp::Service<traj_offboard::srv::SetTarget>::SharedPtr set_target_srv_;
    rclcpp::Client<traj_offboard::srv::GetTrajectorySetpoint>::SharedPtr get_traj_setpoint_client_;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string offboard_state_topic_;
    std::string bridge_feedback_topic_;
    std::string set_target_service_;
    std::string get_traj_service_;

    Pose uav_pose_{};
    double home_x_{0.0};
    double home_y_{0.0};
    double home_z_{0.0};
    double distance_m_{0.0};

    bool position_received_{false};
    bool position_raw_valid_{false};
    bool home_valid_{false};
    bool distance_received_{false};
    bool distance_raw_valid_{false};
    bool imu_received_{false};
    bool imu_raw_valid_{false};

    rclcpp::Time last_position_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_home_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_distance_sensor_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};

    std_msgs::msg::String offboard_state_;
    std::string previous_offboard_state_{"SELF_CHECK"};
    px4_msgs::msg::TrajectorySetpoint target_pose_{};
    px4_msgs::msg::TrajectorySetpoint last_cmd_{};
    bool update_target_{true};
    bool has_target_{false};
    bool pending_request_{false};

    FlightState flight_state_{FlightState::WAITING_FOR_UAV_START};
    bool takeoff_complete_{false};
    bool offboard_mode_sent_{false};
    uint64_t offboard_setpoint_counter_{0U};
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

    double feedback_timeout_s_{1.0};
    double distance_sensor_timeout_s_{1.0};
    double imu_timeout_s_{1.0};
    double takeoff_height_{5.0};
    double position_tolerance_{0.2};
    double yaw_tolerance_{0.15};
    int distance_sensor_min_signal_quality_{1};
    int offboard_warmup_setpoints_{20};

    static double wrapAngle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    uint64_t timestampUs()
    {
        return get_clock()->now().nanoseconds() / 1000;
    }

    bool isFresh(const rclcpp::Time & stamp, double timeout_s) const
    {
        return stamp.nanoseconds() != 0 && (now() - stamp).seconds() <= timeout_s;
    }

    bool hasValidPositionFeedback() const
    {
        return position_received_ && position_raw_valid_ && home_valid_ &&
               isFresh(last_position_time_, feedback_timeout_s_);
    }

    bool isTrajectoryState(const std::string & state) const
    {
        return state == "TRANSIT_TO_AREA" ||
               state == "SEARCH_ADJUST_AUTO" ||
               state == "APPROACH_PLANT" ||
               state == "SAMP_ADJUST_AUTO" ||
               state == "RETREAT" ||
               state == "BACK_HOME";
    }

    void handleVehicleLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        static constexpr double half_pi = 1.5707963267948966;
        uav_pose_.x = static_cast<double>(msg->y) - home_y_;
        uav_pose_.y = static_cast<double>(msg->x) - home_x_;
        uav_pose_.z = -static_cast<double>(msg->z) + home_z_;
        if (std::isfinite(msg->heading)) {
            uav_pose_.yaw = wrapAngle(half_pi - msg->heading);
        }
        position_raw_valid_ = msg->xy_valid && msg->z_valid && msg->heading_good_for_control;
        position_received_ = true;
        last_position_time_ = now();
    }

    void handleVehicleAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        (void)msg;
    }

    void handleVehicleImu(const px4_msgs::msg::VehicleImu::SharedPtr msg)
    {
        imu_raw_valid_ = msg->delta_angle_dt > 0 &&
                         msg->delta_velocity_dt > 0 &&
                         finiteArray(msg->delta_angle) &&
                         finiteArray(msg->delta_velocity);
        imu_received_ = true;
        last_imu_time_ = now();
    }

    void handleHomePosition(const px4_msgs::msg::HomePosition::SharedPtr msg)
    {
        home_x_ = msg->x;
        home_y_ = msg->y;
        home_z_ = msg->z;
        home_valid_ = msg->valid_lpos && msg->valid_alt;
        last_home_time_ = now();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Bridge PX4 home | valid=%s ned=(%.2f, %.2f, %.2f)",
                             home_valid_ ? "true" : "false", home_x_, home_y_, home_z_);
    }

    void handleDistanceSensor(const px4_msgs::msg::DistanceSensor::SharedPtr msg)
    {
        distance_m_ = msg->current_distance;
        distance_raw_valid_ =
            msg->mode == px4_msgs::msg::DistanceSensor::MODE_ENABLED &&
            msg->current_distance >= msg->min_distance &&
            msg->current_distance <= msg->max_distance &&
            msg->signal_quality >= distance_sensor_min_signal_quality_;
        distance_received_ = true;
        last_distance_sensor_time_ = now();
    }

    void handleOffboardState(const std_msgs::msg::String::SharedPtr msg)
    {
        offboard_state_ = *msg;
    }

    void handleSetTarget(const traj_offboard::srv::SetTarget::Request::SharedPtr request,
                         traj_offboard::srv::SetTarget::Response::SharedPtr response)
    {
        target_pose_ = request->target;
        has_target_ = true;
        update_target_ = true;
        response->success = true;
        RCLCPP_INFO(get_logger(),
                    "Bridge target accepted | enu=(%.2f, %.2f, %.2f, yaw %.2f)",
                    target_pose_.position[0], target_pose_.position[1],
                    target_pose_.position[2], target_pose_.yaw);
    }

    void publishBridgeFeedback()
    {
        traj_offboard::msg::BridgeFeedback msg{};
        msg.stamp_us = timestampUs();
        msg.position_valid = hasValidPositionFeedback();
        msg.x = uav_pose_.x;
        msg.y = uav_pose_.y;
        msg.z = uav_pose_.z;
        msg.yaw = uav_pose_.yaw;
        msg.home_valid = home_valid_;
        msg.distance_valid = distance_received_ && distance_raw_valid_ &&
                             isFresh(last_distance_sensor_time_, distance_sensor_timeout_s_);
        msg.distance_m = distance_m_;
        msg.imu_valid = imu_received_ && imu_raw_valid_ && isFresh(last_imu_time_, imu_timeout_s_);
        msg.last_position_time_us = last_position_time_.nanoseconds() / 1000;
        msg.last_home_time_us = last_home_time_.nanoseconds() / 1000;
        msg.last_distance_sensor_time_us = last_distance_sensor_time_.nanoseconds() / 1000;
        msg.last_imu_time_us = last_imu_time_.nanoseconds() / 1000;
        bridge_feedback_pub_->publish(msg);
    }

    void publishOffboardControlMode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = true;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = timestampUs();
        offboard_ctrl_mode_pub_->publish(msg);
    }

    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
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
        msg.timestamp = timestampUs();
        vehicle_cmd_pub_->publish(msg);
    }

    px4_msgs::msg::TrajectorySetpoint convertENUToNED(
        const px4_msgs::msg::TrajectorySetpoint & enu_setpoint) const
    {
        px4_msgs::msg::TrajectorySetpoint ned_setpoint = enu_setpoint;
        ned_setpoint.position[0] = enu_setpoint.position[1] + home_x_;
        ned_setpoint.position[1] = enu_setpoint.position[0] + home_y_;
        ned_setpoint.position[2] = -enu_setpoint.position[2] + home_z_;
        ned_setpoint.velocity[0] = enu_setpoint.velocity[1];
        ned_setpoint.velocity[1] = enu_setpoint.velocity[0];
        ned_setpoint.velocity[2] = -enu_setpoint.velocity[2];
        ned_setpoint.acceleration[0] = enu_setpoint.acceleration[1];
        ned_setpoint.acceleration[1] = enu_setpoint.acceleration[0];
        ned_setpoint.acceleration[2] = -enu_setpoint.acceleration[2];
        ned_setpoint.jerk[0] = enu_setpoint.jerk[1];
        ned_setpoint.jerk[1] = enu_setpoint.jerk[0];
        ned_setpoint.jerk[2] = -enu_setpoint.jerk[2];

        static constexpr float half_pi = 1.57079632679f;
        const float yaw_ned = half_pi - enu_setpoint.yaw;
        ned_setpoint.yaw = std::atan2(std::sin(yaw_ned), std::cos(yaw_ned));
        ned_setpoint.yawspeed = -enu_setpoint.yawspeed;
        return ned_setpoint;
    }

    px4_msgs::msg::TrajectorySetpoint makePositionHoldSetpoint(
        float x, float y, float z, float yaw) const
    {
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
        setpoint.yaw = yaw;
        setpoint.yawspeed = 0.0f;
        return setpoint;
    }

    px4_msgs::msg::TrajectorySetpoint defaultTakeoffSetpoint() const
    {
        return makePositionHoldSetpoint(0.0f, 0.0f, static_cast<float>(takeoff_height_), 0.0f);
    }

    px4_msgs::msg::TrajectorySetpoint publishConvertedSetpoint(
        px4_msgs::msg::TrajectorySetpoint enu_setpoint)
    {
        enu_setpoint.timestamp = timestampUs();
        traj_setpoint_pub_->publish(convertENUToNED(enu_setpoint));
        last_cmd_ = enu_setpoint;
        last_cmd_time_ = now();
        return enu_setpoint;
    }

    bool isArrivedAtPosition(const px4_msgs::msg::TrajectorySetpoint & setpoint) const
    {
        if (!hasValidPositionFeedback()) {
            return false;
        }
        const double dx = uav_pose_.x - setpoint.position[0];
        const double dy = uav_pose_.y - setpoint.position[1];
        const double dz = uav_pose_.z - setpoint.position[2];
        const double dyaw = wrapAngle(uav_pose_.yaw - setpoint.yaw);
        return std::abs(dx) <= position_tolerance_ &&
               std::abs(dy) <= position_tolerance_ &&
               std::abs(dz) <= position_tolerance_ &&
               std::abs(dyaw) <= yaw_tolerance_;
    }

    void publishTrajectorySetpoint()
    {
        if (!has_target_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Bridge trajectory | waiting for FSM target");
            publishHoldSetpoint();
            return;
        }
        if (!get_traj_setpoint_client_->service_is_ready()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "Bridge trajectory | waiting for service=%s",
                                 get_traj_service_.c_str());
            publishHoldSetpoint();
            return;
        }
        if (pending_request_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Bridge trajectory | previous service request still pending");
            return;
        }
        if (!hasValidPositionFeedback()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "Bridge trajectory | waiting for valid PX4 position feedback");
            return;
        }

        px4_msgs::msg::TrajectorySetpoint current_state{};
        current_state.position[0] = static_cast<float>(uav_pose_.x);
        current_state.position[1] = static_cast<float>(uav_pose_.y);
        current_state.position[2] = static_cast<float>(uav_pose_.z);
        current_state.yaw = static_cast<float>(uav_pose_.yaw);

        auto request = std::make_shared<traj_offboard::srv::GetTrajectorySetpoint::Request>();
        request->current_state = current_state;
        request->target = target_pose_;
        request->update_target = update_target_;
        pending_request_ = true;
        get_traj_setpoint_client_->async_send_request(
            request,
            [this](rclcpp::Client<traj_offboard::srv::GetTrajectorySetpoint>::SharedFuture future) {
                pending_request_ = false;
                try {
                    const auto response = future.get();
                    if (response->success) {
                        publishConvertedSetpoint(response->trajectory_setpoint);
                    } else {
                        RCLCPP_ERROR(get_logger(), "Bridge trajectory | service returned success=false");
                        publishConvertedSetpoint(last_cmd_);
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(get_logger(), "Bridge trajectory | service call failed: %s", e.what());
                    publishConvertedSetpoint(last_cmd_);
                }
            });
        update_target_ = false;
    }

    void publishTakeoffSetpoint(const px4_msgs::msg::TrajectorySetpoint & takeoff_setpoint)
    {
        if (!get_traj_setpoint_client_->service_is_ready()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "Bridge takeoff | waiting for service=%s",
                                 get_traj_service_.c_str());
            publishConvertedSetpoint(takeoff_setpoint);
            return;
        }
        if (pending_request_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Bridge takeoff | previous service request still pending");
            return;
        }

        px4_msgs::msg::TrajectorySetpoint current_state{};
        if (hasValidPositionFeedback()) {
            current_state.position[0] = static_cast<float>(uav_pose_.x);
            current_state.position[1] = static_cast<float>(uav_pose_.y);
            current_state.position[2] = static_cast<float>(uav_pose_.z);
            current_state.yaw = static_cast<float>(uav_pose_.yaw);
        }

        auto request = std::make_shared<traj_offboard::srv::GetTrajectorySetpoint::Request>();
        request->current_state = current_state;
        request->target = takeoff_setpoint;
        request->update_target = true;
        pending_request_ = true;
        get_traj_setpoint_client_->async_send_request(
            request,
            [this](rclcpp::Client<traj_offboard::srv::GetTrajectorySetpoint>::SharedFuture future) {
                pending_request_ = false;
                try {
                    const auto response = future.get();
                    if (response->success) {
                        publishConvertedSetpoint(response->trajectory_setpoint);
                    } else {
                        publishConvertedSetpoint(last_cmd_);
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(get_logger(), "Bridge takeoff | service call failed: %s", e.what());
                    publishConvertedSetpoint(last_cmd_);
                }
            });
    }

    void publishHoldSetpoint()
    {
        if (hasValidPositionFeedback()) {
            publishConvertedSetpoint(makePositionHoldSetpoint(
                static_cast<float>(uav_pose_.x), static_cast<float>(uav_pose_.y),
                static_cast<float>(uav_pose_.z), static_cast<float>(uav_pose_.yaw)));
            return;
        }
        if (last_cmd_time_.nanoseconds() != 0) {
            publishConvertedSetpoint(last_cmd_);
            return;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Bridge hold | waiting for valid PX4 position or previous command");
    }

    void enterTakeoff()
    {
        flight_state_ = FlightState::TAKEOFF;
        takeoff_complete_ = false;
        offboard_mode_sent_ = false;
        offboard_setpoint_counter_ = 0;
        RCLCPP_INFO(get_logger(), "Bridge state -> TAKEOFF | trigger=UAV_START");
    }

    void handleTakeoff()
    {
        const auto takeoff_setpoint = has_target_ ? target_pose_ : defaultTakeoffSetpoint();
        publishTakeoffSetpoint(takeoff_setpoint);

        if (offboard_setpoint_counter_ < static_cast<uint64_t>(offboard_warmup_setpoints_)) {
            ++offboard_setpoint_counter_;
            return;
        }

        if (!offboard_mode_sent_) {
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
            offboard_mode_sent_ = true;
            RCLCPP_INFO(get_logger(), "Bridge PX4 command | mode=OFFBOARD arm=true");
        }

        if (isArrivedAtPosition(takeoff_setpoint)) {
            takeoff_complete_ = true;
            flight_state_ = FlightState::OFFBOARD_ACTIVE;
            RCLCPP_INFO(get_logger(),
                        "Bridge state -> OFFBOARD_ACTIVE | takeoff complete pos=(%.2f, %.2f, %.2f)",
                        uav_pose_.x, uav_pose_.y, uav_pose_.z);
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                                 "Bridge takeoff | pos=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f)",
                                 uav_pose_.x, uav_pose_.y, uav_pose_.z,
                                 takeoff_setpoint.position[0],
                                 takeoff_setpoint.position[1],
                                 takeoff_setpoint.position[2]);
        }
    }

    void controlLoopOnTimer()
    {
        publishOffboardControlMode();
        publishBridgeFeedback();

        const auto state = offboard_state_.data;
        const bool entered_uav_start =
            state == "UAV_START" && previous_offboard_state_ != "UAV_START";
        if (entered_uav_start) {
            enterTakeoff();
            handleTakeoff();
            previous_offboard_state_ = state;
            return;
        }

        if (flight_state_ == FlightState::WAITING_FOR_UAV_START) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Bridge waiting | expected FSM state UAV_START current=%s",
                                 state.c_str());
            previous_offboard_state_ = state;
            return;
        }

        if (flight_state_ == FlightState::TAKEOFF) {
            handleTakeoff();
            previous_offboard_state_ = state;
            return;
        }

        if (isTrajectoryState(state)) {
            publishTrajectorySetpoint();
        } else {
            publishHoldSetpoint();
        }
        previous_offboard_state_ = state;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControlBridge>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
