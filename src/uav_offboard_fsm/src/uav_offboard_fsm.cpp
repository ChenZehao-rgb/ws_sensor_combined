#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <traj_offboard/srv/set_target.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class UavOffboardFsm : public rclcpp::Node {
  public:
    UavOffboardFsm() : rclcpp::Node("uav_offboard_fsm") {
        takeoff_height_ = declare_parameter<double>("takeoff_height", 5.0);
        position_tolerance_ = declare_parameter<double>("position_tolerance", 0.25);
        yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.15);
        mission_enabled_ = declare_parameter<bool>("mission_enabled", true);
        require_distance_sensor_ = declare_parameter<bool>("require_distance_sensor", false);
        distance_sensor_timeout_s_ = declare_parameter<double>("distance_sensor_timeout_s", 1.0);
        state_feedback_timeout_s_ = declare_parameter<double>("state_feedback_timeout_s", 1.0);
        search_yaw_offset_rad_ = declare_parameter<double>("search_yaw_offset_rad", 0.35);
        search_lateral_offset_m_ = declare_parameter<double>("search_lateral_offset_m", 0.4);
        approach_distance_m_ = declare_parameter<double>("approach_distance_m", 1.0);
        approach_target_distance_m_ = declare_parameter<double>("approach_target_distance_m", 0.7);
        approach_distance_tolerance_m_ = declare_parameter<double>("approach_distance_tolerance_m", 0.1);
        retreat_distance_m_ = declare_parameter<double>("retreat_distance_m", 1.0);

        const auto state_feedback_topic =
            declare_parameter<std::string>("state_feedback_topic", "/online_traj_generator/ruckig_state");
        const auto distance_sensor_topic =
            declare_parameter<std::string>("distance_sensor_topic", "/fmu/out/distance_sensor");
        const auto vehicle_local_position_topic =
            declare_parameter<std::string>("vehicle_local_position_topic", "/fmu/out/vehicle_local_position");
        const auto home_position_topic =
            declare_parameter<std::string>("home_position_topic", "/fmu/out/home_position");

        const std::vector<double> default_transit = {
            0.0, 0.0, takeoff_height_, 0.0,
            5.0, 0.0, takeoff_height_, 0.0,
            5.0, 5.0, takeoff_height_, 1.57079632679,
        };
        transit_waypoints_ =
            parseWaypointParameter(declare_parameter<std::vector<double>>("transit_waypoints", default_transit));
        if (transit_waypoints_.empty()) {
            transit_waypoints_ = {
                {0.0, 0.0, takeoff_height_, 0.0},
                {5.0, 0.0, takeoff_height_, 0.0},
                {5.0, 5.0, takeoff_height_, 1.57079632679},
            };
        }

        takeoff_waypoint_ = {0.0, 0.0, takeoff_height_, 0.0};
        home_waypoint_ = {0.0, 0.0, takeoff_height_, 0.0};

        offboard_state_pub_ =
            create_publisher<std_msgs::msg::String>("/uav_offboard_fsm/offboard_state", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/uav_offboard_fsm/status", 10);

        set_target_client_ =
            create_client<traj_offboard::srv::SetTarget>("online_traj_generator/set_target");

        control_command_sub_ = create_subscription<std_msgs::msg::String>(
            "/uav_offboard_fsm/control_command", 10,
            std::bind(&UavOffboardFsm::handleControlCommand, this, std::placeholders::_1));

        mission_state_sub_ = create_subscription<std_msgs::msg::String>(
            "/uav_offboard_fsm/mission_state", 10,
            std::bind(&UavOffboardFsm::handleMissionState, this, std::placeholders::_1));

        ruckig_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            state_feedback_topic, 10,
            std::bind(&UavOffboardFsm::handleRuckigState, this, std::placeholders::_1));

        vehicle_local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            vehicle_local_position_topic, rclcpp::SensorDataQoS(),
            std::bind(&UavOffboardFsm::handleVehicleLocalPosition, this, std::placeholders::_1));

        home_position_sub_ = create_subscription<px4_msgs::msg::HomePosition>(
            home_position_topic, rclcpp::SensorDataQoS(),
            std::bind(&UavOffboardFsm::handleHomePosition, this, std::placeholders::_1));

        distance_sensor_sub_ = create_subscription<px4_msgs::msg::DistanceSensor>(
            distance_sensor_topic, rclcpp::SensorDataQoS(),
            std::bind(&UavOffboardFsm::handleDistanceSensor, this, std::placeholders::_1));

        timer_ = create_wall_timer(50ms, std::bind(&UavOffboardFsm::controlLoopOnTimer, this));
    }

  private:
    struct Waypoint {
        double x;
        double y;
        double z;
        double yaw;
    };

    struct ManualCommand {
        double dx{0.0};
        double dy{0.0};
        double dz{0.0};
        double dyaw{0.0};
        int confirmations{0};
        bool target_prepared{false};
    };

    enum class ControlState {
        SELF_CHECK,
        TAKEOFF,
        HOVERING,
        TRANSIT_TO_AREA,
        SEARCH_ADJUST,
        APPROACH_PLANT,
        RETREAT,
        BACK_HOME,
        MANUAL_OPERA
    };

    enum class CommandType {
        TRANSIT_TO_AREA,
        SEARCH_ADJUST,
        APPROACH_PLANT,
        RETREAT,
        BACK_HOME,
        MANUAL_OPERA,
        CONFIRM,
        SELF_CHECK,
        TAKEOFF
    };

    struct ParsedCommand {
        CommandType type;
        std::optional<ManualCommand> manual;
    };

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr offboard_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_state_sub_;
    rclcpp::Client<traj_offboard::srv::SetTarget>::SharedPtr set_target_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ruckig_state_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr home_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distance_sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<ControlState> control_state_{ControlState::SELF_CHECK};
    ControlState previous_state_{ControlState::SELF_CHECK};
    mutable std::mutex fsm_mutex_;

    std::optional<sensor_msgs::msg::JointState> latest_actual_state_;
    std::optional<sensor_msgs::msg::JointState> latest_reference_state_;
    rclcpp::Time last_actual_state_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_reference_state_time_{0, 0, RCL_ROS_TIME};
    double home_x_{0.0};
    double home_y_{0.0};
    double home_z_{0.0};
    mutable std::mutex latest_state_mutex_;
    std::optional<double> latest_distance_m_;
    rclcpp::Time last_distance_sensor_time_{0, 0, RCL_ROS_TIME};

    bool ready_for_takeoff_{false};
    bool self_check_requested_{false};
    bool ready_for_transit_{false};
    bool is_arrived_task_aera_{false};
    bool adjust_completed_{false};
    bool approach_completed_{false};
    bool back_home_{false};
    bool mission_enabled_{true};
    bool require_distance_sensor_{false};

    double takeoff_height_{5.0};
    double position_tolerance_{0.25};
    double yaw_tolerance_{0.15};
    double distance_sensor_timeout_s_{1.0};
    double state_feedback_timeout_s_{1.0};
    double search_yaw_offset_rad_{0.35};
    double search_lateral_offset_m_{0.4};
    double approach_distance_m_{1.0};
    double approach_target_distance_m_{0.7};
    double approach_distance_tolerance_m_{0.1};
    double retreat_distance_m_{1.0};

    Waypoint takeoff_waypoint_{};
    Waypoint home_waypoint_{};
    std::vector<Waypoint> transit_waypoints_;
    std::vector<Waypoint> search_waypoints_;
    std::vector<Waypoint> approach_waypoints_;
    std::vector<Waypoint> retreat_waypoints_;
    std::vector<Waypoint> back_home_waypoints_;
    std::size_t transit_index_{0};
    std::size_t search_index_{0};
    std::size_t approach_index_{0};
    std::size_t retreat_index_{0};
    std::size_t back_home_index_{0};

    std::optional<Waypoint> active_target_;
    bool active_target_sent_{false};
    bool target_request_pending_{false};
    rclcpp::Time last_target_sent_time_{0, 0, RCL_ROS_TIME};

    ManualCommand manual_command_{};

    void controlLoopOnTimer();
    void onStateEntry(ControlState state);
    void transitionTo(ControlState state);

    void handleSelfCheck();
    void handleTakeoff();
    void handleHovering();
    void handleTransitToArea();
    void handleSearchAdjust();
    void handleApproachPlant();
    void handleRetreat();
    void handleBackHome();
    void handleManualOperation();

    bool handleWaypointSequence(std::vector<Waypoint> & waypoints, std::size_t & index,
                                const std::string & label);
    bool handleActiveTargetReached();
    void setActiveTarget(const Waypoint & waypoint);
    void clearActiveTarget();
    void sendActiveTarget();

    bool isSelfCheckOK();
    bool isUAVTakeoff();
    bool isWaypointReached(const Waypoint & waypoint, const sensor_msgs::msg::JointState & state);
    std::optional<Waypoint> currentWaypoint();
    Waypoint currentOrHoverWaypoint();
    bool hasFreshDistanceSensor();
    bool isFreshStateTime(const rclcpp::Time & stamp) const;

    void generateSearchAdjustWaypoints();
    void generateApproachWaypoints();
    void generateRetreatWaypoints();
    Waypoint offsetBodyFrame(const Waypoint & base, double forward_m, double right_m) const;

    void publishOffboardState(ControlState state);
    void publishStatus(ControlState state);
    void handleControlCommand(const std_msgs::msg::String::SharedPtr msg);
    void handleMissionState(const std_msgs::msg::String::SharedPtr msg);
    void handleRuckigState(const sensor_msgs::msg::JointState::SharedPtr msg);
    void handleVehicleLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void handleHomePosition(const px4_msgs::msg::HomePosition::SharedPtr msg);
    void handleDistanceSensor(const px4_msgs::msg::DistanceSensor::SharedPtr msg);

    static std::vector<Waypoint> parseWaypointParameter(const std::vector<double> & flat);
    static std::string stateToString(ControlState state);
    static std::optional<ParsedCommand> parseCommand(const std::string & command);
    static std::string upperCopy(std::string value);
    static double wrapAngle(double angle);
};

void UavOffboardFsm::controlLoopOnTimer()
{
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    const auto current_state = control_state_.load();
    const bool state_changed = current_state != previous_state_;
    if (state_changed) {
        onStateEntry(current_state);
    }

    publishOffboardState(current_state);
    publishStatus(current_state);

    switch (current_state) {
        case ControlState::SELF_CHECK:
            handleSelfCheck();
            break;
        case ControlState::TAKEOFF:
            handleTakeoff();
            break;
        case ControlState::HOVERING:
            handleHovering();
            break;
        case ControlState::TRANSIT_TO_AREA:
            handleTransitToArea();
            break;
        case ControlState::SEARCH_ADJUST:
            handleSearchAdjust();
            break;
        case ControlState::APPROACH_PLANT:
            handleApproachPlant();
            break;
        case ControlState::RETREAT:
            handleRetreat();
            break;
        case ControlState::BACK_HOME:
            handleBackHome();
            break;
        case ControlState::MANUAL_OPERA:
            handleManualOperation();
            break;
    }

    previous_state_ = current_state;
}

void UavOffboardFsm::onStateEntry(ControlState state)
{
    clearActiveTarget();
    RCLCPP_INFO(get_logger(), "FSM entered %s", stateToString(state).c_str());

    switch (state) {
        case ControlState::SELF_CHECK:
            ready_for_takeoff_ = false;
            ready_for_transit_ = false;
            is_arrived_task_aera_ = false;
            adjust_completed_ = false;
            approach_completed_ = false;
            back_home_ = false;
            manual_command_ = {};
            break;
        case ControlState::TRANSIT_TO_AREA:
            transit_index_ = 0;
            is_arrived_task_aera_ = false;
            break;
        case ControlState::SEARCH_ADJUST:
            search_index_ = 0;
            adjust_completed_ = false;
            generateSearchAdjustWaypoints();
            break;
        case ControlState::APPROACH_PLANT:
            approach_index_ = 0;
            approach_completed_ = false;
            generateApproachWaypoints();
            break;
        case ControlState::RETREAT:
            retreat_index_ = 0;
            generateRetreatWaypoints();
            break;
        case ControlState::BACK_HOME:
            back_home_index_ = 0;
            back_home_ = false;
            back_home_waypoints_ = {home_waypoint_};
            break;
        case ControlState::MANUAL_OPERA:
            manual_command_.confirmations = 0;
            manual_command_.target_prepared = false;
            break;
        case ControlState::TAKEOFF:
        case ControlState::HOVERING:
            break;
    }
}

void UavOffboardFsm::transitionTo(ControlState state)
{
    if (control_state_.load() == state) {
        return;
    }
    control_state_.store(state);
}

void UavOffboardFsm::handleSelfCheck()
{
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "State: SELF_CHECK");
    if (!self_check_requested_ && !ready_for_takeoff_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waiting for keyboard SELF_CHECK command");
        return;
    }

    if (ready_for_takeoff_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Self check passed; waiting for keyboard TAKEOFF command");
        return;
    }

    if (!isSelfCheckOK()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Self check pending: mission_enabled=%s distance_sensor_ok=%s",
                             mission_enabled_ ? "true" : "false",
                             hasFreshDistanceSensor() ? "true" : "false");
        return;
    }

    ready_for_takeoff_ = true;
    self_check_requested_ = false;
    RCLCPP_INFO(get_logger(), "Self check passed, ready_for_takeoff=1; waiting for keyboard TAKEOFF command");
}

void UavOffboardFsm::handleTakeoff()
{
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "State: TAKEOFF");
    if (!ready_for_takeoff_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Takeoff blocked before self check");
        transitionTo(ControlState::SELF_CHECK);
        return;
    }

    if (!isUAVTakeoff()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for takeoff completion");
        return;
    }

    ready_for_transit_ = true;
    RCLCPP_INFO(get_logger(), "Takeoff completed, ready_for_transit=1");
    transitionTo(ControlState::HOVERING);
}

void UavOffboardFsm::handleHovering()
{
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "State: HOVERING, waiting for transit/search/approach/retreat/home/manual command");
}

void UavOffboardFsm::handleTransitToArea()
{
    if (!ready_for_transit_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TRANSIT_TO_AREA blocked until ready_for_transit=1");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (handleWaypointSequence(transit_waypoints_, transit_index_, "transit")) {
        is_arrived_task_aera_ = true;
        RCLCPP_INFO(get_logger(), "Transit completed, is_arrived_task_aera=1");
        transitionTo(ControlState::HOVERING);
    }
}

void UavOffboardFsm::handleSearchAdjust()
{
    if (!is_arrived_task_aera_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "SEARCH_ADJUST blocked until is_arrived_task_aera=1");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (handleWaypointSequence(search_waypoints_, search_index_, "search adjust")) {
        adjust_completed_ = true;
        RCLCPP_INFO(get_logger(), "Search adjust completed, adjust_completed=1");
        transitionTo(ControlState::HOVERING);
    }
}

void UavOffboardFsm::handleApproachPlant()
{
    if (!adjust_completed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "APPROACH_PLANT blocked until adjust_completed=1");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (latest_distance_m_ &&
        *latest_distance_m_ <= approach_target_distance_m_ + approach_distance_tolerance_m_) {
        approach_completed_ = true;
        clearActiveTarget();
        RCLCPP_INFO(get_logger(), "Approach completed by distance sensor, approach_completed=1");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (handleWaypointSequence(approach_waypoints_, approach_index_, "approach")) {
        approach_completed_ = true;
        RCLCPP_INFO(get_logger(), "Approach completed by target arrival, approach_completed=1");
        transitionTo(ControlState::HOVERING);
    }
}

void UavOffboardFsm::handleRetreat()
{
    if (!approach_completed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "RETREAT blocked until approach_completed=1");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (handleWaypointSequence(retreat_waypoints_, retreat_index_, "retreat")) {
        adjust_completed_ = true;
        approach_completed_ = false;
        RCLCPP_INFO(get_logger(), "Retreat completed, adjust_completed=1 approach_completed=0");
        transitionTo(ControlState::HOVERING);
    }
}

void UavOffboardFsm::handleBackHome()
{
    if (!ready_for_transit_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "BACK_HOME blocked until takeoff is complete");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (handleWaypointSequence(back_home_waypoints_, back_home_index_, "back home")) {
        back_home_ = true;
        RCLCPP_INFO(get_logger(), "Back home completed, back_home=1");
        transitionTo(ControlState::HOVERING);
    }
}

void UavOffboardFsm::handleManualOperation()
{
    if (manual_command_.confirmations < 2) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "MANUAL_OPERA waiting for double confirmation (%d/2)",
                             manual_command_.confirmations);
        return;
    }

    if (!manual_command_.target_prepared) {
        const auto base = currentOrHoverWaypoint();
        const Waypoint target{
            base.x + manual_command_.dx,
            base.y + manual_command_.dy,
            base.z + manual_command_.dz,
            wrapAngle(base.yaw + manual_command_.dyaw)};
        setActiveTarget(target);
        manual_command_.target_prepared = true;
        RCLCPP_INFO(get_logger(),
                    "Manual target accepted after double confirmation: dx=%.2f dy=%.2f dz=%.2f dyaw=%.2f",
                    manual_command_.dx, manual_command_.dy, manual_command_.dz,
                    manual_command_.dyaw);
    }

    if (handleActiveTargetReached()) {
        manual_command_ = {};
        RCLCPP_INFO(get_logger(), "Manual operation completed");
        transitionTo(ControlState::HOVERING);
    }
}

bool UavOffboardFsm::handleWaypointSequence(std::vector<Waypoint> & waypoints,
                                            std::size_t & index,
                                            const std::string & label)
{
    if (waypoints.empty()) {
        RCLCPP_WARN(get_logger(), "%s waypoint list is empty; treating stage as complete", label.c_str());
        return true;
    }

    if (index >= waypoints.size()) {
        return true;
    }

    if (!active_target_) {
        setActiveTarget(waypoints[index]);
        RCLCPP_INFO(get_logger(), "Dispatching %s waypoint %zu/%zu: %.2f %.2f %.2f yaw %.2f",
                    label.c_str(), index + 1, waypoints.size(), active_target_->x,
                    active_target_->y, active_target_->z, active_target_->yaw);
    }

    if (!handleActiveTargetReached()) {
        return false;
    }

    RCLCPP_INFO(get_logger(), "%s waypoint %zu reached", label.c_str(), index + 1);
    ++index;
    clearActiveTarget();
    return index >= waypoints.size();
}

bool UavOffboardFsm::handleActiveTargetReached()
{
    if (!active_target_) {
        return false;
    }

    if (!active_target_sent_ && !target_request_pending_) {
        sendActiveTarget();
    }

    if (!active_target_sent_) {
        return false;
    }

    const auto waypoint = currentWaypoint();

    if (!waypoint) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waiting for UAV position feedback");
        return false;
    }

    sensor_msgs::msg::JointState state_msg;
    state_msg.position = {waypoint->x, waypoint->y, waypoint->z, waypoint->yaw};
    return isWaypointReached(*active_target_, state_msg);
}

void UavOffboardFsm::setActiveTarget(const Waypoint & waypoint)
{
    active_target_ = waypoint;
    active_target_sent_ = false;
    target_request_pending_ = false;
}

void UavOffboardFsm::clearActiveTarget()
{
    active_target_.reset();
    active_target_sent_ = false;
    target_request_pending_ = false;
}

void UavOffboardFsm::sendActiveTarget()
{
    if (!active_target_) {
        return;
    }
    if (!set_target_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "traj_offboard set_target service not available");
        return;
    }

    auto request = std::make_shared<traj_offboard::srv::SetTarget::Request>();
    request->target.position = {
        static_cast<float>(active_target_->x),
        static_cast<float>(active_target_->y),
        static_cast<float>(active_target_->z)};
    request->target.velocity = {0.0f, 0.0f, 0.0f};
    request->target.acceleration = {0.0f, 0.0f, 0.0f};
    request->target.yaw = static_cast<float>(active_target_->yaw);
    request->target.yawspeed = 0.0f;

    const auto target = *active_target_;
    target_request_pending_ = true;
    last_target_sent_time_ = now();
    set_target_client_->async_send_request(
        request,
        [this, target](rclcpp::Client<traj_offboard::srv::SetTarget>::SharedFuture resp_fut) {
            std::lock_guard<std::mutex> lock(fsm_mutex_);
            target_request_pending_ = false;
            try {
                const auto resp = resp_fut.get();
                if (resp->success) {
                    active_target_sent_ = true;
                    RCLCPP_INFO(get_logger(), "Target sent to traj_offboard: %.2f %.2f %.2f yaw %.2f",
                                target.x, target.y, target.z, target.yaw);
                } else {
                    active_target_sent_ = false;
                    RCLCPP_ERROR(get_logger(), "traj_offboard rejected target");
                }
            } catch (const std::exception & e) {
                active_target_sent_ = false;
                RCLCPP_ERROR(get_logger(), "set_target service call failed: %s", e.what());
            }
        });
}

bool UavOffboardFsm::isSelfCheckOK()
{
    return mission_enabled_ && (!require_distance_sensor_ || hasFreshDistanceSensor());
}

bool UavOffboardFsm::isUAVTakeoff()
{
    const auto state_copy = currentWaypoint();
    if (!state_copy) {
        return false;
    }
    sensor_msgs::msg::JointState state_msg;
    state_msg.position = {state_copy->x, state_copy->y, state_copy->z, state_copy->yaw};
    return isWaypointReached(takeoff_waypoint_, state_msg);
}

bool UavOffboardFsm::isWaypointReached(const Waypoint & waypoint,
                                       const sensor_msgs::msg::JointState & state)
{
    if (state.position.size() < 4) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Joint state has insufficient position entries: %zu",
                             state.position.size());
        return false;
    }

    const double dx = waypoint.x - state.position[0];
    const double dy = waypoint.y - state.position[1];
    const double dz = waypoint.z - state.position[2];
    const double yaw_error = wrapAngle(waypoint.yaw - state.position[3]);

    return std::abs(dx) <= position_tolerance_ &&
           std::abs(dy) <= position_tolerance_ &&
           std::abs(dz) <= position_tolerance_ &&
           std::abs(yaw_error) <= yaw_tolerance_;
}

std::optional<UavOffboardFsm::Waypoint> UavOffboardFsm::currentWaypoint()
{
    std::optional<sensor_msgs::msg::JointState> state_copy;
    {
        std::lock_guard<std::mutex> lock(latest_state_mutex_);
        if (latest_actual_state_ && isFreshStateTime(last_actual_state_time_)) {
            state_copy = latest_actual_state_;
        } else if (latest_reference_state_ && isFreshStateTime(last_reference_state_time_)) {
            state_copy = latest_reference_state_;
        }
    }
    if (!state_copy || state_copy->position.size() < 4) {
        return std::nullopt;
    }
    return Waypoint{
        state_copy->position[0],
        state_copy->position[1],
        state_copy->position[2],
        state_copy->position[3]};
}

UavOffboardFsm::Waypoint UavOffboardFsm::currentOrHoverWaypoint()
{
    const auto current = currentWaypoint();
    if (current) {
        return *current;
    }
    return {0.0, 0.0, takeoff_height_, 0.0};
}

bool UavOffboardFsm::isFreshStateTime(const rclcpp::Time & stamp) const
{
    if (stamp.nanoseconds() == 0) {
        return false;
    }
    return (now() - stamp).seconds() <= state_feedback_timeout_s_;
}

bool UavOffboardFsm::hasFreshDistanceSensor()
{
    if (!latest_distance_m_) {
        return false;
    }
    if (last_distance_sensor_time_.nanoseconds() == 0) {
        return false;
    }
    return (now() - last_distance_sensor_time_).seconds() <= distance_sensor_timeout_s_;
}

void UavOffboardFsm::generateSearchAdjustWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    search_waypoints_.clear();
    search_waypoints_.push_back({base.x, base.y, base.z, wrapAngle(base.yaw + search_yaw_offset_rad_)});
    search_waypoints_.push_back(offsetBodyFrame(base, 0.0, -search_lateral_offset_m_));
    search_waypoints_.push_back(offsetBodyFrame(base, 0.0, search_lateral_offset_m_));
    search_waypoints_.push_back({base.x, base.y, base.z, base.yaw});
}

void UavOffboardFsm::generateApproachWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    double travel_distance = approach_distance_m_;
    if (latest_distance_m_) {
        travel_distance =
            std::clamp(*latest_distance_m_ - approach_target_distance_m_, 0.0, approach_distance_m_);
    }

    approach_waypoints_.clear();
    if (travel_distance <= approach_distance_tolerance_m_) {
        return;
    }
    approach_waypoints_.push_back(offsetBodyFrame(base, travel_distance, 0.0));
}

void UavOffboardFsm::generateRetreatWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    retreat_waypoints_.clear();
    retreat_waypoints_.push_back(offsetBodyFrame(base, -retreat_distance_m_, 0.0));
}

UavOffboardFsm::Waypoint UavOffboardFsm::offsetBodyFrame(const Waypoint & base,
                                                         double forward_m,
                                                         double right_m) const
{
    const double forward_x = std::cos(base.yaw);
    const double forward_y = std::sin(base.yaw);
    const double right_x = std::sin(base.yaw);
    const double right_y = -std::cos(base.yaw);
    return {
        base.x + forward_m * forward_x + right_m * right_x,
        base.y + forward_m * forward_y + right_m * right_y,
        base.z,
        base.yaw};
}

void UavOffboardFsm::publishOffboardState(ControlState state)
{
    std_msgs::msg::String msg;
    msg.data = stateToString(state);
    offboard_state_pub_->publish(msg);
}

void UavOffboardFsm::publishStatus(ControlState state)
{
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "state=" << stateToString(state)
        << " self_check_requested=" << (self_check_requested_ ? 1 : 0)
        << " ready_for_takeoff=" << (ready_for_takeoff_ ? 1 : 0)
        << " ready_for_transit=" << (ready_for_transit_ ? 1 : 0)
        << " is_arrived_task_aera=" << (is_arrived_task_aera_ ? 1 : 0)
        << " adjust_completed=" << (adjust_completed_ ? 1 : 0)
        << " approach_completed=" << (approach_completed_ ? 1 : 0)
        << " back_home=" << (back_home_ ? 1 : 0);
    msg.data = out.str();
    status_pub_->publish(msg);
}

void UavOffboardFsm::handleControlCommand(const std_msgs::msg::String::SharedPtr msg)
{
    const auto parsed = parseCommand(msg->data);
    if (!parsed) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Unknown control command: %s", msg->data.c_str());
        return;
    }

    std::lock_guard<std::mutex> lock(fsm_mutex_);
    const auto state = control_state_.load();

    if (parsed->type == CommandType::CONFIRM) {
        if (state != ControlState::MANUAL_OPERA) {
            RCLCPP_WARN(get_logger(), "CONFIRM ignored outside MANUAL_OPERA");
            return;
        }
        manual_command_.confirmations = std::min(2, manual_command_.confirmations + 1);
        RCLCPP_INFO(get_logger(), "Manual confirmation accepted (%d/2)",
                    manual_command_.confirmations);
        return;
    }

    if (parsed->type == CommandType::SELF_CHECK) {
        ready_for_takeoff_ = false;
        ready_for_transit_ = false;
        is_arrived_task_aera_ = false;
        adjust_completed_ = false;
        approach_completed_ = false;
        back_home_ = false;
        manual_command_ = {};
        self_check_requested_ = true;
        transitionTo(ControlState::SELF_CHECK);
        RCLCPP_INFO(get_logger(), "Keyboard SELF_CHECK command accepted");
        return;
    }

    if (parsed->type == CommandType::TAKEOFF) {
        if (state == ControlState::SELF_CHECK && ready_for_takeoff_) {
            transitionTo(ControlState::TAKEOFF);
            RCLCPP_INFO(get_logger(), "Keyboard TAKEOFF command accepted");
        } else {
            RCLCPP_WARN(get_logger(),
                        "TAKEOFF ignored while state=%s ready_for_takeoff=%d",
                        stateToString(state).c_str(), ready_for_takeoff_ ? 1 : 0);
        }
        return;
    }

    if (state != ControlState::HOVERING) {
        RCLCPP_WARN(get_logger(), "Command %s ignored while state=%s; commands are accepted from HOVERING",
                    msg->data.c_str(), stateToString(state).c_str());
        return;
    }

    switch (parsed->type) {
        case CommandType::TRANSIT_TO_AREA:
            transitionTo(ControlState::TRANSIT_TO_AREA);
            break;
        case CommandType::SEARCH_ADJUST:
            transitionTo(ControlState::SEARCH_ADJUST);
            break;
        case CommandType::APPROACH_PLANT:
            transitionTo(ControlState::APPROACH_PLANT);
            break;
        case CommandType::RETREAT:
            transitionTo(ControlState::RETREAT);
            break;
        case CommandType::BACK_HOME:
            transitionTo(ControlState::BACK_HOME);
            break;
        case CommandType::MANUAL_OPERA:
            if (!parsed->manual) {
                RCLCPP_WARN(get_logger(), "MANUAL_OPERA requires: MANUAL x y z yaw");
                return;
            }
            manual_command_ = *parsed->manual;
            transitionTo(ControlState::MANUAL_OPERA);
            break;
        case CommandType::CONFIRM:
        case CommandType::SELF_CHECK:
        case CommandType::TAKEOFF:
            break;
    }
}

void UavOffboardFsm::handleMissionState(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    const auto state = upperCopy(msg->data);
    if (state == "ENABLED" || state == "RUNNING" || state == "AUTO") {
        mission_enabled_ = true;
    } else if (state == "DISABLED" || state == "ABORT" || state == "STOP") {
        mission_enabled_ = false;
    } else {
        RCLCPP_WARN(get_logger(), "Unknown mission state: %s", msg->data.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "Mission enabled set to %s", mission_enabled_ ? "true" : "false");
}

void UavOffboardFsm::handleRuckigState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    latest_reference_state_ = *msg;
    last_reference_state_time_ = now();
}

void UavOffboardFsm::handleVehicleLocalPosition(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    state.name = {"pos_x", "pos_y", "pos_z", "yaw"};

    double home_x = 0.0;
    double home_y = 0.0;
    double home_z = 0.0;
    double previous_yaw = 0.0;
    bool has_previous_yaw = false;
    {
        std::lock_guard<std::mutex> lock(latest_state_mutex_);
        home_x = home_x_;
        home_y = home_y_;
        home_z = home_z_;
        if (latest_actual_state_ && latest_actual_state_->position.size() >= 4) {
            previous_yaw = latest_actual_state_->position[3];
            has_previous_yaw = true;
        }
    }

    const double yaw =
        std::isfinite(msg->heading) ? wrapAngle(1.5707963267948966 - msg->heading) :
                                      (has_previous_yaw ? previous_yaw : 0.0);

    state.position = {
        static_cast<double>(msg->y) - home_y,
        static_cast<double>(msg->x) - home_x,
        -static_cast<double>(msg->z) + home_z,
        yaw};

    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    latest_actual_state_ = state;
    last_actual_state_time_ = state.header.stamp;
}

void UavOffboardFsm::handleHomePosition(const px4_msgs::msg::HomePosition::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    home_x_ = msg->x;
    home_y_ = msg->y;
    home_z_ = msg->z;
}

void UavOffboardFsm::handleDistanceSensor(const px4_msgs::msg::DistanceSensor::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    if (msg->current_distance >= msg->min_distance && msg->current_distance <= msg->max_distance &&
        msg->signal_quality != 0) {
        latest_distance_m_ = msg->current_distance;
        last_distance_sensor_time_ = now();
    }
}

std::vector<UavOffboardFsm::Waypoint>
UavOffboardFsm::parseWaypointParameter(const std::vector<double> & flat)
{
    std::vector<Waypoint> waypoints;
    if (flat.size() % 4 != 0) {
        return waypoints;
    }
    for (std::size_t i = 0; i < flat.size(); i += 4) {
        waypoints.push_back({flat[i], flat[i + 1], flat[i + 2], flat[i + 3]});
    }
    return waypoints;
}

std::string UavOffboardFsm::stateToString(ControlState state)
{
    switch (state) {
        case ControlState::SELF_CHECK:
            return "SELF_CHECK";
        case ControlState::TAKEOFF:
            return "TAKEOFF";
        case ControlState::HOVERING:
            return "HOVERING";
        case ControlState::TRANSIT_TO_AREA:
            return "TRANSIT_TO_AREA";
        case ControlState::SEARCH_ADJUST:
            return "SEARCH_ADJUST";
        case ControlState::APPROACH_PLANT:
            return "APPROACH_PLANT";
        case ControlState::RETREAT:
            return "RETREAT";
        case ControlState::BACK_HOME:
            return "BACK_HOME";
        case ControlState::MANUAL_OPERA:
            return "MANUAL_OPERA";
    }
    return "UNKNOWN";
}

std::optional<UavOffboardFsm::ParsedCommand>
UavOffboardFsm::parseCommand(const std::string & command)
{
    std::istringstream stream(command);
    std::string token;
    stream >> token;
    if (token.empty()) {
        return std::nullopt;
    }
    token = upperCopy(token);

    if (token == "TRANSIT_TO_AREA" || token == "TRANSIT" || token == "TRAJECTORY_TRACKING" ||
        token == "R") {
        return ParsedCommand{CommandType::TRANSIT_TO_AREA, std::nullopt};
    }
    if (token == "SEARCH_ADJUST" || token == "SEARCH" || token == "ADJUST" || token == "A") {
        return ParsedCommand{CommandType::SEARCH_ADJUST, std::nullopt};
    }
    if (token == "APPROACH_PLANT" || token == "APPROACH" || token == "P") {
        return ParsedCommand{CommandType::APPROACH_PLANT, std::nullopt};
    }
    if (token == "RETREAT" || token == "E") {
        return ParsedCommand{CommandType::RETREAT, std::nullopt};
    }
    if (token == "BACK_HOME" || token == "HOME" || token == "B") {
        return ParsedCommand{CommandType::BACK_HOME, std::nullopt};
    }
    if (token == "CONFIRM" || token == "YES" || token == "Y") {
        return ParsedCommand{CommandType::CONFIRM, std::nullopt};
    }
    if (token == "SELF_CHECK" || token == "SELFCHECK" || token == "RESET" || token == "S") {
        return ParsedCommand{CommandType::SELF_CHECK, std::nullopt};
    }
    if (token == "TAKEOFF" || token == "T") {
        return ParsedCommand{CommandType::TAKEOFF, std::nullopt};
    }
    if (token == "MANUAL_OPERA" || token == "MANUAL" || token == "M") {
        ManualCommand manual;
        if (!(stream >> manual.dx >> manual.dy >> manual.dz >> manual.dyaw)) {
            return ParsedCommand{CommandType::MANUAL_OPERA, std::nullopt};
        }
        return ParsedCommand{CommandType::MANUAL_OPERA, manual};
    }

    return std::nullopt;
}

std::string UavOffboardFsm::upperCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return value;
}

double UavOffboardFsm::wrapAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UavOffboardFsm>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
