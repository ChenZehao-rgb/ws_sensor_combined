#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <traj_offboard/srv/set_target.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

// 正整数参数归一化：ROS2 整数参数在 Humble 中通常是 long，这里统一限制为 >=1 的 int。
int positiveInt(long value)
{
    return static_cast<int>(std::max(1L, value));
}

}  // namespace

class UavOffboardFsm : public rclcpp::Node {
  public:
    // 构造函数：声明并读取所有 ROS2 参数，随后按参数配置创建发布器、订阅器、服务客户端和状态机定时器。
    UavOffboardFsm() : rclcpp::Node("uav_offboard_fsm") {
        const std::vector<double> default_takeoff = {0.0, 0.0, 5.0, 0.0};
        takeoff_waypoint_ =
            parseSingleWaypointParameter(declare_parameter<std::vector<double>>("takeoff_waypoint", default_takeoff),
                                         {0.0, 0.0, 5.0, 0.0});
        home_waypoint_ =
            parseSingleWaypointParameter(declare_parameter<std::vector<double>>("home_waypoint", default_takeoff),
                                         takeoff_waypoint_);
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
        target_velocity_ =
            parseVector3Parameter(declare_parameter<std::vector<double>>("target_velocity", {0.0, 0.0, 0.0}),
                                  {0.0, 0.0, 0.0});
        target_acceleration_ =
            parseVector3Parameter(declare_parameter<std::vector<double>>("target_acceleration", {0.0, 0.0, 0.0}),
                                  {0.0, 0.0, 0.0});
        target_yawspeed_ = declare_parameter<double>("target_yawspeed", 0.0);
        heading_yaw_offset_rad_ = declare_parameter<double>("heading_yaw_offset_rad", 1.5707963267948966);
        distance_sensor_min_signal_quality_ = declare_parameter<int>("distance_sensor_min_signal_quality", 1);
        control_loop_period_ms_ = positiveInt(declare_parameter<int>("control_loop_period_ms", 50));
        executor_threads_ = positiveInt(declare_parameter<int>("executor_threads", 2));
        publisher_queue_depth_ = positiveInt(declare_parameter<int>("publisher_queue_depth", 10));
        subscriber_queue_depth_ = positiveInt(declare_parameter<int>("subscriber_queue_depth", 10));
        state_feedback_queue_depth_ = positiveInt(declare_parameter<int>("state_feedback_queue_depth", 10));
        sensor_queue_depth_ = positiveInt(declare_parameter<int>("sensor_queue_depth", 10));
        log_throttle_ms_ = positiveInt(declare_parameter<int>("log_throttle_ms", 2000));
        takeoff_wait_log_throttle_ms_ = positiveInt(
            declare_parameter<int>("takeoff_wait_log_throttle_ms", 1000));
        hovering_log_throttle_ms_ = positiveInt(declare_parameter<int>("hovering_log_throttle_ms", 3000));
        manual_confirmations_required_ = positiveInt(
            declare_parameter<int>("manual_confirmations_required", 2));

        const auto offboard_state_topic =
            declare_parameter<std::string>("offboard_state_topic", "/uav_offboard_fsm/offboard_state");
        const auto status_topic =
            declare_parameter<std::string>("status_topic", "/uav_offboard_fsm/status");
        const auto set_target_service =
            declare_parameter<std::string>("set_target_service", "online_traj_generator/set_target");
        const auto control_command_topic =
            declare_parameter<std::string>("control_command_topic", "/uav_offboard_fsm/control_command");
        const auto mission_state_topic =
            declare_parameter<std::string>("mission_state_topic", "/uav_offboard_fsm/mission_state");

        const auto state_feedback_topic =
            declare_parameter<std::string>("state_feedback_topic", "/online_traj_generator/ruckig_state");
        const auto distance_sensor_topic =
            declare_parameter<std::string>("distance_sensor_topic", "/fmu/out/distance_sensor");
        const auto vehicle_local_position_topic =
            declare_parameter<std::string>("vehicle_local_position_topic", "/fmu/out/vehicle_local_position");
        const auto home_position_topic =
            declare_parameter<std::string>("home_position_topic", "/fmu/out/home_position");

        const std::vector<double> default_transit = {
            0.0, 0.0, takeoff_waypoint_.z, 0.0,
            5.0, 0.0, takeoff_waypoint_.z, 0.0,
            5.0, 5.0, takeoff_waypoint_.z, 1.57079632679,
        };
        transit_waypoints_ =
            parseWaypointParameter(declare_parameter<std::vector<double>>("transit_waypoints", default_transit));
        if (transit_waypoints_.empty()) {
            transit_waypoints_ = {
                {0.0, 0.0, takeoff_waypoint_.z, 0.0},
                {5.0, 0.0, takeoff_waypoint_.z, 0.0},
                {5.0, 5.0, takeoff_waypoint_.z, 1.57079632679},
            };
        }

        transit_command_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "command_aliases.transit_to_area", {"TRANSIT_TO_AREA", "TRANSIT", "TRAJECTORY_TRACKING", "R"}));
        search_command_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "command_aliases.search_adjust", {"SEARCH_ADJUST", "SEARCH", "ADJUST", "A"}));
        approach_command_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "command_aliases.approach_plant", {"APPROACH_PLANT", "APPROACH", "P"}));
        retreat_command_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "command_aliases.retreat", {"RETREAT", "E"}));
        back_home_command_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "command_aliases.back_home", {"BACK_HOME", "HOME", "B"}));
        confirm_command_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "command_aliases.confirm", {"CONFIRM", "YES", "Y"}));
        self_check_command_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "command_aliases.self_check", {"SELF_CHECK", "SELFCHECK", "RESET", "S"}));
        takeoff_command_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "command_aliases.takeoff", {"TAKEOFF", "T"}));
        manual_command_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "command_aliases.manual_opera", {"MANUAL_OPERA", "MANUAL", "M"}));
        mission_enabled_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "mission_state_enabled_aliases", {"ENABLED", "RUNNING", "AUTO"}));
        mission_disabled_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "mission_state_disabled_aliases", {"DISABLED", "ABORT", "STOP"}));

        offboard_state_pub_ =
            create_publisher<std_msgs::msg::String>(offboard_state_topic, publisher_queue_depth_);
        status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, publisher_queue_depth_);

        set_target_client_ =
            create_client<traj_offboard::srv::SetTarget>(set_target_service);

        control_command_sub_ = create_subscription<std_msgs::msg::String>(
            control_command_topic, subscriber_queue_depth_,
            std::bind(&UavOffboardFsm::handleControlCommand, this, std::placeholders::_1));

        mission_state_sub_ = create_subscription<std_msgs::msg::String>(
            mission_state_topic, subscriber_queue_depth_,
            std::bind(&UavOffboardFsm::handleMissionState, this, std::placeholders::_1));

        ruckig_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            state_feedback_topic, state_feedback_queue_depth_,
            std::bind(&UavOffboardFsm::handleRuckigState, this, std::placeholders::_1));

        auto sensor_qos = rclcpp::SensorDataQoS();
        sensor_qos.keep_last(static_cast<std::size_t>(sensor_queue_depth_));

        vehicle_local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            vehicle_local_position_topic, sensor_qos,
            std::bind(&UavOffboardFsm::handleVehicleLocalPosition, this, std::placeholders::_1));

        home_position_sub_ = create_subscription<px4_msgs::msg::HomePosition>(
            home_position_topic, sensor_qos,
            std::bind(&UavOffboardFsm::handleHomePosition, this, std::placeholders::_1));

        distance_sensor_sub_ = create_subscription<px4_msgs::msg::DistanceSensor>(
            distance_sensor_topic, sensor_qos,
            std::bind(&UavOffboardFsm::handleDistanceSensor, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::milliseconds(control_loop_period_ms_),
                                   std::bind(&UavOffboardFsm::controlLoopOnTimer, this));
    }

    // 返回 MultiThreadedExecutor 的线程数量，main() 用它按参数创建执行器。
    int executorThreads() const { return executor_threads_; }

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
    std::array<double, 3> target_velocity_{0.0, 0.0, 0.0};
    std::array<double, 3> target_acceleration_{0.0, 0.0, 0.0};
    double target_yawspeed_{0.0};
    double heading_yaw_offset_rad_{1.5707963267948966};
    int distance_sensor_min_signal_quality_{1};
    int control_loop_period_ms_{50};
    int executor_threads_{2};
    int publisher_queue_depth_{10};
    int subscriber_queue_depth_{10};
    int state_feedback_queue_depth_{10};
    int sensor_queue_depth_{10};
    int log_throttle_ms_{2000};
    int takeoff_wait_log_throttle_ms_{1000};
    int hovering_log_throttle_ms_{3000};
    int manual_confirmations_required_{2};

    std::vector<std::string> transit_command_aliases_;
    std::vector<std::string> search_command_aliases_;
    std::vector<std::string> approach_command_aliases_;
    std::vector<std::string> retreat_command_aliases_;
    std::vector<std::string> back_home_command_aliases_;
    std::vector<std::string> confirm_command_aliases_;
    std::vector<std::string> self_check_command_aliases_;
    std::vector<std::string> takeoff_command_aliases_;
    std::vector<std::string> manual_command_aliases_;
    std::vector<std::string> mission_enabled_aliases_;
    std::vector<std::string> mission_disabled_aliases_;

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

    static Waypoint parseSingleWaypointParameter(const std::vector<double> & flat,
                                                 const Waypoint & fallback);
    static std::vector<Waypoint> parseWaypointParameter(const std::vector<double> & flat);
    static std::array<double, 3> parseVector3Parameter(const std::vector<double> & flat,
                                                       const std::array<double, 3> & fallback);
    static std::string stateToString(ControlState state);
    std::optional<ParsedCommand> parseCommand(const std::string & command) const;
    static bool tokenMatches(const std::string & token, const std::vector<std::string> & aliases);
    static std::vector<std::string> upperCopyList(std::vector<std::string> values);
    static std::string upperCopy(std::string value);
    static double wrapAngle(double angle);
};

// 状态机主循环：定时检查当前状态，处理状态进入动作，发布状态信息，并调用对应状态处理函数。
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

// 状态进入回调：每次切换到新状态时清空旧目标，并初始化该状态需要的标志位、索引和航点序列。
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

// 状态切换函数：只更新状态枚举，真正的进入动作由下一次定时循环统一执行。
void UavOffboardFsm::transitionTo(ControlState state)
{
    if (control_state_.load() == state) {
        return;
    }
    control_state_.store(state);
}

// 自检状态处理：等待键盘 SELF_CHECK 指令，检查任务使能和可选测距传感器状态，通过后允许起飞。
void UavOffboardFsm::handleSelfCheck()
{
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "State: SELF_CHECK");
    if (!self_check_requested_ && !ready_for_takeoff_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "Waiting for keyboard SELF_CHECK command");
        return;
    }

    if (ready_for_takeoff_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "Self check passed; waiting for keyboard TAKEOFF command");
        return;
    }

    if (!isSelfCheckOK()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "Self check pending: mission_enabled=%s distance_sensor_ok=%s",
                             mission_enabled_ ? "true" : "false",
                             hasFreshDistanceSensor() ? "true" : "false");
        return;
    }

    ready_for_takeoff_ = true;
    self_check_requested_ = false;
    RCLCPP_INFO(get_logger(), "Self check passed, ready_for_takeoff=1; waiting for keyboard TAKEOFF command");
}

// 起飞状态处理：确认已经自检通过，并等待无人机到达起飞航点，到达后进入悬停状态。
void UavOffboardFsm::handleTakeoff()
{
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "State: TAKEOFF");
    if (!ready_for_takeoff_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "Takeoff blocked before self check");
        transitionTo(ControlState::SELF_CHECK);
        return;
    }

    if (!isUAVTakeoff()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), takeoff_wait_log_throttle_ms_,
                             "Waiting for takeoff completion");
        return;
    }

    ready_for_transit_ = true;
    RCLCPP_INFO(get_logger(), "Takeoff completed, ready_for_transit=1");
    transitionTo(ControlState::HOVERING);
}

// 悬停状态处理：悬停本身不下发新目标，只作为等待下一条任务指令的空闲状态。
void UavOffboardFsm::handleHovering()
{
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                         "State: HOVERING, waiting for transit/search/approach/retreat/home/manual command");
}

// 前往任务区域处理：按 transit_waypoints 参数中的航点顺序飞行，全部到达后标记已到达任务区。
void UavOffboardFsm::handleTransitToArea()
{
    if (!ready_for_transit_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
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

// 搜索微调处理：在已到达任务区的前提下执行自动生成的偏航和横移搜索航点。
void UavOffboardFsm::handleSearchAdjust()
{
    if (!is_arrived_task_aera_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
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

// 靠近植株处理：优先用测距传感器判断是否达到目标距离，否则按生成的前进航点执行接近动作。
void UavOffboardFsm::handleApproachPlant()
{
    if (!adjust_completed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
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

// 后退处理：接近动作完成后沿机体系后方退回指定距离，完成后允许继续调整或执行其他任务。
void UavOffboardFsm::handleRetreat()
{
    if (!approach_completed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
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

// 返航处理：按 home_waypoint 参数回到本地坐标系中的返航点，完成后设置返航完成标志。
void UavOffboardFsm::handleBackHome()
{
    if (!ready_for_transit_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
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

// 手动操作处理：收到 MANUAL 指令后需要达到参数要求的确认次数，随后按当前位置叠加手动偏移量下发目标。
void UavOffboardFsm::handleManualOperation()
{
    if (manual_command_.confirmations < manual_confirmations_required_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "MANUAL_OPERA waiting for confirmation (%d/%d)",
                             manual_command_.confirmations, manual_confirmations_required_);
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
                    "Manual target accepted after confirmation: dx=%.2f dy=%.2f dz=%.2f dyaw=%.2f",
                    manual_command_.dx, manual_command_.dy, manual_command_.dz,
                    manual_command_.dyaw);
    }

    if (handleActiveTargetReached()) {
        manual_command_ = {};
        RCLCPP_INFO(get_logger(), "Manual operation completed");
        transitionTo(ControlState::HOVERING);
    }
}

// 航点序列处理：负责发送当前航点、等待到达、推进索引，并在序列全部完成时返回 true。
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

// 当前目标到达判断：确保目标已经发送到轨迹节点，然后根据最新位置反馈判断是否进入容差范围。
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
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "Waiting for UAV position feedback");
        return false;
    }

    sensor_msgs::msg::JointState state_msg;
    state_msg.position = {waypoint->x, waypoint->y, waypoint->z, waypoint->yaw};
    return isWaypointReached(*active_target_, state_msg);
}

// 设置新的活动目标：保存目标航点，并将发送状态重置为未发送。
void UavOffboardFsm::setActiveTarget(const Waypoint & waypoint)
{
    active_target_ = waypoint;
    active_target_sent_ = false;
    target_request_pending_ = false;
}

// 清空活动目标：取消当前航点并重置服务调用状态，通常在切换状态或航点到达后使用。
void UavOffboardFsm::clearActiveTarget()
{
    active_target_.reset();
    active_target_sent_ = false;
    target_request_pending_ = false;
}

// 下发活动目标：通过 traj_offboard 的 set_target 服务把目标位置、速度、加速度和偏航速度发送给轨迹节点。
void UavOffboardFsm::sendActiveTarget()
{
    if (!active_target_) {
        return;
    }
    if (!set_target_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "traj_offboard set_target service not available");
        return;
    }

    auto request = std::make_shared<traj_offboard::srv::SetTarget::Request>();
    request->target.position = {
        static_cast<float>(active_target_->x),
        static_cast<float>(active_target_->y),
        static_cast<float>(active_target_->z)};
    request->target.velocity = {
        static_cast<float>(target_velocity_[0]),
        static_cast<float>(target_velocity_[1]),
        static_cast<float>(target_velocity_[2])};
    request->target.acceleration = {
        static_cast<float>(target_acceleration_[0]),
        static_cast<float>(target_acceleration_[1]),
        static_cast<float>(target_acceleration_[2])};
    request->target.yaw = static_cast<float>(active_target_->yaw);
    request->target.yawspeed = static_cast<float>(target_yawspeed_);

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

// 自检条件判断：任务必须处于使能状态；如果参数要求测距传感器，则还必须有新鲜测距数据。
bool UavOffboardFsm::isSelfCheckOK()
{
    return mission_enabled_ && (!require_distance_sensor_ || hasFreshDistanceSensor());
}

// 起飞完成判断：读取当前无人机位置，并检查是否已经到达 takeoff_waypoint 参数指定的起飞航点。
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

// 航点到达判断：比较当前位置和目标航点的 x/y/z/yaw 误差，全部进入参数容差后认为到达。
bool UavOffboardFsm::isWaypointReached(const Waypoint & waypoint,
                                       const sensor_msgs::msg::JointState & state)
{
    if (state.position.size() < 4) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
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

// 当前航点读取：优先使用真实 PX4 位置反馈，若其超时则退回轨迹参考状态，二者都不可用时返回空。
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

// 获取当前位置或悬停默认点：有新鲜反馈时返回当前位置，否则返回起飞航点作为保底悬停目标。
UavOffboardFsm::Waypoint UavOffboardFsm::currentOrHoverWaypoint()
{
    const auto current = currentWaypoint();
    if (current) {
        return *current;
    }
    return takeoff_waypoint_;
}

// 状态反馈新鲜度判断：时间戳必须有效，且距离当前时间不超过 state_feedback_timeout_s 参数。
bool UavOffboardFsm::isFreshStateTime(const rclcpp::Time & stamp) const
{
    if (stamp.nanoseconds() == 0) {
        return false;
    }
    return (now() - stamp).seconds() <= state_feedback_timeout_s_;
}

// 测距数据新鲜度判断：必须已经收到有效距离，且距离当前时间不超过 distance_sensor_timeout_s 参数。
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

// 生成搜索微调航点：围绕当前位置先偏航，再左右横移，最后回到原始姿态。
void UavOffboardFsm::generateSearchAdjustWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    search_waypoints_.clear();
    search_waypoints_.push_back({base.x, base.y, base.z, wrapAngle(base.yaw + search_yaw_offset_rad_)});
    search_waypoints_.push_back(offsetBodyFrame(base, 0.0, -search_lateral_offset_m_));
    search_waypoints_.push_back(offsetBodyFrame(base, 0.0, search_lateral_offset_m_));
    search_waypoints_.push_back({base.x, base.y, base.z, base.yaw});
}

// 生成靠近航点：根据当前测距结果计算前进距离，目标是接近到 approach_target_distance_m 附近。
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

// 生成后退航点：从当前位置沿机体系后方移动 retreat_distance_m，用于离开植株或障碍物。
void UavOffboardFsm::generateRetreatWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    retreat_waypoints_.clear();
    retreat_waypoints_.push_back(offsetBodyFrame(base, -retreat_distance_m_, 0.0));
}

// 机体系偏移转换：把 forward/right 偏移按 base.yaw 旋转到本地坐标系，保持高度和偏航不变。
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

// 发布给 offboard_control_bridge 的状态字符串，使底层桥接节点知道当前应执行的飞行模式。
void UavOffboardFsm::publishOffboardState(ControlState state)
{
    std_msgs::msg::String msg;
    msg.data = stateToString(state);
    offboard_state_pub_->publish(msg);
}

// 发布状态机诊断信息：把关键布尔标志拼成一行字符串，便于 ros2 topic echo 或录包排查。
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

// 控制指令回调：解析键盘或上层节点发来的字符串指令，并按当前状态决定是否接受和切换状态。
void UavOffboardFsm::handleControlCommand(const std_msgs::msg::String::SharedPtr msg)
{
    const auto parsed = parseCommand(msg->data);
    if (!parsed) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
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
        manual_command_.confirmations =
            std::min(manual_confirmations_required_, manual_command_.confirmations + 1);
        RCLCPP_INFO(get_logger(), "Manual confirmation accepted (%d/%d)",
                    manual_command_.confirmations, manual_confirmations_required_);
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

// 任务使能状态回调：接收外部任务状态字符串，根据参数化别名把 mission_enabled_ 打开或关闭。
void UavOffboardFsm::handleMissionState(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    const auto state = upperCopy(msg->data);
    if (tokenMatches(state, mission_enabled_aliases_)) {
        mission_enabled_ = true;
    } else if (tokenMatches(state, mission_disabled_aliases_)) {
        mission_enabled_ = false;
    } else {
        RCLCPP_WARN(get_logger(), "Unknown mission state: %s", msg->data.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "Mission enabled set to %s", mission_enabled_ ? "true" : "false");
}

// 轨迹参考状态回调：保存在线轨迹生成器输出的参考状态，作为真实位置反馈缺失时的备用状态。
void UavOffboardFsm::handleRuckigState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    latest_reference_state_ = *msg;
    last_reference_state_time_ = now();
}

// PX4 本地位置回调：把 PX4 的 NED/FRD 风格位置换算为本状态机使用的本地 x/y/z/yaw 表示。
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
        std::isfinite(msg->heading) ? wrapAngle(heading_yaw_offset_rad_ - msg->heading) :
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

// PX4 Home 位置回调：记录 PX4 home 原点，用于后续把 PX4 位置转换为相对本地坐标。
void UavOffboardFsm::handleHomePosition(const px4_msgs::msg::HomePosition::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    home_x_ = msg->x;
    home_y_ = msg->y;
    home_z_ = msg->z;
}

// 测距传感器回调：只接受范围内且信号质量达到参数阈值的数据，并记录接收时间用于超时判断。
void UavOffboardFsm::handleDistanceSensor(const px4_msgs::msg::DistanceSensor::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    if (msg->current_distance >= msg->min_distance && msg->current_distance <= msg->max_distance &&
        msg->signal_quality >= distance_sensor_min_signal_quality_) {
        latest_distance_m_ = msg->current_distance;
        last_distance_sensor_time_ = now();
    }
}

// 单航点参数解析：要求参数正好包含 [x, y, z, yaw] 四个 double，格式错误时使用传入的 fallback。
UavOffboardFsm::Waypoint
UavOffboardFsm::parseSingleWaypointParameter(const std::vector<double> & flat,
                                             const Waypoint & fallback)
{
    if (flat.size() != 4) {
        return fallback;
    }
    return {flat[0], flat[1], flat[2], flat[3]};
}

// 多航点参数解析：把扁平数组按每 4 个值一组转换为航点序列，格式错误时返回空序列。
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

// 三维向量参数解析：用于目标速度和加速度，要求正好 3 个 double，否则使用 fallback。
std::array<double, 3>
UavOffboardFsm::parseVector3Parameter(const std::vector<double> & flat,
                                      const std::array<double, 3> & fallback)
{
    if (flat.size() != 3) {
        return fallback;
    }
    return {flat[0], flat[1], flat[2]};
}

// 状态枚举转字符串：统一生成发布给外部节点和日志使用的状态名。
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

// 控制指令解析：读取指令首个 token，与参数化命令别名匹配；手动模式还会解析 dx/dy/dz/dyaw。
std::optional<UavOffboardFsm::ParsedCommand>
UavOffboardFsm::parseCommand(const std::string & command) const
{
    std::istringstream stream(command);
    std::string token;
    stream >> token;
    if (token.empty()) {
        return std::nullopt;
    }
    token = upperCopy(token);

    if (tokenMatches(token, transit_command_aliases_)) {
        return ParsedCommand{CommandType::TRANSIT_TO_AREA, std::nullopt};
    }
    if (tokenMatches(token, search_command_aliases_)) {
        return ParsedCommand{CommandType::SEARCH_ADJUST, std::nullopt};
    }
    if (tokenMatches(token, approach_command_aliases_)) {
        return ParsedCommand{CommandType::APPROACH_PLANT, std::nullopt};
    }
    if (tokenMatches(token, retreat_command_aliases_)) {
        return ParsedCommand{CommandType::RETREAT, std::nullopt};
    }
    if (tokenMatches(token, back_home_command_aliases_)) {
        return ParsedCommand{CommandType::BACK_HOME, std::nullopt};
    }
    if (tokenMatches(token, confirm_command_aliases_)) {
        return ParsedCommand{CommandType::CONFIRM, std::nullopt};
    }
    if (tokenMatches(token, self_check_command_aliases_)) {
        return ParsedCommand{CommandType::SELF_CHECK, std::nullopt};
    }
    if (tokenMatches(token, takeoff_command_aliases_)) {
        return ParsedCommand{CommandType::TAKEOFF, std::nullopt};
    }
    if (tokenMatches(token, manual_command_aliases_)) {
        ManualCommand manual;
        if (!(stream >> manual.dx >> manual.dy >> manual.dz >> manual.dyaw)) {
            return ParsedCommand{CommandType::MANUAL_OPERA, std::nullopt};
        }
        return ParsedCommand{CommandType::MANUAL_OPERA, manual};
    }

    return std::nullopt;
}

// token 匹配工具：判断大写后的输入 token 是否位于某组参数化别名中。
bool UavOffboardFsm::tokenMatches(const std::string & token,
                                  const std::vector<std::string> & aliases)
{
    return std::find(aliases.begin(), aliases.end(), token) != aliases.end();
}

// 字符串数组大写化工具：启动时把所有命令/状态别名统一转成大写，后续匹配不受大小写影响。
std::vector<std::string> UavOffboardFsm::upperCopyList(std::vector<std::string> values)
{
    for (auto & value : values) {
        value = upperCopy(value);
    }
    return values;
}

// 字符串大写化工具：用于命令解析和任务状态解析。
std::string UavOffboardFsm::upperCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return value;
}

// 角度归一化工具：把任意弧度角压到 [-pi, pi]，避免偏航角跨越 pi 时误差突变。
double UavOffboardFsm::wrapAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

// 程序入口：初始化 ROS2，创建状态机节点，按参数指定线程数启动 MultiThreadedExecutor。
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UavOffboardFsm>();
    rclcpp::executors::MultiThreadedExecutor exec(
        rclcpp::ExecutorOptions(), static_cast<std::size_t>(node->executorThreads()));
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
