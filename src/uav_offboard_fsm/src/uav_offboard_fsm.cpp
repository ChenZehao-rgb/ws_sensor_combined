#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <traj_offboard/msg/bridge_feedback.hpp>
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

// 正整数参数归一化：ROS2 Humble 中整数参数可能经由 long 参与比较，这里统一限制为 >=1 的 int。
int positiveInt(long value)
{
    return static_cast<int>(std::max(1L, value));
}

}  // namespace

class UavOffboardFsm : public rclcpp::Node {
  public:
    // 构造函数：读取流程参数，创建状态发布、命令订阅、bridge feedback 订阅和轨迹目标服务客户端。
    UavOffboardFsm() : rclcpp::Node("uav_offboard_fsm")
    {
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
        require_distance_sensor_ = declare_parameter<bool>("require_distance_sensor", true);
        bridge_feedback_timeout_s_ = declare_parameter<double>("bridge_feedback_timeout_s", 1.0);
        search_yaw_offset_rad_ = declare_parameter<double>("search_yaw_offset_rad", 0.35);
        search_lateral_offset_m_ = declare_parameter<double>("search_lateral_offset_m", 0.4);
        approach_distance_m_ = declare_parameter<double>("approach_distance_m", 1.0);
        approach_target_distance_m_ = declare_parameter<double>("approach_target_distance_m", 0.7);
        approach_distance_tolerance_m_ = declare_parameter<double>("approach_distance_tolerance_m", 0.1);
        retreat_distance_m_ = declare_parameter<double>("retreat_distance_m", 1.0);
        sample_adjust_forward_m_ = declare_parameter<double>("sample_adjust_forward_m", 0.2);
        sample_adjust_right_m_ = declare_parameter<double>("sample_adjust_right_m", 0.0);
        sample_adjust_z_offset_m_ = declare_parameter<double>("sample_adjust_z_offset_m", 0.0);
        sample_adjust_yaw_offset_rad_ = declare_parameter<double>("sample_adjust_yaw_offset_rad", 0.0);
        target_velocity_ =
            parseVector3Parameter(declare_parameter<std::vector<double>>("target_velocity", {0.0, 0.0, 0.0}),
                                  {0.0, 0.0, 0.0});
        target_acceleration_ =
            parseVector3Parameter(declare_parameter<std::vector<double>>("target_acceleration", {0.0, 0.0, 0.0}),
                                  {0.0, 0.0, 0.0});
        target_yawspeed_ = declare_parameter<double>("target_yawspeed", 0.0);
        control_loop_period_ms_ = positiveInt(declare_parameter<int>("control_loop_period_ms", 50));
        executor_threads_ = positiveInt(declare_parameter<int>("executor_threads", 2));
        publisher_queue_depth_ = positiveInt(declare_parameter<int>("publisher_queue_depth", 10));
        subscriber_queue_depth_ = positiveInt(declare_parameter<int>("subscriber_queue_depth", 10));
        log_throttle_ms_ = positiveInt(declare_parameter<int>("log_throttle_ms", 2000));
        takeoff_wait_log_throttle_ms_ =
            positiveInt(declare_parameter<int>("takeoff_wait_log_throttle_ms", 1000));
        hovering_log_throttle_ms_ = positiveInt(declare_parameter<int>("hovering_log_throttle_ms", 3000));

        offboard_state_topic_ =
            declare_parameter<std::string>("offboard_state_topic", "/uav_offboard_fsm/offboard_state");
        status_topic_ = declare_parameter<std::string>("status_topic", "/uav_offboard_fsm/status");
        set_target_service_ =
            declare_parameter<std::string>("set_target_service", "online_traj_generator/set_target");
        control_command_topic_ =
            declare_parameter<std::string>("control_command_topic", "/uav_offboard_fsm/control_command");
        mission_state_topic_ =
            declare_parameter<std::string>("mission_state_topic", "/uav_offboard_fsm/mission_state");
        bridge_feedback_topic_ =
            declare_parameter<std::string>("bridge_feedback_topic", "/uav_offboard_fsm/bridge_feedback");

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

        mission_enabled_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "mission_state_enabled_aliases", {"ENABLED", "RUNNING", "AUTO"}));
        mission_disabled_aliases_ = upperCopyList(declare_parameter<std::vector<std::string>>(
            "mission_state_disabled_aliases", {"DISABLED", "ABORT", "STOP"}));

        offboard_state_pub_ =
            create_publisher<std_msgs::msg::String>(offboard_state_topic_, publisher_queue_depth_);
        status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, publisher_queue_depth_);
        set_target_client_ = create_client<traj_offboard::srv::SetTarget>(set_target_service_);

        control_command_sub_ = create_subscription<std_msgs::msg::String>(
            control_command_topic_, subscriber_queue_depth_,
            std::bind(&UavOffboardFsm::handleControlCommand, this, std::placeholders::_1));
        mission_state_sub_ = create_subscription<std_msgs::msg::String>(
            mission_state_topic_, subscriber_queue_depth_,
            std::bind(&UavOffboardFsm::handleMissionState, this, std::placeholders::_1));
        bridge_feedback_sub_ = create_subscription<traj_offboard::msg::BridgeFeedback>(
            bridge_feedback_topic_, subscriber_queue_depth_,
            std::bind(&UavOffboardFsm::handleBridgeFeedback, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::milliseconds(control_loop_period_ms_),
                                   std::bind(&UavOffboardFsm::controlLoopOnTimer, this));

        RCLCPP_INFO(get_logger(),
                    "FSM ready | command_topic=%s offboard_state_topic=%s feedback_topic=%s set_target_service=%s",
                    control_command_topic_.c_str(), offboard_state_topic_.c_str(),
                    bridge_feedback_topic_.c_str(), set_target_service_.c_str());
        RCLCPP_INFO(get_logger(),
                    "FSM mission defaults | takeoff=(%.2f, %.2f, %.2f, yaw %.2f) require_distance_sensor=%s",
                    takeoff_waypoint_.x, takeoff_waypoint_.y, takeoff_waypoint_.z,
                    takeoff_waypoint_.yaw, require_distance_sensor_ ? "true" : "false");
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

    enum class ControlState {
        SELF_CHECK,
        UAV_START,
        TRANSIT_TO_AREA,
        HOVERING,
        SEARCH_ADJUST_AUTO,
        SEARCH_ADJUST_MANUAL,
        APPROACH_PLANT,
        SAMP_ADJUST_AUTO,
        SAMP_ADJUST_MANUAL,
        RETREAT,
        BACK_HOME,
        UAV_TASK_TERM
    };

    enum class CommandType {
        SELF_CHECK,
        WAIT_TASK_ENABLE_AUTH,
        NAV_TO_TASK_DOM,
        SEARCH_ADJUST_AUTO,
        SEARCH_ADJUST_MANUAL,
        TARG_GOT,
        CONFIRM,
        NO,
        SAMP_ADJUST_AUTO,
        SAMP_ADJUST_MANUAL,
        ARM_CONFIG_PREP,
        SAMPL_OPERA,
        UAV_PRE_BACK_HOME,
        BACK_HOME
    };

    enum class PendingFailure {
        NONE,
        SEARCH,
        SAMPLE
    };

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr offboard_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_state_sub_;
    rclcpp::Subscription<traj_offboard::msg::BridgeFeedback>::SharedPtr bridge_feedback_sub_;
    rclcpp::Client<traj_offboard::srv::SetTarget>::SharedPtr set_target_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<ControlState> control_state_{ControlState::SELF_CHECK};
    ControlState previous_state_{ControlState::SELF_CHECK};
    mutable std::mutex fsm_mutex_;
    mutable std::mutex feedback_mutex_;
    std::optional<traj_offboard::msg::BridgeFeedback> latest_bridge_feedback_;
    rclcpp::Time last_bridge_feedback_time_{0, 0, RCL_ROS_TIME};

    bool self_check_requested_{false};
    bool wait_task_enable_auth_{false};
    bool uav_check_succeed_{false};
    bool uav_takeoff_succeed_{false};
    bool uav_arrived_task_aera_{false};
    bool adjust_completed_{false};
    bool uav_search_succeed_{false};
    bool targ_got_pending_confirm_{false};
    bool no_pending_confirm_{false};
    PendingFailure pending_failure_{PendingFailure::NONE};
    bool uav_approach_succeed_{false};
    bool uav_adjust_succeed_{false};
    bool arm_config_prep_{false};
    bool sampl_opera_{false};
    bool uav_ready_for_back_{false};
    bool back_home_{false};
    bool mission_enabled_{true};
    bool require_distance_sensor_{true};

    double position_tolerance_{0.25};
    double yaw_tolerance_{0.15};
    double bridge_feedback_timeout_s_{1.0};
    double search_yaw_offset_rad_{0.35};
    double search_lateral_offset_m_{0.4};
    double approach_distance_m_{1.0};
    double approach_target_distance_m_{0.7};
    double approach_distance_tolerance_m_{0.1};
    double retreat_distance_m_{1.0};
    double sample_adjust_forward_m_{0.2};
    double sample_adjust_right_m_{0.0};
    double sample_adjust_z_offset_m_{0.0};
    double sample_adjust_yaw_offset_rad_{0.0};
    std::array<double, 3> target_velocity_{0.0, 0.0, 0.0};
    std::array<double, 3> target_acceleration_{0.0, 0.0, 0.0};
    double target_yawspeed_{0.0};
    int control_loop_period_ms_{50};
    int executor_threads_{2};
    int publisher_queue_depth_{10};
    int subscriber_queue_depth_{10};
    int log_throttle_ms_{2000};
    int takeoff_wait_log_throttle_ms_{1000};
    int hovering_log_throttle_ms_{3000};

    std::string offboard_state_topic_;
    std::string status_topic_;
    std::string set_target_service_;
    std::string control_command_topic_;
    std::string mission_state_topic_;
    std::string bridge_feedback_topic_;
    std::vector<std::string> mission_enabled_aliases_;
    std::vector<std::string> mission_disabled_aliases_;

    Waypoint takeoff_waypoint_{};
    Waypoint home_waypoint_{};
    std::vector<Waypoint> transit_waypoints_;
    std::vector<Waypoint> search_waypoints_;
    std::vector<Waypoint> approach_waypoints_;
    std::vector<Waypoint> sample_adjust_waypoints_;
    std::vector<Waypoint> retreat_waypoints_;
    std::vector<Waypoint> back_home_waypoints_;
    std::size_t transit_index_{0};
    std::size_t search_index_{0};
    std::size_t approach_index_{0};
    std::size_t sample_adjust_index_{0};
    std::size_t retreat_index_{0};
    std::size_t back_home_index_{0};

    std::optional<Waypoint> active_target_;
    bool active_target_sent_{false};
    bool target_request_pending_{false};
    rclcpp::Time last_target_sent_time_{0, 0, RCL_ROS_TIME};

    void controlLoopOnTimer();
    void onStateEntry(ControlState state);
    void transitionTo(ControlState state);
    void resetMissionProgress();

    void handleSelfCheck();
    void handleUavStart();
    void handleTransitToArea();
    void handleHovering();
    void handleSearchAdjustAuto();
    void handleSearchAdjustManual();
    void handleApproachPlant();
    void handleSampleAdjustAuto();
    void handleSampleAdjustManual();
    void handleRetreat();
    void handleBackHome();
    void handleTaskTerm();

    bool handleWaypointSequence(std::vector<Waypoint> & waypoints, std::size_t & index,
                                const std::string & label);
    bool handleActiveTargetReached();
    void setActiveTarget(const Waypoint & waypoint);
    void clearActiveTarget();
    void sendActiveTarget();

    bool isSelfCheckOK();
    bool isUavTakeoffComplete();
    bool isWaypointReached(const Waypoint & waypoint, const Waypoint & current) const;
    bool hasFreshBridgeFeedback() const;
    bool hasValidDistanceSensor() const;
    std::optional<double> latestDistanceM() const;
    std::optional<Waypoint> currentWaypoint() const;
    Waypoint currentOrHoverWaypoint() const;

    void generateSearchAdjustWaypoints();
    void generateApproachWaypoints();
    void generateSampleAdjustWaypoints();
    void generateRetreatWaypoints();
    Waypoint offsetBodyFrame(const Waypoint & base, double forward_m, double right_m) const;

    void publishOffboardState(ControlState state);
    void publishStatus(ControlState state);
    void handleControlCommand(const std_msgs::msg::String::SharedPtr msg);
    void handleMissionState(const std_msgs::msg::String::SharedPtr msg);
    void handleBridgeFeedback(const traj_offboard::msg::BridgeFeedback::SharedPtr msg);

    static Waypoint parseSingleWaypointParameter(const std::vector<double> & flat,
                                                 const Waypoint & fallback);
    static std::vector<Waypoint> parseWaypointParameter(const std::vector<double> & flat);
    static std::array<double, 3> parseVector3Parameter(const std::vector<double> & flat,
                                                       const std::array<double, 3> & fallback);
    static std::string stateToString(ControlState state);
    static int stateToId(ControlState state);
    static std::optional<CommandType> parseCommand(const std::string & command);
    static bool tokenMatches(const std::string & token, const std::vector<std::string> & aliases);
    static std::vector<std::string> upperCopyList(std::vector<std::string> values);
    static std::string upperCopy(std::string value);
    static double wrapAngle(double angle);
};

// 状态机主循环：发布当前主状态和诊断信息，并把状态处理限制在纯流程流转和轨迹目标发送。
void UavOffboardFsm::controlLoopOnTimer()
{
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    const auto current_state = control_state_.load();
    if (current_state != previous_state_) {
        onStateEntry(current_state);
    }

    publishOffboardState(current_state);
    publishStatus(current_state);

    switch (current_state) {
        case ControlState::SELF_CHECK:
            handleSelfCheck();
            break;
        case ControlState::UAV_START:
            handleUavStart();
            break;
        case ControlState::TRANSIT_TO_AREA:
            handleTransitToArea();
            break;
        case ControlState::HOVERING:
            handleHovering();
            break;
        case ControlState::SEARCH_ADJUST_AUTO:
            handleSearchAdjustAuto();
            break;
        case ControlState::SEARCH_ADJUST_MANUAL:
            handleSearchAdjustManual();
            break;
        case ControlState::APPROACH_PLANT:
            handleApproachPlant();
            break;
        case ControlState::SAMP_ADJUST_AUTO:
            handleSampleAdjustAuto();
            break;
        case ControlState::SAMP_ADJUST_MANUAL:
            handleSampleAdjustManual();
            break;
        case ControlState::RETREAT:
            handleRetreat();
            break;
        case ControlState::BACK_HOME:
            handleBackHome();
            break;
        case ControlState::UAV_TASK_TERM:
            handleTaskTerm();
            break;
    }

    previous_state_ = current_state;
}

// 状态进入动作：初始化该主状态所需的航点队列和流程标志，不直接访问 PX4 topic。
void UavOffboardFsm::onStateEntry(ControlState state)
{
    clearActiveTarget();
    RCLCPP_INFO(get_logger(), "FSM state -> %s", stateToString(state).c_str());

    switch (state) {
        case ControlState::SELF_CHECK:
            resetMissionProgress();
            break;
        case ControlState::UAV_START:
            uav_takeoff_succeed_ = false;
            setActiveTarget(takeoff_waypoint_);
            break;
        case ControlState::TRANSIT_TO_AREA:
            transit_index_ = 0;
            uav_arrived_task_aera_ = false;
            break;
        case ControlState::SEARCH_ADJUST_AUTO:
            search_index_ = 0;
            adjust_completed_ = false;
            uav_search_succeed_ = false;
            targ_got_pending_confirm_ = false;
            no_pending_confirm_ = false;
            pending_failure_ = PendingFailure::NONE;
            generateSearchAdjustWaypoints();
            break;
        case ControlState::SEARCH_ADJUST_MANUAL:
            adjust_completed_ = false;
            uav_search_succeed_ = false;
            targ_got_pending_confirm_ = false;
            no_pending_confirm_ = false;
            pending_failure_ = PendingFailure::NONE;
            break;
        case ControlState::APPROACH_PLANT:
            approach_index_ = 0;
            uav_approach_succeed_ = false;
            generateApproachWaypoints();
            break;
        case ControlState::SAMP_ADJUST_AUTO:
            sample_adjust_index_ = 0;
            uav_adjust_succeed_ = false;
            no_pending_confirm_ = false;
            pending_failure_ = PendingFailure::NONE;
            generateSampleAdjustWaypoints();
            break;
        case ControlState::SAMP_ADJUST_MANUAL:
            uav_adjust_succeed_ = false;
            no_pending_confirm_ = false;
            pending_failure_ = PendingFailure::NONE;
            break;
        case ControlState::RETREAT:
            retreat_index_ = 0;
            uav_ready_for_back_ = false;
            generateRetreatWaypoints();
            break;
        case ControlState::BACK_HOME:
            back_home_index_ = 0;
            back_home_ = false;
            back_home_waypoints_ = {home_waypoint_};
            break;
        case ControlState::HOVERING:
        case ControlState::UAV_TASK_TERM:
            break;
    }
}

// 状态切换函数：只更新主状态，真正的进入动作在下一次主循环统一执行。
void UavOffboardFsm::transitionTo(ControlState state)
{
    if (control_state_.load() == state) {
        return;
    }
    control_state_.store(state);
}

// 任务进度复位：重新自检时清空流程图中的所有完成/授权/确认标志。
void UavOffboardFsm::resetMissionProgress()
{
    wait_task_enable_auth_ = false;
    uav_check_succeed_ = false;
    uav_takeoff_succeed_ = false;
    uav_arrived_task_aera_ = false;
    adjust_completed_ = false;
    uav_search_succeed_ = false;
    targ_got_pending_confirm_ = false;
    no_pending_confirm_ = false;
    pending_failure_ = PendingFailure::NONE;
    uav_approach_succeed_ = false;
    uav_adjust_succeed_ = false;
    arm_config_prep_ = false;
    sampl_opera_ = false;
    uav_ready_for_back_ = false;
    back_home_ = false;
}

// SELF_CHECK：检查任务总使能和 bridge 转发的传感器通信状态，通过后等待 WAIT_TASK_ENABLE_AUTH。
void UavOffboardFsm::handleSelfCheck()
{
    if (!self_check_requested_ && !uav_check_succeed_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "SELF_CHECK | waiting for command SELF_CHECK");
        return;
    }

    uav_check_succeed_ = isSelfCheckOK();
    if (!uav_check_succeed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "SELF_CHECK | pending mission_enabled=%s bridge_feedback=%s distance_sensor=%s",
                             mission_enabled_ ? "true" : "false",
                             hasFreshBridgeFeedback() ? "true" : "false",
                             hasValidDistanceSensor() ? "true" : "false");
        return;
    }

    if (wait_task_enable_auth_) {
        RCLCPP_INFO(get_logger(), "SELF_CHECK complete | WAIT_TASK_ENABLE_AUTH accepted");
        transitionTo(ControlState::UAV_START);
        return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                         "SELF_CHECK | uavCheckSucceed=1 waiting WAIT_TASK_ENABLE_AUTH");
}

// UAV_START：发送起飞目标给 traj_offboard，并用 bridge feedback 的真实位置判断 uavTakeoffSucceed。
void UavOffboardFsm::handleUavStart()
{
    if (!uav_check_succeed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "UAV_START blocked | uavCheckSucceed=0");
        transitionTo(ControlState::SELF_CHECK);
        return;
    }

    if (!active_target_) {
        setActiveTarget(takeoff_waypoint_);
    }

    if (!handleActiveTargetReached()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), takeoff_wait_log_throttle_ms_,
                             "UAV_START | waiting for takeoff target=(%.2f, %.2f, %.2f, yaw %.2f)",
                             takeoff_waypoint_.x, takeoff_waypoint_.y,
                             takeoff_waypoint_.z, takeoff_waypoint_.yaw);
        return;
    }

    uav_takeoff_succeed_ = true;
    RCLCPP_INFO(get_logger(), "UAV_START complete | uavTakeoffSucceed=1 waiting NAV_TO_TASK_DOM");
    transitionTo(ControlState::HOVERING);
}

// TRANSIT_TO_AREA：按预录航点进入任务区，全部到达后回到 HOVERING。
void UavOffboardFsm::handleTransitToArea()
{
    if (!uav_takeoff_succeed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "TRANSIT_TO_AREA blocked | uavTakeoffSucceed=0");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (handleWaypointSequence(transit_waypoints_, transit_index_, "transit_to_area")) {
        uav_arrived_task_aera_ = true;
        RCLCPP_INFO(get_logger(), "TRANSIT_TO_AREA complete | uavArrivedTaskAera=1");
        transitionTo(ControlState::HOVERING);
    }
}

// HOVERING：任务区调度/等待状态，只等待上层精确命令，不自动跨阶段。
void UavOffboardFsm::handleHovering()
{
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                         "HOVERING | waiting for exact mission command");
}

// SEARCH_ADJUST_AUTO：围绕当前位置执行偏航/横移搜索，完成后等待 TARG_GOT+CONFIRM 或 NO+CONFIRM。
void UavOffboardFsm::handleSearchAdjustAuto()
{
    if (!uav_arrived_task_aera_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "SEARCH_ADJUST_AUTO blocked | uavArrivedTaskAera=0");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (handleWaypointSequence(search_waypoints_, search_index_, "search_adjust_auto")) {
        adjust_completed_ = true;
        RCLCPP_INFO(get_logger(), "SEARCH_ADJUST_AUTO complete | waiting TARG_GOT+CONFIRM or NO+CONFIRM");
        transitionTo(ControlState::HOVERING);
    }
}

// SEARCH_ADJUST_MANUAL：保持悬停，等待人工搜索后的 TARG_GOT+CONFIRM 或 NO+CONFIRM。
void UavOffboardFsm::handleSearchAdjustManual()
{
    if (!uav_arrived_task_aera_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "SEARCH_ADJUST_MANUAL blocked | uavArrivedTaskAera=0");
        transitionTo(ControlState::HOVERING);
        return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                         "SEARCH_ADJUST_MANUAL | waiting TARG_GOT+CONFIRM or NO+CONFIRM");
}

// APPROACH_PLANT：根据 bridge 的测距和真实位置反馈缓慢抵近植株，成功后回到 HOVERING。
void UavOffboardFsm::handleApproachPlant()
{
    if (!uav_search_succeed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "APPROACH_PLANT blocked | uavSearchSucceed=0");
        transitionTo(ControlState::HOVERING);
        return;
    }

    const auto distance_m = latestDistanceM();
    if (distance_m && *distance_m <= approach_target_distance_m_ + approach_distance_tolerance_m_) {
        uav_approach_succeed_ = true;
        clearActiveTarget();
        RCLCPP_INFO(get_logger(), "APPROACH_PLANT complete | uavApproachSucceed=1 source=distance_sensor");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (handleWaypointSequence(approach_waypoints_, approach_index_, "approach_plant")) {
        uav_approach_succeed_ = true;
        RCLCPP_INFO(get_logger(), "APPROACH_PLANT complete | uavApproachSucceed=1 source=target_arrival");
        transitionTo(ControlState::HOVERING);
    }
}

// SAMP_ADJUST_AUTO：接近目标后执行参数化微调，成功后回到 HOVERING 等待机械臂/采样命令。
void UavOffboardFsm::handleSampleAdjustAuto()
{
    if (!uav_approach_succeed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "SAMP_ADJUST_AUTO blocked | uavApproachSucceed=0");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (handleWaypointSequence(sample_adjust_waypoints_, sample_adjust_index_, "samp_adjust_auto")) {
        uav_adjust_succeed_ = true;
        RCLCPP_INFO(get_logger(), "SAMP_ADJUST_AUTO complete | uavAdjustSucceed=1 waiting ARM_CONFIG_PREP+SAMPL_OPERA");
        transitionTo(ControlState::HOVERING);
    }
}

// SAMP_ADJUST_MANUAL：保持悬停，等待人工 CONFIRM 标记成功，或 NO+CONFIRM 标记失败终止。
void UavOffboardFsm::handleSampleAdjustManual()
{
    if (!uav_approach_succeed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "SAMP_ADJUST_MANUAL blocked | uavApproachSucceed=0");
        transitionTo(ControlState::HOVERING);
        return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                         "SAMP_ADJUST_MANUAL | waiting CONFIRM(success) or NO+CONFIRM(fail)");
}

// RETREAT：返航前沿机体系后方退出安全距离，完成后等待 BACK_HOME。
void UavOffboardFsm::handleRetreat()
{
    if (!uav_takeoff_succeed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "RETREAT blocked | uavTakeoffSucceed=0");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (handleWaypointSequence(retreat_waypoints_, retreat_index_, "retreat")) {
        uav_ready_for_back_ = true;
        RCLCPP_INFO(get_logger(), "RETREAT complete | uavReadyForBack=1");
        transitionTo(ControlState::HOVERING);
    }
}

// BACK_HOME：按 home_waypoint 返航，到达后设置 backHome 诊断标志。
void UavOffboardFsm::handleBackHome()
{
    if (!uav_takeoff_succeed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "BACK_HOME blocked | uavTakeoffSucceed=0");
        transitionTo(ControlState::HOVERING);
        return;
    }

    if (handleWaypointSequence(back_home_waypoints_, back_home_index_, "back_home")) {
        back_home_ = true;
        RCLCPP_INFO(get_logger(), "BACK_HOME complete | backHome=1");
        transitionTo(ControlState::HOVERING);
    }
}

// UAV_TASK_TERM：任务终止等待态，由 bridge 悬停保持，只接受预返航或直接返航命令。
void UavOffboardFsm::handleTaskTerm()
{
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                         "UAV_TASK_TERM | hovering; waiting UAV_PRE_BACK_HOME or BACK_HOME");
}

// 航点序列处理：发送当前航点，等待真实位置到达，再推进到下一个航点。
bool UavOffboardFsm::handleWaypointSequence(std::vector<Waypoint> & waypoints,
                                            std::size_t & index,
                                            const std::string & label)
{
    if (waypoints.empty()) {
        RCLCPP_WARN(get_logger(), "Waypoint stage skipped | stage=%s reason=empty_list", label.c_str());
        return true;
    }
    if (index >= waypoints.size()) {
        return true;
    }
    if (!active_target_) {
        setActiveTarget(waypoints[index]);
        RCLCPP_INFO(get_logger(),
                    "Waypoint dispatch | stage=%s index=%zu/%zu target=(%.2f, %.2f, %.2f, yaw %.2f)",
                    label.c_str(), index + 1, waypoints.size(), active_target_->x,
                    active_target_->y, active_target_->z, active_target_->yaw);
    }
    if (!handleActiveTargetReached()) {
        return false;
    }
    RCLCPP_INFO(get_logger(), "Waypoint reached | stage=%s index=%zu", label.c_str(), index + 1);
    ++index;
    clearActiveTarget();
    return index >= waypoints.size();
}

// 当前活动目标完成判断：确保目标已发给 traj_offboard，再用 bridge feedback 的真实位置判定到达。
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
    const auto current = currentWaypoint();
    if (!current) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "Target active | waiting for bridge position feedback");
        return false;
    }
    return isWaypointReached(*active_target_, *current);
}

// 设置活动目标：保存将要发送给 traj_offboard 的 ENU 航点。
void UavOffboardFsm::setActiveTarget(const Waypoint & waypoint)
{
    active_target_ = waypoint;
    active_target_sent_ = false;
    target_request_pending_ = false;
}

// 清空活动目标：切换状态或航点完成后停止等待旧目标。
void UavOffboardFsm::clearActiveTarget()
{
    active_target_.reset();
    active_target_sent_ = false;
    target_request_pending_ = false;
}

// 下发活动目标：通过 traj_offboard 的 set_target 服务把目标位置、速度、加速度和偏航速度发送给 bridge/轨迹节点。
void UavOffboardFsm::sendActiveTarget()
{
    if (!active_target_) {
        return;
    }
    if (!set_target_client_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "Target pending | service not available: %s",
                             set_target_service_.c_str());
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
        [this, target](rclcpp::Client<traj_offboard::srv::SetTarget>::SharedFuture future) {
            std::lock_guard<std::mutex> lock(fsm_mutex_);
            target_request_pending_ = false;
            try {
                const auto response = future.get();
                if (response->success) {
                    active_target_sent_ = true;
                    RCLCPP_INFO(get_logger(),
                                "Target sent | target=(%.2f, %.2f, %.2f, yaw %.2f)",
                                target.x, target.y, target.z, target.yaw);
                } else {
                    active_target_sent_ = false;
                    RCLCPP_ERROR(get_logger(), "Target rejected | service=%s",
                                 set_target_service_.c_str());
                }
            } catch (const std::exception & e) {
                active_target_sent_ = false;
                RCLCPP_ERROR(get_logger(), "Target service failed | error=%s", e.what());
            }
        });
}

// 自检条件：任务使能、bridge feedback 在线，并按参数要求检查测距通信。
bool UavOffboardFsm::isSelfCheckOK()
{
    return mission_enabled_ &&
           hasFreshBridgeFeedback() &&
           (!require_distance_sensor_ || hasValidDistanceSensor());
}

// 起飞完成判断：保留独立函数名，语义对应流程图 uavTakeoffSucceed。
bool UavOffboardFsm::isUavTakeoffComplete()
{
    const auto current = currentWaypoint();
    return current && isWaypointReached(takeoff_waypoint_, *current);
}

// 航点到达判断：比较 x/y/z/yaw 误差，全部进入容差后才认为到达。
bool UavOffboardFsm::isWaypointReached(const Waypoint & waypoint, const Waypoint & current) const
{
    const double dx = waypoint.x - current.x;
    const double dy = waypoint.y - current.y;
    const double dz = waypoint.z - current.z;
    const double yaw_error = wrapAngle(waypoint.yaw - current.yaw);
    return std::abs(dx) <= position_tolerance_ &&
           std::abs(dy) <= position_tolerance_ &&
           std::abs(dz) <= position_tolerance_ &&
           std::abs(yaw_error) <= yaw_tolerance_;
}

// bridge feedback 新鲜度判断：只判断内部桥接话题是否在线，不直接判断 PX4 topic。
bool UavOffboardFsm::hasFreshBridgeFeedback() const
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return latest_bridge_feedback_ &&
           last_bridge_feedback_time_.nanoseconds() != 0 &&
           (now() - last_bridge_feedback_time_).seconds() <= bridge_feedback_timeout_s_;
}

// 测距有效性判断：只读取 bridge 已经验证过的 distance_valid 标志。
bool UavOffboardFsm::hasValidDistanceSensor() const
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return latest_bridge_feedback_ &&
           latest_bridge_feedback_->distance_valid &&
           last_bridge_feedback_time_.nanoseconds() != 0 &&
           (now() - last_bridge_feedback_time_).seconds() <= bridge_feedback_timeout_s_;
}

// 最近测距值读取：仅在 bridge 判定测距有效且 feedback 新鲜时返回。
std::optional<double> UavOffboardFsm::latestDistanceM() const
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!latest_bridge_feedback_ ||
        !latest_bridge_feedback_->distance_valid ||
        last_bridge_feedback_time_.nanoseconds() == 0 ||
        (now() - last_bridge_feedback_time_).seconds() > bridge_feedback_timeout_s_) {
        return std::nullopt;
    }
    return latest_bridge_feedback_->distance_m;
}

// 当前真实位置读取：仅使用 bridge 转换后的 ENU 位姿，不再直接订阅 PX4 输出 topic。
std::optional<UavOffboardFsm::Waypoint> UavOffboardFsm::currentWaypoint() const
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!latest_bridge_feedback_ ||
        !latest_bridge_feedback_->position_valid ||
        last_bridge_feedback_time_.nanoseconds() == 0 ||
        (now() - last_bridge_feedback_time_).seconds() > bridge_feedback_timeout_s_) {
        return std::nullopt;
    }
    return Waypoint{
        latest_bridge_feedback_->x,
        latest_bridge_feedback_->y,
        latest_bridge_feedback_->z,
        latest_bridge_feedback_->yaw};
}

// 生成相对当前位置的航点时，如果 bridge 暂无位姿，则退回起飞航点作为保底参考。
UavOffboardFsm::Waypoint UavOffboardFsm::currentOrHoverWaypoint() const
{
    const auto current = currentWaypoint();
    if (current) {
        return *current;
    }
    return takeoff_waypoint_;
}

// 生成搜索航点：以当前位置为原点做航向角变化和左右横移。
void UavOffboardFsm::generateSearchAdjustWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    search_waypoints_.clear();
    search_waypoints_.push_back({base.x, base.y, base.z, wrapAngle(base.yaw + search_yaw_offset_rad_)});
    search_waypoints_.push_back(offsetBodyFrame(base, 0.0, -search_lateral_offset_m_));
    search_waypoints_.push_back(offsetBodyFrame(base, 0.0, search_lateral_offset_m_));
    search_waypoints_.push_back({base.x, base.y, base.z, base.yaw});
}

// 生成靠近航点：优先根据测距差值计算前进距离，否则使用最大单次接近距离。
void UavOffboardFsm::generateApproachWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    double travel_distance = approach_distance_m_;
    const auto distance_m = latestDistanceM();
    if (distance_m) {
        travel_distance = std::clamp(*distance_m - approach_target_distance_m_, 0.0, approach_distance_m_);
    }
    approach_waypoints_.clear();
    if (travel_distance <= approach_distance_tolerance_m_) {
        return;
    }
    approach_waypoints_.push_back(offsetBodyFrame(base, travel_distance, 0.0));
}

// 生成采样微调航点：以当前位置为基准，按机体系前后/左右/高度/偏航偏移生成单个目标。
void UavOffboardFsm::generateSampleAdjustWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    auto target = offsetBodyFrame(base, sample_adjust_forward_m_, sample_adjust_right_m_);
    target.z += sample_adjust_z_offset_m_;
    target.yaw = wrapAngle(base.yaw + sample_adjust_yaw_offset_rad_);
    sample_adjust_waypoints_.clear();
    sample_adjust_waypoints_.push_back(target);
}

// 生成后退航点：从当前位置沿机体系后方退出 retreat_distance_m。
void UavOffboardFsm::generateRetreatWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    retreat_waypoints_.clear();
    retreat_waypoints_.push_back(offsetBodyFrame(base, -retreat_distance_m_, 0.0));
}

// 机体系偏移转换：把 forward/right 偏移按当前 yaw 转换成本地 ENU 坐标。
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

// 发布给 bridge 的主状态字符串；topic 名保持 /uav_offboard_fsm/offboard_state。
void UavOffboardFsm::publishOffboardState(ControlState state)
{
    std_msgs::msg::String msg;
    msg.data = stateToString(state);
    offboard_state_pub_->publish(msg);
}

// 发布状态机诊断：主流程标志按流程图命名，便于 ros2 topic echo 和录包验证。
void UavOffboardFsm::publishStatus(ControlState state)
{
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "state=" << stateToString(state)
        << " state_id=" << stateToId(state)
        << " uavCheckSucceed=" << (uav_check_succeed_ ? 1 : 0)
        << " uavTakeoffSucceed=" << (uav_takeoff_succeed_ ? 1 : 0)
        << " uavArrivedTaskAera=" << (uav_arrived_task_aera_ ? 1 : 0)
        << " uavSearchSucceed=" << (uav_search_succeed_ ? 1 : 0)
        << " uavApproachSucceed=" << (uav_approach_succeed_ ? 1 : 0)
        << " uavAdjustSucceed=" << (uav_adjust_succeed_ ? 1 : 0)
        << " uavReadyForBack=" << (uav_ready_for_back_ ? 1 : 0)
        << " armConfigPrep=" << (arm_config_prep_ ? 1 : 0)
        << " samplOpera=" << (sampl_opera_ ? 1 : 0)
        << " backHome=" << (back_home_ ? 1 : 0)
        << " selfCheckRequested=" << (self_check_requested_ ? 1 : 0)
        << " waitTaskEnableAuth=" << (wait_task_enable_auth_ ? 1 : 0)
        << " targGotPendingConfirm=" << (targ_got_pending_confirm_ ? 1 : 0)
        << " noPendingConfirm=" << (no_pending_confirm_ ? 1 : 0)
        << " missionEnabled=" << (mission_enabled_ ? 1 : 0)
        << " bridgeFeedbackFresh=" << (hasFreshBridgeFeedback() ? 1 : 0)
        << " distanceSensorValid=" << (hasValidDistanceSensor() ? 1 : 0);
    msg.data = out.str();
    status_pub_->publish(msg);
}

// 控制命令回调：只接受流程图定义的精确 token，不再支持命令别名。
void UavOffboardFsm::handleControlCommand(const std_msgs::msg::String::SharedPtr msg)
{
    const auto parsed = parseCommand(msg->data);
    if (!parsed) {
        RCLCPP_WARN(get_logger(), "Command rejected | unknown=%s", msg->data.c_str());
        return;
    }

    std::lock_guard<std::mutex> lock(fsm_mutex_);
    const auto state = control_state_.load();
    const auto command = *parsed;

    if (command == CommandType::SELF_CHECK) {
        resetMissionProgress();
        self_check_requested_ = true;
        transitionTo(ControlState::SELF_CHECK);
        RCLCPP_INFO(get_logger(), "Command accepted | SELF_CHECK");
        return;
    }

    if (command == CommandType::WAIT_TASK_ENABLE_AUTH) {
        if (state != ControlState::SELF_CHECK) {
            RCLCPP_WARN(get_logger(), "Command rejected | WAIT_TASK_ENABLE_AUTH current=%s",
                        stateToString(state).c_str());
            return;
        }
        wait_task_enable_auth_ = true;
        if (uav_check_succeed_) {
            transitionTo(ControlState::UAV_START);
            RCLCPP_INFO(get_logger(), "Command accepted | WAIT_TASK_ENABLE_AUTH -> UAV_START");
        } else {
            RCLCPP_INFO(get_logger(), "Command accepted | WAIT_TASK_ENABLE_AUTH pending uavCheckSucceed");
        }
        return;
    }

    switch (command) {
        case CommandType::NAV_TO_TASK_DOM:
            if (state != ControlState::HOVERING || !uav_takeoff_succeed_) {
                RCLCPP_WARN(get_logger(), "Command rejected | NAV_TO_TASK_DOM current=%s uavTakeoffSucceed=%s",
                            stateToString(state).c_str(), uav_takeoff_succeed_ ? "true" : "false");
                return;
            }
            transitionTo(ControlState::TRANSIT_TO_AREA);
            RCLCPP_INFO(get_logger(), "Command accepted | NAV_TO_TASK_DOM");
            break;
        case CommandType::SEARCH_ADJUST_AUTO:
            if (state != ControlState::HOVERING || !uav_arrived_task_aera_) {
                RCLCPP_WARN(get_logger(), "Command rejected | SEARCH_ADJUST_AUTO current=%s uavArrivedTaskAera=%s",
                            stateToString(state).c_str(), uav_arrived_task_aera_ ? "true" : "false");
                return;
            }
            transitionTo(ControlState::SEARCH_ADJUST_AUTO);
            RCLCPP_INFO(get_logger(), "Command accepted | SEARCH_ADJUST_AUTO");
            break;
        case CommandType::SEARCH_ADJUST_MANUAL:
            if (state != ControlState::HOVERING || !uav_arrived_task_aera_) {
                RCLCPP_WARN(get_logger(), "Command rejected | SEARCH_ADJUST_MANUAL current=%s uavArrivedTaskAera=%s",
                            stateToString(state).c_str(), uav_arrived_task_aera_ ? "true" : "false");
                return;
            }
            transitionTo(ControlState::SEARCH_ADJUST_MANUAL);
            RCLCPP_INFO(get_logger(), "Command accepted | SEARCH_ADJUST_MANUAL");
            break;
        case CommandType::TARG_GOT:
            if (!uav_arrived_task_aera_ ||
                !(state == ControlState::HOVERING ||
                  state == ControlState::SEARCH_ADJUST_AUTO ||
                  state == ControlState::SEARCH_ADJUST_MANUAL) ||
                uav_approach_succeed_) {
                RCLCPP_WARN(get_logger(), "Command rejected | TARG_GOT current=%s uavArrivedTaskAera=%s",
                            stateToString(state).c_str(), uav_arrived_task_aera_ ? "true" : "false");
                return;
            }
            clearActiveTarget();
            uav_search_succeed_ = true;
            targ_got_pending_confirm_ = true;
            no_pending_confirm_ = false;
            pending_failure_ = PendingFailure::NONE;
            RCLCPP_INFO(get_logger(), "Command accepted | TARG_GOT waiting CONFIRM");
            break;
        case CommandType::NO: {
            const bool search_failure_context =
                state == ControlState::SEARCH_ADJUST_AUTO ||
                state == ControlState::SEARCH_ADJUST_MANUAL ||
                (state == ControlState::HOVERING && uav_arrived_task_aera_ &&
                 !uav_search_succeed_ && !uav_approach_succeed_);
            const bool sample_failure_context =
                state == ControlState::SAMP_ADJUST_AUTO ||
                state == ControlState::SAMP_ADJUST_MANUAL ||
                (state == ControlState::HOVERING && uav_approach_succeed_ && !uav_adjust_succeed_);
            if (!search_failure_context && !sample_failure_context) {
                RCLCPP_WARN(get_logger(), "Command rejected | NO current=%s",
                            stateToString(state).c_str());
                return;
            }
            clearActiveTarget();
            no_pending_confirm_ = true;
            targ_got_pending_confirm_ = false;
            pending_failure_ = search_failure_context ? PendingFailure::SEARCH : PendingFailure::SAMPLE;
            if (pending_failure_ == PendingFailure::SEARCH) {
                uav_search_succeed_ = false;
                adjust_completed_ = false;
            } else {
                uav_adjust_succeed_ = false;
                arm_config_prep_ = false;
                sampl_opera_ = false;
            }
            RCLCPP_INFO(get_logger(), "Command accepted | NO waiting CONFIRM");
            break;
        }
        case CommandType::CONFIRM:
            if (no_pending_confirm_ && pending_failure_ != PendingFailure::NONE) {
                no_pending_confirm_ = false;
                pending_failure_ = PendingFailure::NONE;
                clearActiveTarget();
                transitionTo(ControlState::UAV_TASK_TERM);
                RCLCPP_INFO(get_logger(), "Command accepted | NO+CONFIRM -> UAV_TASK_TERM");
                return;
            }
            if (targ_got_pending_confirm_) {
                targ_got_pending_confirm_ = false;
                adjust_completed_ = true;
                uav_search_succeed_ = true;
                transitionTo(ControlState::APPROACH_PLANT);
                RCLCPP_INFO(get_logger(), "Command accepted | TARG_GOT+CONFIRM -> APPROACH_PLANT");
                return;
            }
            if (state == ControlState::SAMP_ADJUST_MANUAL) {
                uav_adjust_succeed_ = true;
                transitionTo(ControlState::HOVERING);
                RCLCPP_INFO(get_logger(), "Command accepted | CONFIRM -> SAMP_ADJUST_MANUAL success");
                return;
            }
            RCLCPP_WARN(get_logger(), "Command rejected | CONFIRM has no pending action state=%s",
                        stateToString(state).c_str());
            break;
        case CommandType::SAMP_ADJUST_AUTO:
            if (state != ControlState::HOVERING || !uav_approach_succeed_) {
                RCLCPP_WARN(get_logger(), "Command rejected | SAMP_ADJUST_AUTO current=%s uavApproachSucceed=%s",
                            stateToString(state).c_str(), uav_approach_succeed_ ? "true" : "false");
                return;
            }
            transitionTo(ControlState::SAMP_ADJUST_AUTO);
            RCLCPP_INFO(get_logger(), "Command accepted | SAMP_ADJUST_AUTO");
            break;
        case CommandType::SAMP_ADJUST_MANUAL:
            if (state != ControlState::HOVERING || !uav_approach_succeed_) {
                RCLCPP_WARN(get_logger(), "Command rejected | SAMP_ADJUST_MANUAL current=%s uavApproachSucceed=%s",
                            stateToString(state).c_str(), uav_approach_succeed_ ? "true" : "false");
                return;
            }
            transitionTo(ControlState::SAMP_ADJUST_MANUAL);
            RCLCPP_INFO(get_logger(), "Command accepted | SAMP_ADJUST_MANUAL");
            break;
        case CommandType::ARM_CONFIG_PREP:
            if (state != ControlState::HOVERING || !uav_adjust_succeed_) {
                RCLCPP_WARN(get_logger(), "Command rejected | ARM_CONFIG_PREP current=%s uavAdjustSucceed=%s",
                            stateToString(state).c_str(), uav_adjust_succeed_ ? "true" : "false");
                return;
            }
            arm_config_prep_ = true;
            RCLCPP_INFO(get_logger(), "Command accepted | ARM_CONFIG_PREP");
            break;
        case CommandType::SAMPL_OPERA:
            if (state != ControlState::HOVERING || !uav_adjust_succeed_ || !arm_config_prep_) {
                RCLCPP_WARN(get_logger(), "Command rejected | SAMPL_OPERA current=%s uavAdjustSucceed=%s armConfigPrep=%s",
                            stateToString(state).c_str(), uav_adjust_succeed_ ? "true" : "false",
                            arm_config_prep_ ? "true" : "false");
                return;
            }
            sampl_opera_ = true;
            RCLCPP_INFO(get_logger(), "Command accepted | SAMPL_OPERA");
            break;
        case CommandType::UAV_PRE_BACK_HOME:
            if (state == ControlState::UAV_TASK_TERM) {
                transitionTo(ControlState::RETREAT);
                RCLCPP_INFO(get_logger(), "Command accepted | UAV_PRE_BACK_HOME from UAV_TASK_TERM");
                return;
            }
            if (state != ControlState::HOVERING || !uav_takeoff_succeed_ ||
                !arm_config_prep_ || !sampl_opera_) {
                RCLCPP_WARN(get_logger(),
                            "Command rejected | UAV_PRE_BACK_HOME current=%s uavTakeoffSucceed=%s armConfigPrep=%s samplOpera=%s",
                            stateToString(state).c_str(), uav_takeoff_succeed_ ? "true" : "false",
                            arm_config_prep_ ? "true" : "false", sampl_opera_ ? "true" : "false");
                return;
            }
            transitionTo(ControlState::RETREAT);
            RCLCPP_INFO(get_logger(), "Command accepted | UAV_PRE_BACK_HOME");
            break;
        case CommandType::BACK_HOME:
            if (state == ControlState::UAV_TASK_TERM) {
                transitionTo(ControlState::BACK_HOME);
                RCLCPP_INFO(get_logger(), "Command accepted | BACK_HOME from UAV_TASK_TERM");
                return;
            }
            if (state != ControlState::HOVERING || !uav_ready_for_back_) {
                RCLCPP_WARN(get_logger(), "Command rejected | BACK_HOME current=%s uavReadyForBack=%s",
                            stateToString(state).c_str(), uav_ready_for_back_ ? "true" : "false");
                return;
            }
            transitionTo(ControlState::BACK_HOME);
            RCLCPP_INFO(get_logger(), "Command accepted | BACK_HOME");
            break;
        case CommandType::SELF_CHECK:
        case CommandType::WAIT_TASK_ENABLE_AUTH:
            break;
    }
}

// 任务总状态回调：只影响 SELF_CHECK 的总任务使能条件。
void UavOffboardFsm::handleMissionState(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    const auto state = upperCopy(msg->data);
    if (tokenMatches(state, mission_enabled_aliases_)) {
        mission_enabled_ = true;
    } else if (tokenMatches(state, mission_disabled_aliases_)) {
        mission_enabled_ = false;
    } else {
        RCLCPP_WARN(get_logger(), "Mission state ignored | unknown=%s", msg->data.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "Mission state updated | mission_enabled=%s",
                mission_enabled_ ? "true" : "false");
}

// bridge feedback 回调：保存 bridge 汇总后的真实位置、测距和传感器有效性。
void UavOffboardFsm::handleBridgeFeedback(const traj_offboard::msg::BridgeFeedback::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    latest_bridge_feedback_ = *msg;
    last_bridge_feedback_time_ = now();
}

// 单航点参数解析：要求参数正好包含 [x, y, z, yaw] 四个值。
UavOffboardFsm::Waypoint
UavOffboardFsm::parseSingleWaypointParameter(const std::vector<double> & flat,
                                             const Waypoint & fallback)
{
    if (flat.size() != 4) {
        return fallback;
    }
    return {flat[0], flat[1], flat[2], flat[3]};
}

// 多航点参数解析：把 [x, y, z, yaw, ...] 扁平数组转换为航点序列。
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

// 三维向量参数解析：用于目标速度和加速度。
std::array<double, 3>
UavOffboardFsm::parseVector3Parameter(const std::vector<double> & flat,
                                      const std::array<double, 3> & fallback)
{
    if (flat.size() != 3) {
        return fallback;
    }
    return {flat[0], flat[1], flat[2]};
}

// 状态枚举转字符串：字符串与流程图主状态名严格一致。
std::string UavOffboardFsm::stateToString(ControlState state)
{
    switch (state) {
        case ControlState::SELF_CHECK:
            return "SELF_CHECK";
        case ControlState::UAV_START:
            return "UAV_START";
        case ControlState::TRANSIT_TO_AREA:
            return "TRANSIT_TO_AREA";
        case ControlState::HOVERING:
            return "HOVERING";
        case ControlState::SEARCH_ADJUST_AUTO:
            return "SEARCH_ADJUST_AUTO";
        case ControlState::SEARCH_ADJUST_MANUAL:
            return "SEARCH_ADJUST_MANUAL";
        case ControlState::APPROACH_PLANT:
            return "APPROACH_PLANT";
        case ControlState::SAMP_ADJUST_AUTO:
            return "SAMP_ADJUST_AUTO";
        case ControlState::SAMP_ADJUST_MANUAL:
            return "SAMP_ADJUST_MANUAL";
        case ControlState::RETREAT:
            return "RETREAT";
        case ControlState::BACK_HOME:
            return "BACK_HOME";
        case ControlState::UAV_TASK_TERM:
            return "UAV_TASK_TERM";
    }
    return "UNKNOWN";
}

// 状态枚举转流程图编号：编号固定为 0-11。
int UavOffboardFsm::stateToId(ControlState state)
{
    switch (state) {
        case ControlState::SELF_CHECK:
            return 0;
        case ControlState::UAV_START:
            return 1;
        case ControlState::TRANSIT_TO_AREA:
            return 2;
        case ControlState::HOVERING:
            return 3;
        case ControlState::SEARCH_ADJUST_AUTO:
            return 4;
        case ControlState::SEARCH_ADJUST_MANUAL:
            return 5;
        case ControlState::APPROACH_PLANT:
            return 6;
        case ControlState::SAMP_ADJUST_AUTO:
            return 7;
        case ControlState::SAMP_ADJUST_MANUAL:
            return 8;
        case ControlState::RETREAT:
            return 9;
        case ControlState::BACK_HOME:
            return 10;
        case ControlState::UAV_TASK_TERM:
            return 11;
    }
    return -1;
}

// 控制命令解析：只比较首个 token 的精确字符串，不再查旧命令别名。
std::optional<UavOffboardFsm::CommandType>
UavOffboardFsm::parseCommand(const std::string & command)
{
    std::istringstream stream(command);
    std::string token;
    stream >> token;
    if (token == "SELF_CHECK") {
        return CommandType::SELF_CHECK;
    }
    if (token == "WAIT_TASK_ENABLE_AUTH") {
        return CommandType::WAIT_TASK_ENABLE_AUTH;
    }
    if (token == "NAV_TO_TASK_DOM") {
        return CommandType::NAV_TO_TASK_DOM;
    }
    if (token == "SEARCH_ADJUST_AUTO") {
        return CommandType::SEARCH_ADJUST_AUTO;
    }
    if (token == "SEARCH_ADJUST_MANUAL") {
        return CommandType::SEARCH_ADJUST_MANUAL;
    }
    if (token == "TARG_GOT") {
        return CommandType::TARG_GOT;
    }
    if (token == "CONFIRM") {
        return CommandType::CONFIRM;
    }
    if (token == "NO") {
        return CommandType::NO;
    }
    if (token == "SAMP_ADJUST_AUTO") {
        return CommandType::SAMP_ADJUST_AUTO;
    }
    if (token == "SAMP_ADJUST_MANUAL") {
        return CommandType::SAMP_ADJUST_MANUAL;
    }
    if (token == "ARM_CONFIG_PREP") {
        return CommandType::ARM_CONFIG_PREP;
    }
    if (token == "SAMPL_OPERA") {
        return CommandType::SAMPL_OPERA;
    }
    if (token == "UAV_PRE_BACK_HOME") {
        return CommandType::UAV_PRE_BACK_HOME;
    }
    if (token == "BACK_HOME") {
        return CommandType::BACK_HOME;
    }
    return std::nullopt;
}

// token 匹配工具：仅用于 mission_state 使能/禁用字符串。
bool UavOffboardFsm::tokenMatches(const std::string & token,
                                  const std::vector<std::string> & aliases)
{
    return std::find(aliases.begin(), aliases.end(), token) != aliases.end();
}

// 字符串数组大写化工具：用于 mission_state 兼容已有外部状态字符串。
std::vector<std::string> UavOffboardFsm::upperCopyList(std::vector<std::string> values)
{
    for (auto & value : values) {
        value = upperCopy(value);
    }
    return values;
}

// 字符串大写化工具：用于 mission_state 解析。
std::string UavOffboardFsm::upperCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return value;
}

// 角度归一化工具：把任意弧度角压到 [-pi, pi]。
double UavOffboardFsm::wrapAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

// 程序入口：初始化 ROS2，创建状态机节点，按参数指定线程数启动 MultiThreadedExecutor。
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UavOffboardFsm>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(),
                                                  node->executorThreads());
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
