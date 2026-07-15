#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <status_interfaces_pkg/msg/task_fsm.hpp>
#include <status_interfaces_pkg/msg/status.hpp>
#include <status_interfaces_pkg/srv/actuator_control.hpp>
#include <status_interfaces_pkg/srv/switch_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <traj_offboard/srv/set_target.hpp>
#include <traj_offboard/msg/traj_complete_flag.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

// ANSI color helpers for terminal log highlights.
// green = state switch / milestone success; blue = setpoint (target) change events.
#define LOG_COLOR_GREEN "\033[1;32m"
#define LOG_COLOR_BLUE  "\033[1;34m"
#define LOG_COLOR_RESET "\033[0m"

class UavOffboardFsm : public rclcpp::Node {
  public:
    // 构造函数：声明并读取所有 ROS2 参数，随后按参数配置创建发布器、订阅器、服务客户端和状态机定时器。
    UavOffboardFsm() : rclcpp::Node("uav_offboard_fsm") {
        const std::vector<double> default_takeoff = {0.0, 0.0, 5.0, 0.0}; //需要根据轨迹航线确定起始高度
        takeoff_waypoint_ =
            parseSingleWaypointParameter(declare_parameter<std::vector<double>>("takeoff_waypoint", default_takeoff),
                                         {0.0, 0.0, 5.0, 0.0});
        home_waypoint_ =
            parseSingleWaypointParameter(declare_parameter<std::vector<double>>("home_waypoint", default_takeoff),
                                         takeoff_waypoint_);

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
        use_xy_adjust_ = declare_parameter<bool>("use_xy_adjust", use_xy_adjust_);
        use_z_adjust_ = declare_parameter<bool>("use_z_adjust", use_z_adjust_);
        direct_to_uav_hold_after_self_check_ =
            declare_parameter<bool>("direct_to_uav_hold_after_self_check",
                                    direct_to_uav_hold_after_self_check_);
        hold_adjust_pos_des_timeout_s_ =
            declare_parameter<double>("hold_adjust_pos_des_timeout_s", hold_adjust_pos_des_timeout_s_);
        hold_adjust_min_update_period_s_ =
            declare_parameter<double>("hold_adjust_min_update_period_s", hold_adjust_min_update_period_s_);
        hold_adjust_target_update_tolerance_m_ =
            declare_parameter<double>("hold_adjust_target_update_tolerance_m",
                                      hold_adjust_target_update_tolerance_m_);
        hold_adjust_max_xy_step_m_ =
            declare_parameter<double>("hold_adjust_max_xy_step_m", hold_adjust_max_xy_step_m_);
        hold_adjust_max_z_step_m_ =
            declare_parameter<double>("hold_adjust_max_z_step_m", hold_adjust_max_z_step_m_);
        hold_adjust_max_velocity_xyz_ = parseVector3Parameter(
            declare_parameter<std::vector<double>>(
                "hold_adjust_max_velocity_xyz",
                {hold_adjust_max_velocity_xyz_[0],
                 hold_adjust_max_velocity_xyz_[1],
                 hold_adjust_max_velocity_xyz_[2]}),
            hold_adjust_max_velocity_xyz_);

        // 必须在创建任何 pub/sub/timer/service/client 之前先建好 callback group。
        cbg_fsm_     = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cbg_sensor_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cbg_command_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cbg_service_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sensor_sub_opts;
        sensor_sub_opts.callback_group = cbg_sensor_;
        rclcpp::SubscriptionOptions command_sub_opts;
        command_sub_opts.callback_group = cbg_command_;

        // 发给底层offboard状态定义
        offboard_state_pub_ =
            create_publisher<std_msgs::msg::String>("/uav_offboard_fsm/offboard_state", publisher_queue_depth_);
        // 对外发布当前状态索引，消息定义在 status_interfaces_pkg/msg/Status.msg 中。
        status_pub_ =
            create_publisher<status_interfaces_pkg::msg::Status>("/uav_offboard_fsm/status", publisher_queue_depth_);
        vehicle_command_publisher_ =
            create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", publisher_queue_depth_);

        //请求轨迹生成服务的客户端，发送目标点给在线轨迹生成器，后者调用ruckig库计算轨迹并发布状态反馈
        //客户端响应回调会取 fsm_mutex_，与 controlLoopOnTimer 同组以天然串行。
        set_target_client_ =
            create_client<traj_offboard::srv::SetTarget>("online_traj_generator/set_target",
                                                         rmw_qos_profile_services_default, cbg_fsm_);
        switch_status_client_ =
            create_client<status_interfaces_pkg::srv::SwitchStatus>("/ground_station/switch_status",
                                                                    rmw_qos_profile_services_default, cbg_fsm_);
        // actuator_control 服务与 actuator_timer_ 共享 cbg_service_：
        // 二者都会调用 publish_vehicle_command，互斥执行可避免服务更新设定值与定时重发交叉。
        actuator_control_srv_ = create_service<status_interfaces_pkg::srv::ActuatorControl>(
            "/uav_offboard_fsm/actuator_control",
            std::bind(&UavOffboardFsm::handleActuatorControl, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cbg_service_);

        status_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                          std::bind(&UavOffboardFsm::statusPublishOnTimer, this),
                                          cbg_fsm_);
        // keyboard control command
        control_command_sub_ = create_subscription<std_msgs::msg::String>(
            "/uav_offboard_fsm/control_command", subscriber_queue_depth_,
            std::bind(&UavOffboardFsm::handleControlCommand, this, std::placeholders::_1),
            command_sub_opts);
        // main state control command
        main_task_status_sub_ = create_subscription<status_interfaces_pkg::msg::TaskFSM>(
            "/main_task_fsm/task_states", subscriber_queue_depth_,
            std::bind(&UavOffboardFsm::handleMainTaskStatus, this, std::placeholders::_1),
            command_sub_opts);

        ruckig_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/online_traj_generator/ruckig_state", state_feedback_queue_depth_,
            std::bind(&UavOffboardFsm::handleRuckigState, this, std::placeholders::_1),
            sensor_sub_opts);

        traj_complete_flag_sub_ = create_subscription<traj_offboard::msg::TrajCompleteFlag>(
            "/traj_offboard/traj_complete_flag", subscriber_queue_depth_,
            std::bind(&UavOffboardFsm::handleTrajCompleteFlag, this, std::placeholders::_1),
            sensor_sub_opts);
        auto sensor_qos = rclcpp::SensorDataQoS(); //best effort, duable volatile
        sensor_qos.keep_last(static_cast<std::size_t>(sensor_queue_depth_));

        vehicle_local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", sensor_qos,
            std::bind(&UavOffboardFsm::handleVehicleLocalPosition, this, std::placeholders::_1),
            sensor_sub_opts);

        home_position_sub_ = create_subscription<px4_msgs::msg::HomePosition>(
            "/fmu/out/home_position", sensor_qos,
            std::bind(&UavOffboardFsm::handleHomePosition, this, std::placeholders::_1),
            sensor_sub_opts);

        distance_sensor_sub_ = create_subscription<px4_msgs::msg::DistanceSensor>(
            "/fmu/out/distance_sensor", sensor_qos,
            std::bind(&UavOffboardFsm::handleDistanceSensor, this, std::placeholders::_1),
            sensor_sub_opts);

        manual_control_setpoint_sub_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
            "/fmu/out/manual_control_setpoint", sensor_qos,
            std::bind(&UavOffboardFsm::handleManualControlSetpoint, this, std::placeholders::_1),
            sensor_sub_opts);

        whole_body_uav_setpoint_sub_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/whole_body_planner/uav_setpoint", subscriber_queue_depth_,
            std::bind(&UavOffboardFsm::handleWholeBodyUavSetpoint, this, std::placeholders::_1),
            sensor_sub_opts);

        // actuator_outputs_sub_ = create_subscription<px4_msgs::msg::ActuatorOutputs>(
        //     "/fmu/out/actuator_outputs", sensor_qos,
        //     std::bind(&UavOffboardFsm::handleActuatorOutputs, this, std::placeholders::_1));
        timer_ = create_wall_timer(std::chrono::milliseconds(control_loop_period_ms_),
                                   std::bind(&UavOffboardFsm::controlLoopOnTimer, this),
                                   cbg_fsm_);
        // 5Hz 持续向 FMU 重发当前 actuator 设定值；与 actuator_control 服务共用 cbg_service_，
        // 因二者都会触发 publish_vehicle_command，互斥执行避免与服务回调交叉。
        actuator_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&UavOffboardFsm::publishActuatorCommand, this),
                                            cbg_service_);

        RCLCPP_INFO(get_logger(),
                    LOG_COLOR_GREEN "uav_offboard_fsm ready | initial=%s loop=%dms; waiting for SELF_CHECK" LOG_COLOR_RESET,
                    stateToString(control_state_.load()).c_str(), control_loop_period_ms_);
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
    using Vector3 = std::array<double, 3>;
    struct TimedHoldSetpoint {
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        rclcpp::Time stamp;
    };

    enum class ControlState : int {
        SELF_CHECK = 0,
        UAV_START = 1,
        TRANSIT_TO_AREA = 2,
        UAV_ARRIVED_AERA = 3,
        SEARCH_ADJUST_AUTO = 4,
        SEARCH_ADJUST_MANUAL = 5,
        APPROACH_PLANT = 6,
        UAV_PRE_HOLD = 7,
        SAMP_ADJUST_AUTO = 8,
        SAMP_ADJUST_MANUAL = 9,
        UAV_HOLD = 10,
        RETREAT = 11,
        UAV_BACK_HOME = 12,
        UAV_TASK_TERM = 13
    };

    enum class CommandType {
        PRE_CHECK,
        WAIT_TASK_ENABLE_AUTH,
        NAV_TO_TASK_DOM,
        ARRIVE_TASK_DOM,
        UAV_SEARCH_TARGS,
        SEARCH_ADJUST_AUTO,
        SEARCH_ADJUST_MANUAL,
        TARG_GOT,
        TARG_READY,
        UAV_POSE_ADAP,
        SAMP_ADJUST_AUTO,
        SAMP_ADJUST_MANUAL,
        ARM_CONFIG_PREP,
        SAMPL_OPERA,
        UAV_PRE_BACK_HOME,
        BACK_HOME,
        TASK_TERM,
        NO,
        CONFIRM
    };

    struct ParsedCommand {
        CommandType type;
    };

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr offboard_state_pub_;
    rclcpp::Publisher<status_interfaces_pkg::msg::Status>::SharedPtr status_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Service<status_interfaces_pkg::srv::ActuatorControl>::SharedPtr actuator_control_srv_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_command_sub_;
    rclcpp::Subscription<status_interfaces_pkg::msg::TaskFSM>::SharedPtr main_task_status_sub_;
    rclcpp::Client<traj_offboard::srv::SetTarget>::SharedPtr set_target_client_;
    rclcpp::Client<status_interfaces_pkg::srv::SwitchStatus>::SharedPtr switch_status_client_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ruckig_state_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr home_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distance_sensor_sub_;
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_setpoint_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr whole_body_uav_setpoint_sub_;
    // rclcpp::Subscription<px4_msgs::msg::ActuatorOutputs>::SharedPtr actuator_outputs_sub_;

    rclcpp::Subscription<traj_offboard::msg::TrajCompleteFlag>::SharedPtr traj_complete_flag_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr actuator_timer_;

    std::atomic<ControlState> control_state_{ControlState::SELF_CHECK};
    std::atomic<int> last_main_task_status_{-1};
    std::atomic<int64_t> last_main_task_dispatch_ns_{0};
    ControlState previous_state_{ControlState::SELF_CHECK};
    mutable std::mutex fsm_mutex_;
    // Callback group 分配：
    //  cbg_fsm_      —— 状态机主循环、状态发布定时器、客户端响应（共用 fsm_mutex_，互斥即可）
    //  cbg_sensor_   —— 高频传感器订阅（位置、home、测距、ruckig 反馈、轨迹完成标志）
    //  cbg_command_  —— 上层/键盘指令订阅
    //  cbg_service_  —— actuator_control 服务与 actuator_timer_（共用，避免向 FMU 的重发与服务更新交叉）
    rclcpp::CallbackGroup::SharedPtr cbg_fsm_;
    rclcpp::CallbackGroup::SharedPtr cbg_sensor_;
    rclcpp::CallbackGroup::SharedPtr cbg_command_;
    rclcpp::CallbackGroup::SharedPtr cbg_service_;

    std::optional<sensor_msgs::msg::JointState> latest_actual_state_;
    std::optional<sensor_msgs::msg::JointState> latest_reference_state_;
    rclcpp::Time last_actual_state_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_reference_state_time_{0, 0, RCL_ROS_TIME};
    double home_x_{0.0};
    double home_y_{0.0};
    double home_z_{0.0};
    bool home_position_valid_{false};
    mutable std::mutex latest_state_mutex_;
    std::optional<double> latest_distance_m_;
    rclcpp::Time last_distance_sensor_time_{0, 0, RCL_ROS_TIME};
    std::optional<Vector3> latest_hold_uav_setpoint_;
    std::optional<Vector3> latest_hold_uav_velocity_;
    std::optional<Vector3> latest_hold_uav_acceleration_;
    rclcpp::Time last_hold_pos_des_time_{0, 0, RCL_ROS_TIME};

    traj_offboard::msg::TrajCompleteFlag traj_complete_flag_;
    //状态机内部状态标志位和参数
    bool ready_for_takeoff_{false};
    bool self_check_requested_{false};
    bool ready_for_transit_{false};
    bool is_arrived_task_aera_{false};
    bool adjust_completed_{false};
    bool uav_search_succeed_{false};
    bool approach_completed_{false};
    bool uav_adjust_succeed_{false};
    bool arm_config_prepared_{false};
    bool sampl_opera_completed_{false};
    bool uav_ready_for_back_{false};
    bool back_home_{false};
    bool targ_got_confirm_pending_{false};
    bool task_term_confirm_pending_{false};
    bool switch_status_request_pending_{false};
    bool require_distance_sensor_{false};
    bool require_external_switch_service_{false};
    bool hold_adjust_started_{false};
    bool hold_adjust_stale_hold_sent_{false};
    std::optional<Vector3> hold_adjust_desired_local_;
    rclcpp::Time hold_adjust_last_planned_pos_des_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time hold_adjust_last_target_update_time_{0, 0, RCL_ROS_TIME};

    double position_tolerance_{0.1};
    double yaw_tolerance_{0.1}; // 0.1 radian，约5.7度
    double distance_sensor_timeout_s_{1.0};
    double vehicle_local_position_timeout_s_{1.0};
    double state_feedback_timeout_s_{1.0};
    double search_yaw_offset_rad_{0.35};
    double search_lateral_offset_m_{0.4};
    double approach_target_distance_m_{0.7};
    double approach_distance_tolerance_m_{0.1};
    // 无测距传感器（仿真）时允许的最大体系前向位移（也作为有测距时的安全上限）。
    double approach_max_travel_m_{3.0};
    // 靠近段期望的体系前向速度（仅 x 轴），y、z 方向恒为 0。
    double approach_speed_m_s_{0.1};
    // RETREAT 段目标：测距传感器读数达到 retreat_target_distance_m_ 时停止；
    // 仿真模式（无测距）则以 retreat_max_travel_m_ 作为体系后向最大位移；体系 x 速度上限 retreat_speed_m_s_。
    double retreat_target_distance_m_{5.0};
    double retreat_max_travel_m_{5.0};
    double retreat_speed_m_s_{0.5};
    double sample_adjust_forward_m_{0.2};
    double sample_adjust_right_m_{0.0};
    double sample_adjust_z_offset_m_{0.0};
    double sample_adjust_yaw_offset_rad_{0.0};
    bool direct_to_uav_hold_after_self_check_{false};
    // UAV_HOLD 内 whole_body_planner 的 UAV 期望位置持续调整；xy/z 可独立开启。
    bool use_xy_adjust_{false};
    bool use_z_adjust_{false};
    double hold_adjust_pos_des_timeout_s_{1.0};
    double hold_adjust_min_update_period_s_{0.0};
    double hold_adjust_target_update_tolerance_m_{0.02};
    double hold_adjust_max_xy_step_m_{0.5};
    double hold_adjust_max_z_step_m_{0.3};
    // UAV_HOLD 调整段 Ruckig 平动速度上限；分量 <=0 表示沿用全局默认。
    Vector3 hold_adjust_max_velocity_xyz_{0.0, 0.0, 0.0};
    std::array<double, 3> target_velocity_{0.0, 0.0, 0.0};
    std::array<double, 3> target_acceleration_{0.0, 0.0, 0.0};
    // 当前段下发给 Ruckig 的平动速度上限覆盖；分量 <=0 表示让 OTG 沿用 VEL_LIMIT 默认。
    std::array<double, 3> target_max_velocity_xyz_{0.0, 0.0, 0.0};
    double target_yawspeed_{0.0};
    double heading_yaw_offset_rad_{1.5707963267948966};
    int distance_sensor_min_signal_quality_{1};
    int control_loop_period_ms_{50};
    int executor_threads_{4};
    int publisher_queue_depth_{10};
    int subscriber_queue_depth_{10};
    int state_feedback_queue_depth_{10};
    int sensor_queue_depth_{10};
    int log_throttle_ms_{2000};
    int takeoff_wait_log_throttle_ms_{1000};
    int hovering_log_throttle_ms_{3000};
    int main_task_repeat_dispatch_period_ms_{500};
    int switch_status_urgency_{5};

    Waypoint takeoff_waypoint_{};             // 起飞完成时由 traj_first_setpoint 覆盖
    Waypoint home_waypoint_{};
    Waypoint transit_end_waypoint_{};         // TRANSIT_TO_AREA 结束时由 traj_last_setpoint 填充
    bool transit_end_waypoint_valid_{false};
    std::vector<Waypoint> transit_waypoints_;
    std::vector<Waypoint> search_waypoints_;
    std::vector<Waypoint> approach_waypoints_;
    std::vector<Waypoint> sample_adjust_waypoints_;
    std::vector<Waypoint> hold_adjust_waypoints_;
    std::vector<Waypoint> retreat_waypoints_;
    std::vector<Waypoint> back_home_waypoints_;
    std::size_t transit_index_{0};
    std::size_t search_index_{0};
    std::size_t approach_index_{0};
    std::size_t sample_adjust_index_{0};
    std::size_t hold_adjust_index_{0};
    std::size_t retreat_index_{0};
    std::size_t back_home_index_{0};

    std::optional<Waypoint> active_target_;
    bool active_target_sent_{false};
    bool target_request_pending_{false};
    rclcpp::Time last_target_sent_time_{0, 0, RCL_ROS_TIME};

    // 由 actuator_timer_ 以 5Hz 持续向 FMU 重发 VEHICLE_CMD_DO_SET_ACTUATOR；
    // close=0 请求先连续两个周期发送初始化值，再恢复发送实际关闭值。
    std::atomic<float> actuator_close{0.0f};
    std::atomic<float> actuator_cut{-1.0f};
    std::atomic<int> actuator_close_init_cycles_remaining{0};
    std::atomic<bool> actuator_command_enabled_{false};
    std::atomic<bool> manual_control_setpoint_received_{false};

    void publishActuatorCommand();

    void controlLoopOnTimer();
    void onStateEntry(ControlState state);
    void transitionTo(ControlState state);
    void resetMissionProgress();

    void handleSelfCheck();
    void handleUavStart();
    void handleUavArrivedArea();
    void handleTransitToArea();
    void handleSearchAdjustAuto();
    void handleSearchAdjustManual();
    void handleApproachPlant();
    void handleUavPreHold();
    void handleSampleAdjustAuto();
    void handleSampleAdjustManual();
    void handleUavHold();
    void handleRetreat();
    void handleActuatorControl(
        const std::shared_ptr<status_interfaces_pkg::srv::ActuatorControl::Request> request,
        std::shared_ptr<status_interfaces_pkg::srv::ActuatorControl::Response> response);
    void handleBackHome();
    void handleTaskTerm();

    bool handleWaypointSequence(std::vector<Waypoint> & waypoints, std::size_t & index,
                                const std::string & label);
    bool handleActiveTargetReached();
    void setActiveTarget(const Waypoint & waypoint);
    void clearActiveTarget();
    bool sendActiveTarget(bool throttle_success_log = false);

    void publish_vehicle_command(uint16_t command, float param1, float param2);

    bool isSelfCheckOK();
    bool isUAVTakeoff();
    bool isWaypointReached(const Waypoint & waypoint, const sensor_msgs::msg::JointState & state);
    std::optional<Waypoint> currentWaypoint();
    Waypoint currentOrHoverWaypoint();
    Waypoint waypointFromSetpoint(const px4_msgs::msg::TrajectorySetpoint & setpoint) const;
    bool hasFreshDistanceSensor();
    bool hasFreshVehicleLocalPosition();
    bool hasValidHomePosition();
    bool isFreshStateTime(const rclcpp::Time & stamp) const;

    void generateSearchAdjustWaypoints();
    void generateApproachWaypoints();
    void generateSampleAdjustWaypoints();
    bool handleUavHoldAdjust();
    bool generateHoldAdjustWaypoints();
    bool isHoldAdjustTargetUpdateNeeded(const Waypoint & target) const;
    std::optional<TimedHoldSetpoint> latestFreshHoldUavSetpoint();
    void generateRetreatWaypoints();
    Waypoint offsetBodyFrame(const Waypoint & base, double forward_m, double right_m) const;

    void publishOffboardState(ControlState state);
    void publishStatus(ControlState state);
    void statusPublishOnTimer();
    void handleControlCommand(const std_msgs::msg::String::SharedPtr msg);
    void handleParsedCommand(CommandType command_type, const std::string & source,
                             bool quiet_reject = false);
    void handleMainTaskStatus(const status_interfaces_pkg::msg::TaskFSM::SharedPtr msg);
    void handleRuckigState(const sensor_msgs::msg::JointState::SharedPtr msg);
    void handleVehicleLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void handleHomePosition(const px4_msgs::msg::HomePosition::SharedPtr msg);
    void handleDistanceSensor(const px4_msgs::msg::DistanceSensor::SharedPtr msg);
    void handleManualControlSetpoint(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg);
    void handleWholeBodyUavSetpoint(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);
    // void handleActuatorOutputs(const px4_msgs::msg::ActuatorOutputs::SharedPtr msg);
    void requestSwitchChoice(ControlState current_state, const std::vector<ControlState> & candidates,
                             const std::string & reason);
    void applyApprovedTransition(ControlState current_state, ControlState target_state,
                                 const std::string & reason);
    bool canAcceptTargetTransition(ControlState current_state, ControlState target_state) const;

    static Waypoint parseSingleWaypointParameter(const std::vector<double> & flat,
                                                 const Waypoint & fallback);
    static std::vector<Waypoint> parseWaypointParameter(const std::vector<double> & flat);
    static std::array<double, 3> parseVector3Parameter(const std::vector<double> & flat,
                                                       const std::array<double, 3> & fallback);
    static std::string stateToString(ControlState state);
    static int stateToId(ControlState state);
    static std::optional<ControlState> statusIdToState(uint8_t status);
    std::optional<ParsedCommand> parseCommand(const std::string & command) const;
    std::optional<ParsedCommand> commandFromMainTaskStatus(uint8_t status) const;
    static bool tokenMatches(const std::string & token, const std::vector<std::string> & aliases);
    static std::vector<std::string> upperCopyList(std::vector<std::string> values);
    static std::string upperCopy(std::string value);
    static double wrapAngle(double angle);

    void handleTrajCompleteFlag(const traj_offboard::msg::TrajCompleteFlag::SharedPtr msg);
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

    switch (current_state) {
        case ControlState::SELF_CHECK:        handleSelfCheck();        break;
        case ControlState::UAV_START:         handleUavStart();         break;
        case ControlState::TRANSIT_TO_AREA:   handleTransitToArea();    break;
        case ControlState::UAV_ARRIVED_AERA:  handleUavArrivedArea();   break;
        case ControlState::SEARCH_ADJUST_AUTO:   handleSearchAdjustAuto();   break;
        case ControlState::SEARCH_ADJUST_MANUAL: handleSearchAdjustManual(); break;
        case ControlState::APPROACH_PLANT:    handleApproachPlant();    break;
        case ControlState::UAV_PRE_HOLD:      handleUavPreHold();       break;
        case ControlState::SAMP_ADJUST_AUTO:  handleSampleAdjustAuto(); break;
        case ControlState::SAMP_ADJUST_MANUAL:handleSampleAdjustManual();break;
        case ControlState::UAV_HOLD:          handleUavHold();          break;
        case ControlState::RETREAT:           handleRetreat();          break;
        case ControlState::UAV_BACK_HOME:     handleBackHome();         break;
        case ControlState::UAV_TASK_TERM:     handleTaskTerm();         break;
    }

    previous_state_ = current_state;
}

// 状态进入回调：每次切换到新状态时清空旧目标，并初始化该状态需要的标志位、索引和航点序列。
void UavOffboardFsm::onStateEntry(ControlState state)
{
    clearActiveTarget();
    // 默认航点末端速度为 0、不覆盖 Ruckig 速度上限；APPROACH_PLANT 在 generateApproachWaypoints 中重置这两项。
    target_velocity_ = {0.0, 0.0, 0.0};
    target_max_velocity_xyz_ = {0.0, 0.0, 0.0};
    RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "FSM state -> %s" LOG_COLOR_RESET, stateToString(state).c_str());

    switch (state) {
        case ControlState::SELF_CHECK:
            resetMissionProgress();
            break;
        case ControlState::TRANSIT_TO_AREA:
            transit_index_ = 0;
            is_arrived_task_aera_ = false;
            break;
        case ControlState::UAV_ARRIVED_AERA:
            targ_got_confirm_pending_ = false;
            task_term_confirm_pending_ = false;
            break;
        case ControlState::SEARCH_ADJUST_AUTO:
            search_index_ = 0;
            adjust_completed_ = false;
            uav_search_succeed_ = false;
            targ_got_confirm_pending_ = false;
            task_term_confirm_pending_ = false;
            generateSearchAdjustWaypoints();
            break;
        case ControlState::SEARCH_ADJUST_MANUAL:
            adjust_completed_ = false;
            uav_search_succeed_ = false;
            targ_got_confirm_pending_ = false;
            task_term_confirm_pending_ = false;
            break;
        case ControlState::APPROACH_PLANT:
            approach_index_ = 0;
            approach_completed_ = false;
            uav_adjust_succeed_ = false;
            arm_config_prepared_ = false;
            sampl_opera_completed_ = false;
            targ_got_confirm_pending_ = false;
            generateApproachWaypoints();
            break;
        case ControlState::UAV_PRE_HOLD:
            task_term_confirm_pending_ = false;
            break;
        case ControlState::SAMP_ADJUST_AUTO:
            sample_adjust_index_ = 0;
            uav_adjust_succeed_ = false;
            arm_config_prepared_ = false;
            sampl_opera_completed_ = false;
            task_term_confirm_pending_ = false;
            generateSampleAdjustWaypoints();
            break;
        case ControlState::SAMP_ADJUST_MANUAL:
            uav_adjust_succeed_ = false;
            arm_config_prepared_ = false;
            sampl_opera_completed_ = false;
            task_term_confirm_pending_ = false;
            break;
        case ControlState::UAV_HOLD:
            arm_config_prepared_ = false;
            sampl_opera_completed_ = false;
            task_term_confirm_pending_ = false;
            hold_adjust_started_ = false;
            hold_adjust_stale_hold_sent_ = false;
            hold_adjust_desired_local_.reset();
            hold_adjust_last_planned_pos_des_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            hold_adjust_last_target_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            hold_adjust_waypoints_.clear();
            hold_adjust_index_ = 0;
            break;
        case ControlState::RETREAT:
            retreat_index_ = 0;
            uav_ready_for_back_ = false;
            task_term_confirm_pending_ = false;
            generateRetreatWaypoints();
            break;
        case ControlState::UAV_BACK_HOME:
            back_home_index_ = 0;
            back_home_ = false;
            home_waypoint_ = waypointFromSetpoint(traj_complete_flag_.traj_first_setpoint);
            {
                const auto current = currentOrHoverWaypoint();
                const double cruise_z = current.z + 5.0;
                const double dx = home_waypoint_.x - current.x;
                const double dy = home_waypoint_.y - current.y;
                const double horiz_dist = std::hypot(dx, dy);
                const double heading_yaw = (horiz_dist > position_tolerance_)
                                               ? wrapAngle(std::atan2(dy, dx))
                                               : current.yaw;

                const Waypoint lifted{current.x, current.y, cruise_z, current.yaw};
                const Waypoint cruise_yaw{current.x, current.y, cruise_z, heading_yaw};
                const Waypoint cruise{home_waypoint_.x, home_waypoint_.y, cruise_z, heading_yaw};
                const Waypoint final_yaw_above{home_waypoint_.x, home_waypoint_.y, cruise_z, home_waypoint_.yaw};
                back_home_waypoints_ = {lifted, cruise_yaw, cruise, final_yaw_above, home_waypoint_};
            }
            break;
        case ControlState::UAV_START:
        case ControlState::UAV_TASK_TERM:
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

// 任务进度复位：从自检重新开始时清空所有流程图标志，避免上一次任务状态影响新任务。
void UavOffboardFsm::resetMissionProgress()
{
    ready_for_takeoff_ = false;
    ready_for_transit_ = false;
    is_arrived_task_aera_ = false;
    adjust_completed_ = false;
    uav_search_succeed_ = false;
    approach_completed_ = false;
    uav_adjust_succeed_ = false;
    arm_config_prepared_ = false;
    sampl_opera_completed_ = false;
    uav_ready_for_back_ = false;
    back_home_ = false;
    targ_got_confirm_pending_ = false;
    task_term_confirm_pending_ = false;
    switch_status_request_pending_ = false;
    transit_end_waypoint_valid_ = false;
    hold_adjust_started_ = false;
    hold_adjust_stale_hold_sent_ = false;
    hold_adjust_desired_local_.reset();
    hold_adjust_last_planned_pos_des_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    hold_adjust_last_target_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    hold_adjust_waypoints_.clear();
    hold_adjust_index_ = 0;
}

// 自检状态处理：等待 SELF_CHECK 指令，检查总任务使能和可选测距通信，通过后置 uavCheckSucceed=1。
void UavOffboardFsm::handleSelfCheck()
{
    if (!self_check_requested_ && !ready_for_takeoff_) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                              "SELF_CHECK | waiting for command SELF_CHECK");
        return;
    }

    if (ready_for_takeoff_) {
        RCLCPP_DEBUG_ONCE(get_logger(),
                          "SELF_CHECK | uavCheckSucceed=1; waiting for command WAIT_TASK_ENABLE_AUTH");
        return;
    }

    if (!isSelfCheckOK()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "SELF_CHECK | pending distance_sensor_ok=%s vehicle_local_position_ok=%s home_position_ok=%s",
                             hasFreshDistanceSensor() ? "true" : "false",
                             hasFreshVehicleLocalPosition() ? "true" : "false",
                             hasValidHomePosition() ? "true" : "false");
        return;
    }

    ready_for_takeoff_ = true;
    self_check_requested_ = false;
    RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "SELF_CHECK complete | uavCheckSucceed=1 waiting WAIT_TASK_ENABLE_AUTH" LOG_COLOR_RESET);
}

// 起飞状态处理：由 WAIT_TASK_ENABLE_AUTH 授权进入，等待无人机到达起飞高度并置 uavTakeoffSucceed=1。
void UavOffboardFsm::handleUavStart()
{
    if (!ready_for_takeoff_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "UAV_START blocked | self-check is not complete");
        transitionTo(ControlState::SELF_CHECK);
        return;
    }

    if (ready_for_transit_) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), takeoff_wait_log_throttle_ms_,
                              "UAV_START | uavTakeoffSucceed=1; waiting NAV_TO_TASK_DOM");
        return;
    }

    if (!traj_complete_flag_.take_off_completed) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), takeoff_wait_log_throttle_ms_,
                              "UAV_START | waiting for vehicle at target=(%.2f, %.2f, %.2f, yaw %.2f)",
                              takeoff_waypoint_.x, takeoff_waypoint_.y,
                              takeoff_waypoint_.z, takeoff_waypoint_.yaw);
        return;
    }

    ready_for_transit_ = true;
    RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "UAV_START complete | uavTakeoffSucceed=1 waiting NAV_TO_TASK_DOM" LOG_COLOR_RESET);
}

// 到达任务区保持状态：无人机悬停并等待搜索、目标已获取或任务终止等上层状态指令。
void UavOffboardFsm::handleUavArrivedArea()
{
    if (targ_got_confirm_pending_) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                              "UAV_ARRIVED_AERA | TARG_GOT received; waiting service approval for APPROACH_PLANT");
        return;
    }
    if (task_term_confirm_pending_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                             "UAV_ARRIVED_AERA | NO received; waiting keyboard CONFIRM to execute UAV_TASK_TERM");
        return;
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                          "UAV_ARRIVED_AERA | waiting UAV_SEARCH_TARGS, TARG_GOT or TASK_TERM");
}

// 前往任务区域处理：按 transit_waypoints 参数中的航点顺序飞行，全部到达后标记已到达任务区。
void UavOffboardFsm::handleTransitToArea()
{
    if (!ready_for_transit_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "TRANSIT_TO_AREA blocked | UAV_START is not complete");
        transitionTo(ControlState::UAV_START);
        return;
    }

    if (traj_complete_flag_.trajectory_completed) {
        is_arrived_task_aera_ = true;
        RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "TRANSIT_TO_AREA complete | uavArrivedTaskAera=1" LOG_COLOR_RESET);
        transitionTo(ControlState::UAV_ARRIVED_AERA);
    }
}

// 自动搜索处理：到达任务区后执行偏航/横移搜索；锁定目标必须由 TARG_GOT 经 SwitchStatus 批准。
void UavOffboardFsm::handleSearchAdjustAuto()
{
    if (!is_arrived_task_aera_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "SEARCH_ADJUST_AUTO blocked | task area has not been reached");
        transitionTo(ControlState::UAV_ARRIVED_AERA);
        return;
    }

    if (targ_got_confirm_pending_) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                              "SEARCH_ADJUST_AUTO | TARG_GOT received; waiting SwitchStatus service response for APPROACH_PLANT");
        return;
    }
    if (task_term_confirm_pending_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                             "SEARCH_ADJUST_AUTO | NO received; waiting keyboard CONFIRM to execute UAV_TASK_TERM");
        return;
    }

    if (handleWaypointSequence(search_waypoints_, search_index_, "search adjust auto")) {
        adjust_completed_ = true;
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                              "SEARCH_ADJUST_AUTO complete | uavSearchSucceed=0 waiting TARG_GOT or TASK_TERM");
    }
}

// 手动搜索处理：保持悬停，等待 TARG_GOT 经 SwitchStatus 批准，或 NO + CONFIRM 终止。
void UavOffboardFsm::handleSearchAdjustManual()
{
    if (!is_arrived_task_aera_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "SEARCH_ADJUST_MANUAL blocked | task area has not been reached");
        transitionTo(ControlState::UAV_ARRIVED_AERA);
        return;
    }

    if (targ_got_confirm_pending_) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                              "SEARCH_ADJUST_MANUAL | TARG_GOT received; waiting SwitchStatus service response for APPROACH_PLANT");
        return;
    }
    if (task_term_confirm_pending_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                             "SEARCH_ADJUST_MANUAL | NO received; waiting keyboard CONFIRM to execute UAV_TASK_TERM");
        return;
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                          "SEARCH_ADJUST_MANUAL | waiting for TARG_GOT or TASK_TERM");
}

// 靠近植株处理：根据相机/测距语义缓慢抵近机械臂作业范围，成功后进入 UAV_PRE_HOLD 等待 TARG_READY。
void UavOffboardFsm::handleApproachPlant()
{
    if (!uav_search_succeed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "APPROACH_PLANT blocked | target has not been confirmed");
        transitionTo(ControlState::UAV_ARRIVED_AERA);
        return;
    }

    if (latest_distance_m_ &&
        *latest_distance_m_ <= approach_target_distance_m_ + approach_distance_tolerance_m_) {
        approach_completed_ = true;
        clearActiveTarget();
        RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "APPROACH_PLANT complete | uavApproachSucceed=1 TARG_READY source=distance_sensor" LOG_COLOR_RESET);
        transitionTo(ControlState::UAV_PRE_HOLD);
        return;
    }

    if (handleWaypointSequence(approach_waypoints_, approach_index_, "approach")) {
        approach_completed_ = true;
        RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "APPROACH_PLANT complete | uavApproachSucceed=1 TARG_READY source=target_arrival" LOG_COLOR_RESET);
        transitionTo(ControlState::UAV_PRE_HOLD);
    }
}

// 接近完成保持状态：无人机保持在植株附近，等待总状态机下发 UAV_POSE_ADAP 并选择采样微调模式。
// 进入时不下发新的 set_target：APPROACH 段生成的航点已经以末端速度 0 收尾，
// 让 Ruckig 沿用 bridge 中已缓存的 APPROACH 目标自然完成轨迹（与 RETREAT 行为一致），
// 避免"target_pos=当前参考位置 + target_vel=0"导致的前冲-回拉 overshoot 与小位置变化。
void UavOffboardFsm::handleUavPreHold()
{
    if (!approach_completed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "UAV_PRE_HOLD blocked | approach is not complete");
        transitionTo(ControlState::UAV_ARRIVED_AERA);
        return;
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                          "UAV_PRE_HOLD | holding APPROACH endpoint; waiting UAV_POSE_ADAP or SAMP_ADJUST_AUTO/MANUAL");
}

// 自动采样微调处理：TARG_READY 后执行姿态/位置微调，成功后等待 ARM_CONFIG_PREP + SAMPL_OPERA。
void UavOffboardFsm::handleSampleAdjustAuto()
{
    if (!approach_completed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "SAMP_ADJUST_AUTO blocked | approach is not complete");
        transitionTo(ControlState::UAV_PRE_HOLD);
        return;
    }

    if (task_term_confirm_pending_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                             "SAMP_ADJUST_AUTO | NO received; waiting keyboard CONFIRM to execute UAV_TASK_TERM");
        return;
    }

    if (handleWaypointSequence(sample_adjust_waypoints_, sample_adjust_index_, "sample adjust auto")) {
        uav_adjust_succeed_ = true;
        RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "SAMP_ADJUST_AUTO complete | uavAdjustSucceed=1 waiting ARM_CONFIG_PREP" LOG_COLOR_RESET);
        transitionTo(ControlState::UAV_HOLD);
    }
}

// 手动采样微调处理：保持悬停，CONFIRM 表示微调成功，NO + CONFIRM 表示人工授权终止。
void UavOffboardFsm::handleSampleAdjustManual()
{
    if (!approach_completed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "SAMP_ADJUST_MANUAL blocked | approach is not complete");
        transitionTo(ControlState::UAV_PRE_HOLD);
        return;
    }

    if (task_term_confirm_pending_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                             "SAMP_ADJUST_MANUAL | NO received; waiting keyboard CONFIRM to execute UAV_TASK_TERM");
        return;
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                          "SAMP_ADJUST_MANUAL | waiting keyboard CONFIRM for uavAdjustSucceed=1 or NO");
}

// 采样保持状态：微调完成后等待机械臂准备和采样完成指令，再允许进入预返航流程。
void UavOffboardFsm::handleUavHold()
{
    if (!uav_adjust_succeed_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "UAV_HOLD blocked | sample adjust is not complete");
        transitionTo(ControlState::UAV_PRE_HOLD);
        return;
    }

    if (!handleUavHoldAdjust()) {
        return;
    }

    if (!arm_config_prepared_) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                              "UAV_HOLD | waiting ARM_CONFIG_PREP");
        return;
    }
    if (!sampl_opera_completed_) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                              "UAV_HOLD | waiting SAMPL_OPERA");
        return;
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                          "UAV_HOLD | sampling complete; waiting UAV_PRE_BACK_HOME");
}

// UAV_HOLD 内 UAV 期望位置实时调整：xy/z 可独立启用。
// 以 FSM 控制循环频率消费 whole_body_planner 最新 setpoint；若 set_target 服务仍 pending，
// 跳过当前周期，下一周期继续使用缓存的最新 setpoint。
bool UavOffboardFsm::handleUavHoldAdjust()
{
    if (!use_xy_adjust_ && !use_z_adjust_) {
        return true;
    }

    const auto desired_setpoint = latestFreshHoldUavSetpoint();
    if (!desired_setpoint) {
        if (!hold_adjust_started_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                                 "UAV_HOLD adjust pending | waiting fresh whole_body_planner uav setpoint");
            return false;
        }

        if (!target_request_pending_ &&
            (!hold_adjust_stale_hold_sent_ || !active_target_sent_)) {
            const auto hold_target = currentOrHoverWaypoint();
            target_max_velocity_xyz_ = hold_adjust_max_velocity_xyz_;
            target_velocity_ = {0.0, 0.0, 0.0};
            target_acceleration_ = {0.0, 0.0, 0.0};
            setActiveTarget(hold_target);
            hold_adjust_stale_hold_sent_ = sendActiveTarget(true);
            if (hold_adjust_stale_hold_sent_) {
                RCLCPP_WARN(get_logger(),
                            "UAV_HOLD adjust stale | sent current-position hold target");
            } else {
                clearActiveTarget();
            }
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                             "UAV_HOLD adjust paused | whole_body_planner uav setpoint is stale");
        return true;
    }

    if (desired_setpoint->stamp.nanoseconds() <=
        hold_adjust_last_planned_pos_des_time_.nanoseconds()) {
        return true;
    }

    if (target_request_pending_) {
        return true;
    }

    const auto now_time = now();
    if (hold_adjust_min_update_period_s_ > 0.0 &&
        hold_adjust_last_target_update_time_.nanoseconds() != 0 &&
        (now_time - hold_adjust_last_target_update_time_).seconds() <
            hold_adjust_min_update_period_s_) {
        return true;
    }

    hold_adjust_desired_local_ = desired_setpoint->position;
    if (!generateHoldAdjustWaypoints()) {
        hold_adjust_last_planned_pos_des_time_ = desired_setpoint->stamp;
        return true;
    }
    if (hold_adjust_waypoints_.empty()) {
        hold_adjust_last_planned_pos_des_time_ = desired_setpoint->stamp;
        hold_adjust_started_ = true;
        hold_adjust_stale_hold_sent_ = false;
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                              "UAV_HOLD adjust | desired UAV position already reached on enabled axes");
        return true;
    }

    const auto target = hold_adjust_waypoints_.front();
    if (!isHoldAdjustTargetUpdateNeeded(target)) {
        hold_adjust_last_planned_pos_des_time_ = desired_setpoint->stamp;
        hold_adjust_started_ = true;
        hold_adjust_stale_hold_sent_ = false;
        return true;
    }

    target_max_velocity_xyz_ = hold_adjust_max_velocity_xyz_;
    target_velocity_ = desired_setpoint->velocity;
    target_acceleration_ = desired_setpoint->acceleration;
    if (!use_xy_adjust_) {
        target_velocity_[0] = 0.0;
        target_velocity_[1] = 0.0;
        target_acceleration_[0] = 0.0;
        target_acceleration_[1] = 0.0;
    }
    if (!use_z_adjust_) {
        target_velocity_[2] = 0.0;
        target_acceleration_[2] = 0.0;
    }
    setActiveTarget(target);
    if (!sendActiveTarget(true)) {
        clearActiveTarget();
        return true;
    }
    hold_adjust_last_planned_pos_des_time_ = desired_setpoint->stamp;
    hold_adjust_last_target_update_time_ = now_time;
    hold_adjust_started_ = true;
    hold_adjust_stale_hold_sent_ = false;

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                          "UAV_HOLD realtime target sent | desired_fsm=(%.3f, %.3f, %.3f)",
                          desired_setpoint->position[0], desired_setpoint->position[1],
                          desired_setpoint->position[2]);
    return true;
}

bool UavOffboardFsm::generateHoldAdjustWaypoints()
{
    hold_adjust_waypoints_.clear();
    hold_adjust_index_ = 0;
    if (!hold_adjust_desired_local_) {
        RCLCPP_WARN(get_logger(), "UAV_HOLD adjust skipped | no desired UAV setpoint");
        return false;
    }

    const auto base = currentOrHoverWaypoint();
    const auto desired = *hold_adjust_desired_local_;
    Waypoint target = base;
    if (use_xy_adjust_) {
        target.x = desired[0];
        target.y = desired[1];
    }
    if (use_z_adjust_) {
        target.z = desired[2];
    }
    target.yaw = base.yaw;

    bool clamped = false;
    if (use_xy_adjust_ && hold_adjust_max_xy_step_m_ > 0.0) {
        const double dx = target.x - base.x;
        const double dy = target.y - base.y;
        const double dist_xy = std::hypot(dx, dy);
        if (dist_xy > hold_adjust_max_xy_step_m_) {
            const double scale = hold_adjust_max_xy_step_m_ / dist_xy;
            target.x = base.x + dx * scale;
            target.y = base.y + dy * scale;
            clamped = true;
        }
    }
    if (use_z_adjust_ && hold_adjust_max_z_step_m_ > 0.0) {
        const double dz = target.z - base.z;
        const double clamped_dz =
            std::clamp(dz, -hold_adjust_max_z_step_m_, hold_adjust_max_z_step_m_);
        if (std::abs(clamped_dz - dz) > 1e-9) {
            target.z = base.z + clamped_dz;
            clamped = true;
        }
    }

    const double update_tol = std::max(0.0, hold_adjust_target_update_tolerance_m_);
    const bool need_position_adjust =
        (use_xy_adjust_ &&
         (std::abs(target.x - base.x) > update_tol ||
          std::abs(target.y - base.y) > update_tol)) ||
        (use_z_adjust_ && std::abs(target.z - base.z) > update_tol);
    if (need_position_adjust) {
        hold_adjust_waypoints_.push_back(target);
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                          "UAV_HOLD adjust plan | current=(%.3f, %.3f, %.3f, yaw %.3f) desired_fsm=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f, yaw %.3f) waypoints=%zu use_xy=%s use_z=%s clamped=%s",
                          base.x, base.y, base.z, base.yaw,
                          desired[0], desired[1], desired[2],
                          target.x, target.y, target.z, target.yaw,
                          hold_adjust_waypoints_.size(),
                          use_xy_adjust_ ? "true" : "false",
                          use_z_adjust_ ? "true" : "false",
                          clamped ? "true" : "false");
    if (clamped) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "UAV_HOLD adjust safety clamp active | max_xy_step=%.3f max_z_step=%.3f",
                             hold_adjust_max_xy_step_m_, hold_adjust_max_z_step_m_);
    }
    return true;
}

bool UavOffboardFsm::isHoldAdjustTargetUpdateNeeded(const Waypoint & target) const
{
    if (!active_target_) {
        return true;
    }

    const double update_tol = std::max(0.0, hold_adjust_target_update_tolerance_m_);
    return std::abs(target.x - active_target_->x) > update_tol ||
           std::abs(target.y - active_target_->y) > update_tol ||
           std::abs(target.z - active_target_->z) > update_tol ||
           std::abs(wrapAngle(target.yaw - active_target_->yaw)) > yaw_tolerance_;
}

std::optional<UavOffboardFsm::TimedHoldSetpoint> UavOffboardFsm::latestFreshHoldUavSetpoint()
{
    std::optional<Vector3> pos_copy;
    std::optional<Vector3> vel_copy;
    std::optional<Vector3> acc_copy;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    {
        std::lock_guard<std::mutex> lock(latest_state_mutex_);
        pos_copy = latest_hold_uav_setpoint_;
        vel_copy = latest_hold_uav_velocity_;
        acc_copy = latest_hold_uav_acceleration_;
        stamp = last_hold_pos_des_time_;
    }
    if (!pos_copy || stamp.nanoseconds() == 0 ||
        (now() - stamp).seconds() > hold_adjust_pos_des_timeout_s_) {
        return std::nullopt;
    }
    return TimedHoldSetpoint{
        *pos_copy,
        vel_copy.value_or(Vector3{0.0, 0.0, 0.0}),
        acc_copy.value_or(Vector3{0.0, 0.0, 0.0}),
        stamp};
}

// 后退处理：APPROACH 的逆过程。沿机体系后方慢速退回安全距离，完成后允许执行 BACK_HOME。
// 优先使用测距传感器：读数达到 retreat_target_distance_m_ 即视为完成；否则走完 retreat_waypoints_ 中的航点。
void UavOffboardFsm::handleRetreat()
{
    if (!ready_for_transit_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "RETREAT blocked | UAV_START is not complete");
        transitionTo(ControlState::UAV_TASK_TERM);
        return;
    }

    if (latest_distance_m_ &&
        *latest_distance_m_ >= retreat_target_distance_m_ - approach_distance_tolerance_m_) {
        if (!uav_ready_for_back_) {
            uav_ready_for_back_ = true;
            clearActiveTarget();
            RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "RETREAT complete | uavReadyForBack=1 source=distance_sensor latest=%.2fm" LOG_COLOR_RESET,
                        *latest_distance_m_);
        }
        return;
    }

    if (handleWaypointSequence(retreat_waypoints_, retreat_index_, "retreat")) {
        if (!uav_ready_for_back_) {
            uav_ready_for_back_ = true;
            RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "RETREAT complete | uavReadyForBack=1 source=target_arrival" LOG_COLOR_RESET);
        }
    }
}

// 返航处理：按 home_waypoint 参数回到本地坐标系中的返航点，完成后设置返航完成标志。
void UavOffboardFsm::handleBackHome()
{
    if (!ready_for_transit_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "BACK_HOME blocked | UAV_START is not complete");
        transitionTo(ControlState::UAV_TASK_TERM);
        return;
    }

    if (handleWaypointSequence(back_home_waypoints_, back_home_index_, "back home")) {
        if (!back_home_) {
            back_home_ = true;
            RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "UAV_BACK_HOME complete | back_home=true" LOG_COLOR_RESET);
        }
    }
}

// 任务终止处理：保持悬停，等待人工选择预返航后退或直接返航。
void UavOffboardFsm::handleTaskTerm()
{
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                         "UAV_TASK_TERM | hovering; waiting for UAV_PRE_BACK_HOME or BACK_HOME");
}

void UavOffboardFsm::publish_vehicle_command(uint16_t command, float param1, float param2)
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
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    if (vehicle_command_publisher_->get_subscription_count() > 0) {
        vehicle_command_publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(),
                   "Published vehicle_command - command: %d, param1: %.2f, param2: %.2f",
                    command, param1, param2);
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), log_throttle_ms_,
                             "No subscribers for /fmu/in/vehicle_command");
    }
}

void UavOffboardFsm::handleActuatorControl(
    const std::shared_ptr<status_interfaces_pkg::srv::ActuatorControl::Request> request,
    std::shared_ptr<status_interfaces_pkg::srv::ActuatorControl::Response> response)
{
    if (vehicle_command_publisher_->get_subscription_count() == 0) {
        RCLCPP_WARN(get_logger(), "ActuatorControl rejected | no FMU subscriber");
        response->success = false;
        return;
    }
    // 仅更新设定值，由 actuator_timer_ 以 5Hz 持续重发给 FMU；
    // close=0 时重置两个初始化周期，close=1 时取消尚未完成的初始化。
    actuator_close.store(request->close ? 1.0f : -1.0f);
    actuator_cut.store(request->cut ? 1.0f : -1.0f);
    actuator_close_init_cycles_remaining.store(request->close ? 0 : 0);
    response->success = true;
    RCLCPP_INFO(get_logger(),
                 LOG_COLOR_BLUE "ActuatorControl | close=%d cut=%d -> setpoint updated" LOG_COLOR_RESET,
                static_cast<int>(request->close), static_cast<int>(request->cut));
}

// 5hz timer 回调函数：持续发布当前的执行器命令，确保 FMU 能及时收到最新的指令状态。
void UavOffboardFsm::publishActuatorCommand()
{
    if (!actuator_command_enabled_.load()) {
        if (actuator_close_init_cycles_remaining.load() > 0) {
            publish_vehicle_command(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR,
                0.0f,
                -1.0f);
            actuator_close_init_cycles_remaining.fetch_sub(1);
            RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), log_throttle_ms_,
            "Actuator command blocked init| %s",
            manual_control_setpoint_received_.load()
                ? "manual_control_setpoint aux1 >= 0.0 or invalid"
                : "waiting for manual_control_setpoint aux1 < 0.0");
            return;
        }
        publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR,
        -1.0f,
        -1.0f);

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), log_throttle_ms_,
            "Actuator command blocked | %s",
            manual_control_setpoint_received_.load()
                ? "manual_control_setpoint aux1 >= 0.0 or invalid"
                : "waiting for manual_control_setpoint aux1 < 0.0");
        return;
    }

    const float close = actuator_close.load();
    const float cut = actuator_cut.load();
    if (close == -1.0f && actuator_close_init_cycles_remaining.load() > 0) {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR,
            0.0f,
            cut);
        actuator_close_init_cycles_remaining.fetch_sub(1);
        return;
    }

    publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR,
        close,
        cut);
}

// 航点序列处理：负责发送当前航点、等待到达、推进索引，并在序列全部完成时返回 true。
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
                    LOG_COLOR_BLUE "Waypoint dispatch | stage=%s index=%zu/%zu target=(%.2f, %.2f, %.2f, yaw %.2f)" LOG_COLOR_RESET,
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
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                              "Target active | waiting for fresh UAV position feedback");
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
bool UavOffboardFsm::sendActiveTarget(bool throttle_success_log)
{
    if (!active_target_) {
        return false;
    }
    if (!set_target_client_->service_is_ready()) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                              "Target pending | service not available: online_traj_generator/set_target");
        return false;
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
    request->max_velocity_xyz = {
        target_max_velocity_xyz_[0],
        target_max_velocity_xyz_[1],
        target_max_velocity_xyz_[2],
    };

    const auto target = *active_target_;
    target_request_pending_ = true;
    last_target_sent_time_ = now();
    set_target_client_->async_send_request(
        request,
        [this, target, throttle_success_log](rclcpp::Client<traj_offboard::srv::SetTarget>::SharedFuture resp_fut) {
            std::lock_guard<std::mutex> lock(fsm_mutex_);
            target_request_pending_ = false;
            try {
                const auto resp = resp_fut.get();
                if (resp->success) {
                    active_target_sent_ = true;
                    if (throttle_success_log) {
                        RCLCPP_INFO_THROTTLE(
                            get_logger(), *get_clock(), 1000,
                            LOG_COLOR_BLUE "Target accepted by bridge | service=online_traj_generator/set_target target=(%.2f, %.2f, %.2f, yaw %.2f)" LOG_COLOR_RESET,
                            target.x, target.y, target.z, target.yaw);
                    } else {
                        RCLCPP_INFO(get_logger(),
                                    LOG_COLOR_BLUE "Target accepted by bridge | service=online_traj_generator/set_target target=(%.2f, %.2f, %.2f, yaw %.2f)" LOG_COLOR_RESET,
                                    target.x, target.y, target.z, target.yaw);
                    }
                } else {
                    active_target_sent_ = false;
                    RCLCPP_ERROR(get_logger(), "Target rejected | service=online_traj_generator/set_target");
                }
            } catch (const std::exception & e) {
                active_target_sent_ = false;
                RCLCPP_ERROR(get_logger(), "Target service failed | error=%s", e.what());
            }
        });
    return true;
}

// 自检条件判断：必须有新鲜的本地位置反馈和有效的 PX4 home 原点；如果参数要求测距传感器，则还必须有新鲜测距数据。
bool UavOffboardFsm::isSelfCheckOK()
{
    return (!require_distance_sensor_ || hasFreshDistanceSensor()) &&
           hasFreshVehicleLocalPosition() && hasValidHomePosition() && traj_complete_flag_.offboard_mode_active;
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
                             "Position feedback invalid | expected=4 actual=%zu",
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

// 当前航点读取：只返回最新轨迹参考设定点(ruckig_state)，不使用真实 PX4 位置反馈，
// 以保证下一段轨迹从已下发设定点连续衔接(flatness)；参考点超时或不可用时返回空。
std::optional<UavOffboardFsm::Waypoint> UavOffboardFsm::currentWaypoint()
{
    std::optional<sensor_msgs::msg::JointState> state_copy;
    {
        std::lock_guard<std::mutex> lock(latest_state_mutex_);
        if (latest_reference_state_ && isFreshStateTime(last_reference_state_time_)) {
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

// 获取当前位置或悬停默认点：有新鲜参考设定点时返回它；否则回退到最近的里程碑点——
// 转场结束后用 traj_last_setpoint，起飞后/兜底用 traj_first_setpoint(takeoff_waypoint_)。
UavOffboardFsm::Waypoint UavOffboardFsm::currentOrHoverWaypoint()
{
    const auto current = currentWaypoint();
    if (current) {
        return *current;
    }
    if (transit_end_waypoint_valid_) {
        return transit_end_waypoint_;
    }
    return takeoff_waypoint_;
}

// 轨迹设定点转航点：取 px4 TrajectorySetpoint 的 position[0..2] 与 yaw 构造本状态机的 Waypoint。
UavOffboardFsm::Waypoint
UavOffboardFsm::waypointFromSetpoint(const px4_msgs::msg::TrajectorySetpoint & setpoint) const
{
    return Waypoint{
        setpoint.position[0],
        setpoint.position[1],
        setpoint.position[2],
        setpoint.yaw};
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

bool UavOffboardFsm::hasFreshVehicleLocalPosition()
{
    if (!latest_actual_state_.has_value()) {
        return false;
    }
    if (last_actual_state_time_.nanoseconds() == 0) {
        return false;
    }
    return (now() - last_actual_state_time_).seconds() <= vehicle_local_position_timeout_s_;
}

// Home 位置有效性判断：必须已收到 PX4 home，且其本地坐标 xyz 已设置(valid_lpos)并为有限值。
bool UavOffboardFsm::hasValidHomePosition()
{
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return home_position_valid_;
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

// 生成靠近航点：仅沿体系 x 轴慢速前进（y、z 速度为 0），由测距传感器判定到达 approach_target_distance_m。
// - 有新鲜测距：前进距离 = clamp(latest - target, 0, approach_max_travel_m_)；handleApproachPlant 内监测距离传感器以提前终止。
// - 无测距（仿真）：前进距离 = approach_max_travel_m_(默认 3m)，到达航点即视为完成。
// 同时把 target_velocity_ 设为体系 x 方向的 approach_speed_m_s_，体系 y、z 分量为 0。
void UavOffboardFsm::generateApproachWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    approach_waypoints_.clear();

    const bool sensor_ok = hasFreshDistanceSensor();
    double travel_distance = approach_max_travel_m_;
    if (sensor_ok) {
        travel_distance = std::clamp(*latest_distance_m_ - approach_target_distance_m_,
                                     0.0, approach_max_travel_m_);
        if (travel_distance <= approach_distance_tolerance_m_) {
            RCLCPP_INFO(get_logger(),
                        "Approach skip | distance_sensor=%.2fm already within target=%.2fm (tol=%.2fm)",
                        *latest_distance_m_, approach_target_distance_m_,
                        approach_distance_tolerance_m_);
            target_velocity_ = {0.0, 0.0, 0.0};
            return;
        }
        RCLCPP_INFO(get_logger(),
                    "Approach plan | source=distance_sensor latest=%.2fm target=%.2fm travel=%.2fm speed=%.2fm/s",
                    *latest_distance_m_, approach_target_distance_m_, travel_distance,
                    approach_speed_m_s_);
    } else {
        RCLCPP_INFO(get_logger(),
                    "Approach plan | source=simulation (no distance sensor) travel=%.2fm speed=%.2fm/s",
                    travel_distance, approach_speed_m_s_);
    }

    approach_waypoints_.push_back(offsetBodyFrame(base, travel_distance, 0.0));

    // 末端速度设为 0：1) 3m 是安全上限，到点必须能立刻刹车；2) Ruckig 要求 |target_v[i]| < max_v[i]，
    // 若末端速度等于上限会触发 ErrorInvalidInput；巡航段的 0.1 m/s 由 target_max_velocity_xyz_ 单独约束。
    target_velocity_ = {0.0, 0.0, 0.0};
    // Ruckig 平动速度 per-axis 上限压到 approach_speed_m_s_，控制巡航速度。
    target_max_velocity_xyz_ = {
        approach_speed_m_s_,
        approach_speed_m_s_,
        approach_speed_m_s_,
    };
}

// 生成采样微调航点：以当前位置为基准，按参数配置的机体系前后/左右/高度/偏航偏移生成单个微调目标。
void UavOffboardFsm::generateSampleAdjustWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    auto target = offsetBodyFrame(base, sample_adjust_forward_m_, sample_adjust_right_m_);
    target.z += sample_adjust_z_offset_m_;
    target.yaw = wrapAngle(base.yaw + sample_adjust_yaw_offset_rad_);

    sample_adjust_waypoints_.clear();
    sample_adjust_waypoints_.push_back(target);
}

// 生成后退航点：APPROACH 的逆过程。仅沿体系 -x 慢速后退（y、z 速度 0），由测距传感器判定退到 retreat_target_distance_m_。
// - 有新鲜测距：后退距离 = clamp(target - latest, 0, retreat_max_travel_m_)；handleRetreat 内监测距离传感器以提前终止。
// - 无测距（仿真）：后退距离 = retreat_max_travel_m_(默认 5m)，到达航点即视为完成。
// 末端速度设为 0；巡航速度通过 target_max_velocity_xyz_ = retreat_speed_m_s_ 控制（Ruckig per-axis 上限）。
void UavOffboardFsm::generateRetreatWaypoints()
{
    const auto base = currentOrHoverWaypoint();
    retreat_waypoints_.clear();

    const bool sensor_ok = hasFreshDistanceSensor();
    double travel_distance = retreat_max_travel_m_;
    if (sensor_ok) {
        travel_distance = std::clamp(retreat_target_distance_m_ - *latest_distance_m_,
                                     0.0, retreat_max_travel_m_);
        if (travel_distance <= approach_distance_tolerance_m_) {
            RCLCPP_INFO(get_logger(),
                        "Retreat skip | distance_sensor=%.2fm already beyond target=%.2fm (tol=%.2fm)",
                        *latest_distance_m_, retreat_target_distance_m_,
                        approach_distance_tolerance_m_);
            target_velocity_ = {0.0, 0.0, 0.0};
            target_max_velocity_xyz_ = {0.0, 0.0, 0.0};
            return;
        }
        RCLCPP_INFO(get_logger(),
                    "Retreat plan | source=distance_sensor latest=%.2fm target=%.2fm travel=%.2fm speed=%.2fm/s",
                    *latest_distance_m_, retreat_target_distance_m_, travel_distance,
                    retreat_speed_m_s_);
    } else {
        RCLCPP_INFO(get_logger(),
                    "Retreat plan | source=simulation (no distance sensor) travel=%.2fm speed=%.2fm/s",
                    travel_distance, retreat_speed_m_s_);
    }

    // 沿体系后向（-x）移动 travel_distance。
    retreat_waypoints_.push_back(offsetBodyFrame(base, -travel_distance, 0.0));

    // 末端速度 0（同 APPROACH，避免 |target_v|=max_v 触发 Ruckig ErrorInvalidInput）。
    target_velocity_ = {0.0, 0.0, 0.0};
    // Ruckig 平动速度 per-axis 上限压到 retreat_speed_m_s_，控制后退巡航速度。
    target_max_velocity_xyz_ = {
        retreat_speed_m_s_,
        retreat_speed_m_s_,
        retreat_speed_m_s_,
    };
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

// 发布状态机状态：/status 使用交互包中的索引消息。
void UavOffboardFsm::publishStatus(ControlState state)
{
        status_interfaces_pkg::msg::Status status_msg;
        status_msg.uav_control_state = static_cast<uint8_t>(stateToId(state));
        status_msg.uav_check_succeed     = ready_for_takeoff_;
        status_msg.uav_takeoff_succeed   = ready_for_transit_;
        status_msg.uav_arrived_task_aera = is_arrived_task_aera_;
        status_msg.uav_search_succeed    = uav_search_succeed_;
        status_msg.uav_approach_succeed  = approach_completed_;
        status_msg.uav_adjust_succeed    = uav_adjust_succeed_;
        status_msg.uav_ready_for_back    = uav_ready_for_back_;
        status_msg.uav_back_home_succeed = back_home_;
        status_pub_->publish(status_msg);
}

// 状态发布定时器：周期性向外发布 Status 和可读诊断文本。
void UavOffboardFsm::statusPublishOnTimer()
{
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    publishStatus(control_state_.load());
}

// 控制指令回调：键盘临时控制仍使用字符串，解析后交给统一的流程指令处理函数。
void UavOffboardFsm::handleControlCommand(const std_msgs::msg::String::SharedPtr msg)
{
        const auto parsed = parseCommand(msg->data);
        if (!parsed) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                                 "Command rejected | unknown=%s", msg->data.c_str());
            return;
        }
        handleParsedCommand(parsed->type, msg->data);
}

// 流程指令处理：统一处理键盘模拟命令和 main_task_state 状态命令，并按最终流程图执行转换。
// quiet_reject=true 时（来自 50Hz main_task_state 派发），拒绝消息只走 DEBUG，避免终端被刷屏。
void UavOffboardFsm::handleParsedCommand(CommandType command_type, const std::string & source,
                                         bool quiet_reject)
{
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    const auto state = control_state_.load();

#define REJECT_WARN(...) \
    do { if (quiet_reject) { RCLCPP_DEBUG(get_logger(), __VA_ARGS__); } \
         else { RCLCPP_WARN(get_logger(), __VA_ARGS__); } } while (0)

    if (command_type == CommandType::PRE_CHECK) {
        resetMissionProgress();
        self_check_requested_ = true;
        transitionTo(ControlState::SELF_CHECK);
        RCLCPP_INFO(get_logger(), "Command accepted | source=%s -> SELF_CHECK", source.c_str());
        return;
    }

    if (command_type == CommandType::CONFIRM) {
        if (task_term_confirm_pending_) {
            clearActiveTarget();
            targ_got_confirm_pending_ = false;
            task_term_confirm_pending_ = false;
            transitionTo(ControlState::UAV_TASK_TERM);
            RCLCPP_INFO(get_logger(), "Command accepted | NO + CONFIRM -> UAV_TASK_TERM");
            return;
        }

        if (state == ControlState::SAMP_ADJUST_MANUAL && approach_completed_) {
            uav_adjust_succeed_ = true;
            arm_config_prepared_ = false;
            sampl_opera_completed_ = false;
            transitionTo(ControlState::UAV_HOLD);
            RCLCPP_INFO(get_logger(), "Command accepted | SAMP_ADJUST_MANUAL CONFIRM uavAdjustSucceed=1");
            return;
        }

        REJECT_WARN( "Command rejected | CONFIRM has no pending action in state=%s",
                    stateToString(state).c_str());
        return;
    }

    if (command_type == CommandType::WAIT_TASK_ENABLE_AUTH) {
        if (state == ControlState::SELF_CHECK && ready_for_takeoff_) {
            if (direct_to_uav_hold_after_self_check_) {
                ready_for_transit_ = true;
                is_arrived_task_aera_ = true;
                adjust_completed_ = true;
                uav_search_succeed_ = true;
                approach_completed_ = true;
                uav_adjust_succeed_ = true;
                arm_config_prepared_ = false;
                sampl_opera_completed_ = false;
                targ_got_confirm_pending_ = false;
                task_term_confirm_pending_ = false;
                clearActiveTarget();
                transitionTo(ControlState::UAV_HOLD);
                RCLCPP_INFO(get_logger(),
                            LOG_COLOR_GREEN "Command accepted | WAIT_TASK_ENABLE_AUTH -> UAV_HOLD direct mode" LOG_COLOR_RESET);
            } else {
                transitionTo(ControlState::UAV_START);
                RCLCPP_INFO(get_logger(), "Command accepted | WAIT_TASK_ENABLE_AUTH -> UAV_START");
            }
        } else {
            REJECT_WARN(
                        "Command rejected | WAIT_TASK_ENABLE_AUTH current=%s uavCheckSucceed=%s",
                        stateToString(state).c_str(), ready_for_takeoff_ ? "true" : "false");
        }
        return;
    }

    switch (command_type) {
        case CommandType::NAV_TO_TASK_DOM:
            if (state != ControlState::UAV_START || !ready_for_transit_) {
                REJECT_WARN(
                            "Command rejected | NAV_TO_TASK_DOM current=%s uavTakeoffSucceed=%s",
                            stateToString(state).c_str(), ready_for_transit_ ? "true" : "false");
                return;
            }
            transitionTo(ControlState::TRANSIT_TO_AREA);
            RCLCPP_INFO(get_logger(), "Command accepted | NAV_TO_TASK_DOM -> TRANSIT_TO_AREA");
            break;
        case CommandType::ARRIVE_TASK_DOM:
            if (state == ControlState::TRANSIT_TO_AREA && is_arrived_task_aera_) {
                transitionTo(ControlState::UAV_ARRIVED_AERA);
                RCLCPP_INFO(get_logger(), "Command accepted | ARRIVE_TASK_DOM -> UAV_ARRIVED_AERA");
                return;
            }
            REJECT_WARN(
                        "Command rejected | ARRIVE_TASK_DOM current=%s uavArrivedTaskAera=%s",
                        stateToString(state).c_str(), is_arrived_task_aera_ ? "true" : "false");
            break;
        case CommandType::UAV_SEARCH_TARGS:
            if (state != ControlState::UAV_ARRIVED_AERA || !is_arrived_task_aera_) {
                REJECT_WARN(
                            "Command rejected | UAV_SEARCH_TARGS current=%s uavArrivedTaskAera=%s",
                            stateToString(state).c_str(), is_arrived_task_aera_ ? "true" : "false");
                return;
            }
            requestSwitchChoice(state,
                                {ControlState::SEARCH_ADJUST_AUTO,
                                    ControlState::SEARCH_ADJUST_MANUAL},
                                    "UAV_SEARCH_TARGS");
            break;
        case CommandType::SEARCH_ADJUST_AUTO:
            if (state != ControlState::UAV_ARRIVED_AERA || !is_arrived_task_aera_ || approach_completed_) {
                REJECT_WARN(
                            "Command rejected | SEARCH_ADJUST_AUTO current=%s uavArrivedTaskAera=%s TARG_READY=%s",
                            stateToString(state).c_str(), is_arrived_task_aera_ ? "true" : "false",
                        approach_completed_ ? "true" : "false");
                return;
            }
            targ_got_confirm_pending_ = false;
            task_term_confirm_pending_ = false;
            transitionTo(ControlState::SEARCH_ADJUST_AUTO);
            RCLCPP_INFO(get_logger(), "Command accepted | SEARCH_ADJUST_AUTO");
            break;
        case CommandType::SEARCH_ADJUST_MANUAL:
            if (state != ControlState::UAV_ARRIVED_AERA || !is_arrived_task_aera_ || approach_completed_) {
                REJECT_WARN(
                            "Command rejected | SEARCH_ADJUST_MANUAL current=%s uavArrivedTaskAera=%s TARG_READY=%s",
                            stateToString(state).c_str(), is_arrived_task_aera_ ? "true" : "false",
                        approach_completed_ ? "true" : "false");
            return;
            }
            targ_got_confirm_pending_ = false;
            task_term_confirm_pending_ = false;
            transitionTo(ControlState::SEARCH_ADJUST_MANUAL);
            RCLCPP_INFO(get_logger(), "Command accepted | SEARCH_ADJUST_MANUAL");
            break;
        case CommandType::TARG_GOT:
            if (!is_arrived_task_aera_ ||
                !(state == ControlState::UAV_ARRIVED_AERA ||
                    state == ControlState::SEARCH_ADJUST_AUTO ||
                    state == ControlState::SEARCH_ADJUST_MANUAL)) {
                REJECT_WARN(
                            "Command rejected | TARG_GOT current=%s uavArrivedTaskAera=%s",
                        stateToString(state).c_str(), is_arrived_task_aera_ ? "true" : "false");
                return;
            }
            if (approach_completed_) {
                REJECT_WARN(
                            "Command rejected | TARG_GOT current=%s TARG_READY=true",
                            stateToString(state).c_str());
                return;
            }
            clearActiveTarget();
            task_term_confirm_pending_ = false;
            requestSwitchChoice(state, {ControlState::APPROACH_PLANT}, "TARG_GOT");
            RCLCPP_INFO(get_logger(), "Command accepted | TARG_GOT -> APPROACH_PLANT, waiting for approval");
            break;
        case CommandType::TARG_READY:
            if (state == ControlState::APPROACH_PLANT && approach_completed_) {
                transitionTo(ControlState::UAV_PRE_HOLD);
                RCLCPP_INFO(get_logger(), "Command accepted | TARG_READY -> UAV_PRE_HOLD");
                return;
            }
            if (state == ControlState::UAV_PRE_HOLD && approach_completed_) {
                RCLCPP_INFO(get_logger(), "Command accepted | TARG_READY already in UAV_PRE_HOLD");
                return;
            }
            REJECT_WARN(
                        "Command rejected | TARG_READY current=%s uavApproachSucceed=%s",
                        stateToString(state).c_str(), approach_completed_ ? "true" : "false");
            break;
        case CommandType::UAV_POSE_ADAP:
            if (state != ControlState::UAV_PRE_HOLD || !approach_completed_) {
                REJECT_WARN(
                            "Command rejected | UAV_POSE_ADAP current=%s uavApproachSucceed=%s",
                            stateToString(state).c_str(), approach_completed_ ? "true" : "false");
                return;
            }
            requestSwitchChoice(state,
                                {ControlState::SAMP_ADJUST_AUTO,
                                    ControlState::SAMP_ADJUST_MANUAL},
                                "UAV_POSE_ADAP");
            break;
        case CommandType::SAMP_ADJUST_AUTO:
            if (state != ControlState::UAV_PRE_HOLD || !approach_completed_) {
                REJECT_WARN(
                            "Command rejected | SAMP_ADJUST_AUTO current=%s uavApproachSucceed=%s",
                            stateToString(state).c_str(), approach_completed_ ? "true" : "false");
            return;
            }
            task_term_confirm_pending_ = false;
            transitionTo(ControlState::SAMP_ADJUST_AUTO);
            RCLCPP_INFO(get_logger(), "Command accepted | SAMP_ADJUST_AUTO");
            break;
        case CommandType::SAMP_ADJUST_MANUAL:
            if (state != ControlState::UAV_PRE_HOLD || !approach_completed_) {
                REJECT_WARN(
                            "Command rejected | SAMP_ADJUST_MANUAL current=%s uavApproachSucceed=%s",
                            stateToString(state).c_str(), approach_completed_ ? "true" : "false");
                return;
            }
            task_term_confirm_pending_ = false;
            transitionTo(ControlState::SAMP_ADJUST_MANUAL);
            RCLCPP_INFO(get_logger(), "Command accepted | SAMP_ADJUST_MANUAL");
            break;
        case CommandType::ARM_CONFIG_PREP:
            if (state == ControlState::UAV_PRE_HOLD && approach_completed_) {
                uav_adjust_succeed_ = true;
                arm_config_prepared_ = true;
                transitionTo(ControlState::UAV_HOLD);
                RCLCPP_DEBUG_ONCE(get_logger(),
                                  LOG_COLOR_GREEN "Command accepted | ARM_CONFIG_PREP -> UAV_HOLD" LOG_COLOR_RESET);
                return;
            }
            if (state == ControlState::UAV_HOLD && uav_adjust_succeed_) {
                arm_config_prepared_ = true;
                RCLCPP_DEBUG_ONCE(get_logger(),
                                  LOG_COLOR_GREEN "Command accepted | ARM_CONFIG_PREP waiting SAMPL_OPERA" LOG_COLOR_RESET);
                return;
            }
            REJECT_WARN(
                        "Command rejected | ARM_CONFIG_PREP current=%s uavAdjustSucceed=%s",
                        stateToString(state).c_str(), uav_adjust_succeed_ ? "true" : "false");
        break;
        case CommandType::SAMPL_OPERA:
            if (state == ControlState::UAV_HOLD && uav_adjust_succeed_ && arm_config_prepared_) {
                if (!sampl_opera_completed_) {
                    sampl_opera_completed_ = true;
                }
                RCLCPP_DEBUG(get_logger(), "Command accepted | SAMPL_OPERA waiting UAV_PRE_BACK_HOME");
                return;
            }
            REJECT_WARN(
                        "Command rejected | SAMPL_OPERA current=%s uavAdjustSucceed=%s ARM_CONFIG_PREP=%s",
                        stateToString(state).c_str(), uav_adjust_succeed_ ? "true" : "false",
                        arm_config_prepared_ ? "true" : "false");
        break;
        case CommandType::UAV_PRE_BACK_HOME:
            if (state == ControlState::UAV_TASK_TERM) {
                transitionTo(ControlState::RETREAT);
                RCLCPP_INFO(get_logger(), "Command accepted | UAV_PRE_BACK_HOME from UAV_TASK_TERM");
                return;
            }
            if (state != ControlState::UAV_HOLD || !ready_for_transit_ ||
                !uav_adjust_succeed_ || !arm_config_prepared_ || !sampl_opera_completed_) {
                REJECT_WARN(
                            "Command rejected | UAV_PRE_BACK_HOME current=%s uavTakeoffSucceed=%s uavAdjustSucceed=%s ARM_CONFIG_PREP=%s SAMPL_OPERA=%s",
                        stateToString(state).c_str(), ready_for_transit_ ? "true" : "false",
                        uav_adjust_succeed_ ? "true" : "false",
                        arm_config_prepared_ ? "true" : "false",
                        sampl_opera_completed_ ? "true" : "false");
                return;
            }
            transitionTo(ControlState::RETREAT);
            RCLCPP_INFO(get_logger(), "Command accepted | UAV_PRE_BACK_HOME -> RETREAT");
        break;
        case CommandType::BACK_HOME:
            if (state == ControlState::UAV_TASK_TERM) {
                transitionTo(ControlState::UAV_BACK_HOME);
                RCLCPP_INFO(get_logger(), "Command accepted | BACK_HOME from UAV_TASK_TERM");
                return;
            }
            if (state != ControlState::RETREAT || !uav_ready_for_back_) {
                REJECT_WARN(
                            "Command rejected | BACK_HOME current=%s uavReadyForBack=%s",
                            stateToString(state).c_str(), uav_ready_for_back_ ? "true" : "false");
                return;
            }
            transitionTo(ControlState::UAV_BACK_HOME);
            RCLCPP_INFO(get_logger(), "Command accepted | BACK_HOME");
            break;
        case CommandType::TASK_TERM:
            clearActiveTarget();
            targ_got_confirm_pending_ = false;
            task_term_confirm_pending_ = false;
            transitionTo(ControlState::UAV_TASK_TERM);
            RCLCPP_INFO(get_logger(), "Command accepted | TASK_TERM -> UAV_TASK_TERM");
            break;
        case CommandType::NO: 
        {
            const bool search_failure_context =
                state == ControlState::SEARCH_ADJUST_AUTO ||
                state == ControlState::SEARCH_ADJUST_MANUAL ||
                (state == ControlState::UAV_ARRIVED_AERA && is_arrived_task_aera_ &&
                    adjust_completed_ && !uav_search_succeed_ && !approach_completed_);
            const bool sample_failure_context =
                state == ControlState::SAMP_ADJUST_AUTO ||
                state == ControlState::SAMP_ADJUST_MANUAL ||
                (state == ControlState::UAV_PRE_HOLD && approach_completed_ && !uav_adjust_succeed_);

            if (!search_failure_context && !sample_failure_context) {
                REJECT_WARN(
                            "Command rejected | NO current=%s search_failure_context=%s sample_failure_context=%s",
                            stateToString(state).c_str(),
                            search_failure_context ? "true" : "false",
                            sample_failure_context ? "true" : "false");
                return;
            }
            if (search_failure_context) {
                adjust_completed_ = false;
                uav_search_succeed_ = false;
            }
            if (sample_failure_context) {
                uav_adjust_succeed_ = false;
                arm_config_prepared_ = false;
                sampl_opera_completed_ = false;
            }
            clearActiveTarget();
            targ_got_confirm_pending_ = false;
            task_term_confirm_pending_ = true;
            RCLCPP_INFO(get_logger(), "Command accepted | NO waiting CONFIRM for UAV_TASK_TERM");
        }
        break;
        case CommandType::CONFIRM:
        case CommandType::PRE_CHECK:
        case CommandType::WAIT_TASK_ENABLE_AUTH:
        break;
    }

#undef REJECT_WARN
}

// 主状态机状态回调：订阅 status_interfaces_pkg/Status，将 main_task_state 索引映射为无人机流程指令。
void UavOffboardFsm::handleMainTaskStatus(const status_interfaces_pkg::msg::TaskFSM::SharedPtr msg)
{
        const auto now_ns = now().nanoseconds();
        const int status_value = static_cast<int>(msg->main_task_state);
        const int previous = last_main_task_status_.load();
        const int64_t last_dispatch_ns = last_main_task_dispatch_ns_.load();
        const int64_t repeat_period_ns =
            static_cast<int64_t>(main_task_repeat_dispatch_period_ms_) * 1000000LL;
        // if main_task_state 没有变化，且距离上次派发未超过设定的重复派发周期，则认为是重复消息，忽略不处理。
        if (previous == status_value && now_ns - last_dispatch_ns < repeat_period_ns) {
            return;
        }
        last_main_task_status_.store(status_value);
        last_main_task_dispatch_ns_.store(now_ns);

        const auto parsed = commandFromMainTaskStatus(msg->main_task_state);
        if (!parsed) {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                                  "Main task state ignored | status=%u", msg->main_task_state);
            return;
        }

        std::ostringstream source;
        // 构造消息源字符串，包含 main_task_state 的整数值
        source << "main_task_state=" << static_cast<int>(msg->main_task_state);
        // 50Hz 派发，拒绝消息走 DEBUG 避免刷屏；键盘路径仍保持 WARN。
        handleParsedCommand(parsed->type, source.str(), /*quiet_reject=*/true);
}

// 请求式切换：向地面站/人工选择端发送可选目标状态；单候选时表示请求批准该固定目标。
void UavOffboardFsm::requestSwitchChoice(ControlState current_state,
                                          const std::vector<ControlState> & candidates,
                                          const std::string & reason)
{
    if (candidates.empty()) {
        RCLCPP_WARN(get_logger(), "SwitchStatus skipped | empty candidate list reason=%s",
                    reason.c_str());
        return;
    }

    // if (switch_status_request_pending_) {
    //     RCLCPP_WARN(get_logger(), "SwitchStatus skipped | request already pending reason=%s",
    //                 reason.c_str());
    //     return;
    // }

    if (!switch_status_client_->service_is_ready()) {
        if (require_external_switch_service_) {
            RCLCPP_WARN(get_logger(), "SwitchStatus unavailable | reason=%s", reason.c_str());
            return;
        }
        if (candidates.size() == 1) {
            RCLCPP_WARN(get_logger(),
                        "SwitchStatus unavailable | reason=%s; single-candidate local fallback accepted",
                        reason.c_str());
            applyApprovedTransition(current_state, candidates.front(), reason + "_local_fallback");
            return;
        }
        RCLCPP_WARN(get_logger(),
                    "SwitchStatus unavailable | reason=%s; use direct keyboard mode command",
                    reason.c_str());
        return;
    }

    auto request = std::make_shared<status_interfaces_pkg::srv::SwitchStatus::Request>();
    request->current_status = static_cast<uint8_t>(stateToId(current_state));
    request->urgency = static_cast<uint8_t>(switch_status_urgency_);
    for (const auto candidate : candidates) {
        request->switchable_statuses.push_back(static_cast<uint8_t>(stateToId(candidate)));
    }

    switch_status_request_pending_ = true;
    const bool tracks_targ_got =
        std::find(candidates.begin(), candidates.end(), ControlState::APPROACH_PLANT) != candidates.end();
    if (tracks_targ_got) {
        targ_got_confirm_pending_ = true;
    }
    switch_status_client_->async_send_request(
        request,
        [this, current_state, candidates, reason, tracks_targ_got](
            rclcpp::Client<status_interfaces_pkg::srv::SwitchStatus>::SharedFuture resp_fut) {
            std::lock_guard<std::mutex> lock(fsm_mutex_);
            switch_status_request_pending_ = false;
            if (tracks_targ_got) {
                targ_got_confirm_pending_ = false;
            }
            try {
                const auto response = resp_fut.get();
                if (response->current_status != static_cast<uint8_t>(stateToId(current_state))) {
                    RCLCPP_WARN(get_logger(),
                                "SwitchStatus ignored | stale response current=%u expected=%d",
                                response->current_status, stateToId(current_state));
                    return;
                }
                const auto target_state = statusIdToState(response->target_status);
                if (!target_state) {
                    RCLCPP_WARN(get_logger(), "SwitchStatus ignored | unknown target=%u",
                                response->target_status);
                    return;
                }
                if (*target_state == current_state) {
                    RCLCPP_INFO(get_logger(), "SwitchStatus refused | reason=%s current=%s",
                                reason.c_str(), stateToString(current_state).c_str());
                    return;
                }
                if (std::find(candidates.begin(), candidates.end(), *target_state) == candidates.end()) {
                    RCLCPP_WARN(get_logger(), "SwitchStatus ignored | target=%s is not switchable",
                                stateToString(*target_state).c_str());
                    return;
                }
                applyApprovedTransition(current_state, *target_state, reason);
            } catch (const std::exception & e) {
                RCLCPP_ERROR(get_logger(), "SwitchStatus failed | reason=%s error=%s",
                                reason.c_str(), e.what());
            }
        });
}

// 已获批准的切换：集中处理批准后的标志位更新、执行结果发布和状态跳转。
void UavOffboardFsm::applyApprovedTransition(ControlState current_state,
                                              ControlState target_state,
                                              const std::string & reason)
{
        const auto live_state = control_state_.load();
        if (live_state != current_state) {
            RCLCPP_WARN(get_logger(), "Transition ignored | stale approval reason=%s expected=%s actual=%s",
                        reason.c_str(), stateToString(current_state).c_str(),
                        stateToString(live_state).c_str());
            return;
        }
        if (!canAcceptTargetTransition(current_state, target_state)) {
            RCLCPP_WARN(get_logger(), "Transition rejected | reason=%s current=%s target=%s",
                        reason.c_str(), stateToString(current_state).c_str(),
                        stateToString(target_state).c_str());
            return;
        }

        clearActiveTarget();
        targ_got_confirm_pending_ = false;
        task_term_confirm_pending_ = false;

        if (target_state == ControlState::APPROACH_PLANT) {
            adjust_completed_ = true;
            uav_search_succeed_ = true;
        } else if (target_state == ControlState::UAV_HOLD) {
            uav_adjust_succeed_ = true;
        }

        transitionTo(target_state);
        RCLCPP_INFO(get_logger(), LOG_COLOR_GREEN "Transition approved | reason=%s %s -> %s" LOG_COLOR_RESET,
                    reason.c_str(), stateToString(current_state).c_str(),
                    stateToString(target_state).c_str());
}

// 服务控制可接受性检查：只允许流程图中当前阶段合法、且必要完成标志已经满足的跳转。
bool UavOffboardFsm::canAcceptTargetTransition(ControlState current_state,
                                               ControlState target_state) const
{
        if (target_state == current_state) {
            return true;
        }

        switch (current_state) {
            case ControlState::SELF_CHECK:
                return target_state == ControlState::UAV_START && ready_for_takeoff_;
            case ControlState::UAV_START:
                return (target_state == ControlState::TRANSIT_TO_AREA && ready_for_transit_) ||
                       target_state == ControlState::UAV_TASK_TERM;
            case ControlState::TRANSIT_TO_AREA:
                return (target_state == ControlState::UAV_ARRIVED_AERA && is_arrived_task_aera_) ||
                       target_state == ControlState::UAV_TASK_TERM;
            case ControlState::UAV_ARRIVED_AERA:
                return target_state == ControlState::SEARCH_ADJUST_AUTO ||
                       target_state == ControlState::SEARCH_ADJUST_MANUAL ||
                       target_state == ControlState::APPROACH_PLANT ||
                       target_state == ControlState::UAV_TASK_TERM;
            case ControlState::SEARCH_ADJUST_AUTO:
            case ControlState::SEARCH_ADJUST_MANUAL:
                return target_state == ControlState::APPROACH_PLANT ||
                       target_state == ControlState::UAV_TASK_TERM;
            case ControlState::APPROACH_PLANT:
                return (target_state == ControlState::UAV_PRE_HOLD && approach_completed_) ||
                       target_state == ControlState::UAV_TASK_TERM;
            case ControlState::UAV_PRE_HOLD:
                return target_state == ControlState::SAMP_ADJUST_AUTO ||
                       target_state == ControlState::SAMP_ADJUST_MANUAL ||
                       target_state == ControlState::UAV_TASK_TERM;
            case ControlState::SAMP_ADJUST_AUTO:
            case ControlState::SAMP_ADJUST_MANUAL:
                return target_state == ControlState::UAV_HOLD ||
                       target_state == ControlState::UAV_TASK_TERM;
            case ControlState::UAV_HOLD:
                return (target_state == ControlState::RETREAT && sampl_opera_completed_) ||
                       target_state == ControlState::UAV_BACK_HOME ||
                       target_state == ControlState::UAV_TASK_TERM;
            case ControlState::RETREAT:
                return (target_state == ControlState::UAV_BACK_HOME && uav_ready_for_back_) ||
                       target_state == ControlState::UAV_TASK_TERM;
            case ControlState::UAV_BACK_HOME:
                return target_state == ControlState::UAV_TASK_TERM;
            case ControlState::UAV_TASK_TERM:
                return target_state == ControlState::RETREAT ||
                       target_state == ControlState::UAV_BACK_HOME;
        }
        return false;
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
    home_position_valid_ = msg->valid_lpos && std::isfinite(msg->x) &&
                           std::isfinite(msg->y) && std::isfinite(msg->z);
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

// 手动通道 aux1 作为执行器命令安全门：aux1 < 0.5 时允许 actuator_timer_ 正常重发，否则禁止向 PX4 发布。
void UavOffboardFsm::handleManualControlSetpoint(
    const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
{
    const bool enabled = msg->valid && std::isfinite(msg->aux3) && msg->aux3 < 0.0f;
    const bool was_received = manual_control_setpoint_received_.exchange(true);
    const bool was_enabled = actuator_command_enabled_.exchange(enabled);
    // 第一次收到 manual control 消息, 状态发生变化，比如从允许变成禁止，或者从禁止变成允许
    if (!was_received || was_enabled != enabled) {
        if (enabled) {
            RCLCPP_INFO(get_logger(),
                        LOG_COLOR_GREEN "Actuator command gate enabled | aux3=%.2f" LOG_COLOR_RESET,
                        msg->aux3);
        } else {
            RCLCPP_WARN(get_logger(),
                        "Actuator command gate blocked | valid=%s aux3=%.2f",
                        msg->valid ? "true" : "false", msg->aux3);
        }
    }
}

void UavOffboardFsm::handleWholeBodyUavSetpoint(
    const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
{
    const auto finiteOrZero = [](float value) -> double {
        return std::isfinite(value) ? static_cast<double>(value) : 0.0;
    };
    const Vector3 pos_ned{
        static_cast<double>(msg->position[0]),
        static_cast<double>(msg->position[1]),
        static_cast<double>(msg->position[2])};
    const Vector3 vel_ned{
        finiteOrZero(msg->velocity[0]),
        finiteOrZero(msg->velocity[1]),
        finiteOrZero(msg->velocity[2])};
    // const Vector3 acc_ned{
    //     finiteOrZero(msg->acceleration[0]),
    //     finiteOrZero(msg->acceleration[1]),
    //     finiteOrZero(msg->acceleration[2])};
    if (!std::isfinite(pos_ned[0]) || !std::isfinite(pos_ned[1]) ||
        !std::isfinite(pos_ned[2])) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "Whole body UAV setpoint ignored | non-finite position");
        return;
    }

    Vector3 pos_fsm{};
    Vector3 vel_fsm{};
    // Vector3 acc_fsm{};
    bool home_valid = false;
    const auto stamp = now();
    {
        std::lock_guard<std::mutex> lock(latest_state_mutex_);
        home_valid = home_position_valid_;
        if (home_valid) {
            pos_fsm = {
                pos_ned[1] - home_y_,
                pos_ned[0] - home_x_,
                -pos_ned[2] + home_z_};
            vel_fsm = {
                vel_ned[1],
                vel_ned[0],
                -vel_ned[2]};
            // acc_fsm = {
            //     acc_ned[1],
            //     acc_ned[0],
            //     -acc_ned[2]};
            latest_hold_uav_setpoint_ = pos_fsm;
            latest_hold_uav_velocity_ = vel_fsm;
            // latest_hold_uav_acceleration_ = acc_fsm;
            last_hold_pos_des_time_ = stamp;
        }
    }
    if (!home_valid) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
                             "Whole body UAV setpoint ignored | waiting valid home position");
        return;
    }
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), hovering_log_throttle_ms_,
                          "Whole body UAV setpoint received | ned=(%.3f, %.3f, %.3f) fsm=(%.3f, %.3f, %.3f) vel_fsm=(%.3f, %.3f, %.3f)",
                          pos_ned[0], pos_ned[1], pos_ned[2],
                          pos_fsm[0], pos_fsm[1], pos_fsm[2],
                          vel_fsm[0], vel_fsm[1], vel_fsm[2]);
}

// void UavOffboardFsm::handleActuatorOutputs(const px4_msgs::msg::ActuatorOutputs::SharedPtr msg)
// {
    
// }
// 轨迹完成标志回调：缓存里程碑标志，并在起飞/转场完成时用对应的轨迹设定点更新起飞点与转场终点。
void UavOffboardFsm::handleTrajCompleteFlag(const traj_offboard::msg::TrajCompleteFlag::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    traj_complete_flag_ = *msg;
    if (msg->take_off_completed) {
        takeoff_waypoint_ = waypointFromSetpoint(msg->traj_first_setpoint);
    }
    if (msg->trajectory_completed) {
        transit_end_waypoint_ = waypointFromSetpoint(msg->traj_last_setpoint);
        transit_end_waypoint_valid_ = true;
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
        case ControlState::UAV_START:
            return "UAV_START";
            case ControlState::TRANSIT_TO_AREA:
                return "TRANSIT_TO_AREA";
            case ControlState::UAV_ARRIVED_AERA:
                return "UAV_ARRIVED_AERA";
            case ControlState::SEARCH_ADJUST_AUTO:
                return "SEARCH_ADJUST_AUTO";
            case ControlState::SEARCH_ADJUST_MANUAL:
                return "SEARCH_ADJUST_MANUAL";
            case ControlState::APPROACH_PLANT:
                return "APPROACH_PLANT";
            case ControlState::UAV_PRE_HOLD:
                return "UAV_PRE_HOLD";
            case ControlState::SAMP_ADJUST_AUTO:
                return "SAMP_ADJUST_AUTO";
            case ControlState::SAMP_ADJUST_MANUAL:
                return "SAMP_ADJUST_MANUAL";
            case ControlState::UAV_HOLD:
                return "UAV_HOLD";
            case ControlState::RETREAT:
                return "RETREAT";
            case ControlState::UAV_BACK_HOME:
                return "UAV_BACK_HOME";
            case ControlState::UAV_TASK_TERM:
                return "UAV_TASK_TERM";
        }
        return "UNKNOWN";
}

// 状态枚举转流程图编号：编号固定为 0-13，与最终版无人机状态表一致。
int UavOffboardFsm::stateToId(ControlState state)
{
        return static_cast<int>(state);
}

// 状态索引转枚举：用于服务请求/响应中的 uint8 状态值合法性检查。
std::optional<UavOffboardFsm::ControlState> UavOffboardFsm::statusIdToState(uint8_t status)
{
        switch (status) {
            case 0:
                return ControlState::SELF_CHECK;
            case 1:
                return ControlState::UAV_START;
            case 2:
                return ControlState::TRANSIT_TO_AREA;
            case 3:
                return ControlState::UAV_ARRIVED_AERA;
            case 4:
                return ControlState::SEARCH_ADJUST_AUTO;
            case 5:
                return ControlState::SEARCH_ADJUST_MANUAL;
            case 6:
                return ControlState::APPROACH_PLANT;
            case 7:
                return ControlState::UAV_PRE_HOLD;
            case 8:
                return ControlState::SAMP_ADJUST_AUTO;
            case 9:
                return ControlState::SAMP_ADJUST_MANUAL;
            case 10:
                return ControlState::UAV_HOLD;
            case 11:
                return ControlState::RETREAT;
            case 12:
                return ControlState::UAV_BACK_HOME;
            case 13:
                return ControlState::UAV_TASK_TERM;
            default:
                return std::nullopt;
        }
}

// 控制指令解析：读取首个 token，并精确匹配流程图指令；不再支持旧命令别名。
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

        if (token == "SELF_CHECK" || token == "PRE_CHECK") {
            return ParsedCommand{CommandType::PRE_CHECK};
        }
        if (token == "WAIT_TASK_ENABLE_AUTH") {
            return ParsedCommand{CommandType::WAIT_TASK_ENABLE_AUTH};
        }
        if (token == "NAV_TO_TASK_DOM") {
            return ParsedCommand{CommandType::NAV_TO_TASK_DOM};
        }
        if (token == "ARRIVE_TASK_DOM") {
            return ParsedCommand{CommandType::ARRIVE_TASK_DOM};
        }
        if (token == "UAV_SEARCH_TARGS") {
            return ParsedCommand{CommandType::UAV_SEARCH_TARGS};
        }
        if (token == "SEARCH_ADJUST_AUTO") {
            return ParsedCommand{CommandType::SEARCH_ADJUST_AUTO};
        }
    if (token == "SEARCH_ADJUST_MANUAL") {
        return ParsedCommand{CommandType::SEARCH_ADJUST_MANUAL};
    }
        if (token == "TARG_GOT") {
            return ParsedCommand{CommandType::TARG_GOT};
        }
        if (token == "TARG_READY") {
            return ParsedCommand{CommandType::TARG_READY};
        }
        if (token == "UAV_POSE_ADAP") {
            return ParsedCommand{CommandType::UAV_POSE_ADAP};
        }
        if (token == "SAMP_ADJUST_AUTO") {
            return ParsedCommand{CommandType::SAMP_ADJUST_AUTO};
        }
    if (token == "SAMP_ADJUST_MANUAL") {
        return ParsedCommand{CommandType::SAMP_ADJUST_MANUAL};
    }
    if (token == "ARM_CONFIG_PREP") {
        return ParsedCommand{CommandType::ARM_CONFIG_PREP};
    }
    if (token == "SAMPL_OPERA") {
        return ParsedCommand{CommandType::SAMPL_OPERA};
    }
    if (token == "UAV_PRE_BACK_HOME") {
        return ParsedCommand{CommandType::UAV_PRE_BACK_HOME};
    }
        if (token == "BACK_HOME") {
            return ParsedCommand{CommandType::BACK_HOME};
        }
        if (token == "TASK_TERM") {
            return ParsedCommand{CommandType::TASK_TERM};
        }
        if (token == "NO") {
            return ParsedCommand{CommandType::NO};
        }
    if (token == "CONFIRM") {
        return ParsedCommand{CommandType::CONFIRM};
    }

        return std::nullopt;
}

// main_task_state 索引映射：当前可由键盘模拟，也可直接订阅总状态机发布的 Status.status。
std::optional<UavOffboardFsm::ParsedCommand>
UavOffboardFsm::commandFromMainTaskStatus(uint8_t status) const
{
        switch (status) {
            case 1:
                return ParsedCommand{CommandType::PRE_CHECK};
            case 2:
                return ParsedCommand{CommandType::WAIT_TASK_ENABLE_AUTH};
            case 3:
                return ParsedCommand{CommandType::NAV_TO_TASK_DOM};
            case 4:
                return ParsedCommand{CommandType::ARRIVE_TASK_DOM};
            case 6:
                return ParsedCommand{CommandType::UAV_SEARCH_TARGS};
            case 7:
                return ParsedCommand{CommandType::TARG_GOT};
            case 8:
                return ParsedCommand{CommandType::TARG_READY};
            case 9:
                return ParsedCommand{CommandType::UAV_POSE_ADAP};
            case 10:
                return ParsedCommand{CommandType::ARM_CONFIG_PREP};
            case 11:
                return ParsedCommand{CommandType::SAMPL_OPERA};
            case 12:
                return ParsedCommand{CommandType::UAV_PRE_BACK_HOME};
            case 13:
                return ParsedCommand{CommandType::BACK_HOME};
            case 30:
                return ParsedCommand{CommandType::TASK_TERM};
            default:
                return std::nullopt;
        }
}

// token 匹配工具：判断大写后的任务状态 token 是否位于允许/禁止任务状态集合中。
bool UavOffboardFsm::tokenMatches(const std::string & token,
                                  const std::vector<std::string> & aliases)
{
    return std::find(aliases.begin(), aliases.end(), token) != aliases.end();
}

// 字符串数组大写化工具：启动时把任务状态集合统一转成大写，后续匹配不受大小写影响。
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
