#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

class MockPx4Sim : public rclcpp::Node {
  public:
    MockPx4Sim() : rclcpp::Node("mock_px4_sim") {
        plant_x_ = declare_parameter<double>("plant_x", 5.0);
        plant_y_ = declare_parameter<double>("plant_y", 6.5);
        plant_z_ = declare_parameter<double>("plant_z", 5.0);
        min_distance_ = declare_parameter<double>("min_distance", 0.2);
        max_distance_ = declare_parameter<double>("max_distance", 8.0);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
        qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos_profile.depth = 5;
        auto sensor_qos =
            rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
                        qos_profile);

        local_position_pub_ =
            create_publisher<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
                                                                  sensor_qos);
        attitude_pub_ =
            create_publisher<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", sensor_qos);
        imu_pub_ = create_publisher<px4_msgs::msg::VehicleImu>("/fmu/out/vehicle_imu", sensor_qos);
        home_pub_ = create_publisher<px4_msgs::msg::HomePosition>("/fmu/out/home_position", sensor_qos);
        distance_pub_ =
            create_publisher<px4_msgs::msg::DistanceSensor>("/fmu/out/distance_sensor", sensor_qos);

        trajectory_sub_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10,
            std::bind(&MockPx4Sim::handleTrajectorySetpoint, this, std::placeholders::_1));
        vehicle_command_sub_ = create_subscription<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10,
            std::bind(&MockPx4Sim::handleVehicleCommand, this, std::placeholders::_1));

        timer_ = create_wall_timer(50ms, std::bind(&MockPx4Sim::onTimer, this));
    }

  private:
    struct Pose2d {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double yaw{0.0};
    };

    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleImu>::SharedPtr imu_pub_;
    rclcpp::Publisher<px4_msgs::msg::HomePosition>::SharedPtr home_pub_;
    rclcpp::Publisher<px4_msgs::msg::DistanceSensor>::SharedPtr distance_pub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    Pose2d pose_{};
    Pose2d target_{};
    bool has_target_{false};
    double home_x_{0.0};
    double home_y_{0.0};
    double home_z_{0.0};
    double plant_x_{5.0};
    double plant_y_{6.5};
    double plant_z_{5.0};
    double min_distance_{0.2};
    double max_distance_{8.0};

    static double wrapAngle(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    uint64_t timestampUs() const {
        return now().nanoseconds() / 1000;
    }

    void handleTrajectorySetpoint(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        static constexpr double half_pi = 1.5707963267948966;
        if (std::isfinite(msg->position[0]) && std::isfinite(msg->position[1]) &&
            std::isfinite(msg->position[2])) {
            target_.x = msg->position[1] - home_y_;
            target_.y = msg->position[0] - home_x_;
            target_.z = -msg->position[2] + home_z_;
        }
        if (std::isfinite(msg->yaw)) {
            target_.yaw = wrapAngle(half_pi - msg->yaw);
        }
        has_target_ = true;
    }

    void handleVehicleCommand(const px4_msgs::msg::VehicleCommand::SharedPtr msg) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Mock PX4 command received | command=%u", msg->command);
    }

    void onTimer() {
        if (has_target_) {
            pose_ = target_;
        }
        publishHome();
        publishLocalPosition();
        publishAttitude();
        publishImu();
        publishDistance();
    }

    void publishHome() {
        px4_msgs::msg::HomePosition msg{};
        msg.timestamp = timestampUs();
        msg.x = static_cast<float>(home_x_);
        msg.y = static_cast<float>(home_y_);
        msg.z = static_cast<float>(home_z_);
        msg.yaw = 0.0f;
        msg.valid_lpos = true;
        msg.valid_hpos = true;
        msg.valid_alt = true;
        home_pub_->publish(msg);
    }

    void publishLocalPosition() {
        px4_msgs::msg::VehicleLocalPosition msg{};
        msg.timestamp = timestampUs();
        msg.timestamp_sample = msg.timestamp;
        msg.xy_valid = true;
        msg.z_valid = true;
        msg.v_xy_valid = true;
        msg.v_z_valid = true;
        msg.x = static_cast<float>(pose_.y + home_x_);
        msg.y = static_cast<float>(pose_.x + home_y_);
        msg.z = static_cast<float>(-pose_.z + home_z_);
        msg.heading = static_cast<float>(wrapAngle(1.5707963267948966 - pose_.yaw));
        msg.heading_good_for_control = true;
        local_position_pub_->publish(msg);
    }

    void publishAttitude() {
        px4_msgs::msg::VehicleAttitude msg{};
        msg.timestamp = timestampUs();
        msg.timestamp_sample = msg.timestamp;
        const double half_yaw = pose_.yaw * 0.5;
        msg.q[0] = 0.0f;
        msg.q[1] = 0.0f;
        msg.q[2] = static_cast<float>(-std::sin(half_yaw));
        msg.q[3] = static_cast<float>(std::cos(half_yaw));
        attitude_pub_->publish(msg);
    }

    void publishImu() {
        px4_msgs::msg::VehicleImu msg{};
        msg.timestamp = timestampUs();
        msg.timestamp_sample = msg.timestamp;
        msg.delta_angle_dt = 50000;
        msg.delta_velocity_dt = 50000;
        imu_pub_->publish(msg);
    }

    void publishDistance() {
        px4_msgs::msg::DistanceSensor msg{};
        msg.timestamp = timestampUs();
        msg.min_distance = static_cast<float>(min_distance_);
        msg.max_distance = static_cast<float>(max_distance_);
        const double forward_x = std::cos(pose_.yaw);
        const double forward_y = std::sin(pose_.yaw);
        const double dx = plant_x_ - pose_.x;
        const double dy = plant_y_ - pose_.y;
        const double along_forward = dx * forward_x + dy * forward_y;
        msg.current_distance =
            static_cast<float>(std::clamp(along_forward, min_distance_, max_distance_));
        msg.signal_quality = 100;
        msg.type = px4_msgs::msg::DistanceSensor::MAV_DISTANCE_SENSOR_RADAR;
        msg.orientation = px4_msgs::msg::DistanceSensor::ROTATION_FORWARD_FACING;
        msg.mode = px4_msgs::msg::DistanceSensor::MODE_ENABLED;
        distance_pub_->publish(msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockPx4Sim>());
    rclcpp::shutdown();
    return 0;
}
