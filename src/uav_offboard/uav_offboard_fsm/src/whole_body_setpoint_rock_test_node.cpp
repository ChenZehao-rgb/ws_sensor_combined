#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

class WholeBodySetpointRockTest : public rclcpp::Node {
  public:
    WholeBodySetpointRockTest() : rclcpp::Node("whole_body_setpoint_rock_test")
    {
        output_topic_ = declare_parameter<std::string>(
            "output_topic", "/whole_body_planner/uav_setpoint");
        local_position_topic_ = declare_parameter<std::string>(
            "local_position_topic", "/fmu/out/vehicle_local_position");
        rate_hz_ = declare_parameter<double>("rate_hz", 20.0);
        trajectory_type_ = declare_parameter<std::string>("trajectory_type", "sine");
        amplitude_m_ = declare_parameter<double>("amplitude_m", 0.2);
        frequency_hz_ = declare_parameter<double>("frequency_hz", 0.2);
        fixed_velocity_m_s_ = declare_parameter<double>("fixed_velocity_m_s", 0.1);
        fixed_velocity_north_m_s_ = declare_parameter<double>("fixed_velocity_north_m_s", 0.0);
        fixed_velocity_east_m_s_ = declare_parameter<double>("fixed_velocity_east_m_s", 0.0);
        axis_ = declare_parameter<std::string>("axis", "east");
        use_manual_base_ = declare_parameter<bool>("use_manual_base", false);
        manual_base_ned_ = parseManualBase(
            declare_parameter<std::vector<double>>("manual_base_ned", {0.0, 0.0, 0.0}));

        if (rate_hz_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid rate_hz=%.3f, using 20Hz", rate_hz_);
            rate_hz_ = 20.0;
        }
        if (frequency_hz_ < 0.0) {
            RCLCPP_WARN(get_logger(), "Invalid frequency_hz=%.3f, using 0.2Hz", frequency_hz_);
            frequency_hz_ = 0.2;
        }
        if (!std::isfinite(fixed_velocity_m_s_)) {
            RCLCPP_WARN(get_logger(), "Invalid fixed_velocity_m_s, using 0.1m/s");
            fixed_velocity_m_s_ = 0.1;
        }
        if (!std::isfinite(fixed_velocity_north_m_s_)) {
            RCLCPP_WARN(get_logger(), "Invalid fixed_velocity_north_m_s, using 0.0m/s");
            fixed_velocity_north_m_s_ = 0.0;
        }
        if (!std::isfinite(fixed_velocity_east_m_s_)) {
            RCLCPP_WARN(get_logger(), "Invalid fixed_velocity_east_m_s, using 0.0m/s");
            fixed_velocity_east_m_s_ = 0.0;
        }
        if (trajectory_type_ != "sine" && trajectory_type_ != "fixed_velocity") {
            RCLCPP_WARN(get_logger(),
                        "Invalid trajectory_type=%s, using sine",
                        trajectory_type_.c_str());
            trajectory_type_ = "sine";
        }
        axis_index_ = axisToIndex(axis_);
        fixed_velocity_use_north_east_ = isNorthEastAxis(axis_);
        if (axis_index_ < 0 &&
            !(trajectory_type_ == "fixed_velocity" && fixed_velocity_use_north_east_)) {
            RCLCPP_WARN(get_logger(), "Invalid axis=%s, using east", axis_.c_str());
            axis_ = "east";
            axis_index_ = 1;
            fixed_velocity_use_north_east_ = false;
        }

        setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(output_topic_, 10);

        auto sensor_qos = rclcpp::SensorDataQoS();
        local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            local_position_topic_, sensor_qos,
            std::bind(&WholeBodySetpointRockTest::handleVehicleLocalPosition, this,
                      std::placeholders::_1));

        if (use_manual_base_) {
            base_ned_ = manual_base_ned_;
            base_ready_ = true;
            waveform_started_ = false;
            RCLCPP_INFO(get_logger(),
                        "Rock test manual base | ned=(%.3f, %.3f, %.3f)",
                        base_ned_[0], base_ned_[1], base_ned_[2]);
        }

        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&WholeBodySetpointRockTest::publishSetpoint, this));

        if (trajectory_type_ == "fixed_velocity") {
            if (fixed_velocity_use_north_east_) {
                RCLCPP_INFO(
                    get_logger(),
                    "WholeBodySetpointRockTest ready | out=%s base=%s type=%s axis=%s velocity_north=%.3fm/s velocity_east=%.3fm/s rate=%.1fHz",
                    output_topic_.c_str(),
                    use_manual_base_ ? "manual_base_ned" : local_position_topic_.c_str(),
                    trajectory_type_.c_str(), axis_.c_str(), fixed_velocity_north_m_s_,
                    fixed_velocity_east_m_s_, rate_hz_);
            } else {
                RCLCPP_INFO(
                    get_logger(),
                    "WholeBodySetpointRockTest ready | out=%s base=%s type=%s axis=%s velocity=%.3fm/s rate=%.1fHz",
                    output_topic_.c_str(),
                    use_manual_base_ ? "manual_base_ned" : local_position_topic_.c_str(),
                    trajectory_type_.c_str(), axis_.c_str(), fixed_velocity_m_s_, rate_hz_);
            }
        } else {
            RCLCPP_INFO(
                get_logger(),
                "WholeBodySetpointRockTest ready | out=%s base=%s type=%s axis=%s amplitude=%.3fm frequency=%.3fHz rate=%.1fHz",
                output_topic_.c_str(),
                use_manual_base_ ? "manual_base_ned" : local_position_topic_.c_str(),
                trajectory_type_.c_str(), axis_.c_str(), amplitude_m_, frequency_hz_,
                rate_hz_);
        }
    }

  private:
    static std::array<double, 3> parseManualBase(const std::vector<double> & value)
    {
        if (value.size() != 3) {
            return {0.0, 0.0, 0.0};
        }
        return {value[0], value[1], value[2]};
    }

    static int axisToIndex(const std::string & axis)
    {
        if (axis == "north" || axis == "x" || axis == "n") {
            return 0;
        }
        if (axis == "east" || axis == "y" || axis == "e") {
            return 1;
        }
        if (axis == "down" || axis == "z" || axis == "d") {
            return 2;
        }
        return -1;
    }

    static bool isNorthEastAxis(const std::string & axis)
    {
        return axis == "north_east" || axis == "northeast" || axis == "ne" ||
               axis == "xy" || axis == "horizontal" || axis == "both";
    }

    void handleVehicleLocalPosition(
        const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        if (use_manual_base_ || base_ready_) {
            return;
        }
        if (!std::isfinite(msg->x) || !std::isfinite(msg->y) || !std::isfinite(msg->z)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting valid vehicle local position");
            return;
        }

        base_ned_ = {
            static_cast<double>(msg->x),
            static_cast<double>(msg->y),
            static_cast<double>(msg->z)};
        base_ready_ = true;
        waveform_started_ = false;
        RCLCPP_INFO(get_logger(),
                    "Rock test captured base from vehicle_local_position | ned=(%.3f, %.3f, %.3f)",
                    base_ned_[0], base_ned_[1], base_ned_[2]);
    }

    void publishSetpoint()
    {
        if (!base_ready_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Rock test waiting base position from %s",
                                 local_position_topic_.c_str());
            return;
        }

        const auto now_time = now();
        if (!waveform_started_) {
            start_time_ = now_time;
            waveform_started_ = true;
        }
        const double t = (now_time - start_time_).seconds();
        auto target = base_ned_;
        std::array<float, 3> velocity_ned{0.0f, 0.0f, 0.0f};
        std::array<float, 3> acceleration_ned{0.0f, 0.0f, 0.0f};
        if (trajectory_type_ == "fixed_velocity") {
            if (fixed_velocity_use_north_east_) {
                target[0] += fixed_velocity_north_m_s_ * t;
                target[1] += fixed_velocity_east_m_s_ * t;
                velocity_ned[0] = static_cast<float>(fixed_velocity_north_m_s_);
                velocity_ned[1] = static_cast<float>(fixed_velocity_east_m_s_);
            } else {
                const double offset = fixed_velocity_m_s_ * t;
                target[static_cast<std::size_t>(axis_index_)] += offset;
                velocity_ned[static_cast<std::size_t>(axis_index_)] =
                    static_cast<float>(fixed_velocity_m_s_);
            }
        } else {
            constexpr double kPi = 3.14159265358979323846;
            const double omega = 2.0 * kPi * frequency_hz_;
            const double phase = omega * t;
            const double offset = amplitude_m_ * std::cos(phase);
            const double velocity = -amplitude_m_ * omega * std::sin(phase);
            const double acceleration = -amplitude_m_ * omega * omega * std::cos(phase);
            target[static_cast<std::size_t>(axis_index_)] += offset;
            velocity_ned[static_cast<std::size_t>(axis_index_)] =
                static_cast<float>(velocity);
            acceleration_ned[static_cast<std::size_t>(axis_index_)] =
                static_cast<float>(acceleration);
        }

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = now().nanoseconds() / 1000;
        // target.fill(0.0f);
        msg.position = {
            static_cast<float>(target[0]),
            static_cast<float>(target[1]),
            static_cast<float>(target[2])};
        // test vel and acc to 0.0
        // velocity_ned.fill(0.0f);
        // acceleration_ned.fill(0.0f);
        msg.velocity = velocity_ned;
        msg.acceleration = acceleration_ned;
        msg.yaw = 0.0f;
        msg.yawspeed = 0.0f;
        setpoint_pub_->publish(msg);
    }

    std::string output_topic_;
    std::string local_position_topic_;
    std::string trajectory_type_;
    std::string axis_;
    double rate_hz_{20.0};
    double amplitude_m_{0.2};
    double frequency_hz_{0.2};
    double fixed_velocity_m_s_{0.1};
    double fixed_velocity_north_m_s_{0.0};
    double fixed_velocity_east_m_s_{0.0};
    bool use_manual_base_{false};
    bool base_ready_{false};
    bool fixed_velocity_use_north_east_{false};
    int axis_index_{1};
    std::array<double, 3> manual_base_ned_{0.0, 0.0, 0.0};
    std::array<double, 3> base_ned_{0.0, 0.0, 0.0};
    rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
    bool waveform_started_{false};

    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WholeBodySetpointRockTest>());
    rclcpp::shutdown();
    return 0;
}
