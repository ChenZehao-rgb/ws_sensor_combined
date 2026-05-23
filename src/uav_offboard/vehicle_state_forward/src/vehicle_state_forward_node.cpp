#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

class VehicleStateForwardNode : public rclcpp::Node
{
public:
    VehicleStateForwardNode() : Node("vehicle_state_forward_node")
    {
        RCLCPP_INFO(this->get_logger(), "Vehicle State Forward Node started");

        rclcpp::QoS qos(10);
        qos.best_effort()
           .transient_local();

        global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", qos,
            [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                last_global_position_ = msg;
            });

        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
                last_attitude_ = msg;
            });

        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                last_vehicle_status_ = msg;
            });

        home_position_sub_ = this->create_subscription<px4_msgs::msg::HomePosition>(
            "/fmu/out/home_position", qos,
            [this](const px4_msgs::msg::HomePosition::SharedPtr msg) {
                last_home_position_ = msg;
            });

        global_position_pub_ = this->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
            "/ground_station/vehicle_global_position", 10);

        attitude_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitude>(
            "/ground_station/vehicle_attitude", 10);

        vehicle_status_pub_ = this->create_publisher<px4_msgs::msg::VehicleStatus>(
            "/ground_station/vehicle_status", 10);

        home_position_pub_ = this->create_publisher<px4_msgs::msg::HomePosition>(
            "/ground_station/home_position", 10);

        using namespace std::chrono_literals;
        constexpr auto period_20hz = 50ms;
        constexpr auto period_2hz = 500ms;

        timer_20hz_1_ = this->create_wall_timer(period_20hz, [this]() {
            this->publishGlobalPosition();
        });

        timer_20hz_2_ = this->create_wall_timer(period_20hz, [this]() {
            this->publishAttitude();
        });

        timer_2hz_1_ = this->create_wall_timer(period_2hz, [this]() {
            this->publishVehicleStatus();
        });

        timer_2hz_2_ = this->create_wall_timer(period_2hz, [this]() {
            this->publishHomePosition();
        });
    }

private:
    void publishGlobalPosition()
    {
        if (last_global_position_ == nullptr) {
            return;
        }

        global_position_pub_->publish(*last_global_position_);
    }

    void publishAttitude()
    {
        if (last_attitude_ == nullptr) {
            return;
        }

        attitude_pub_->publish(*last_attitude_);
    }

    void publishVehicleStatus()
    {
        if (last_vehicle_status_ == nullptr) {
            return;
        }

        vehicle_status_pub_->publish(*last_vehicle_status_);
    }

    void publishHomePosition()
    {
        if (last_home_position_ == nullptr) {
            return;
        }

        home_position_pub_->publish(*last_home_position_);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr home_position_sub_;

    rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_pub_;
    rclcpp::Publisher<px4_msgs::msg::HomePosition>::SharedPtr home_position_pub_;

    rclcpp::TimerBase::SharedPtr timer_20hz_1_;
    rclcpp::TimerBase::SharedPtr timer_20hz_2_;
    rclcpp::TimerBase::SharedPtr timer_2hz_1_;
    rclcpp::TimerBase::SharedPtr timer_2hz_2_;

    px4_msgs::msg::VehicleGlobalPosition::SharedPtr last_global_position_;
    px4_msgs::msg::VehicleAttitude::SharedPtr last_attitude_;
    px4_msgs::msg::VehicleStatus::SharedPtr last_vehicle_status_;
    px4_msgs::msg::HomePosition::SharedPtr last_home_position_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleStateForwardNode>());
    rclcpp::shutdown();
    return 0;
}
