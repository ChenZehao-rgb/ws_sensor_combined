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
        // qos profile setting
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos, std::bind(&OffboardControlBridge::VehicleLocalPositionCallback, this, std::placeholders::_1));
        vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos, std::bind(&OffboardControlBridge::VehicleAttitudeCallback, this, std::placeholders::_1));
        vehicle_imu_sub_ = this->create_subscription<px4_msgs::msg::VehicleImu>(
            "/fmu/out/vehicle_imu", qos, std::bind(&OffboardControlBridge::VehicleImuCallback, this, std::placeholders::_1));
        // Service to set target for online trajectory generator
        set_target_srv_ = this->create_service<traj_offboard::srv::SetTarget>(
            "online_traj_generator/set_target", std::bind(&OffboardControlBridge::handle_set_target, this, std::placeholders::_1, std::placeholders::_2));

        auto onTimer = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                // Switch to offboard mode and arm after sending initial setpoints
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
                RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000, "Offboard & arm commands sent");
            }

            publish_offboard_control_mode();

            if (offboard_setpoint_counter_ < 11) {
                ++offboard_setpoint_counter_;
            }
        };

        // Control timer: pair OffboardControlMode with a setpoint
        timer_ = this->create_wall_timer(100ms, onTimer);
    }

  private:
    // State holders
    geometry_msgs::msg::PoseStamped uav_pose_;
    geometry_msgs::msg::TwistStamped uav_twist_;
    sensor_msgs::msg::Imu uav_imu_;
    px4_msgs::msg::TrajectorySetpoint target_pose_;
    bool have_target_{false};

    // ROS interfaces
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleImu>::SharedPtr vehicle_imu_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr targ_point_pub_;
    rclcpp::Service<traj_offboard::srv::SetTarget>::SharedPtr set_target_srv_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_cmd_sub_;

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

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControlBridge>());
    rclcpp::shutdown();
    return 0;
}
