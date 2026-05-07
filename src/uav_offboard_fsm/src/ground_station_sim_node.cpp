#include <rclcpp/rclcpp.hpp>
#include <status_interfaces_pkg/srv/airdrop_status.hpp>
#include <status_interfaces_pkg/srv/switch_status.hpp>

class GroundStationSim : public rclcpp::Node {
  public:
    GroundStationSim() : rclcpp::Node("ground_station_sim")
    {
        const auto switch_service =
            declare_parameter<std::string>("switch_status_service", "/ground_station/switch_status");
        const auto airdrop_service =
            declare_parameter<std::string>("airdrop_status_service", "/ground_station/airdrop_status");

        switch_status_srv_ = create_service<status_interfaces_pkg::srv::SwitchStatus>(
            switch_service,
            std::bind(&GroundStationSim::handleSwitchStatus, this,
                      std::placeholders::_1, std::placeholders::_2));

        airdrop_status_srv_ = create_service<status_interfaces_pkg::srv::AirdropStatus>(
            airdrop_service,
            std::bind(&GroundStationSim::handleAirdropStatus, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
                    "GroundStationSim ready | switch_service=%s airdrop_service=%s",
                    switch_service.c_str(), airdrop_service.c_str());
    }

  private:
    // 请求式：自动选择 switchable_statuses 中的第一个候选，原样回传 current_status 供时效校验。
    void handleSwitchStatus(
        const std::shared_ptr<status_interfaces_pkg::srv::SwitchStatus::Request> request,
        std::shared_ptr<status_interfaces_pkg::srv::SwitchStatus::Response> response)
    {
        response->current_status = request->current_status;
        if (request->switchable_statuses.empty()) {
            response->target_status = request->current_status;
            RCLCPP_WARN(get_logger(),
                        "SwitchStatus | empty switchable list, echoing current_status=%u",
                        request->current_status);
            return;
        }
        response->target_status = request->switchable_statuses[0];
        RCLCPP_INFO(get_logger(),
                    "SwitchStatus | current=%u -> target=%u (auto first)",
                    request->current_status, response->target_status);
    }

    // 控制式：仿真模式下始终批准切换请求。
    void handleAirdropStatus(
        const std::shared_ptr<status_interfaces_pkg::srv::AirdropStatus::Request> request,
        std::shared_ptr<status_interfaces_pkg::srv::AirdropStatus::Response> response)
    {
        response->success = true;
        RCLCPP_INFO(get_logger(),
                    "AirdropStatus | approved current=%u -> target=%u urgency=%u",
                    request->current_status, request->target_status, request->urgency);
    }

    rclcpp::Service<status_interfaces_pkg::srv::SwitchStatus>::SharedPtr switch_status_srv_;
    rclcpp::Service<status_interfaces_pkg::srv::AirdropStatus>::SharedPtr airdrop_status_srv_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundStationSim>());
    rclcpp::shutdown();
    return 0;
}
