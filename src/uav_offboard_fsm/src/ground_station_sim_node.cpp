#include <rclcpp/rclcpp.hpp>
#include <status_interfaces_pkg/srv/switch_status.hpp>

class GroundStationSim : public rclcpp::Node {
  public:
    GroundStationSim() : rclcpp::Node("ground_station_sim")
    {
        const auto switch_service =
            declare_parameter<std::string>("switch_status_service", "/ground_station/switch_status");

        switch_status_srv_ = create_service<status_interfaces_pkg::srv::SwitchStatus>(
            switch_service,
            std::bind(&GroundStationSim::handleSwitchStatus, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
                    "GroundStationSim ready | switch_service=%s",
                    switch_service.c_str());
    }

  private:
    // 请求式：自动选择 switchable_statuses 中的第一个候选，原样回传 current_status 供时效校验。
    uint32_t count = 1;
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
        // while(count < 5000000)
        // {
        //     count ++;
        //     RCLCPP_WARN(get_logger(),
        //                 "Waiting for repose, count=%d",
        //                 count);
        // }
        response->target_status = request->switchable_statuses[0];
        RCLCPP_INFO(get_logger(),
                    "SwitchStatus | current=%u -> target=%u (auto first)",
                    request->current_status, response->target_status);
    }

    rclcpp::Service<status_interfaces_pkg::srv::SwitchStatus>::SharedPtr switch_status_srv_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundStationSim>());
    rclcpp::shutdown();
    return 0;
}
