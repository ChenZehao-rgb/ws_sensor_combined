#include <rclcpp/rclcpp.hpp>
#include <waypoint_data_pkg/msg/waypoint_data.hpp>
#include <waypoint_data_pkg/msg/waypoint_data_response.hpp>

#include <map>
#include <fstream>
#include <string>
#include <chrono>

// CSV 保存路径（硬编码绝对路径）
static constexpr const char * CSV_PATH = "/home/sia/ws_sensor_combined/src/uav_offboard/waypoints.csv";
// 超时时间（秒）- 连续两个航点之间的最大间隔
static constexpr double TIMEOUT_SEC = 5.0;

class WaypointReceiverNode : public rclcpp::Node
{
public:
  WaypointReceiverNode()
  : Node("waypoint_receiver_node")
  {
    // QoS：与 Win 端 FastDDS 一致，使用 ParametersQoS（reliable + transient_local depth=1000）
    auto qos = rclcpp::QoS(100).reliable();

    sub_ = this->create_subscription<waypoint_data_pkg::msg::WaypointData>(
      "/path_agent/waypoint_data",
      qos,
      std::bind(&WaypointReceiverNode::on_waypoint, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<waypoint_data_pkg::msg::WaypointDataResponse>(
      "/path_agent/waypoint_data_response",
      qos
    );

    RCLCPP_INFO(this->get_logger(), "WaypointReceiver 已启动，等待航点数据...");
    RCLCPP_INFO(this->get_logger(), "订阅: /path_agent/waypoint_data");
    RCLCPP_INFO(this->get_logger(), "发布: /path_agent/waypoint_data_response");
    RCLCPP_INFO(this->get_logger(), "CSV 路径: %s", CSV_PATH);
  }

private:
  using WaypointData = waypoint_data_pkg::msg::WaypointData;
  using WaypointDataResponse = waypoint_data_pkg::msg::WaypointDataResponse;

  rclcpp::Subscription<WaypointData>::SharedPtr sub_;
  rclcpp::Publisher<WaypointDataResponse>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  // 用 map 缓存，key=point_index，自动去重且按索引有序
  std::map<int64_t, WaypointData> waypoint_map_;
  int64_t expected_total_{0};

  void on_waypoint(const WaypointData::SharedPtr msg)
  {
    // 首次收到时记录总数
    if (waypoint_map_.empty()) {
      expected_total_ = msg->point_num;
      RCLCPP_INFO(
        this->get_logger(),
        "开始接收航点数据，预期总数: %ld", expected_total_);
    }

    // 总数不一致时（对方中途改变了 point_num），重置
    if (msg->point_num != expected_total_) {
      RCLCPP_WARN(
        this->get_logger(),
        "point_num 变化（%ld -> %ld），重置缓存", expected_total_, msg->point_num);
      reset();
      expected_total_ = msg->point_num;
    }

    // 存入 map（重复 index 会覆盖，防止对方重发）
    int64_t idx = msg->point_index;
    bool is_new = (waypoint_map_.find(idx) == waypoint_map_.end());
    waypoint_map_[idx] = *msg;

    if (is_new) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "收到航点 [%ld/%ld]", idx, expected_total_ - 1);
      // 每次收到新航点都重置超时计时器
      start_timeout_timer();
    }

    // 判断是否收齐
    if (static_cast<int64_t>(waypoint_map_.size()) == expected_total_) {
      RCLCPP_INFO(this->get_logger(), "航点数据接收完毕，共 %ld 条，开始写入 CSV...", expected_total_);
      handle_complete();
    }
  }

  void handle_complete()
  {
    // 停止超时计时器
    if (timeout_timer_) {
      timeout_timer_->cancel();
      timeout_timer_.reset();
    }

    // 写 CSV
    bool write_ok = write_csv();
    if (!write_ok) {
      publish_response(false, "CSV 写入失败，请检查路径: " + std::string(CSV_PATH));
      reset();
      return;
    }

    // 验证行数
    int64_t line_count = count_csv_data_lines();
    bool count_ok = (line_count == expected_total_);

    WaypointDataResponse response;
    if (count_ok) {
      response.success = true;
      response.message = "航点接受成功，总数量：" + std::to_string(expected_total_);
      RCLCPP_INFO(this->get_logger(), "%s", response.message.c_str());
    } else {
      response.success = false;
      response.message = "CSV 行数验证失败，预期: " + std::to_string(expected_total_) +
        "，实际写入: " + std::to_string(line_count);
      RCLCPP_ERROR(this->get_logger(), "%s", response.message.c_str());
    }

    pub_->publish(response);
    reset();
  }

  bool write_csv()
  {
    std::ofstream ofs(CSV_PATH, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开文件: %s", CSV_PATH);
      return false;
    }

    // 表头
    ofs << "index,x,y,z,yaw,vn,ve,vd,an,ae,ad\n";

    // 数据行（map 已按 key 升序排列）
    for (const auto & [idx, wp] : waypoint_map_) {
      ofs << idx << ","
          << wp.position.position.x << ","
          << wp.position.position.y << ","
          << wp.position.position.z << ","
          << wp.yaw << ","
          << wp.velocity.x << ","
          << wp.velocity.y << ","
          << wp.velocity.z << ","
          << wp.acceleration.x << ","
          << wp.acceleration.y << ","
          << wp.acceleration.z << "\n";
    }

    ofs.close();
    RCLCPP_INFO(this->get_logger(), "CSV 已写入: %s", CSV_PATH);
    return true;
  }

  int64_t count_csv_data_lines()
  {
    std::ifstream ifs(CSV_PATH);
    if (!ifs.is_open()) {
      return -1;
    }
    int64_t count = 0;
    std::string line;
    bool first = true;
    while (std::getline(ifs, line)) {
      if (first) {
        first = false;  // 跳过表头
        continue;
      }
      if (!line.empty()) {
        ++count;
      }
    }
    return count;
  }

  void start_timeout_timer()
  {
    if (timeout_timer_) {
      timeout_timer_->cancel();
    }
    timeout_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(TIMEOUT_SEC),
      std::bind(&WaypointReceiverNode::on_timeout, this)
    );
  }

  void on_timeout()
  {
    int64_t received = static_cast<int64_t>(waypoint_map_.size());
    RCLCPP_WARN(
      this->get_logger(),
      "接收超时（%g 秒），已收到 %ld/%ld 个航点，数据已丢弃",
      TIMEOUT_SEC, received, expected_total_);

    std::string msg = "超时，已接收 " + std::to_string(received) +
      "/" + std::to_string(expected_total_) + " 个航点，数据已丢弃";
    publish_response(false, msg);
    reset();
  }

  void publish_response(bool success, const std::string & message)
  {
    WaypointDataResponse response;
    response.success = success;
    response.message = message;
    pub_->publish(response);
  }

  void reset()
  {
    waypoint_map_.clear();
    expected_total_ = 0;
    if (timeout_timer_) {
      timeout_timer_->cancel();
      timeout_timer_.reset();
    }
    RCLCPP_INFO(this->get_logger(), "缓存已清空，等待下一批航点数据...");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointReceiverNode>());
  rclcpp::shutdown();
  return 0;
}
