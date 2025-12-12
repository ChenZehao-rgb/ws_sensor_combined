#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

class KeyboardControlNode : public rclcpp::Node {
  public:
    KeyboardControlNode() : rclcpp::Node("uav_keyboard_control") {
        command_pub_ = this->create_publisher<std_msgs::msg::String>("/uav_offboard_fsm/control_command", 10);
        startKeyboardListener();
    }

    ~KeyboardControlNode() override { stopKeyboardListener(); }

  private:
    void startKeyboardListener();
    void stopKeyboardListener();
    void keyboardLoop();
    void publishCommand(const std::string &state);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    std::thread keyboard_thread_;
    std::atomic<bool> stop_keyboard_{false};
    termios original_termios_{};
    bool termios_configured_{false};
};

void KeyboardControlNode::startKeyboardListener() {
    if (!isatty(STDIN_FILENO)) {
        RCLCPP_WARN(this->get_logger(), "STDIN is not a TTY; keyboard control disabled.");
        return;
    }
    if (tcgetattr(STDIN_FILENO, &original_termios_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get terminal attributes; keyboard control disabled.");
        return;
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1; // 100 ms timeout
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set terminal to raw mode; keyboard control disabled.");
        return;
    }

    termios_configured_ = true;
    stop_keyboard_.store(false);
    keyboard_thread_ = std::thread(&KeyboardControlNode::keyboardLoop, this);
    RCLCPP_INFO(this->get_logger(),
                "Keyboard control enabled: Shift+S (self-check), Shift+T (takeoff), Shift+G (goto point), Shift+R (trajectory tracking), Shift+C (circle tracking).");
}

void KeyboardControlNode::stopKeyboardListener() {
    stop_keyboard_.store(true);
    if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
    }
    if (termios_configured_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        termios_configured_ = false;
    }
}

void KeyboardControlNode::keyboardLoop() {
    while (rclcpp::ok() && !stop_keyboard_.load()) {
        char ch = 0;
        ssize_t bytes = read(STDIN_FILENO, &ch, 1);
        if (bytes <= 0) {
            continue;
        }

        switch (ch) {
        case 'S':
            publishCommand("SELFCHECK");
            break;
        case 'T':
            publishCommand("TAKEOFF");
            break;
        case 'G':
            publishCommand("GOTOPOINT");
            break;
        case 'R':
            publishCommand("TRAJECTORY_TRACKING");
            break;
        case 'C':
            publishCommand("CIRCLE_TRACKING");
        default:
            break;
        }
    }
}

void KeyboardControlNode::publishCommand(const std::string &state) {
    std_msgs::msg::String msg;
    msg.data = state;
    command_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing control command: %s", state.c_str());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
