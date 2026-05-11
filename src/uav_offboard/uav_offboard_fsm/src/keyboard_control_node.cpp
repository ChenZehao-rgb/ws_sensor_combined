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
                    "Keyboard control ready\n"
                    "  Shift+S: SELF_CHECK  Shift+T: WAIT_TASK_ENABLE_AUTH  Shift+R: NAV_TO_TASK_DOM\n"
                    "  Shift+U: UAV_SEARCH_TARGS  Shift+A: SEARCH_ADJUST_AUTO  Shift+Z: SEARCH_ADJUST_MANUAL\n"
                    "  Shift+P: TARG_GOT  Shift+D: TARG_READY  Shift+G: UAV_POSE_ADAP\n"
                    "  Shift+C: SAMP_ADJUST_AUTO  Shift+V: SAMP_ADJUST_MANUAL  Shift+O: ARM_CONFIG_PREP\n"
                    "  Shift+L: SAMPL_OPERA  Shift+E: UAV_PRE_BACK_HOME  Shift+B: BACK_HOME\n"
                    "  Shift+Q: TASK_TERM  Shift+X: NO  Shift+Y: CONFIRM");
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
            publishCommand("SELF_CHECK");
            break;
        case 'T':
            publishCommand("WAIT_TASK_ENABLE_AUTH");
            break;
            case 'R':
                publishCommand("NAV_TO_TASK_DOM");
                break;
            case 'U':
                publishCommand("UAV_SEARCH_TARGS");
                break;
            case 'A':
                publishCommand("SEARCH_ADJUST_AUTO");
                break;
        case 'Z':
            publishCommand("SEARCH_ADJUST_MANUAL");
            break;
            case 'P':
                publishCommand("TARG_GOT");
                break;
            case 'D':
                publishCommand("TARG_READY");
                break;
            case 'G':
                publishCommand("UAV_POSE_ADAP");
                break;
        case 'C':
            publishCommand("SAMP_ADJUST_AUTO");
            break;
        case 'V':
            publishCommand("SAMP_ADJUST_MANUAL");
            break;
        case 'O':
            publishCommand("ARM_CONFIG_PREP");
            break;
        case 'L':
            publishCommand("SAMPL_OPERA");
            break;
        case 'E':
            publishCommand("UAV_PRE_BACK_HOME");
            break;
            case 'B':
                publishCommand("BACK_HOME");
                break;
            case 'Q':
                publishCommand("TASK_TERM");
                break;
        case 'X':
            publishCommand("NO");
            break;
        case 'Y':
            publishCommand("CONFIRM");
            break;
        default:
            break;
        }
    }
}

void KeyboardControlNode::publishCommand(const std::string &state) {
    std_msgs::msg::String msg;
    msg.data = state;
    command_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Command published | %s", state.c_str());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
