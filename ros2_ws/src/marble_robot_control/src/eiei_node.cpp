// Keyboard teleop for ROS 2 to control motors (cmd_vel) and an arm via micro-ROS
// Publishes:
//  - geometry_msgs/msg/Twist on "/cmd_vel" (linear.x, angular.z)
//  - std_msgs/msg/Int8 on "/arm_cmd" (simple command codes)
// Key mapping (simple and Pico-friendly):
//  - w/s: forward/backward
//  - a/d: rotate left/right
//  - space: stop (zero twist)
//  - o: gripper open  (1)
//  - p: gripper close (2)
//  - u: arm up        (3)
//  - m: arm down      (4)
//  - q: quit

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <iostream>
#include <string>
#include <chrono>
#include <unordered_map>
#include <type_traits>


class KeyboardPublisher : public rclcpp::Node
{
public:
    KeyboardPublisher() : Node("keyboard_publisher")
    {
        // Parameters (speeds and publish rate)
        linear_speed_ = this->declare_parameter<double>("linear_speed", 0.3);
        angular_speed_ = this->declare_parameter<double>("angular_speed", 1.0);
        pub_rate_hz_   = this->declare_parameter<int>("publish_rate_hz", 50);

        // Create publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        arm_pub_     = this->create_publisher<std_msgs::msg::Int8>("arm_cmd", 10);

        // Create timer to check for keyboard input (fast polling)
        input_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&KeyboardPublisher::check_keyboard, this)
        );

        // Create timer to publish the current cmd_vel at a steady rate
        auto pub_period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1, pub_rate_hz_)));
        pub_timer_ = this->create_wall_timer(
            pub_period,
            std::bind(&KeyboardPublisher::publish_cmd_vel, this)
        );
        
        // Store original terminal settings
        tcgetattr(STDIN_FILENO, &original_termios_);
        
        // Set terminal to non-canonical mode with no echo
        struct termios new_termios = original_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        new_termios.c_cc[VTIME] = 0;  // No timeout
        new_termios.c_cc[VMIN] = 1;   // Read at least 1 character
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        
        // Set stdin to non-blocking
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        
        RCLCPP_INFO(this->get_logger(), "Keyboard teleop started. Use WASD to move, space to stop, q to quit.");
        std::cout << "Mappings: w/s forward/back, a/d rotate, space stop, o/p gripper open/close, u/m arm up/down, q quit." << std::endl;
    }
    
    ~KeyboardPublisher()
    {
        restore_terminal();
    }

private:
    void check_keyboard()
    {
        char key;
        // Read all available characters
        while (read(STDIN_FILENO, &key, 1) > 0) {
            if (key == 'q') {
                RCLCPP_INFO(this->get_logger(), "Quit command received");
                restore_terminal();
                rclcpp::shutdown();
                return;
            }

            handle_key(key);
        }
    }
    
    void handle_key(char key)
    {
        // Movement
        switch (key) {
            case 'w':
            case 'W':
                twist_.linear.x = linear_speed_;
                break;
            case 's':
            case 'S':
                twist_.linear.x = -linear_speed_;
                break;
            case 'a':
            case 'A':
                twist_.angular.z = angular_speed_;
                break;
            case 'd':
            case 'D':
                twist_.angular.z = -angular_speed_;
                break;
            case ' ':  // stop
                twist_.linear.x = 0.0;
                twist_.angular.z = 0.0;
                break;
            default:
                break;
        }

        // Arm commands (publish once per keypress)
        std_msgs::msg::Int8 arm_msg;
        bool send_arm = false;
        switch (key) {
            case 'o':
            case 'O':
                arm_msg.data = 1; // gripper open
                send_arm = true;
                break;
            case 'p':
            case 'P':
                arm_msg.data = 2; // gripper close
                send_arm = true;
                break;
            case 'u':
            case 'U':
                arm_msg.data = 3; // arm up
                send_arm = true;
                break;
            case 'm':
            case 'M':
                arm_msg.data = 4; // arm down
                send_arm = true;
                break;
            default:
                break;
        }

        if (send_arm) {
            arm_pub_->publish(arm_msg);
            RCLCPP_INFO(this->get_logger(), "arm_cmd: %d", arm_msg.data);
        }

        // Mark movement updated for logging
        last_move_time_ = std::chrono::steady_clock::now();
    }

    void publish_cmd_vel()
    {
        cmd_vel_pub_->publish(twist_);
        // Log sparsely when movement changes or every 1s while moving
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time_).count() > 1000) {
            if (twist_.linear.x != 0.0 || twist_.angular.z != 0.0) {
                RCLCPP_INFO(this->get_logger(), "cmd_vel: lin %.2f ang %.2f", twist_.linear.x, twist_.angular.z);
                last_log_time_ = now;
            }
        }
    }

    void restore_terminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
    }
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr arm_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr input_timer_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    struct termios original_termios_;
    
    // Current command and timing
    geometry_msgs::msg::Twist twist_{};
    double linear_speed_ {0.3};
    double angular_speed_ {1.0};
    int pub_rate_hz_ {50};
    std::chrono::steady_clock::time_point last_move_time_ {std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point last_log_time_ {std::chrono::steady_clock::now()};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<KeyboardPublisher>();
    
    try 
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}