#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8.hpp>

class ReceiverNode : public rclcpp::Node {
public:
  ReceiverNode() : Node("receiver_node") {
    using std::placeholders::_1;
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&ReceiverNode::on_cmd_vel, this, _1));
    arm_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "arm_cmd", 10, std::bind(&ReceiverNode::on_arm_cmd, this, _1));
    RCLCPP_INFO(this->get_logger(), "ReceiverNode started. Subscribing to /cmd_vel and /arm_cmd");
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg) {
    RCLCPP_INFO(this->get_logger(), "cmd_vel received: lin %.2f ang %.2f",
                msg.linear.x, msg.angular.z);
  }

  void on_arm_cmd(const std_msgs::msg::Int8 & msg) {
    const char *label = "unknown";
    switch (msg.data) {
      case 1: label = "gripper_open"; break;
      case 2: label = "gripper_close"; break;
      case 3: label = "arm_up"; break;
      case 4: label = "arm_down"; break;
      default: break;
    }
    RCLCPP_INFO(this->get_logger(), "arm_cmd received: %d (%s)", msg.data, label);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr arm_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReceiverNode>());
  rclcpp::shutdown();
  return 0;
}
