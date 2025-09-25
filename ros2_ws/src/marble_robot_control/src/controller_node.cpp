#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>

// ControllerNode: translates high-level cmd_vel and arm commands into low-level
// motor speed targets and servo positions. PID and kinematics are intentionally
// left as placeholders for you to implement.

struct WheelCmd {
  float left_speed = 0.0f;   // m/s or normalized [-1,1]
  float right_speed = 0.0f;  // m/s or normalized [-1,1]
};

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("controller_node") {
    // Parameters
    wheel_separation_ = this->declare_parameter<double>("wheel_separation", 0.18);
    wheel_radius_     = this->declare_parameter<double>("wheel_radius", 0.033);
    max_speed_        = this->declare_parameter<double>("max_wheel_speed", 0.8); // m/s

    // Subscriptions
    using std::placeholders::_1;
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ControllerNode::on_cmd_vel, this, _1));

    // Encoder feedback (example). Replace with your actual encoder topic/type
    // e.g., sensor_msgs/JointState with names ["wheel_left", "wheel_right"].
    enc_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "wheel_states", 10, std::bind(&ControllerNode::on_encoder, this, _1));

    // Arm/gripper commands (Int8 codes as defined by teleop)
    arm_cmd_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "arm_cmd", 10, std::bind(&ControllerNode::on_arm_cmd, this, _1));

    // Publishers for low-level commands
    // Example: normalized wheel speed targets [-1, 1] or rad/s.
    left_wheel_pub_  = this->create_publisher<std_msgs::msg::Float32>("left_wheel_cmd", 10);
    right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float32>("right_wheel_cmd", 10);

    // Example: servo target position (degrees or normalized [0,1]).
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float32>("gripper_target", 10);
    arm_pub_     = this->create_publisher<std_msgs::msg::Float32>("arm_target", 10);

    // Control timer (e.g., 100 Hz)
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&ControllerNode::control_step, this));

    RCLCPP_INFO(this->get_logger(), "ControllerNode started (PID placeholders ready)");
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg) {
    // Map Twist to wheel targets using differential drive kinematics
    double v = msg.linear.x;     // m/s
    double w = msg.angular.z;    // rad/s

    // Inverse kinematics for differential drive
    double v_left  = v - (w * wheel_separation_ / 2.0);
    double v_right = v + (w * wheel_separation_ / 2.0);

    // Optional clamp to max speed
    v_left  = std::clamp(v_left,  -max_speed_, max_speed_);
    v_right = std::clamp(v_right, -max_speed_, max_speed_);

    // Store as setpoints; convert to wheel angular velocity if needed
    target_left_speed_  = static_cast<float>(v_left);
    target_right_speed_ = static_cast<float>(v_right);
  }

  void on_encoder(const sensor_msgs::msg::JointState & msg) {
    //unfinished
    // - Extract wheel velocities from msg.velocity based on names
    // - Update current_left_speed_ / current_right_speed_
    // - Your PID will run in control_step() to compute motor outputs
    (void)msg; // suppress unused warning for now
  }

  void on_arm_cmd(const std_msgs::msg::Int8 & msg) {
    // Simple mapping to target servo positions; you can replace with your own logic
    switch (msg.data) {
      case 1: // gripper open
        gripper_target_ = 1.0f; // e.g., 1.0 = fully open
        break;
      case 2: // gripper close
        gripper_target_ = 0.0f; // e.g., 0.0 = fully closed
        break;
      case 3: // arm up
        arm_target_ = 1.0f;
        break;
      case 4: // arm down
        arm_target_ = 0.0f;
        break;
      default:
        break;
    }
  }

  void control_step() {
    // PID
    // - Compute error = target - current for each wheel
    // - Update integral and derivative terms with dt from a steady clock
    // - Apply output limits and anti-windup
    // - Send outputs to left_wheel_pub_ and right_wheel_pub_
    // unfinished

    // Placeholder: publish target speeds directly (open-loop)
    std_msgs::msg::Float32 l_msg, r_msg;
    l_msg.data = target_left_speed_;
    r_msg.data = target_right_speed_;
    left_wheel_pub_->publish(l_msg);
    right_wheel_pub_->publish(r_msg);

    // Publish servo targets (you may use a different message/channel)
    std_msgs::msg::Float32 g_msg, a_msg;
    g_msg.data = gripper_target_;
    a_msg.data = arm_target_;
    gripper_pub_->publish(g_msg);
    arm_pub_->publish(a_msg);
  }

  // Parameters
  double wheel_separation_ = 0.18;
  double wheel_radius_ = 0.033; // currently unused in placeholder
  double max_speed_ = 0.8;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr enc_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr arm_cmd_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr arm_pub_;

  // Control state
  float target_left_speed_  = 0.0f;
  float target_right_speed_ = 0.0f;
  float current_left_speed_  = 0.0f; // set from encoders
  float current_right_speed_ = 0.0f; // set from encoders
  float gripper_target_ = 0.0f;
  float arm_target_     = 0.0f;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
