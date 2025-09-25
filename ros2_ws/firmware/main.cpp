#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int8.h>

namespace
{
constexpr float kWheelSeparation = 0.18f;   // meters
constexpr float kMaxWheelSpeed   = 0.8f;    // meters per second
constexpr uint32_t kControlPeriodMs = 10U;  // 100 Hz control loop

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t control_timer;

rcl_publisher_t left_wheel_pub;
rcl_publisher_t right_wheel_pub;
rcl_publisher_t gripper_pub;
rcl_publisher_t arm_pub;

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t arm_cmd_sub;
std_msgs__msg__Int8 arm_cmd_msg;

std_msgs__msg__Float32 left_wheel_msg;
std_msgs__msg__Float32 right_wheel_msg;
std_msgs__msg__Float32 gripper_msg;
std_msgs__msg__Float32 arm_msg;

float target_left_speed  = 0.0f;
float target_right_speed = 0.0f;
float gripper_target     = 0.0f;
float arm_target         = 0.0f;

void error_loop()
{
  while (true)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

float clampf(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

#define RCCHECK(fn)                          \
  {                                          \
    rcl_ret_t temp_rc = fn;                  \
    if ((temp_rc != RCL_RET_OK))             \
    {                                        \
      error_loop();                          \
    }                                        \
  }

#define RCSOFTCHECK(fn)                      \
  {                                          \
    rcl_ret_t temp_rc = fn;                  \
    (void)temp_rc;                           \
  }

void cmd_vel_callback(const void * msgin)
{
  const auto * msg = static_cast<const geometry_msgs__msg__Twist *>(msgin);
  float v = static_cast<float>(msg->linear.x);
  float w = static_cast<float>(msg->angular.z);

  float v_left  = v - (w * kWheelSeparation * 0.5f);
  float v_right = v + (w * kWheelSeparation * 0.5f);

  target_left_speed  = clampf(v_left, -kMaxWheelSpeed, kMaxWheelSpeed);
  target_right_speed = clampf(v_right, -kMaxWheelSpeed, kMaxWheelSpeed);
}

void arm_cmd_callback(const void * msgin)
{
  const auto * msg = static_cast<const std_msgs__msg__Int8 *>(msgin);
  switch (msg->data)
  {
    case 1:  // gripper open
      gripper_target = 1.0f;
      break;
    case 2:  // gripper close
      gripper_target = 0.0f;
      break;
    case 3:  // arm up
      arm_target = 1.0f;
      break;
    case 4:  // arm down
      arm_target = 0.0f;
      break;
    default:
      break;
  }
}

void control_timer_callback(rcl_timer_t * timer, int64_t /*last_call_time*/)
{
  if (timer == nullptr)
  {
    return;
  }

  left_wheel_msg.data  = target_left_speed;
  right_wheel_msg.data = target_right_speed;
  gripper_msg.data     = gripper_target;
  arm_msg.data         = arm_target;

  RCSOFTCHECK(rcl_publish(&left_wheel_pub, &left_wheel_msg, nullptr));
  RCSOFTCHECK(rcl_publish(&right_wheel_pub, &right_wheel_msg, nullptr));
  RCSOFTCHECK(rcl_publish(&gripper_pub, &gripper_msg, nullptr));
  RCSOFTCHECK(rcl_publish(&arm_pub, &arm_msg, nullptr));

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

}  // namespace

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  delay(2000);  // Allow time for the serial session to settle
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
  RCCHECK(rclc_node_init_default(&node, "controller_node", "", &support));

  std_msgs__msg__Float32__init(&left_wheel_msg);
  std_msgs__msg__Float32__init(&right_wheel_msg);
  std_msgs__msg__Float32__init(&gripper_msg);
  std_msgs__msg__Float32__init(&arm_msg);

  geometry_msgs__msg__Twist__init(&cmd_vel_msg);
  std_msgs__msg__Int8__init(&arm_cmd_msg);

  RCCHECK(rclc_publisher_init_default(
      &left_wheel_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "left_wheel_cmd"));

  RCCHECK(rclc_publisher_init_default(
      &right_wheel_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "right_wheel_cmd"));

  RCCHECK(rclc_publisher_init_default(
      &gripper_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "gripper_target"));

  RCCHECK(rclc_publisher_init_default(
      &arm_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "arm_target"));

  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
      &arm_cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
      "arm_cmd"));

  RCCHECK(rclc_timer_init_default(
      &control_timer,
      &support,
      RCL_MS_TO_NS(kControlPeriodMs),
      control_timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &arm_cmd_sub, &arm_cmd_msg, &arm_cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  delay(1);
}
