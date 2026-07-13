/*********************************************************************
 * joystick_servo_example.cpp  (集成 DH AG95 夹爪控制)
 *  - B 键：闭爪 (position = 0)
 *  - X 键：开爪 (position = 100)
 *  - 采用边沿检测，仅在按下瞬间发一次命令
 *********************************************************************/

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <dh_gripper_driver/msg/gripper_ctrl.hpp>        // ★ 新增：夹爪消息类型
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>

#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

// 话题名
const std::string JOY_TOPIC     = "/joy";
const std::string TWIST_TOPIC   = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC   = "/servo_node/delta_joint_cmds";
const std::string GRIPPER_TOPIC = "/gripper/ctrl";         // ★ 新增
const std::string EEF_FRAME_ID  = "tool0";
const std::string BASE_FRAME_ID = "base_link";

// 夹爪默认参数
const double GRIPPER_OPEN_POS  = 100.0;   // AG95 全开
const double GRIPPER_CLOSE_POS = 0.0;     // AG95 全闭
const double GRIPPER_FORCE     = 50.0;    // 夹持力 (%)
const double GRIPPER_SPEED     = 50.0;    // 速度 (%)

// XBOX 1 手柄映射
enum Axis {
  LEFT_STICK_X = 0, LEFT_STICK_Y = 1, LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3, RIGHT_STICK_Y = 4, RIGHT_TRIGGER = 5,
  D_PAD_X = 6, D_PAD_Y = 7
};
enum Button {
  A = 0, B = 1, X = 2, Y = 3,
  LEFT_BUMPER = 4, RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6, MENU = 7, HOME = 8,
  LEFT_STICK_CLICK = 9, RIGHT_STICK_CLICK = 10
};

std::map<Axis, double> AXIS_DEFAULTS   = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;

/** 把手柄轴/按键 转成 Twist 或 JointJog
 *  返回 true -> 发 Twist；false -> 发 JointJog
 *  注意：B / X 已从 joint-jog 触发条件中移除，改作夹爪控制
 */
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint)
{
  // 只保留 A、Y 和 D_PAD 作为点动触发条件
  if (buttons[A] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
  {
    joint->joint_names.push_back("joint_1");
    joint->velocities.push_back(axes[D_PAD_X]);
    joint->joint_names.push_back("joint_2");
    joint->velocities.push_back(axes[D_PAD_Y]);

    joint->joint_names.push_back("joint_6");
    joint->velocities.push_back(buttons[Y] - buttons[A]);
    return false;
  }

  // Twist 命令
  twist->twist.linear.z = axes[RIGHT_STICK_Y];
  twist->twist.linear.y = axes[RIGHT_STICK_X];

  double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
  double lin_x_left  =  0.5 * (axes[LEFT_TRIGGER]  - AXIS_DEFAULTS.at(LEFT_TRIGGER));
  twist->twist.linear.x = lin_x_right + lin_x_left;

  twist->twist.angular.y = axes[LEFT_STICK_Y];
  twist->twist.angular.x = axes[LEFT_STICK_X];

  double roll_positive = buttons[RIGHT_BUMPER];
  double roll_negative = -1 * (buttons[LEFT_BUMPER]);
  twist->twist.angular.z = roll_positive + roll_negative;

  return true;
}

void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons)
{
  if (buttons[CHANGE_VIEW] && frame_name == EEF_FRAME_ID)
    frame_name = BASE_FRAME_ID;
  else if (buttons[MENU] && frame_name == BASE_FRAME_ID)
    frame_name = EEF_FRAME_ID;
}

namespace moveit_servo
{
class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options),
      frame_to_publish_(BASE_FRAME_ID),
      prev_btn_x_(0),
      prev_btn_b_(0)
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    twist_pub_   = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    joint_pub_   = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    // ★ 新增：夹爪命令发布器
    gripper_pub_ = this->create_publisher<dh_gripper_driver::msg::GripperCtrl>(
        GRIPPER_TOPIC, rclcpp::SystemDefaultsQoS());

    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  ~JoyToServoPub() override {}

  // ★ 新增：根据 position 发一条夹爪命令
  void publishGripperCmd(double position)
  {
    auto msg = std::make_unique<dh_gripper_driver::msg::GripperCtrl>();
    msg->initialize = false;
    msg->position   = position;
    msg->force      = GRIPPER_FORCE;
    msg->speed      = GRIPPER_SPEED;
    gripper_pub_->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "Gripper cmd sent: position = %.1f", position);
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // ★ 夹爪控制（边沿检测：仅在 0->1 翻转时发一次）
    int cur_btn_x = (msg->buttons.size() > static_cast<size_t>(X)) ? msg->buttons[X] : 0;
    int cur_btn_b = (msg->buttons.size() > static_cast<size_t>(B)) ? msg->buttons[B] : 0;

    if (cur_btn_x && !prev_btn_x_)
      publishGripperCmd(GRIPPER_OPEN_POS);    // X 键 -> 开爪
    if (cur_btn_b && !prev_btn_b_)
      publishGripperCmd(GRIPPER_CLOSE_POS);   // B 键 -> 闭爪

    prev_btn_x_ = cur_btn_x;
    prev_btn_b_ = cur_btn_b;

    // —— 下面与原流程保持一致 ——
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    updateCmdFrame(frame_to_publish_, msg->buttons);

    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
    {
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp    = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
    else
    {
      joint_msg->header.stamp    = this->now();
      joint_msg->header.frame_id = "Link_3";
      joint_pub_->publish(std::move(joint_msg));
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<dh_gripper_driver::msg::GripperCtrl>::SharedPtr gripper_pub_;   // ★
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  std::string frame_to_publish_;
  int prev_btn_x_;    // ★ 上一帧 X 键状态
  int prev_btn_b_;    // ★ 上一帧 B 键状态
};

}  // namespace moveit_servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoyToServoPub)

