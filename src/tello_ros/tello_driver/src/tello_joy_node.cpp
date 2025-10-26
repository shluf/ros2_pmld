#include "tello_joy_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tello_msgs/srv/tello_action.hpp"

namespace tello_joy
{

  TelloJoyNode::TelloJoyNode(const rclcpp::NodeOptions &options) :
    Node("tello_joy", options)
  {
    using std::placeholders::_1;

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&TelloJoyNode::joy_callback, this, _1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    tello_client_ = create_client<tello_msgs::srv::TelloAction>("tello_action");

    (void) joy_sub_;
  }

  TelloJoyNode::~TelloJoyNode()
  {}


  void TelloJoyNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    // Validate incoming Joy message sizes to avoid out-of-range access
    const auto buttons_sz = joy_msg->buttons.size();
    const auto axes_sz = joy_msg->axes.size();

    const int max_button_index = std::max(joy_button_takeoff_, joy_button_land_);
    const int max_axis_index = std::max(std::max(joy_axis_throttle_, joy_axis_strafe_), std::max(joy_axis_vertical_, joy_axis_yaw_));

    if (static_cast<int>(buttons_sz) <= max_button_index) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Joy message has %zu buttons; expected index up to %d. Ignoring.",
                           buttons_sz, max_button_index);
      return;
    }

    if (static_cast<int>(axes_sz) <= max_axis_index) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Joy message has %zu axes; expected index up to %d. Ignoring.",
                           axes_sz, max_axis_index);
      return;
    }

    if (joy_msg->buttons[joy_button_takeoff_]) {
      auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
      request->cmd = "takeoff";
      tello_client_->async_send_request(request);
    } else if (joy_msg->buttons[joy_button_land_]) {
      auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
      request->cmd = "land";
      tello_client_->async_send_request(request);
    } else {
      geometry_msgs::msg::Twist twist_msg;
      twist_msg.linear.x = joy_msg->axes[joy_axis_throttle_];
      twist_msg.linear.y = joy_msg->axes[joy_axis_strafe_];
      twist_msg.linear.z = joy_msg->axes[joy_axis_vertical_];
      twist_msg.angular.z = joy_msg->axes[joy_axis_yaw_];
      cmd_vel_pub_->publish(twist_msg);
    }
  }

} // namespace tello_joy

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tello_joy::TelloJoyNode)