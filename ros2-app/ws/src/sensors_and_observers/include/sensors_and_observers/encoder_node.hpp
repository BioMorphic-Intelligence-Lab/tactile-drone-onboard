#include <chrono>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "custom_interfaces/srv/get_joint_state.hpp"
#include "cui_devices_amt10x.hpp"

using namespace std::chrono_literals;

class EncoderNode : public rclcpp::Node
{
public:
  EncoderNode();

private:
  const std::string _NAMES[4];
  void timer_callback();

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _publisher;
  rclcpp::Service<custom_interfaces::srv::GetJointState>::SharedPtr _server;

  void _get_joint_state(const std::shared_ptr<custom_interfaces::srv::GetJointState::Request> request,
                              std::shared_ptr<custom_interfaces::srv::GetJointState::Response> response);

  Cui_Devices_Amt10x *_encoders;
};