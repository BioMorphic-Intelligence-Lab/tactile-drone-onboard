#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class Converter : public rclcpp::Node
{
public:
    Converter(std::string topic, uint n_joints)
        : Node("converter"), _TOPIC(topic), _N_JOINTS(n_joints)
    {

      /* Init all the class members */
      this->_joint_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
        topic, rclcpp::SensorDataQoS(), std::bind(&Converter::_joint_callback, this, std::placeholders::_1));

    }
private:
  const std::string _TOPIC;
  const uint _N_JOINTS;

  rclcpp::Time start;

  void _joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_subscriber;
};
