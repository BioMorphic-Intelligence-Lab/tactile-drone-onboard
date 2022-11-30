#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "cui_devices_amt10x.hpp"

using namespace std::chrono_literals;

class EncoderNode : public rclcpp::Node
{
  public:
    EncoderNode(uint16_t ppr, uint8_t encoderA, uint8_t encoderB, uint8_t encoderX)
    : Node("encoder_node"),
     _NAMES{"base_joint_pitch","base_joint_roll", "middle_joint", "top_joint"},
     _encoder(ppr, encoderA, encoderB, encoderX)
    {
      this->_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
      this->_timer = this->create_wall_timer(50ms, std::bind(&EncoderNode::timer_callback, this));

      this->_encoder.begin();
    }

  private:
    const std::string _NAMES[4];
    void timer_callback();

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _publisher;

    Cui_Devices_Amt10x _encoder;

};