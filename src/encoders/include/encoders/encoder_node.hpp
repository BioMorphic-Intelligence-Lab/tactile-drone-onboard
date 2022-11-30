#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/stamped_float32.hpp"
#include "cui_devices_amt10x.hpp"

using namespace std::chrono_literals;

class EncoderNode : public rclcpp::Node
{
  public:
    EncoderNode(uint16_t ppr, uint8_t encoderA, uint8_t encoderB, uint8_t encoderX)
    : Node("encoder_node"), _encoder(ppr, encoderA, encoderB, encoderX)
    {
      this->_publisher = this->create_publisher<interfaces::msg::StampedFloat32>("angle", 10);
      this->_timer = this->create_wall_timer(100ms, std::bind(&EncoderNode::timer_callback, this));

      this->_encoder.begin();
    }

  private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<interfaces::msg::StampedFloat32>::SharedPtr _publisher;

    Cui_Devices_Amt10x _encoder;

};