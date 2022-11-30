#include "encoder_node.hpp"

void EncoderNode::timer_callback()
{
    auto message = interfaces::msg::StampedFloat32();
    message.header.stamp = this->now();
    message.header.frame_id = this->get_name();
    message.val = this->_encoder.getPosition();
    RCLCPP_DEBUG(this->get_logger(), "Publishing: '%f'", message.val);
    this->_publisher->publish(message);
}

int main(int argc, char * argv[])
{
    uint16_t ppr = 2048;
    uint8_t encoderA = 2, encoderB = 3, encoderX = 0;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncoderNode>(ppr, encoderA, encoderB, encoderX));
    rclcpp::shutdown();
    return 0;
}