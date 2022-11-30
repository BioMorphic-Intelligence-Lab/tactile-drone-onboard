#include "encoder_node.hpp"

void EncoderNode::timer_callback()
{
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.header.frame_id = this->get_name();
    message.position = {this->_encoder.getPosition(), 0, 0, 0};
    for(int i = 0; i < 4 ;i++) message.name.push_back(this->_NAMES[i]);

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