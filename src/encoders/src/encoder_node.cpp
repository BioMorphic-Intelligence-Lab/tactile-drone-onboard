#include "encoder_node.hpp"

void EncoderNode::timer_callback()
{
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.header.frame_id = this->get_name();
    for(int i = 0; i < 4 ;i++)
    {
        message.position.push_back(i == 3 ? -this->_encoders[i]->getPosition() : this->_encoders[i]->getPosition());
        message.name.push_back(this->_NAMES[i]);
    }

    this->_publisher->publish(message);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncoderNode>());
    rclcpp::shutdown();
    return 0;
}