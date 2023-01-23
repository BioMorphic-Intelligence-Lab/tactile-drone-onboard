#include "force_estimator_node.hpp"

void ForceEstimatorNode::joint_timer_callback()
{
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.header.frame_id = this->get_name();
    
    float pos[4] = {}, vel[4] = {0.0,0.0,0.0,0.0};
    this->_encoders->getPosition(pos);
    this->_encoders->getVelocity(vel);

    for(int i = 0; i < 4; i++)
    {
        message.position.push_back(i == 3 ? -pos[i] : pos[i]);
        message.velocity.push_back(vel[i]);
        message.name.push_back(this->_NAMES[i]);
    }

    this->_joint_publisher->publish(message);
}

void ForceEstimatorNode::force_timer_callback()
{
    auto message = geometry_msgs::msg::WrenchStamped();
    message.header.stamp = this->now();
    message.header.frame_id = this->get_name();

    // TODO extract the actual contact force estimate
    message.wrench.force.x = 0.0;
    message.wrench.force.y = 0.0;
    message.wrench.force.z = 0.0;
    
    message.wrench.torque.x = 0.0;
    message.wrench.torque.y = 0.0;
    message.wrench.torque.z = 0.0;

    this->_force_publisher->publish(message);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<ForceEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
