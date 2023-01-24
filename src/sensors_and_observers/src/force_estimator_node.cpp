#include "force_estimator_node.hpp"

void ForceEstimatorNode::_estimate_force(float *pos, float *vel)
{
    
}

void ForceEstimatorNode::_run()
{
    for(;;)
    {
        /* Extract encoder position and velocity */
        float pos[4] = {0.0, 0.0, 0.0, 0.0}, vel[4] = {0.0, 0.0, 0.0, 0.0};
        this->_encoders->getPosition(pos);
        this->_encoders->getVelocity(vel);

        /* Compute the estimated contact force and store in class var */
        this->_estimate_force(pos, vel);

    }
}

void ForceEstimatorNode::_begin()
{
    /* Start the thread.*/
    this->_executer =  new std::thread(&ForceEstimatorNode::_run, this);
    this->_executer->detach();
}

void ForceEstimatorNode::_get_force(std::vector<double> & force)
{
    assert(force.size() == 3);

    for(uint i = 0; i < force.size(); i++)
    {
        force.at(i) = this->_force.at(i);
    }
}

void ForceEstimatorNode::_joint_timer_callback()
{
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.header.frame_id = this->get_name();
    
    float pos[4] = {0.0, 0.0, 0.0, 0.0}, vel[4] = {0.0,0.0,0.0,0.0};
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

void ForceEstimatorNode::_force_timer_callback()
{
    auto message = geometry_msgs::msg::WrenchStamped();
    message.header.stamp = this->now();
    message.header.frame_id = this->get_name();

    /* Get the currently estimated force */
    std::vector<double> force(3);
    this->_get_force(force);

    /* Pack the force into the message.
     * The torque is assumed to be zero, thus
     * doesn't need to be filled explicitely */
    message.wrench.force.x = force.at(0);
    message.wrench.force.y = force.at(0);
    message.wrench.force.z = force.at(0);
    
    /* Actually publish the message */
    this->_force_publisher->publish(message);
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<ForceEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
