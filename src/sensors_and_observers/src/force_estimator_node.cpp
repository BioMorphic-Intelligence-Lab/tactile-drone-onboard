#include "force_estimator_node.hpp"

void ForceEstimatorNode::_estimate_force(float *pos, float *vel,
                                         float tendon_force)
{ 
    /* We're computing the external force based on the systems dynamic model, 
     * however since we assume the motion of the overall system is slow the 
     * inertia, corioles and damping contributions are negligible. Thus the
     * external force is computed via:
     *      f_ee = PI(J_ee(q)^T) (G(q) + K(q) - A*tau)
     * where
     *      q is the current arm configuration
     *      PI is the pseudo inverse
     *      J_ee(q) is the ee-Jacobian
     *      G(q) is the gravity contribution
     *      K(q) is the stiffness contribution
     *      A is the input mapping matrix
     *      tau is the force applied on the tendon
     * */

    std::vector<Eigen::Matrix3f> Rs = this->_get_rs(pos);  
    std::vector<Eigen::Vector3f> coms = this->_get_coms(Rs);

    std::cout << coms.at(0)(0) << " " << coms.at(0)(1) << " " << coms.at(0)(2) << std::endl;

}

std::vector<Eigen::Vector3f> ForceEstimatorNode::_get_coms(std::vector<Eigen::Matrix3f> rs)
{
    std::vector<Eigen::Vector3f> coms;
    for(uint i = 0; i < this->_l.size(); i++)
    {
        /* Init translation vector */
        Eigen::Vector3f com = Eigen::Vector3f::Zero();
        
        /* Add the translation of all previous links */
        for(uint j = 0; j < i; j++)
        {
            Eigen::Vector3f bar;
            bar << 0, 0, - this->_l[j];
            com += rs.at(j) * bar;
        }

        /* Add half bar length translation to the CoM */
        Eigen::Vector3f half_bar;
        half_bar << 0, 0, - 0.5 * this->_l[i];
        com+= rs.at(i) * half_bar;

        /* Push back into vector*/
        coms.push_back(com);
    }
    /* Return collection of coms */
    return coms;
}

Eigen::Matrix3f ForceEstimatorNode::_rot_x(double theta)
{
    float sT = sin(theta);
    float cT = cos(theta);

    Eigen::Matrix3f rot;
    rot << 1,  0,   0,
           0, cT, -sT,
           0, sT,  cT;

    return rot;
}

Eigen::Matrix3f ForceEstimatorNode::_rot_y(double theta)
{
    float sT = sin(theta);
    float cT = cos(theta);

    Eigen::Matrix3f rot;
    rot <<  cT,  0, sT,
             0,  1,  0,
           -sT,  0, cT;

    return rot;
}

std::vector<Eigen::Matrix3f> ForceEstimatorNode::_get_rs(float *pos)
{
    /* Init empty vector */
    std::vector<Eigen::Matrix3f> Rs;
    
    /* Add the rotation of the first matrix */
    Rs.push_back(this->_rot_x(pos[0]));

    /* For each link, push back the current rotation matrix,
     * which describes its attitude with respect to the base.
     * Purposefully starting at i = 1 sinze we're doing the first
     * joint separately before since it has a different rotation axis */
    for(uint i = 1; i < this->_k.size(); i++)
    {
        Rs.push_back(Rs.back() * this->_rot_y(pos[i]));
    }

    return Rs;
}

void ForceEstimatorNode::_run()
{
    for(;;)
    {
        /* Extract encoder position and velocity */
        float pos[4] = {0.0, 0.0, 0.0, 0.0}, vel[4] = {0.0, 0.0, 0.0, 0.0};
        this->_encoders->getPosition(pos);
        this->_encoders->getVelocity(vel);

        /* Get the currently applied tendon force */
        float tendon_force = 0.0; // TODO actually get the force

        /* Compute the estimated contact force and store in class var */
        this->_estimate_force(pos, vel, tendon_force);

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
    message.header.frame_id = "ee";

    /* Get the currently estimated force */
    std::vector<double> force(3);
    this->_get_force(force);

    /* Pack the force into the message.
     * The torque is assumed to be zero, thus
     * doesn't need to be filled explicitely */
    message.wrench.force.x = force.at(0);
    message.wrench.force.y = force.at(1);
    message.wrench.force.z = force.at(2);
    
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
