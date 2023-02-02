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

    /* Extract the needed information from the current joint positions*/
    std::vector<Eigen::Matrix3f> Rs = this->_get_rs(pos);
    std::vector<Eigen::Vector3f> joints = this->_get_joint_locs(Rs);  
    std::vector<Eigen::Vector3f> coms = this->_get_coms(joints, Rs);

    /* Compute the various generalized force contributions */
    Eigen::Vector4f gravity_cont = this->_get_gravity_contribution(joints, coms, Rs);
    Eigen::Vector4f stiffness_cont = this->_get_stiffness_contribution(pos); 
    Eigen::Vector4f ctrl_cont = this->_get_ctrl_contribution(tendon_force);

    /* Compute the current EE-Jacobian */
    Eigen::MatrixXf J_EE = this->_get_ee_jacobian(pos); 

    /* Compute the force acting on the end effector */
    Eigen::Vector3f f = J_EE.transpose().completeOrthogonalDecomposition().pseudoInverse() 
                             * (gravity_cont + stiffness_cont - ctrl_cont);

    std::vector<float> force = {1, 0, 0};
    this->_set_force(force); 
}

Eigen::MatrixXf ForceEstimatorNode::_get_ee_jacobian(float * pos)
{
    /* Store all the sine and cosine value such that we don't have 
     * to re-compute them all the time */
    double s[4] = {sin(pos[0]), sin(pos[1]), sin(pos[2]), sin(pos[3])},
           c[4] = {cos(pos[0]), cos(pos[1]), cos(pos[2]), cos(pos[3])};

    /* Init the jacobian matrix */
    Eigen::MatrixXf J_ee(3, this->_k.size());

    /* Fill it with values */
    J_ee << // Effects on ee x
    this->_l[1]*s[0]*s[1]+this->_l[2]*s[0]*(c[2]*s[1]+c[1]*s[2])+this->_l[3]*s[0]*(c[1]*(c[3]*s[2]+c[2]*s[3])+s[1]*(c[2]*c[3]-s[2]*s[3])),
	-this->_l[1]*c[0]*c[1]-this->_l[2]*c[0]*(c[1]*c[2]-s[1]*s[2])-this->_l[3]*c[0]*(-s[1]*(c[3]*s[2]+c[2]*s[3])+c[1]*(c[2]*c[3]-s[2]*s[3])),
    -this->_l[2]*c[0]*(c[1]*c[2]-s[1]*s[2])-this->_l[3]*c[0]*(s[1]*(-c[3]*s[2]-c[2]*s[3])+c[1]*(c[2]*c[3]-s[2]*s[3])),
    -this->_l[3]*c[0]*(s[1]*(-c[3]*s[2]-c[2]*s[3])+c[1]*(c[2]*c[3]-s[2]*s[3])),
    
    // Effects on ee y
    this->_l[1]*c[0]+this->_l[2]*c[0]+this->_l[3]*c[0]+this->_l[0]*c[0], 0, 0, 0,

    // Effect on ee z
    this->_l[0]*s[0]+this->_l[1]*c[1]*s[0]+this->_l[2]*s[0]*(c[1]*c[2]-s[1]*s[2])+this->_l[3]*s[0]*(s[1]*(-c[3]*s[2]-c[2]*s[3])+c[1]*(c[2]*c[3]-s[2]*s[3])),
    this->_l[1]*c[0]*s[1]-this->_l[2]*c[0]*(-c[2]*s[1]-c[1]*s[2])-this->_l[3]*c[0]*(c[1]*(-c[3]*s[2]-c[2]*s[3])-s[1]*(c[2]*c[3]-s[2]*s[3])),
    -this->_l[2]*c[0]*(-c[2]*s[1]-c[1]*s[2])-this->_l[3]*c[0]*(c[1]*(-c[3]*s[2]-c[2]*s[3])+s[1]*(-c[2]*c[3]+s[2]*s[3])),
    -this->_l[3]*c[0]*(c[1]*(-c[3]*s[2]-c[2]*s[3])+s[1]*(-c[2]*c[3]+s[2]*s[3]));

    return J_ee;
}

Eigen::VectorXf ForceEstimatorNode::_get_stiffness_contribution(float * pos)
{
    Eigen::VectorXf tau(this->_k.size());
    for(uint i = 0; i < this->_k.size(); i++)
    {
        tau(i) = this->_k.at(i) * pos[i];
    }

    return tau;
}

Eigen::VectorXf ForceEstimatorNode::_get_ctrl_contribution(float tendon_force)
{
    /* Init the vector. The first joint is passive,
     * hence the control contribution there is always zero. */
    Eigen::VectorXf tau(this->_r.size() + 1);
    tau(0) = 0;

    for(uint i = 1; i < this->_r.size() + 1; i++)
    {
        tau(i) = this->_r.at(i-1) * tendon_force;
    }

    return tau;
}

Eigen::VectorXf ForceEstimatorNode::_get_gravity_contribution(std::vector<Eigen::Vector3f> joint_locs,
                                                              std::vector<Eigen::Vector3f> coms,
                                                              std::vector<Eigen::Matrix3f> rs)
{
    Eigen::VectorXf gravity_contr(rs.size());

    Eigen::Vector3f g;
    g << 0, 0, - 9.81;

    for(uint i = 0; i < rs.size(); i++)
    {
        /* Define the projection vector, i.e. the rotation axis 
         * of the current joint */
        Eigen::Vector3f proj;
        if(i == 0) proj << 1, 0, 0;
        else proj << 0, 1, 0;

        /* Rotate it accordingly */
        proj = rs.at(i) * proj;

        /* Add the contribution of each link to the current joint */
        for(uint j = i; j < rs.size(); j++)
        {
            /* Find the torque produced by link j on joint i */
            Eigen::Vector3f torque = (coms.at(j) - joint_locs.at(i)).cross(this->_m[j] * g);

            /* Not the entire torque has an effect on the joint, but only the component
             * aligned with the joint axis, i.e. we must project the torque onto the 
             * revolute axis of the joint. Once projected the scalar torque acting on 
             * the joint is equivalent to the norm of the projected vector. */
            gravity_contr(i) += ((torque.dot(proj)) * proj).norm();
        }
    }

    /* Finally return the overall generalized contribution */
    return gravity_contr;
}                                                              

std::vector<Eigen::Vector3f> ForceEstimatorNode::_get_joint_locs(std::vector<Eigen::Matrix3f> rs)
{
    /* Init vector of joint locations. First joint is at [0,0,0] */
    std::vector<Eigen::Vector3f> joint_locs = {Eigen::Vector3f::Zero()};
    for(uint i = 0; i < rs.size() - 1; i++)
    {
        /* Create vector representing current link length*/
        Eigen::Vector3f bar;
        bar << 0, 0, -this->_l[i];
 
        /* Push back the correct location - Last location + correctly rotated link length */
        joint_locs.push_back(joint_locs.back() + rs.at(i) * bar);
    }

    /* Return the location of joint position */
    return joint_locs;
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

std::vector<Eigen::Vector3f> ForceEstimatorNode::_get_coms(std::vector<Eigen::Vector3f> joint_locs,
                                                           std::vector<Eigen::Matrix3f> rs)
{
    /* Init vector of coms */
    std::vector<Eigen::Vector3f> coms;

    /* For each joint just add half the link length 
     * - correctly rotated - to obtain the CoM location */
    for(uint i = 0; i < rs.size(); i++)
    {
        Eigen::Vector3f bar;
        bar << 0, 0, - 0.5 * this->_l[i];
        coms.push_back(joint_locs.at(i) + rs.at(i) * bar);
    }

    return coms;
}

Eigen::Matrix3f ForceEstimatorNode::_rot_x(float theta)
{
    float sT = sin(theta);
    float cT = cos(theta);

    Eigen::Matrix3f rot;
    rot << 1,  0,   0,
           0, cT, -sT,
           0, sT,  cT;

    return rot;
}

Eigen::Matrix3f ForceEstimatorNode::_rot_y(float theta)
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

void ForceEstimatorNode::_set_force(std::vector<float> force)
{
    assert(force.size() == this->_force.size());
    for(uint i = 0; i < force.size(); i++)
    {
        this->_force.at(i) = force.at(i);
    }
}

void ForceEstimatorNode::_get_force(std::vector<float> & force)
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
    std::vector<float> force(3);
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