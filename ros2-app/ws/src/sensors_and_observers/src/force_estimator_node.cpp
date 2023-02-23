#include "force_estimator_node.hpp"

std::vector<double> ForceEstimatorNode::_estimate_force(
            double *pos,
            double *vel,
            double tendon_force)
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

    /* Compute the force acting on the end effector. It is negative since we're interested in the force
     * acting on the end-effector and not the force on the environment */
    Eigen::Vector3f f = -this->pseudoInverse(J_EE.transpose())
                             * (gravity_cont + stiffness_cont - ctrl_cont);

    std::vector<double> force = {f(0), f(1), f(2)};  
    return force;
}

Eigen::MatrixXf ForceEstimatorNode::pseudoInverse(const Eigen::MatrixXf &a, double epsilon)
{
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

Eigen::MatrixXf ForceEstimatorNode::_get_ee_jacobian(double * pos)
{
    /* Store all the sine and cosine value such that we don't have 
     * to re-compute them all the time */
    double c0 = cos(pos[0]), s0 = sin(pos[0]),
           c1 = cos(pos[1]), s1 = sin(pos[1]),
           c12 = cos(pos[1] - pos[2]), s12 = sin(pos[1] - pos[2]),
           c123 = cos(pos[1] - pos[2] + pos[3]), s123 = sin(pos[1] - pos[2] + pos[3]);

    /* Init the jacobian matrix */
    Eigen::MatrixXf J_ee(3, this->_k.size());

    /* Fill it with values */
    J_ee <<
    // Effects on ee x
    0, this->_l[2]*c1+this->_l[3]*c12+this->_l[4]*c123, -this->_l[3]*c12-this->_l[4]*c123, this->_l[4]*c123,
    // Effects on ee y
    -c0*(this->_l[1]+this->_l[2]*c1+this->_l[3]*c12+this->_l[4]*c123), s0*(this->_l[2]*s1+this->_l[3]*s12+this->_l[4]*s123), -s0*(this->_l[3]*s12+this->_l[4]*s123), this->_l[4]*s0*s123,
    // Effects on ee z
    -((this->_l[1]+this->_l[2]*c1+this->_l[3]*c12+this->_l[4]*c123)*s0), -c0*(this->_l[2]*s1+this->_l[3]*s12+this->_l[4]*s123), c0*(this->_l[3]*s12+this->_l[4]*s123), -this->_l[4]*c0*s123;
    

    return J_ee;
}

Eigen::VectorXf ForceEstimatorNode::_get_stiffness_contribution(double * pos)
{
    Eigen::VectorXf tau(this->_k.size());
    for(uint i = 0; i < this->_k.size(); i++)
    {
        tau(i) = -this->_k.at(i) * pos[i];
    }

    return tau;
}

Eigen::VectorXf ForceEstimatorNode::_get_ctrl_contribution(double tendon_force)
{
    /* Init the vector. The first joint is passive,
     * hence the control contribution there is always zero. */
    Eigen::VectorXf tau(this->_k.size());
    tau(0) = 0;

    for(uint i = 0; i < this->_r.size(); i++)
    {
        /* All uneven indexed joints are oriented such that they have a negative effect 
         * from positive force, and all even ones have a positive effect. */
        tau(i+1) = ((i+1) % 2 == 0) ? this->_r.at(i) * tendon_force : -this->_r.at(i) * tendon_force;
    }

    return tau;
}

Eigen::VectorXf ForceEstimatorNode::_get_gravity_contribution(std::vector<Eigen::Vector3f> joint_locs,
                                                              std::vector<Eigen::Vector3f> coms,
                                                              std::vector<Eigen::Matrix3f> rs)
{
    Eigen::Vector4f gravity_contr = Eigen::Vector4f::Zero();

    Eigen::Vector3f g;
    g << 0, 0, 9.81;

    for(uint i = 0; i < joint_locs.size(); i++)
    {
        /* Define the projection vector, i.e. the rotation axis 
         * of the current joint (always x-axis) */
        Eigen::Vector3f proj;
        proj << 1, 0, 0;

        /* Rotate it accordingly */
        proj = rs.at(i) * proj;

        /* Add the contribution of each link to the current joint */
        for(uint j = i; j < coms.size(); j++)
        {
            /* Find the torque produced by link j on joint i */
            Eigen::Vector3f torque = (coms.at(j) - joint_locs.at(i)).cross(this->_m[j] * g);

            /* Not the entire torque has an effect on the joint, but only the component
             * aligned with the joint axis, i.e. we must project the torque onto the 
             * revolute axis of the joint. Once projected the scalar torque acting on 
             * the joint is equivalent to the norm of the projected vector. */
            gravity_contr(i) += (torque.dot(proj));
        }
    }

    /* Finally return the overall generalized contribution */
    return gravity_contr;
}                                                              

std::vector<Eigen::Vector3f> ForceEstimatorNode::_get_joint_locs(std::vector<Eigen::Matrix3f> rs)
{
    Eigen::Vector3f bar;
    bar << 0, 0, 1;

    /* Init vector of joint locations. First joint is at [0,0,l1] */
    std::vector<Eigen::Vector3f> joint_locs;
    joint_locs.push_back(this->_l[0] * bar);

    for(uint i = 1; i < this->_l.size()-1; i++)
    {
        /* Push back the correct location - Last location + correctly rotated link length */
        joint_locs.push_back(joint_locs.back() + rs.at(i) * this->_l[i] * bar);
    }

    /* Return the location of joint position */
    return joint_locs;
}


std::vector<Eigen::Vector3f> ForceEstimatorNode::_get_coms(std::vector<Eigen::Matrix3f> rs)
{

    Eigen::Vector3f bar;
    bar << 0, 0, 1;
    
    std::vector<Eigen::Vector3f> coms;
    for(uint i = 1; i < this->_l.size(); i++)
    {
        /* Init translation vector */
        Eigen::Vector3f com = this->_l[0] * bar;
        
        /* Add the translation of all previous links */
        for(uint j = 1; j < i; j++)
        {
            com += rs.at(j) * this->_l[j] * bar;
        }

        /* Add half bar length translation to the CoM */
        Eigen::Vector3f half_bar;
        half_bar << 0, 0, 0.5 * this->_l[i];
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

    Eigen::Vector3f bar;
    bar << 0, 0, 1;

    /* Init vector of coms */
    std::vector<Eigen::Vector3f> coms;

    /* For each joint just add half the link length 
     * - correctly rotated - to obtain the CoM location */
    for(uint i = 1; i < rs.size(); i++)
    {
        coms.push_back(joint_locs.at(i-1) + rs.at(i) * 0.5 * this->_l[i] * bar);
    }

    return coms;
}

Eigen::Matrix3f ForceEstimatorNode::_rot_x(double theta)
{
    double sT = sin(theta);
    double cT = cos(theta);

    Eigen::Matrix3f rot;
    rot << 1,  0,   0,
           0, cT, -sT,
           0, sT,  cT;

    return rot;
}

Eigen::Matrix3f ForceEstimatorNode::_rot_y(double theta)
{
    double sT = sin(theta);
    double cT = cos(theta);

    Eigen::Matrix3f rot;
    rot <<  cT,  0, sT,
             0,  1,  0,
           -sT,  0, cT;

    return rot;
}

Eigen::Matrix3f ForceEstimatorNode::_rot_z(double theta)
{
    double sT = sin(theta);
    double cT = cos(theta);

    Eigen::Matrix3f rot;
    rot <<  cT, -sT,  0,
            sT,  cT,  0,
             0,   0,  1;

    return rot;
}

std::vector<Eigen::Matrix3f> ForceEstimatorNode::_get_rs(double *pos)
{
    /* Init empty vector */
    std::vector<Eigen::Matrix3f> Rs;
    
    /* Add the rotation from the base to the first joint which is identity */
    Rs.push_back(this->_alignments.at(0));

    /* For each link, push back the current rotation matrix,
     * which describes its attitude with respect to the base.
     * Purposefully stopping at size()-1 since the last alignement matrix
     * has no effect on the position of the ee */
    for(uint i = 0; i < this->_alignments.size()-1; i++)
    {
        Rs.push_back(Rs.back() * this->_rot_x(pos[i])* this->_alignments.at(i+1));
    }

    return Rs;
}

void ForceEstimatorNode::_joint_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    double pos[4] = {msg->position[0],
                    msg->position[1],
                    msg->position[2],
                    msg->position[3]},
           vel[4] = {0,//msg->velocity[0],
                    0,//msg->velocity[1],
                    0,//msg->velocity[2],
                    0};//msg->velocity[3]};

    /* Compute the estimated contact force and store in class var */
    std::vector<double> force = this->_estimate_force(pos, vel, this->_f);

    auto wrench_msg = geometry_msgs::msg::WrenchStamped();
    wrench_msg.header.stamp = msg->header.stamp;
    wrench_msg.header.frame_id = "arm_base";

    wrench_msg.wrench.force.x = force.at(0);
    wrench_msg.wrench.force.y = force.at(1);
    wrench_msg.wrench.force.z = force.at(2);

    this->_force_publisher->publish(wrench_msg);

}

void ForceEstimatorNode::_make_frame_alignments()
{
    this->_alignments.push_back(Eigen::Matrix3f::Identity());
    this->_alignments.push_back(this->_rot_z(M_PI_2));
    this->_alignments.push_back(this->_rot_z(M_PI));
    this->_alignments.push_back(this->_rot_z(M_PI));
    this->_alignments.push_back(this->_rot_z(-M_PI_2));
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<ForceEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
