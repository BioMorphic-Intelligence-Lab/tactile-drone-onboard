#include "repeated_contact_based_reference_finder.hpp"

void RepeatedContactBasedReferenceFinder::_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  double dt = (this->now() - this->_beginning).seconds();

  /* For the first 15 seconds we just take of and hover */
  if(dt < 15)
  {
      this->_reference = {0.0, 0.0, 1.5};
      this->_reference_yaw = 0.0;
  }
  /* After that we update the reference according to contact */
  else if(dt > 15)
  {

    /* The first time we enter this block we update the reference to a point inside the wall */
    if(!this->_experiment_running)
    {
      std::vector<double> reference_vect = this->get_parameter("init_reference").as_double_array();
      
      this->_reference_yaw = 0.0;
      this->_reference = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(reference_vect.data(),
                                                             reference_vect.size());

      this->_experiment_running = true;
    }

    /* We only update the reference position if we're close enoug to it, i.e. we can consider it to be reached */
    double error = (this->_base_ref - this->_x).norm();
    double yaw = this->yaw_from_quat(this->_q);
    double yaw_error = fabs(this->to_angle_range(yaw - this->_reference_yaw));

    /* Extract*/
    Eigen::Vector3d force;
    force << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;

    if(error < 0.3 * this->_alpha && // Only update when the reference position is sufficiently achieved (within 30 percent of the step size)
       yaw_error < 5 * M_PI * 0.005556  // Only update when the reference yaw is sufficiently achieved (within 55 Degree)
      )  
    {

      if(force.norm() < 0.2) // Only do the main update when the force magnitude is enough)
      {
        RCLCPP_INFO(this->get_logger(), "No force detected - Moving forward");
        Eigen::Vector3d change = 0.1 * this->_alpha * (this->_q.toRotationMatrix() * Eigen::Vector3d::UnitY());
        change(2) = 0.0;
        this->_reference += change;
      }
      else if(force.x() >= 0) // Only update on positive x-force (in finger base frame) e.g. we are in contact with the wall
      {
        RCLCPP_INFO(this->get_logger(), "Enough force present. Doing Main update");
        
        /* Transform the force into the base coordinate system */
        force = this->_base.transpose() * force;
        
        RCLCPP_DEBUG(this->get_logger(), "Force in Body Frame: x %f, y %f, z %f", force.x(), force.y(), force.z());

        /* The yaw is even more subsceptible to noise,
        * hence we update it only once we experience
        * a larger lateral force */
        if(fabs(force.x()) > 0.1 && force.norm() > 0.1)
        {
          /* Compute the reference yaw 
            * Angle between a vectors and the frame x axis: atan2(f.y / f.x)
            */
          //double relative_angle = atan2(force.x(), force.y());
          this->_reference_yaw += force.x() > 0 ? 5 * M_PI * 0.00555555555 : 
                                                 -5 * M_PI * 0.00555555555;

        }
  
        /* For the position propagation we also need to transform the force vector using the current base orientation */
        force = this->_q.toRotationMatrix() * force;

        RCLCPP_DEBUG(this->get_logger(), "Force in World Frame: x %f, y %f, z %f", force.x(), force.y(), force.z());

        /* Compute the new reference position based on the contact force */
        Eigen::Vector3d change = this->_alpha * (force.cross(Eigen::Vector3d::UnitZ()).normalized());
        change(2) = 0.0;
        this->_reference += change;
        
      }
    }
    // If exceed a force threshold we move back.
    else if(force.norm() > 0.7)
    {
      RCLCPP_INFO(this->get_logger(), "Too much force detected - Moving backwards");
      Eigen::Vector3d change = -0.1 * this->_alpha * (this->_q.toRotationMatrix() * Eigen::Vector3d::UnitY());
      change(2) = 0.0;
      this->_reference += change;
    }
  }
  

  /* Get the current time stamp */
  rclcpp::Time now = this->now();

  /* Fill the EE Reference message */
  geometry_msgs::msg::PoseStamped ee_ref;
  ee_ref.header.stamp = now;
  ee_ref.header.frame_id = "world";
  ee_ref.pose.position.x = this->_reference(0);
  ee_ref.pose.position.y = this->_reference(1);
  ee_ref.pose.position.z = this->_reference(2);
  ee_ref.pose.orientation.z = sin(this->_reference_yaw * 0.5);
  ee_ref.pose.orientation.w = cos(this->_reference_yaw * 0.5);
 
  /* Compute the respective reference position of the base */
  this->_base_ref = this->_reference - rot_z(this->_reference_yaw) * (this->_offset + this->_base * this->_relative_forward_kinematics(this->_nominal_joint_state));

  /* Fill the base reference message */
  geometry_msgs::msg::PoseStamped base_ref_msg;
  base_ref_msg.header.stamp = now;
  base_ref_msg.header.frame_id = "world";

  base_ref_msg.pose.position.x = this->_base_ref.x();
  base_ref_msg.pose.position.y = this->_base_ref.y();
  base_ref_msg.pose.position.z = this->_base_ref.z();
  base_ref_msg.pose.orientation.z = sin(this->_reference_yaw * 0.5);
  base_ref_msg.pose.orientation.w = cos(this->_reference_yaw * 0.5);

  /* Publish all the values */
  this->_reference_publisher->publish(ee_ref);
  this->_trajectory_publisher->publish(base_ref_msg);

  /* We repeat the experiment once we reached the end of the wall */
  if(this->_x.x() < - 1.8)
  {
    this->_beginning = this->now();
  }
}