#include "contact_based_reference_finder.hpp"


void ContactBasedReferenceFinder::_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  
  /* We only update the reference position if we're close enoug to it, i.e. we can consider it to be reached */
  Eigen::Vector3d ee = this->_forward_kinematics(this->_x, this->_q, this->_joint_state);
  double error = (this->_reference - ee).norm();

  if(error < 0.25 * this->_alpha)
  {
    /* Extract*/
    Eigen::Vector3d force, unit_z, unit_x;
    force << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
    unit_x << 1, 0, 0;
    unit_z << 0, 0, 1;

    /* Compute the reference yaw */
    this->_reference_yaw += acos(force.dot(unit_x) / (force.norm()));

    /* Transform the force into the world coordinate system */
    force = this->_base.transpose() * force;
    /* Compute the new reference position based on the contact force */
    this->_reference += this->_alpha * (force.cross(unit_z).normalized());

  }

  rclcpp::Time now = this->now();

  geometry_msgs::msg::PoseStamped ref;
  ref.header.stamp = now;
  ref.header.frame_id = "world";
  ref.pose.position.x = this->_reference(0);
  ref.pose.position.y = this->_reference(1);
  ref.pose.position.z = this->_reference(2);
  ref.pose.orientation.z = sin(this->_reference_yaw * 0.5);
  ref.pose.orientation.w = cos(this->_reference_yaw * 0.5);
 

  /* Compute the respective reference position of the base */
  // TODO this should be done from the nominal joint state, not the actual one
  Eigen::Vector3d base_ref = this->_reference - this->_q.normalized().toRotationMatrix() * (this->_offset + this->_base * this->_relative_forward_kinematics(this->_joint_state));

  geometry_msgs::msg::PoseStamped set;
  set.header.stamp = now;
  set.header.frame_id = "world";

  set.pose.position.x = base_ref.x();
  set.pose.position.y = base_ref.y();
  set.pose.position.z = base_ref.z();
  set.pose.orientation.z = sin(this->_reference_yaw * 0.5);
  set.pose.orientation.w = cos(this->_reference_yaw * 0.5);

  /* Publish all the values */
  this->_trajectory_publisher->publish(set);
  this->_reference_publisher->publish(ref);
}

void ContactBasedReferenceFinder::_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  this->_nav_state = msg->nav_state;
}

void ContactBasedReferenceFinder::_vehicle_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  /* Transform from px4 frame (ned) to ros2 frame (enu)*/
  Eigen::Quaterniond orientation = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
  Eigen::Vector3d position = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);

  orientation = px4_ros_com::frame_transforms::ned_to_enu_orientation(
                  px4_ros_com::frame_transforms::aircraft_to_baselink_orientation(orientation));
  position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(position);            

  this->_x.x() = position.x();
  this->_x.y() = position.y();
  this->_x.z() = position.z(); 

  this->_q = orientation;
}

void ContactBasedReferenceFinder::_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  Eigen::Vector4d q;
  q << msg->position[0], msg->position[1], msg->position[2], msg->position[3];
  this->_joint_state = q;
}

Eigen::Vector3d ContactBasedReferenceFinder::_relative_forward_kinematics(Eigen::Vector4d xi)
{
  Eigen::Vector3d bar;
  bar << 0, 0, 1;

  Eigen::Vector3d ee_pos = 
            this->_l[0] * bar 
          + this->_l[1] * this->_alignments[0] * rot_x(xi[0]) * bar
          + this->_l[2] * this->_alignments[0] * rot_x(xi[0]) * this->_alignments[1] * rot_x(xi[1]) * bar
          + this->_l[3] * this->_alignments[0] * rot_x(xi[0]) * this->_alignments[1] * rot_x(xi[1]) * this->_alignments[2] *  rot_x(xi[2]) * bar
          + this->_l[4] * this->_alignments[0] * rot_x(xi[0]) * this->_alignments[1] * rot_x(xi[1]) * this->_alignments[2] *  rot_x(xi[2]) * this->_alignments[3] * rot_x(xi[3]) * bar;
  return ee_pos;
}

Eigen::Vector3d ContactBasedReferenceFinder::_forward_kinematics(Eigen::Vector3d x, Eigen::Quaterniond q, Eigen::Vector4d xi)
{

  Eigen::Vector3d ee_pos;
  
  ee_pos = x 
          +  q.normalized().toRotationMatrix() * (this->_offset 
          + this->_base *(this->_relative_forward_kinematics(xi)
          ));

  return ee_pos;
}

Eigen::Matrix3d ContactBasedReferenceFinder::rot_x(double theta)
{
  double cT = cos(theta);
  double sT = sin(theta);

  Eigen::Matrix3d rot;
  rot << 1, 0, 0,
         0, cT, -sT,
         0, sT, cT;

  return rot;
}
Eigen::Matrix3d ContactBasedReferenceFinder::rot_y(double theta)
{
  double cT = cos(theta);
  double sT = sin(theta);

  Eigen::Matrix3d rot;
  rot << cT, 0, sT,
         0, 1, 0,
         -sT, 0, cT;

  return rot;
}

Eigen::Matrix3d ContactBasedReferenceFinder::rot_z(double theta)
{
  double cT = cos(theta);
  double sT = sin(theta);

  Eigen::Matrix3d rot;
  rot << cT, -sT, 0,
         sT, cT, 0,
         0, 0, 1;

  return rot;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<ContactBasedReferenceFinder>());
  rclcpp::shutdown();
  return 0;
}
