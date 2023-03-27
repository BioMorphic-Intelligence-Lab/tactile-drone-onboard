#include "contact_based_reference_finder.hpp"

ContactBasedReferenceFinder::ContactBasedReferenceFinder()
        : Node("contact_based_reference_finder"), _experiment_running(false)
{   
    using namespace std::chrono_literals;

    /* Declare all the parameters */
    this->declare_parameter("init_reference", std::vector<double>{0.0, 1.15, 1.6});
    this->declare_parameter("force_topic", "wrench");
    this->declare_parameter("reference_topic", "ee_reference");
    this->declare_parameter("alpha", 0.2);
    this->declare_parameter("robot_params.l", std::vector<double>{0.025, 0.025, 0.0578, 0.058, 0.045});
    this->declare_parameter("robot_params.alignments.offset", std::vector<double>{0.2, 0, 0});
    this->declare_parameter("robot_params.alignments.base", std::vector<double>{M_PI, 0, M_PI});
    this->declare_parameter("robot_params.alignments.align0", std::vector<double>{0, 0, 0});
    this->declare_parameter("robot_params.alignments.align1", std::vector<double>{0, 0, M_PI_2});
    this->declare_parameter("robot_params.alignments.align2", std::vector<double>{0, 0, M_PI});
    this->declare_parameter("robot_params.alignments.align3", std::vector<double>{0, 0, M_PI});
    this->declare_parameter("robot_params.alignments.align4", std::vector<double>{0, 0,-M_PI_2});

    /* Actually get all the parameters */
    this->_reference = {0.0, 0.0, 1.5};
    this->_reference_yaw = M_PI_2;
    this->_alpha  = this->get_parameter("alpha").as_double();
    this->_nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MAX;

    /* Get the robot params */
    this->_l = this->get_parameter("robot_params.l").as_double_array();
    std::vector<double> offset = this->get_parameter("robot_params.alignments.offset").as_double_array();
    std::vector<double> base_rot = this->get_parameter("robot_params.alignments.base").as_double_array();
    std::vector<double> align0 = this->get_parameter("robot_params.alignments.align0").as_double_array();
    std::vector<double> align1 = this->get_parameter("robot_params.alignments.align1").as_double_array();
    std::vector<double> align2 = this->get_parameter("robot_params.alignments.align2").as_double_array();
    std::vector<double> align3 = this->get_parameter("robot_params.alignments.align3").as_double_array();
    std::vector<double> align4 = this->get_parameter("robot_params.alignments.align4").as_double_array();

    /* Create the alignment matrices */
    this->_offset = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(offset.data(), offset.size());
    this->_base = rot_x(base_rot.at(0)) * rot_y(base_rot.at(1)) * rot_z(base_rot.at(2));
    this->_alignments.push_back(rot_x(align0.at(0)) * rot_y(align0.at(1)) * rot_z(align0.at(2)));
    this->_alignments.push_back(rot_x(align1.at(0)) * rot_y(align1.at(1)) * rot_z(align1.at(2)));
    this->_alignments.push_back(rot_x(align2.at(0)) * rot_y(align2.at(1)) * rot_z(align2.at(2)));
    this->_alignments.push_back(rot_x(align3.at(0)) * rot_y(align3.at(1)) * rot_z(align3.at(2)));
    this->_alignments.push_back(rot_x(align4.at(0)) * rot_y(align4.at(1)) * rot_z(align4.at(2)));

    /* Init the subscribers and publishers */
    this->_force_subscription = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        this->get_parameter("force_topic").as_string(), 10, std::bind(&ContactBasedReferenceFinder::_force_callback, this, std::placeholders::_1));
    this->_vehicle_subscription = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(), std::bind(&ContactBasedReferenceFinder::_vehicle_callback, this, std::placeholders::_1));
    this->_status_subscription = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(), std::bind(&ContactBasedReferenceFinder::_status_callback, this, std::placeholders::_1));

    this->_reference_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("reference_topic").as_string(), 10);
    this->_trajectory_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/base_reference", 10);

    /* Init the service clients */
    this->_bias_service_client = this->create_client<std_srvs::srv::Trigger>("calibrate_bias");
    this->_joint_service_client = this->create_client<custom_interfaces::srv::GetJointState>("get_joint_state");
   
    while (!this->_bias_service_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        exit(0);
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bias service not available, waiting again...");
    }

    while (!this->_joint_service_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        exit(0);
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint service not available, waiting again...");
    }

    /* Call the service to calibrate the force offset */
    auto bias_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto bias_result = this->_bias_service_client->async_send_request(bias_request);
    
    this->_update_nominal_configuration();

    this->_beginning = this->now();
}


void ContactBasedReferenceFinder::_update_nominal_configuration()
{

    auto joint_request = std::make_shared<custom_interfaces::srv::GetJointState::Request>();

    auto response_received_callback = [this]
    (rclcpp::Client<custom_interfaces::srv::GetJointState>::SharedFuture future) {
        /* Save the current joint state as nominal */
        this->_nominal_joint_state = {future.get()->joint_state.position[0],
                                    future.get()->joint_state.position[1],
                                    future.get()->joint_state.position[2],
                                    future.get()->joint_state.position[3]};

        RCLCPP_INFO(this->get_logger(), "Nominal Robot Finger configuration set");
    };

    auto joint_result = this->_joint_service_client->async_send_request(joint_request, response_received_callback);
}

void ContactBasedReferenceFinder::_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  double dt = (this->now() - this->_beginning).seconds();

  /* For the first 15 seconds we just take of and hover */
  if(dt < 15)
  {
      this->_reference = {0.0, 0.0, 1.5};
      this->_reference_yaw = M_PI_2;
  }
  /* After that we update the reference according to contact */
  else if(dt > 15)
  {

    /* The first time we enter this block we update the reference to a point inside the wall */
    if(!this->_experiment_running)
    {
      std::vector<double> reference_vect = this->get_parameter("init_reference").as_double_array();
      this->_reference = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(reference_vect.data(), reference_vect.size());
      this->_reference_yaw = M_PI_2;

      this->_experiment_running = true;
    }

    /* We only update the reference position if we're close enoug to it, i.e. we can consider it to be reached */
    Eigen::Vector3d ee = this->_forward_kinematics(this->_x, this->_q, this->_nominal_joint_state);
    double error = (this->_reference - ee).norm();
    double yaw = atan2(2 * ((this->_q.x() * this->_q.y()) + (this->_q.w() * this->_q.z())),
        this->_q.w()*this->_q.w() + this->_q.x()*this->_q.x() - this->_q.y()*this->_q.y() - this->_q.z()*this->_q.z());

    /* Extract*/
    Eigen::Vector3d force;
    force << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;

    if(error < 0.5 * this->_alpha && // Only update when the reference position is sufficiently achieved (within 50 percent of the step size)
       fabs(yaw - this->_reference_yaw) < 15 * M_PI * 0.005556 && // Only update when the reference yaw is sufficiently achieved (within 15 Degree)
       force.norm() > 0.1 && // Only update when the force magnitude is enough
       force.x() > 0)  // Only update on positive x-force e.g. we are in contact with the wall
    {

      /* Transform the force into the world coordinate system */
      force = this->_base.transpose() * force;
      force(1) = -force(2); // TODO Double check this. I'm still doing something wrong with the reference frames....

      RCLCPP_DEBUG(this->get_logger(), "Force in Body Frame: x %f, y %f, z %f", force.x(), force.y(), force.z());

      /* The yaw is even more subsceptible to noise,
       * hence we update it only once we experience
       * a larger lateral force */
      if(fabs(force.y()) > 0.08 && fabs(force.x()) > 0.1)
      {
        /* Compute the reference yaw 
          * Angle between to vectors: arccos(DOT(f, e_x) / |f|)
          * but since we're only considering
          * the angle in the ground plane we
          * treat the vector as a 2D vector.
          * The sign needs to be specifically added. */
        double relative_angle = (force.y() > 0) ? 
                  acos(-force.x() / sqrt(force.x()*force.x() + force.y()*force.y())):
                - acos(-force.x() / sqrt(force.x()*force.x() + force.y()*force.y()));
        this->_reference_yaw += 0.1 * relative_angle;

      }

      /* For the position propagation we also need to transform the force vector using the current base orientation */
      force = this->_q.toRotationMatrix() * force;

      RCLCPP_DEBUG(this->get_logger(), "Force in World Frame: x %f, y %f, z %f", force.x(), force.y(), force.z());

      /* Compute the new reference position based on the contact force */
      this->_reference += this->_alpha * (force.cross(Eigen::Vector3d::UnitZ()).normalized());

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
  Eigen::Vector3d base_ref = this->_reference - rot_z(this->_reference_yaw) * (this->_offset + this->_base * this->_relative_forward_kinematics(this->_nominal_joint_state));

  /* Fill the base reference message */
  geometry_msgs::msg::PoseStamped base_ref_msg;
  base_ref_msg.header.stamp = now;
  base_ref_msg.header.frame_id = "world";

  base_ref_msg.pose.position.x = base_ref.x();
  base_ref_msg.pose.position.y = base_ref.y();
  base_ref_msg.pose.position.z = base_ref.z();
  base_ref_msg.pose.orientation.z = sin(this->_reference_yaw * 0.5);
  base_ref_msg.pose.orientation.w = cos(this->_reference_yaw * 0.5);

  /* Publish all the values */
  this->_reference_publisher->publish(ee_ref);
  this->_trajectory_publisher->publish(base_ref_msg);
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
