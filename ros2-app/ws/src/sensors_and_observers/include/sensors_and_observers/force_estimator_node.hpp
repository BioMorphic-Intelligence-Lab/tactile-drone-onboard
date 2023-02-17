#include <chrono>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

class ForceEstimatorNode : public rclcpp::Node
{
public:
  ForceEstimatorNode()
      : Node("force_estimator"),
        _NAMES{"base_joint_pitch", "base_joint_roll", "middle_joint", "top_joint"}
  {

    /* Declare all the parameters */
    // Arm properties
    this->declare_parameter("arm_properties.m", std::vector<double>{0.005, 0.06, 0.03, 0.01});
    this->declare_parameter("arm_properties.l", std::vector<double>{0.025, 0.0578, 0.058, 0.045});
    this->declare_parameter("arm_properties.k", std::vector<double>{0.1, 0.1, 0.1, 0.1});
    this->declare_parameter("arm_properties.r", std::vector<double>{0.02, 0.02, 0.015});
    this->declare_parameter("arm_properties.f", -0.48812942);

    /* Get all the arm property parameters */
    this->_m = this->get_parameter("arm_properties.m").as_double_array();
    this->_l = this->get_parameter("arm_properties.l").as_double_array();
    this->_k = this->get_parameter("arm_properties.k").as_double_array();
    this->_r = this->get_parameter("arm_properties.r").as_double_array();
    this->_f = this->get_parameter("arm_properties.f").as_double();

    assert(this->_m.size() == this->_l.size());

    /* Init all the class members */
    this->_force_publisher = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench", 10);
    this->_joint_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&ForceEstimatorNode::_joint_callback, this, std::placeholders::_1));

  }

private:
  const std::string _NAMES[4];

  void _joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  std::vector<double> _estimate_force(double *pos, double *vel,
                                      double tendon_force);

  Eigen::Matrix3f _rot_x(double theta);
  Eigen::Matrix3f _rot_y(double theta);
  std::vector<Eigen::Matrix3f> _get_rs(double *pos);
  std::vector<Eigen::Vector3f> _get_joint_locs(std::vector<Eigen::Matrix3f> rs);
  std::vector<Eigen::Vector3f> _get_coms(std::vector<Eigen::Matrix3f> rs);
  std::vector<Eigen::Vector3f> _get_coms(std::vector<Eigen::Vector3f> joint_locs,
                                         std::vector<Eigen::Matrix3f> rs);

  Eigen::VectorXf _get_gravity_contribution(std::vector<Eigen::Vector3f> joint_locs,
                                            std::vector<Eigen::Vector3f> coms,
                                            std::vector<Eigen::Matrix3f> rs);
  Eigen::VectorXf _get_stiffness_contribution(double * pos);     
  Eigen::VectorXf _get_ctrl_contribution(double tendon_force);     
  Eigen::MatrixXf _get_ee_jacobian(double * pos);                

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr _force_publisher;

  /* Arm properties*/
  std::vector<double> _m, _l, _k, _r;
  double _f;
};