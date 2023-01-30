#include <chrono>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "cui_devices_amt10x.hpp"

class ForceEstimatorNode : public rclcpp::Node
{
public:
  ForceEstimatorNode()
      : Node("force_estimator"),
        _NAMES{"base_joint_pitch", "base_joint_roll", "middle_joint", "top_joint"}
  {

    /* Declare all the parameters */
    // PPR for all encoders
    this->declare_parameter("encoder_config.ppr", 2048);
    // Pin idx for the encoders                        Signal: A   B   X
    this->declare_parameter("encoder_config.encoder0_pins", std::vector<int>{0, 2, 3});
    this->declare_parameter("encoder_config.encoder1_pins", std::vector<int>{1, 4, 5});
    this->declare_parameter("encoder_config.encoder2_pins", std::vector<int>{21, 22, 23});
    this->declare_parameter("encoder_config.encoder3_pins", std::vector<int>{26, 27, 25});
    // Arm properties
    this->declare_parameter("arm_properties.m", std::vector<double>{1.0, 1.0, 1.0, 1.0});
    this->declare_parameter("arm_properties.l", std::vector<double>{1.0, 1.0, 1.0, 1.0});
    this->declare_parameter("arm_properties.k", std::vector<double>{1.0, 1.0, 1.0, 1.0});
    this->declare_parameter("arm_properties.r", std::vector<double>{1.0, 1.0, 1.0});

    // Frequency in which to publish the current joint state
    this->declare_parameter("encoder_config.joint_pub_fr", 24);
    this->declare_parameter("force_pub_fr", 24);

    /* Read out all the parameters */
    uint16_t ppr = this->get_parameter("encoder_config.ppr").as_int();

    // Store pin numbers in 2d array
    std::vector<std::vector<long int>> pins = {this->get_parameter("encoder_config.encoder0_pins").as_integer_array(),
                                               this->get_parameter("encoder_config.encoder1_pins").as_integer_array(),
                                               this->get_parameter("encoder_config.encoder2_pins").as_integer_array(),
                                               this->get_parameter("encoder_config.encoder3_pins").as_integer_array()};
    // Convert to classic arrays
    uint8_t A[4] = {uint8_t(pins[0][0]), uint8_t(pins[1][0]), uint8_t(pins[2][0]), uint8_t(pins[3][0])};
    uint8_t B[4] = {uint8_t(pins[0][1]), uint8_t(pins[1][1]), uint8_t(pins[2][1]), uint8_t(pins[3][1])};
    uint8_t X[4] = {uint8_t(pins[0][2]), uint8_t(pins[1][2]), uint8_t(pins[2][2]), uint8_t(pins[3][2])};

    uint16_t f_joint = this->get_parameter("encoder_config.joint_pub_fr").as_int();
    uint16_t f_force = this->get_parameter("force_pub_fr").as_int();

    /* Get all the arm property parameters */
    this->_m = this->get_parameter("arm_properties.m").as_double_array();
    this->_l = this->get_parameter("arm_properties.l").as_double_array();
    this->_k = this->get_parameter("arm_properties.k").as_double_array();
    this->_r = this->get_parameter("arm_properties.r").as_double_array();

    assert(this->_m.size() == this->_l.size());

    /* Init all the class members */
    this->_joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    this->_force_publisher = this->create_publisher<geometry_msgs::msg::WrenchStamped>("force", 10);

    this->_joint_timer = this->create_wall_timer(std::chrono::milliseconds(int(1e3 * 1.0 / f_joint)), std::bind(&ForceEstimatorNode::_joint_timer_callback, this));
    this->_f_timer = this->create_wall_timer(std::chrono::milliseconds(int(1e3 * 1.0 / f_force)), std::bind(&ForceEstimatorNode::_force_timer_callback, this));

    /* Init Encoder Driver instances */
    this->_encoders = new Cui_Devices_Amt10x(4, ppr, A, B, X);

    /* Start the encoders */
    this->_encoders->begin();

    /* Start the force estimator */
    this->_begin();
  }

private:
  const std::string _NAMES[4];
  void _joint_timer_callback();
  void _force_timer_callback();
  void _estimate_force(float *pos, float *vel,
                       float tendon_force);
  void _run();
  void _begin();
  void _get_force(std::vector<double> & force);
  Eigen::Matrix3f _rot_x(double theta);
  Eigen::Matrix3f _rot_y(double theta);
  std::vector<Eigen::Matrix3f> _get_rs(float *pos);
  std::vector<Eigen::Vector3f> _get_coms(std::vector<Eigen::Matrix3f> rs);


  rclcpp::TimerBase::SharedPtr _joint_timer;
  rclcpp::TimerBase::SharedPtr _f_timer;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_publisher;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr _force_publisher;

  Cui_Devices_Amt10x *_encoders;
  std::vector<double> _force = {1.0, 0.0, 0.0};

  /* Arm properties*/
  std::vector<double> _m, _l, _k, _r;

  std::thread* _executer; // Thread that executer the run function
};