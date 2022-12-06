#include <chrono>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "cui_devices_amt10x.hpp"

using namespace std::chrono_literals;

class EncoderNode : public rclcpp::Node
{
  public:
    EncoderNode()
    : Node("encoder_node"),
     _NAMES{"base_joint_pitch","base_joint_roll", "middle_joint", "top_joint"}
    {

      /* Declare all the parameters */

      // PPR for all encoders
      this->declare_parameter("ppr", 2048);
      // Pin idx for the encoders                        Signal: A   B   X
      this->declare_parameter("encoder0_pins", std::vector<int>{ 0,  2,  3});
      this->declare_parameter("encoder1_pins", std::vector<int>{ 1,  4,  5});
      this->declare_parameter("encoder2_pins", std::vector<int>{21, 22, 23});
      this->declare_parameter("encoder3_pins", std::vector<int>{26, 27, 25});

      // Frequency in which to publish the current joint state
      this->declare_parameter("state_pub_fr", 200);

      /* Read out all the parameters */
      uint16_t ppr = this->get_parameter("ppr").as_int();

      // Store pin numbers in 2d array
      std::vector<std::vector<long int>> pins = {this->get_parameter("encoder0_pins").as_integer_array(),
                                                 this->get_parameter("encoder1_pins").as_integer_array(),
                                                 this->get_parameter("encoder2_pins").as_integer_array(),
                                                 this->get_parameter("encoder3_pins").as_integer_array()};

      uint16_t f = this->get_parameter("state_pub_fr").as_int();

      /* Init all the class members */
      this->_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
      this->_timer = this->create_wall_timer(std::chrono::milliseconds(int(10e3 * 1.0 / f)), std::bind(&EncoderNode::timer_callback, this));

      for(uint8_t i = 0; i < pins.size(); i++)
      {
        /* Init Encoder Driver instances */
        this->_encoders[i] = new Cui_Devices_Amt10x(i, ppr, pins[i][0], pins[i][1], pins[i][2]);
      }

      /* Start the encoders */
      for(uint8_t i = 0; i < pins.size(); i++)
      {
        /* Init Encoder Driver instances */
        this->_encoders[i]->begin();
      }
    }

  private:
    const std::string _NAMES[4];
    void timer_callback();

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _publisher;

    Cui_Devices_Amt10x* _encoders[4];

};