#include "encoder_node.hpp"

EncoderNode::EncoderNode()
      : Node("encoder_node"),
        _NAMES{"base_joint_pitch", "base_joint_roll", "middle_joint", "top_joint"}
{

    /* Declare all the parameters */

    // PPR for all encoders
    this->declare_parameter("ppr", 2048);
    // Pin idx for the encoders                        Signal: A   B   X
    this->declare_parameter("encoder0_pins", std::vector<int>{0, 2, 3});
    this->declare_parameter("encoder1_pins", std::vector<int>{1, 4, 5});
    this->declare_parameter("encoder2_pins", std::vector<int>{21, 22, 23});
    this->declare_parameter("encoder3_pins", std::vector<int>{26, 27, 25});

    // Frequency in which to publish the current joint state
    this->declare_parameter("state_pub_fr", 24);

    /* Read out all the parameters */
    uint16_t ppr = this->get_parameter("ppr").as_int();

    // Store pin numbers in 2d array
    std::vector<std::vector<long int>> pins = {this->get_parameter("encoder0_pins").as_integer_array(),
                                                this->get_parameter("encoder1_pins").as_integer_array(),
                                                this->get_parameter("encoder2_pins").as_integer_array(),
                                                this->get_parameter("encoder3_pins").as_integer_array()};
    // Convert to classic arrays
    uint8_t A[4] = {uint8_t(pins[0][0]), uint8_t(pins[1][0]), uint8_t(pins[2][0]), uint8_t(pins[3][0])};
    uint8_t B[4] = {uint8_t(pins[0][1]), uint8_t(pins[1][1]), uint8_t(pins[2][1]), uint8_t(pins[3][1])};
    uint8_t X[4] = {uint8_t(pins[0][2]), uint8_t(pins[1][2]), uint8_t(pins[2][2]), uint8_t(pins[3][2])};

    uint16_t f = this->get_parameter("state_pub_fr").as_int();

    /* Init all the class members */
    this->_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
    this->_server = this->create_service<custom_interfaces::srv::GetJointState>(
		    "get_joint_state", std::bind(&EncoderNode::_get_joint_state, this, std::placeholders::_1, std::placeholders::_2));
    this->_timer = this->create_wall_timer(std::chrono::milliseconds(int(1e3 * 1.0 / f)), std::bind(&EncoderNode::timer_callback, this));

    /* Init Encoder Driver instances */
    this->_encoders = new Cui_Devices_Amt10x(4, ppr, A, B, X);

    /* Start the encoders */
    this->_encoders->begin();
}

void EncoderNode::_get_joint_state(const std::shared_ptr<custom_interfaces::srv::GetJointState::Request> request,
                                         std::shared_ptr<custom_interfaces::srv::GetJointState::Response> response)
{

    /* Unused because its empty anyways */
    (void) request;

    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.header.frame_id = this->get_name();
    
    float pos[4] = {}, vel[4] = {0.0,0.0,0.0,0.0};
    this->_encoders->getPosition(pos);
    this->_encoders->getVelocity(vel);

    for(int i = 0; i < 4; i++)
    {
        message.position.push_back(i == 1 ? pos[i] : -pos[i]);
        message.velocity.push_back(vel[i]);
        message.name.push_back(this->_NAMES[i]);
    }

    response->joint_state = message;
}

void EncoderNode::timer_callback()
{
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.header.frame_id = this->get_name();
    
    float pos[4] = {}, vel[4] = {0.0,0.0,0.0,0.0};
    this->_encoders->getPosition(pos);
    this->_encoders->getVelocity(vel);

    for(int i = 0; i < 4; i++)
    {
        message.position.push_back(i == 1 ? pos[i] : -pos[i]);
        message.velocity.push_back(vel[i]);
        message.name.push_back(this->_NAMES[i]);
    }

    this->_publisher->publish(message);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncoderNode>());
    rclcpp::shutdown();
    return 0;
}