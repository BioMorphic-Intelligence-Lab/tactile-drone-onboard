#include "base_reference_position_publisher.hpp"


BaseReferencePositionPub::BaseReferencePositionPub()
    : Node("base_reference_position_publisher"), _offboard_setpoint_counter(0)
{   

    /* Declare all the parameters */
    this->declare_parameter("frequency", 20.0);

    /* Actually get all the parameters */
    this->_frequency =  this->get_parameter("frequency").as_double();
    this->_timer = this->create_wall_timer(1.0 / this->_frequency * 1s,
                                            std::bind(&BaseReferencePositionPub::_timer_callback, this));
    this->_status_subscription = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(), std::bind(&BaseReferencePositionPub::_status_callback, this, std::placeholders::_1));
    this->_timesync_subscription = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
        "/fmu/out/timesync_status", rclcpp::SensorDataQoS(), std::bind(&BaseReferencePositionPub::_timesync_callback, this, std::placeholders::_1));
    this->_reference_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/base_reference", rclcpp::SensorDataQoS(), std::bind(&BaseReferencePositionPub::_reference_callback, this, std::placeholders::_1));
    
    this->_offboard_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    this->_trajectory_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);

    this->_vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    this->_beginning = this->now();

    /* Init Ref Pose */
    this->_ref_pos = {0.0, 0.0, -2.0};
    this->_ref_yaw = 0.0;

}


/* Callback Functions */
void BaseReferencePositionPub::_timer_callback()
{
    if (_offboard_setpoint_counter == 10) 
    {
        /* On the real system we want to arm and change mode using the remote control
            Uncomment this for the SITL e.g. automatic arming and switch to offboard mode */
            
        // Change to Offboard mode after 10 setpoints
        //this->_publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        // Arm the vehicle
        //this->arm();

    }

    // offboard_control_mode needs to be paired with trajectory_setpoint
    this->_publish_offboard_control_mode();
    this->_publish_trajectory_setpoint();

    // stop the counter after reaching 11
    if (_offboard_setpoint_counter < 11) {
        _offboard_setpoint_counter++;
    }

}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void BaseReferencePositionPub::_publish_trajectory_setpoint()
{
    double t = (this->now() - this->_beginning).seconds();

    // We start by just taking off and hovering above the ground
    Eigen::Vector3d reference = {0, 0, -2.0};
    float yaw = 0;

    // Mission duration
    if(t > 15 && t <= 100)
    {
        reference = this->_ref_pos; 
        yaw = this->_ref_yaw;
    }
    else if(t > 100 && !this->_landed)
    {
        this->_landed = true;
        this->land();
        return;
    }

    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.position = {reference.x(), reference.y(), reference.z()};
    msg.yaw = yaw; // [-PI:PI]
    msg.timestamp = this->get_timestamp();
    this->_trajectory_publisher->publish(msg);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void BaseReferencePositionPub::_publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_timestamp();
    _offboard_publisher->publish(msg);
}

void BaseReferencePositionPub::_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    this->_nav_state = msg->nav_state;
    this->_arming_state = msg->arming_state;
}

void BaseReferencePositionPub::_reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

    Eigen::Vector3d position = {msg->pose.position.x, msg->pose.position.y,  msg->pose.position.z};
    // transform to px4 (ned) frame
    position = px4_ros_com::frame_transforms::enu_to_ned_local_frame(position);
    this->_ref_pos = position;

    // This assumes the quaternion only describes yaw. Also directly transformed into the px4 frame
    this->_ref_yaw = M_PI_2 - 2 * acos(msg->pose.orientation.w);
}

void BaseReferencePositionPub::_timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
{
    this->_timestamp_local = std::chrono::steady_clock::now();
    this->_timestamp_remote.store(msg->timestamp);
}

uint64_t BaseReferencePositionPub::get_timestamp()
{
    auto now = std::chrono::steady_clock::now();
    return this->_timestamp_remote.load() + std::chrono::round<std::chrono::microseconds>(now - this->_timestamp_local).count();
}

/**
 * @brief Send a command to Arm the vehicle
 */
void BaseReferencePositionPub::arm()
{
    this->_publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}
/**
 * @brief Send a command to Disarm the vehicle
 */
void BaseReferencePositionPub::disarm()
{
    _publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}
/**
 * @brief Send a command to takeoff
 */
void BaseReferencePositionPub::takeoff()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param7 = 3.0;
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_timestamp();
    _vehicle_command_pub->publish(msg);

    this->_taken_off = true;
    RCLCPP_INFO(this->get_logger(), "Takeoff command send");
}
/**
 * @brief Send a command to land
 */
void BaseReferencePositionPub::land()
{
    _publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

    RCLCPP_INFO(this->get_logger(), "Land command send");
}


void BaseReferencePositionPub::_publish_vehicle_command(uint16_t command,
                                                            float param1,
                                                            float param2,
                                                            float param3,
                                                            float param4,
                                                            float param5,
                                                            float param6,
                                                            float param7)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;

    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_timestamp();
    this->_vehicle_command_pub->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<BaseReferencePositionPub>());
  rclcpp::shutdown();
  return 0;
}
