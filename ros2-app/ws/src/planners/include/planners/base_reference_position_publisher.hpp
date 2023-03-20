#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_ros_com/frame_transforms.h"

using namespace std::chrono_literals;

class BaseReferencePositionPub : public rclcpp::Node
{
public:
    BaseReferencePositionPub();

private: 

    uint8_t _nav_state, _arming_state;
    bool _taken_off = false, _landed = false;
    uint _offboard_setpoint_counter;

    double _frequency;

    rclcpp::Time _beginning;
    rclcpp::TimerBase::SharedPtr _timer;

    /* Publishers and Subscribers */
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_subscription;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _reference_subscription;
    
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_publisher;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_publisher;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;

    // Time offset
    std::atomic<uint64_t> _timestamp_remote;
    std::chrono::time_point<std::chrono::steady_clock> _timestamp_local;

    // Current High Level Reference
    Eigen::Vector3d _ref_pos;
    double _ref_yaw;

    /* Callback Functions */
    void _timer_callback();

    /**
     * @brief Publish a trajectory setpoint
     *        For this example, it sends a trajectory setpoint to make the
     *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
     */
    void _publish_trajectory_setpoint();

    /**
     * @brief Publish the offboard control mode.
     *        For this example, only position and altitude controls are active.
     */
    void _publish_offboard_control_mode();

    void _status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

    void _reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void _timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg);

    uint64_t get_timestamp();

    /**
     * @brief Send a command to Arm the vehicle
     */
    void arm();

    /**
     * @brief Send a command to Disarm the vehicle
     */
    void disarm();

    /**
     * @brief Send a command to takeoff
     */
    void takeoff();

    /**
     * @brief Send a command to land
     */
    void land();


    void _publish_vehicle_command(uint16_t command,
                                  float param1 = 0.0,
                                  float param2 = 0.0,
                                  float param3 = 0.0,
                                  float param4 = 0.0,
                                  float param5 = 0.0,
                                  float param6 = 0.0,
                                  float param7 = 0.0);
};


