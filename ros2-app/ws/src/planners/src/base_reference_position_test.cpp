#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_ros_com/frame_transforms.h"

using namespace std::chrono_literals;

class BaseReferencePositionTest : public rclcpp::Node
{
public:
    BaseReferencePositionTest()
        : Node("contact_based_reference_finder")
    {   

        this->_timer = this->create_wall_timer(500ms, std::bind(&BaseReferencePositionTest::_timer_callback, this));
        this->_status_subscription = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(), std::bind(&BaseReferencePositionTest::_status_callback, this, std::placeholders::_1));

        this->_reference_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("base_reference", 10);
        this->_offboard_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        this->_trajectory_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        this->_beginning = this->now();

    }

private: 

    uint8_t _nav_state;
    rclcpp::Time _beginning;
    rclcpp::TimerBase::SharedPtr _timer;

    /* Publishers and Subscribers */
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_subscription;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _reference_publisher;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_publisher;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_publisher;

    /* Callback Functions */
    void _timer_callback()
    {
        Eigen::Vector3d reference;
        reference << 0.0, 0.0, 0.0;
        double reference_yaw = 0.0;
        rclcpp::Time now = this->now();
        if(now - this->_beginning < 30s)
        {
            reference(0) = 1.0;
            reference(1) = 1.0;
            reference(2) = 1.0;

            reference_yaw = 0.0;
        }
        else if(now - this->_beginning < 60s)
        {
            
            reference(0) = -1.0;
            reference(1) = -1.0;
            reference(2) = 1.0;

            reference_yaw = M_PI_2;
        }
        else
        {
            reference(0) = -1.0;
            reference(1) = -1.0;
            reference(2) = 2.0;

            reference_yaw = M_PI;
        }

        geometry_msgs::msg::PoseStamped ref;
        ref.header.stamp = now;
        ref.header.frame_id = "world";
        ref.pose.position.x = reference(0);
        ref.pose.position.y = reference(1);
        ref.pose.position.z = reference(2);
        ref.pose.orientation.z = sin(reference_yaw * 0.5);
        ref.pose.orientation.w = cos(reference_yaw * 0.5);

        auto offboard_msg = px4_msgs::msg::OffboardControlMode();
        offboard_msg.timestamp = (uint64_t)(this->now().nanoseconds() * 0.001);
        offboard_msg.position=true;
        offboard_msg.attitude=true;
        offboard_msg.velocity=false;
        offboard_msg.acceleration=false;
        this->_offboard_publisher->publish(offboard_msg);

        if(this->_nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
        {
            px4_msgs::msg::TrajectorySetpoint set;
            set.timestamp = (uint64_t) (now.nanoseconds() * 0.001);
            /* Transform reference back into px4 frame (ned) */
            Eigen::Vector3d base_ref_px4 = px4_ros_com::frame_transforms::enu_to_ned_local_frame(reference);    
            set.position[0] = base_ref_px4.x();
            set.position[1] = base_ref_px4.y();
            set.position[2] = base_ref_px4.z();

            /* The Yaw also needs to be transformed into the px4 coordinate system (ned) */
            set.yaw = M_PI_2 - reference_yaw;
            /* Publish all the values */
            this->_trajectory_publisher->publish(set);
        }

        this->_reference_publisher->publish(ref);

    }

    void _status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        this->_nav_state = msg->nav_state;
    }


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<BaseReferencePositionTest>());
  rclcpp::shutdown();
  return 0;
}
