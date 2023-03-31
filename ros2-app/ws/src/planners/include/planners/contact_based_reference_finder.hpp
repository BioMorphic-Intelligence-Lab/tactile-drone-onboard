#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "custom_interfaces/srv/get_joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_ros_com/frame_transforms.h"

class ContactBasedReferenceFinder : public rclcpp::Node
{
public:
    ContactBasedReferenceFinder();

private: 

    /* Start time */
    rclcpp::Time _beginning;

    /* Current ee position reference */
    Eigen::Vector3d _reference, _base_ref;

    /* Nominal robotic finger state */
    Eigen::Vector4d _nominal_joint_state;

    /* Current base pose and orientation*/
    Eigen::Vector3d _x;
    Eigen::Quaterniond _q;

    /* Step between references and current reference yaw*/
    double _alpha, _reference_yaw;

    /* PX4 navigation state */
    uint8_t _nav_state;

    /* Whether we started being in contact */
    bool _experiment_running;

    /* Robotic Kinematic parameters */
    std::vector<double> _l;
    Eigen::Vector3d _offset;
    Eigen::Matrix3d _base;
    std::vector<Eigen::Matrix3d> _alignments;

    /* Publishers and Subscribers */
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr _force_subscription;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_subscription;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_subscription;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _reference_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _trajectory_publisher;

    /* Service Client to calibrate force bias and get joint state*/
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _bias_service_client;
    rclcpp::Client<custom_interfaces::srv::GetJointState>::SharedPtr _joint_service_client;

    /* Utility Functions */
    Eigen::Vector3d _relative_forward_kinematics(Eigen::Vector4d xi);
    Eigen::Vector3d _forward_kinematics(Eigen::Vector3d x, Eigen::Quaterniond q, Eigen::Vector4d xi);

    void _update_nominal_configuration();

    /* Static Utility Functions */
    static Eigen::Matrix3d rot_x(double theta);
    static Eigen::Matrix3d rot_y(double theta);
    static Eigen::Matrix3d rot_z(double theta);
    static Eigen::Quaterniond ned2enu(Eigen::Quaterniond quat);
    static Eigen::Vector3d ned2enu(Eigen::Vector3d vect);
    static double to_angle_range(double angle);
    static double yaw_from_quat(Eigen::Quaterniond quat);
    

    /* Callback Functions */
    void _force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void _vehicle_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void _status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);


};