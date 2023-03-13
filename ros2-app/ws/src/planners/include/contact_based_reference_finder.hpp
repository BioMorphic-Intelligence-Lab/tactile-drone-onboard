#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_ros_com/frame_transforms.h"

class ContactBasedReferenceFinder : public rclcpp::Node
{
public:
    ContactBasedReferenceFinder()
        : Node("contact_based_reference_finder")
    {   
        /* Declare all the parameters */
        this->declare_parameter("init_reference", std::vector<double>{1.0, 1.0, 1.0});
        this->declare_parameter("force_topic", "wrench");
        this->declare_parameter("reference_topic", "ee_reference");
        this->declare_parameter("alpha", 0.2);
        this->declare_parameter("robot_params.l", std::vector<double>{0.025, 0.025, 0.0578, 0.058, 0.045});
        this->declare_parameter("robot_params.alignments.offset", std::vector<double>{0.2, 0, 0});
        this->declare_parameter("robot_params.alignments.base", std::vector<double>{M_PI, 0, -M_PI_2});
        this->declare_parameter("robot_params.alignments.align0", std::vector<double>{0, 0, 0});
        this->declare_parameter("robot_params.alignments.align1", std::vector<double>{0, 0, M_PI_2});
        this->declare_parameter("robot_params.alignments.align2", std::vector<double>{0, 0, M_PI});
        this->declare_parameter("robot_params.alignments.align3", std::vector<double>{0, 0, M_PI});
        this->declare_parameter("robot_params.alignments.align4", std::vector<double>{0, 0,-M_PI_2});

        /* Actually get all the parameters */
        std::vector<double> reference_vect = this->get_parameter("init_reference").as_double_array();
        this->_reference = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(reference_vect.data(), reference_vect.size());
        this->_reference_yaw = 0;
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
        this->_joint_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::SensorDataQoS(), std::bind(&ContactBasedReferenceFinder::_joint_callback, this, std::placeholders::_1));
        this->_vehicle_subscription = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(), std::bind(&ContactBasedReferenceFinder::_vehicle_callback, this, std::placeholders::_1));
        this->_status_subscription = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(), std::bind(&ContactBasedReferenceFinder::_status_callback, this, std::placeholders::_1));

        this->_reference_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("reference_topic").as_string(), 10);
        this->_trajectory_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/base_reference", 10);
    

    }

private: 

    Eigen::Vector3d _reference;
    Eigen::Vector4d _joint_state;

    Eigen::Vector3d _x;
    Eigen::Quaterniond _q;

    double _alpha, _reference_yaw;

    uint8_t _nav_state;

    std::vector<double> _l;
    Eigen::Vector3d _offset;
    Eigen::Matrix3d _base;
    std::vector<Eigen::Matrix3d> _alignments;

    /* Publishers and Subscribers */
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr _force_subscription;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_subscription;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_subscription;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_subscription;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _reference_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _trajectory_publisher;

    /* Utility Functions */
    Eigen::Vector3d _relative_forward_kinematics(Eigen::Vector4d xi);
    Eigen::Vector3d _forward_kinematics(Eigen::Vector3d x, Eigen::Quaterniond q, Eigen::Vector4d xi);

    /* Static Utility Functions */
    static Eigen::Matrix3d rot_x(double theta);
    static Eigen::Matrix3d rot_y(double theta);
    static Eigen::Matrix3d rot_z(double theta);

    /* Callback Functions */
    void _force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void _joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void _vehicle_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void _status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);


};