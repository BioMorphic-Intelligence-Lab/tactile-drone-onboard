#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "custom_interfaces/srv/get_joint_state.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_ros_com/frame_transforms.h"

class EEPositionTest : public rclcpp::Node
{
public:
    EEPositionTest()
        : Node("ee_position_test")
    {   
        using namespace std::chrono_literals;

        /* Declare all the parameters */
        this->declare_parameter("init_reference", std::vector<double>{1.0, 1.0, 1.0});
        this->declare_parameter("reference_topic", "ee_reference");
        this->declare_parameter("robot_params.l", std::vector<double>{0.025, 0.025, 0.0578, 0.058, 0.045});
        this->declare_parameter("robot_params.alignments.offset", std::vector<double>{0.2, 0, 0});
        this->declare_parameter("robot_params.alignments.base", std::vector<double>{M_PI, 0.0,M_PI});
        this->declare_parameter("robot_params.alignments.align0", std::vector<double>{0, 0, 0});
        this->declare_parameter("robot_params.alignments.align1", std::vector<double>{0, 0, M_PI_2});
        this->declare_parameter("robot_params.alignments.align2", std::vector<double>{0, 0, M_PI});
        this->declare_parameter("robot_params.alignments.align3", std::vector<double>{0, 0, M_PI});
        this->declare_parameter("robot_params.alignments.align4", std::vector<double>{0, 0,-M_PI_2});
        this->declare_parameter("frequency", 20.0);

        /* Actually get all the parameters */
        std::vector<double> reference_vect = this->get_parameter("init_reference").as_double_array();
        this->_reference = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(reference_vect.data(), reference_vect.size());
        this->_reference_yaw = 0;
        this->_frequency =  this->get_parameter("frequency").as_double();

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
        this->_vehicle_subscription = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(), std::bind(&EEPositionTest::_vehicle_callback, this, std::placeholders::_1));
        
        this->_reference_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("reference_topic").as_string(), 10);
        this->_trajectory_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/base_reference", rclcpp::SensorDataQoS());

        /* Init the service clients */
        this->_joint_service_client = this->create_client<custom_interfaces::srv::GetJointState>("get_joint_state");

        /* Call the service to get the nominal joint state */
        while (!this->_joint_service_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                exit(0);
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint service not available, waiting again...");
        }


        this->_update_nominal_configuration();

        this->_timer = this->create_wall_timer(1.0 / this->_frequency * 1s,
                                        std::bind(&EEPositionTest::_timer_callback, this));

        this->_beginning = this->now();

    }

private: 

    /* Timer for publishing references */
    rclcpp::Time _beginning;
    rclcpp::TimerBase::SharedPtr _timer;

    /* Current ee position reference */
    Eigen::Vector3d _reference;
    double _reference_yaw, _frequency;

    /* Nominal robotic finger state */
    Eigen::Vector4d _nominal_joint_state;

    /* Current base pose and orientation*/
    Eigen::Vector3d _x;
    Eigen::Quaterniond _q;

    /* Robotic Kinematic parameters */
    std::vector<double> _l;
    Eigen::Vector3d _offset;
    Eigen::Matrix3d _base;
    std::vector<Eigen::Matrix3d> _alignments;

    /* Publishers and Subscribers */
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_subscription;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _reference_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _trajectory_publisher;

    /* Service Client to calibrate force bias and get joint state*/
    rclcpp::Client<custom_interfaces::srv::GetJointState>::SharedPtr _joint_service_client;

    void _update_nominal_configuration()
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

    void _timer_callback()
    {
        /* We only update the reference position if we're close enoug to it, i.e. we can consider it to be reached */
        Eigen::Vector3d ee = this->_forward_kinematics(this->_x, this->_q, this->_nominal_joint_state);
        double error = (this->_reference - ee).norm();

        if(error < 0.05)
        {
            Eigen::Vector3d addon = {0.2, 0.2, 0};
            double addon_yaw = 0.2;
            this->_reference += addon;
            this->_reference_yaw += addon_yaw;
        }

        Eigen::Vector3d eg_base_ref = this->_reference - rot_z(this->_reference_yaw) * (this->_offset + this->_base * this->_relative_forward_kinematics(this->_nominal_joint_state));

        rclcpp::Time now = this->now();

        // EE Reference
        geometry_msgs::msg::PoseStamped ee_ref;
        ee_ref.header.stamp = now;
        ee_ref.header.frame_id = "world";
        ee_ref.pose.position.x = this->_reference(0);
        ee_ref.pose.position.y = this->_reference(1);
        ee_ref.pose.position.z = this->_reference(2);
        ee_ref.pose.orientation.z = sin(this->_reference_yaw * 0.5);
        ee_ref.pose.orientation.w = cos(this->_reference_yaw * 0.5);

        // Base Ref
        geometry_msgs::msg::PoseStamped base_ref;
        base_ref.header.stamp = now;
        base_ref.header.frame_id = "world";

        base_ref.pose.position.x = eg_base_ref.x();
        base_ref.pose.position.y = eg_base_ref.y();
        base_ref.pose.position.z = eg_base_ref.z();
        base_ref.pose.orientation.z = sin(this->_reference_yaw * 0.5);
        base_ref.pose.orientation.w = cos(this->_reference_yaw * 0.5);

        this->_reference_publisher->publish(ee_ref);
        this->_trajectory_publisher->publish(base_ref);

    }
    /* Utility Functions */
    Eigen::Vector3d _relative_forward_kinematics(Eigen::Vector4d xi)
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

    Eigen::Vector3d _forward_kinematics(Eigen::Vector3d x, Eigen::Quaterniond q, Eigen::Vector4d xi)
    {
        Eigen::Vector3d ee_pos;
        
        ee_pos = x 
                +  q.normalized().toRotationMatrix() * (this->_offset 
                + this->_base *(this->_relative_forward_kinematics(xi)
                ));

        return ee_pos;
    }

    /* Static Utility Functions */
    static Eigen::Matrix3d rot_x(double theta)
    {
        double cT = cos(theta);
        double sT = sin(theta);

        Eigen::Matrix3d rot;
        rot << 1, 0, 0,
                0, cT, -sT,
                0, sT, cT;

        return rot;
    }

    static Eigen::Matrix3d rot_y(double theta)
    {
        double cT = cos(theta);
        double sT = sin(theta);

        Eigen::Matrix3d rot;
        rot << cT, 0, sT,
                0, 1, 0,
                -sT, 0, cT;

        return rot;
    }

    static Eigen::Matrix3d rot_z(double theta)
    {
        double cT = cos(theta);
        double sT = sin(theta);

        Eigen::Matrix3d rot;
        rot << cT, -sT, 0,
                sT, cT, 0,
                0, 0, 1;

        return rot;
    }

    /* Callback Functions */
    void _vehicle_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
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

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<EEPositionTest>());
  rclcpp::shutdown();
  return 0;
}