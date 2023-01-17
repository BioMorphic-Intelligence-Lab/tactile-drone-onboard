#include "force_estimator.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("Main"), "Hello, from the force estimator");

    //rclcpp::spin(std::make_shared</* namespace_name::ClassName */>());
    rclcpp::shutdown();
    return 0;
}
