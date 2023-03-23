#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

using namespace std::chrono_literals;

class FakeWrenchPublisher : public rclcpp::Node
{
public:
    FakeWrenchPublisher()
        : Node("fake_wrench_publisher")
    {   
        timer_ = this->create_wall_timer(
            10ms, std::bind(&FakeWrenchPublisher::timer_callback, this));
        this->_wrench_publisher = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/wrench", 10);

    }

private: 

    void timer_callback()
    {
      auto message = geometry_msgs::msg::WrenchStamped();
      message.header.stamp = this->now();
      message.header.frame_id = "arm_base";

      message.wrench.force.x = 0.2;
      message.wrench.force.y = 0.15;
      message.wrench.force.z = 0.0;
      
      this->_wrench_publisher->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr _wrench_publisher;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<FakeWrenchPublisher>());
  rclcpp::shutdown();
  return 0;
}
