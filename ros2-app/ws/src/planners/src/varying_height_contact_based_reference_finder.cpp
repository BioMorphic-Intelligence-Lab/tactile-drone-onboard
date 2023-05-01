#include "varying_height_contact_based_reference_finder.hpp"

VaryingHeightContactBasedReferenceFinder::VaryingHeightContactBasedReferenceFinder()
{
    this->declare_parameter("high", 1.7);
    this->declare_parameter("low", 1.5);

    this->_high = this->get_parameter("high").as_double();
    this->_low = this->get_parameter("low").as_double();

    this->timer_ = this->create_wall_timer(
      5s, std::bind(&VaryingHeightContactBasedReferenceFinder::timer_callback, this));
    this->_curr = false;
}

void VaryingHeightContactBasedReferenceFinder::timer_callback()
{
  if(this->_curr)
  {
    this->_height = this->_low;
    RCLCPP_INFO(this->get_logger(), "Low");
  }
  else
  {
    this->_height = this->_high;
    RCLCPP_INFO(this->get_logger(), "High");
  }

  this->_curr = !this->_curr;

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<VaryingHeightContactBasedReferenceFinder>());
  rclcpp::shutdown();
  return 0;
}
