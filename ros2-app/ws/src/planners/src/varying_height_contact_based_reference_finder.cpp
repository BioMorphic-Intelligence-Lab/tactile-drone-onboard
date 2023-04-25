#include "varying_height_contact_based_reference_finder.hpp"

VaryingHeightContactBasedReferenceFinder::VaryingHeightContactBasedReferenceFinder()
{
    this->declare_parameter("high", 1.7);
    this->declare_parameter("low", 1.5);

    this->_high = this->get_parameter("high").as_double();
    this->_low = this->get_parameter("low").as_double();
}

void VaryingHeightContactBasedReferenceFinder::_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  /* Simply execute the super function */
  ContactBasedReferenceFinder::_force_callback(msg);

  /* Update the hight reference every 10 seconds between high and low */
  int seconds = (int)(this->now() - this->_beginning).seconds();

  if((seconds / 10) % 2 == 0)
  {
    this->_reference(2) = this->_high;
  }
  else
  {
    this->_reference(2) = this->_low;
  }

}