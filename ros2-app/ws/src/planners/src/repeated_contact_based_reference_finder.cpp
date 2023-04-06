#include "repeated_contact_based_reference_finder.hpp"

void RepeatedContactBasedReferenceFinder::_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  /* Simply execute the super function */
  ContactBasedReferenceFinder::_force_callback(msg);

  /* We repeat the experiment once we reached the end of the wall */
  if(this->_x.x() < - 1.8)
  {
    this->_beginning = this->now();
  }
}