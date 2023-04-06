#include "contact_based_reference_finder.hpp"

class RepeatedContactBasedReferenceFinder : public ContactBasedReferenceFinder
{
public:
    RepeatedContactBasedReferenceFinder();

    void _force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

};