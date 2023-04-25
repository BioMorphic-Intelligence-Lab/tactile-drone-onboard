#include "contact_based_reference_finder.hpp"

class VaryingHeightContactBasedReferenceFinder : public ContactBasedReferenceFinder
{
public:
    VaryingHeightContactBasedReferenceFinder();

    void _force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

private:
    double _high, _low;
};