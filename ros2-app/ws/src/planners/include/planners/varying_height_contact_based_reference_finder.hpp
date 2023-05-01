#include "contact_based_reference_finder.hpp"

using namespace std::chrono_literals;

class VaryingHeightContactBasedReferenceFinder : public ContactBasedReferenceFinder
{
public:
    VaryingHeightContactBasedReferenceFinder();
    
private:

    rclcpp::TimerBase::SharedPtr timer_;    

    void timer_callback();

    double _high, _low;
    bool _curr; // True == high , false == low
};