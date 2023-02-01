#include "cui_devices_amt10x.hpp"

/* Define encoder resolution */
const uint16_t ppr = 2048;

/* Define Pin Indeces (WiringPi indeces) for each signal */
uint8_t A[4] = {0, 1, 21, 26};
uint8_t B[4] = {2, 4, 22, 27};
uint8_t X[4] = {3, 5, 23, 25};

int main()
{
    Cui_Devices_Amt10x encoder(4, ppr, A, B, X);
    encoder.begin();

    std::cout << "Thread started" <<std::endl;
    float angle[4] = {};

    for(int i = 0;;i ++)
    {        
        if(i % 5000000 == 0)
        {
            encoder.getPosition(angle);
            //double vel = encoder.getVelocity();

            std::cout << "Angles: ";
            for(uint8_t j = 0; j < 4; j++)
            {
                std::cout  << 180 * M_1_PI * angle[j] << " [deg] ";
                //std::cout << "Velocity: " << 180 * M_1_PI * vel << " [deg/s]" <<std::endl;
            }
            std::cout << "" << std::endl;
        }
    }

    return 0;
}