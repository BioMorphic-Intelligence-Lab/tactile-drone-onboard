#include "cui_devices_amt10x.hpp"

void Cui_Devices_Amt10x::run()
{
    for(;;)
    {
        this->readABX();
    }
}

void Cui_Devices_Amt10x::begin()
{
    /* Setting up wiringPi and the GPIO pins */
    wiringPiSetup();
    for(uint8_t i = 0; i < this->_NUM; i++)
    {
        pinMode(this->_encoderA[i], INPUT);
        pinMode(this->_encoderB[i], INPUT);
        pinMode(this->_encoderX[i], INPUT);
    }
    
    /* Start the thread. This constantly reads the current angle/velocity
     * and stores it in the member functions which can be accessed via the
     * member functions. */
    this->executer =  new std::thread(&Cui_Devices_Amt10x::run, this);
    this->executer->detach();
}

void Cui_Devices_Amt10x::readABX()
{
    for(uint8_t i = 0; i < this->_NUM; i++)
    {
        this->_currValueX[i] = digitalRead(this->_encoderX[i]);

        switch(this->_prevValueX[i] | this->_currValueX[i])
        {
            /* At a rising edge of the index signal we reset the count*/
            case 0b01:
                /* TODO Currently this only resets in one direction (CCW).
                * Should I reset in both? Can I even?*/
                this->_counter[i] = 0;
            break;
            /* Otherwise the index has no effect */
            case 0b00: case 0b10: case 0b11:
            break;

            default:
                std::cerr << "Unexpected State Reached in Cui_Devices_Amt10x::readABX(). I have no fucking idea what you did..."
                        << "Index: "<< int(i) <<  std::endl;
                std::cerr << "Reached State (X): " << int(this->_prevValueX[i] | this->_currValueX[i]) << std::endl;
                exit(1);
                break;
        }

        /* We bitshift the value of A by one to make space for B resulting in the concatenation of AB */
        this->_currValueAB[i] = ((uint8_t)(digitalRead(this->_encoderA[i])) << 1)  
                              | ((uint8_t)(digitalRead(this->_encoderB[i])));
        
        /* Evaluate the current state by binary or (see table in header) */
        switch(this->_prevValueAB[i] | this->_currValueAB[i])
        {
            /* All CCW cases are cosidered as positive */
            case 0b0010: case 0b0100: case 0b1011: case 0b1101:
                this->_counter[i] = this->_counter[i] + 1;
                break;
            /* All CW case are considered as negative */
            case 0b0001: case 0b0111: case 0b1000: case 0b1110:
                this->_counter[i] = this->_counter[i] - 1;
                break;
            /* For completeness sake these are all the idle case */
            case 0b0000: case 0b0101: case 0b1010: case 0b1111:
                //this->_count_vel *= 0.0; 
                break;
            /* These are the invalid cases that should not occur */
            case 0b0011: case 0b0110: case 0b1001: case 0b1100:
                std::cerr << "Reached Invalid case in Cui_Devices_Amt10x::readABX()!"  
                        << "Index: "<< int(i) <<  std::endl;
                break;
            default:
                std::cerr << "Unexpected State Reached in Cui_Devices_Amt10x::readABX(). I have no fucking idea what you did..." 
                        << "Index: "<< int(i) <<  std::endl;
                std::cerr << "Reached State (AB): " << int(((uint8_t)(digitalRead(this->_encoderA[i])) << 1)  
                                                         | ((uint8_t)(digitalRead(this->_encoderB[i])))) << std::endl;
                exit(1);
                break;
        }

        this->_curr_count_ts = micros();
        int16_t curr_dt = this->_curr_count_ts - this->_prev_count_ts;
        if(curr_dt >= this->_DT)
        {
            this->_count_vel[i] = i == 3 ? 
                    - float(this->_counter[i] - this->_prev_count[i]) / float(curr_dt) :
                      float(this->_counter[i] - this->_prev_count[i]) / float(curr_dt);
            this->_prev_count_ts = this->_curr_count_ts;
            this->_prev_count[i] = this->_counter[i];
        }
        /* Update previous value to current value, bitshifted by two to make space for next values */
        this->_prevValueAB[i] = (this->_currValueAB[i] << 2);
        this->_prevValueX[i] = (this->_currValueX[i] << 1);

    }    
}

void Cui_Devices_Amt10x::getPosition(float (&pos)[])
{
    /* Convert from counts to Radians */
    for(uint8_t i = 0; i < this->_NUM; i++)
    {
        pos[i] = this->_counter[i] * this->_RAD_PER_COUNT;
    }

    return;
}

void Cui_Devices_Amt10x::getVelocity(float (&vel)[])
{
    /* Convert from counts per microsecond to rad per second */
    for(uint8_t i = 0; i < this->_NUM; i++)
    {
        vel[i] = this->_count_vel[i] * this->_RAD_PER_COUNT * 1e6;
    }

    return;
}

