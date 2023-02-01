/***************************************************************************************************/
/*
   NOTE:
   - Quadrature encoder makes two waveforms that are 90° out of phase:
                           _______         _______         __
                  PinA ___|       |_______|       |_______|   PinA
          CCW <--              _______         _______
                  PinB _______|       |_______|       |______ PinB
                               _______         _______
                  PinA _______|       |_______|       |______ PinA
          CW  -->          _______         _______         __
                  PinB ___|       |_______|       |_______|   PinB
          The half of the pulses from top to bottom create full state array:  
          prev.A+B   cur.A+B   (prev.AB+cur.AB)  Array   Encoder State
          -------   ---------   --------------   -----   -------------
            00         00            0000          0     stop/idle
            00         01            0001         -1     CW,  0x01
            00         10            0010          1     CCW, 0x02
            00         11            0011          0     invalid state
            01         00            0100          1     CCW, 0x04
            01         01            0101          0     stop/idle
            01         10            0110          0     invalid state
            01         11            0111         -1     CW, 0x07
            10         00            1000         -1     CW, 0x08
            10         01            1001          0     invalid state
            10         10            1010          0     stop/idle
            10         11            1011          1     CCW, 0x0B
            11         00            1100          0     invalid state
            11         01            1101          1     CCW, 0x0D 
            11         10            1110         -1     CW,  0x0E
            11         11            1111          0     stop/idle
          - CW  states 0b0001, 0b0111, 0b1000, 0b1110
          - CCW states 0b0010, 0b0100, 0b1011, 0b1101
*/
/***************************************************************************************************/
#ifndef CUI_DEVICES_AMT10X_h
#define CUI_DEVICES_AMT10X_h

#include <iostream>
#include <cmath>
#include <cassert>
#include <thread>
#include <wiringPi.h>

class Cui_Devices_Amt10x
{
  public:
    Cui_Devices_Amt10x(uint8_t num, uint16_t ppr, 
                       uint8_t* encoderA,
                       uint8_t* encoderB,
                       uint8_t* encoderX): 
        _NUM{num}, _PPR{ppr}, _RAD_PER_COUNT(2.0 * M_PI / (4.0 * ppr)), _DT(10000)
    {

      /* Init all arrays with all zeros */
      this->_prevValueAB = new uint8_t[this->_NUM];
      this->_currValueAB = new uint8_t[this->_NUM];
      this->_prevValueX = new uint8_t[this->_NUM];
      this->_currValueX = new uint8_t[this->_NUM];

      this->_counter = new int16_t[this->_NUM];
      this->_prev_count = new int16_t[this->_NUM];

      this->_count_vel = new float[this->_NUM];

      this->_encoderA = new uint8_t[this->_NUM];
      this->_encoderB = new uint8_t[this->_NUM];
      this->_encoderX = new uint8_t[this->_NUM];

      for(uint8_t i = 0; i < this->_NUM; i++)
      {
          this->_prevValueAB[i] = 0;
          this->_currValueAB[i] = 0;
          this->_prevValueX[i] = 0;
          this->_currValueX[i] = 0;

          this->_counter[i] = 0;
          this->_prev_count[i] = 0;

          this->_count_vel[i] = 0;


          this->_encoderA[i] = encoderA[i];
          this->_encoderB[i] = encoderB[i];
          this->_encoderX[i] = encoderX[i];
      }
    }

    void begin();

    void getPosition(float (&pos)[]);

    void getVelocity(float (&vel)[]);

  protected:
    int16_t* _counter;        // encoder click counter, limit -32768..32767
    float* _count_vel;      // encoder count velocity, (counts per second)

  private:

    void readABX(); // Function that reads out current values and stores them in class members
    void run(); // Function that is executed in a thread to run the driver permanently

    std::thread* executer; // Thread that executer the run function

    const uint8_t _NUM; // Number of encoders
    const uint16_t _PPR; // Pulses per Revolution: resolution of the encoder
    const float _RAD_PER_COUNT; // Radians per count: convenience variable to comput angle more quickly
    const uint16_t _DT; // Time Delta for Velocity estimate

    uint8_t* _prevValueAB;    // previouse state of "A"+"B"
    uint8_t* _currValueAB;    // current   state of "A"+"B"
    uint8_t* _currValueX;     // current state of pin "X"
    uint8_t* _prevValueX;     // prev state of pin "X"
    uint32_t _prev_count_ts = 0; // timestamp of last count (in micro seconds since program start)
    uint32_t _curr_count_ts = 0; // timestamp of current count (in micro seconds since program start)
    int16_t* _prev_count;

    uint8_t* _encoderA;           // pin "A"
    uint8_t* _encoderB;           // pin "B"
    uint8_t* _encoderX;           // pin "X"
};

#endif