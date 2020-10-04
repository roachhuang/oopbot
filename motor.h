// class declaration, which is the interface that any client using this class will use

#ifndef MY_MOTOR_H // make sure that the class will not be included more than once
#define MY_MOTOR_H

/* This include is necessary to use the specific Arduino functions and types (think of pinMode(), digitalWrite(), byte).
   In the “main” file we don’t need to write it because it’s automatically added when you compile your code.
   But in any other file, you need to add it by yourself.
*/
#include <Arduino.h>

// #define tickPerCm (100 * 2.0 * PI * radius / SPOKE)

class Motor
{
  private:
    const float radius = 0.0325; //wheel radius
    const float circumference = 2 * PI * radius;
    // const float L = 0.115;		 //distance between wheels
    const byte SPOKE = 20;
    const float tickPerCm = circumference * 100 / SPOKE;

    byte En, In1, In2;

  public:
    volatile uint8_t counter;
    Motor(byte enable, byte in1, byte in2);
    void init();
    void forward(byte pwm);
    void backward(byte pwm);
    void stop();
    float getSpeed();
    float getDistance();
};

#endif
