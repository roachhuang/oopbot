#include "motor.h"

Motor::Motor(byte enable, byte in1, byte in2)
{
	En = enable;
	In1 = in1;
	In2 = in2;	
	init();
}

void Motor::init()
{
	pinMode(En, OUTPUT);
	pinMode(In1, OUTPUT);
	pinMode(In2, OUTPUT);
	stop();
	counter = 0;
}

void Motor::forward(byte pwm)
{
	analogWrite(En, abs(pwm));
	digitalWrite(In1, HIGH);
	digitalWrite(In2, LOW);
}

void Motor::backward(byte pwm)
{
	analogWrite(En, abs(pwm));
	digitalWrite(In1, LOW);
	digitalWrite(In2, HIGH);
}

void Motor::stop()
{
	analogWrite(En, 0);
}

float Motor::getSpeed()
{
	// return counter *2;	
	float speed;
	speed = counter * 5 *tickPerCm; // 200ms x 2 = 1s, cm/s
	// counter = 0;
	return speed;	
}
float Motor::getDistance(){
  float d;
  cli();
  d = circumference * counter/SPOKE;
  // counter = 0; 
  sei();
  return d;
}

 
