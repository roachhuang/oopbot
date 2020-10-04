/*
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.9 _turn:=0.8 _key_timeout:=1
  rosrun rosserial_python serial_node.py /dev/ttyUSB0
  rostopic echo /left_wheel_velocity
  rosrun rqt_plot rqt_plot

  roslaunch mypkg oopbot.launch
  rostopic pub -1 /pidCb geometry_msgs/Point32  '{x: 3, y: 0.0, z: 0.0}'
*/
#include "motor.h"
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
// #include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>

#include <ros/time.h>
// #include <TimerOne.h>

// #define LOOP_TIME 500000 // 500ms

#define ENA 6
#define IN1 7 // left
#define IN2 8

#define ENB 9 // right, timerone in mega 2560 uses pin 11
#define IN3 4 // r
#define IN4 5

#define left_encoder_pin 2
#define right_encoder_pin 3 // blue
// const float radius = 0.0325; //wheel radius
const float L = 0.115; //distance between wheels
#define max_linear 60  //cm/s
unsigned long prevTime;

Motor motorL(ENA, IN1, IN2);
Motor motorR(ENB, IN3, IN4);
//////PID
double inputL, outputL, setPointL;
double pidLPart[] = {2.84, 1.2, 0}; //kp = 0.040,ki = 0.0005,kd =0.0011;
PID pidL(&inputL, &outputL, &setPointL, pidLPart[0], pidLPart[1], pidLPart[2], DIRECT);

double inputR, outputR, setPointR;
const double pidRPart[] = {3.18, 0.5, 0.0}; //kp = 0.040,ki = 0.0005,kd =0.0011;
PID pidR(&inputR, &outputR, &setPointR, pidRPart[0], pidRPart[1], pidRPart[2], DIRECT);

float vR, vL;
float theta = 0, x = 0, y = 0;

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

std_msgs::Float64 control_effort;
ros::Publisher control_effort_pub("/control_effort", &control_effort);
std_msgs::Float64 setpoint;
ros::Publisher setpoint_pub("/setpoint", &setpoint);
std_msgs::Float64 state;
// ros::Publisher state_pub("/state", &state);

std_msgs::Int16 left_wheel_vel;
ros::Publisher left_wheel_vel_pub("/left_wheel_velocity", &left_wheel_vel);

std_msgs::Int16 right_wheel_vel;
ros::Publisher right_wheel_vel_pub("/right_wheel_velocity", &right_wheel_vel);

geometry_msgs::Twist sensor_vel;
ros::Publisher sensor_vel_pub("/sensor_velocity", &sensor_vel);

void docount_left() // counts from the speed sensor
{
  motorL.counter++; // increase +1 the counter value
}

void docount_right() // counts from the speed sensor
{
  motorR.counter++; // increase +1 the counter value
}

void timerIsr()
{
  // Timer1.detachInterrupt();  //stop the timer
  // Left Motor Speed

  inputL = left_wheel_vel.data = motorL.getSpeed(); // timerIsr 500ms*2 = 1s, cm/s
  left_wheel_vel_pub.publish(&left_wheel_vel);
  // state_pub.publish(&state);

  inputR = right_wheel_vel.data = motorR.getSpeed();
  right_wheel_vel_pub.publish(&right_wheel_vel);

  if (setPointR != 0) {
    pidR.Compute();
    // may be reset counter here to make speed calculation more accuracy.
    (outputR < 0) ? motorR.backward(abs(outputR)) : motorR.forward(outputR);
  }
  else {
    motorR.stop();
  }
  if (setPointL != 0) {
    pidL.Compute();
    // control_effort.data = outputL;
    // control_effort_pub.publish(&control_effort);
    // may be reset counter here to make speed calculation more accuracy.
    (outputL < 0) ? motorL.backward(abs(outputL)) : motorL.forward(outputL);
  }
  else {
    motorL.stop();
  }
  /*
    sensor_vel.linear.x = radius*(left_wheel_vel.data + right_wheel_vel.data)/2;
    sensor_vel.linear.y = 0;
    sensor_vel.linear.z = 0;
    sensor_vel.angular.x = 0;
    sensor_vel.angular.y = 0;
    sensor_vel.angular.z = radius*(left_wheel_vel.data + right_wheel_vel.data)/L;
    sensor_vel_pub.publish(&sensor_vel);
  */
  // Timer1.attachInterrupt( timerIsr );  //enable the timer
}

ros::Subscriber<std_msgs::Int16> subCmdLeft("cmd_left_wheel", cmdLeftWheelCB);
ros::Subscriber<std_msgs::Int16> subCmdRight("cmd_right_wheel", cmdRightWheelCB);

void cmdVelCB(const geometry_msgs::Twist &twist)
{
  int gain = 100; // convert m to cm/s
  vL = gain * (twist.linear.x - twist.angular.z * L / 2);
  vR = gain * (twist.linear.x + twist.angular.z * L / 2);
  setpoint.data = setPointL = abs(vL);
  setpoint_pub.publish(&setpoint);
  setPointR = abs(vR);
  /*
    if (vl > max_linear)
    vl = max_linear;
    if (vR > max_linear)
    vR = max_linear;
  */
}
void pidCb(const geometry_msgs::Point32 &msg)
{
  pidLPart[0] = msg.x;
  pidLPart[1] = msg.y;
  pidLPart[2] = msg.z;
  pidL.SetTunings(pidLPart[0], pidLPart[1], pidLPart[2]);
}

void updateOdom() {
  float dL, dR, dC;
  
  dL = motorL.getDistance();
  dR = motorR.getDistance();
  dC = (dL + dR) / 2.0;

  //if (theta > 2.0 * PI) theta -= 2.0 * PI;
  //if (theta < 0.0) theta += 2.0 * PI;
  x += dC * cos(theta);
  y += dC * sin(theta);
  theta += ((dR - dL) / L);
  // update  transform
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z  = 0.0;
  // converting from euler angle to quaternion form
  t.transform.rotation = tf::createQuaternionFromYaw(theta);

  // filling the odometry
  t.header.stamp = nh.now();
  t.header.frame_id  = "/odom";     // Global odometry frame
  t.child_frame_id = "/base_link";  // Robot local frame

  /////////////////////////////////////////////////////////////
  /*
       position
        t.pose.pose.position.x = x;
        t.pose.pose.position.y = y;
        t.pose.pose.position.z = 0.0;
        t.pose.pose.orientation  = odom_quat;

        //  velocity
        t.twist.twist.linear.x = vx;
        t.twist.twist.linear.y = vy;
  */

  //  publishing  the odometry  and the new tf
  broadcaster.sendTransform(t);         // broadcasting updated result
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", cmdVelCB);
// pid gains
ros::Subscriber<geometry_msgs::Point32> subPidCb("pidCb", pidCb);

void setup()
{
  // put your setup code here, to run once:
  prevTime = 0;

  //Setup for encoders
  pinMode(right_encoder_pin, INPUT_PULLUP);
  pinMode(left_encoder_pin, INPUT_PULLUP);

  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31KHz PWM to prevent motor noise
  pidL.SetOutputLimits(-255, 255);
  pidL.SetSampleTime(100);
  pidL.SetMode(AUTOMATIC);
  //pidL.SetTunings(left_kp, left_ki, left_kd);

  pidR.SetOutputLimits(-255, 255);
  pidR.SetSampleTime(100);
  pidR.SetMode(AUTOMATIC);
  //pidR.SetTunings(right_kp, right_ki, right_kd);

  attachInterrupt(digitalPinToInterrupt(left_encoder_pin), docount_left, RISING);   // increase counter when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(right_encoder_pin), docount_right, RISING); // increase counter when speed sensor pin goes High

  nh.initNode();
  broadcaster.init(nh);                 // odom data broadcaster init

  nh.subscribe(subCmdRight);
  nh.subscribe(subCmdLeft);
  nh.subscribe(subCmdVel);
  nh.subscribe(subPidCb);

  nh.advertise(left_wheel_vel_pub);
  nh.advertise(right_wheel_vel_pub);
  nh.advertise(sensor_vel_pub);

  nh.advertise(control_effort_pub);
  nh.advertise(setpoint_pub);
  // nh.advertise(state_pub);

  //Timer1.initialize(LOOP_TIME);
  //Timer1.attachInterrupt(timerIsr); // enable the timer
}

void loop()
{  
  unsigned long nowTime = millis();

  //sample the range data from the ultrasound sensor and
  /* publish the range value once every 50 milliseconds
    if ((millis() - range_timer) > 50) {
    range_msg.range = getRange();
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_timer = millis() + 50;
    }
  */


  if ((nowTime - prevTime) > 200) { // publish odom every 100ms
    prevTime = nowTime;
    timerIsr();
    updateOdom();
    motorL.counter = 0;
    motorR.counter = 0;
  }

  // odom_pub.publish(state);
  nh.spinOnce();
}


void cmdLeftWheelCB(const std_msgs::Int16 &msg)
{
  (msg.data >= 0) ? motorL.forward(msg.data) : motorL.backward(msg.data);
}

void cmdRightWheelCB(const std_msgs::Int16 &msg)
{
  (msg.data >= 0) ? motorR.forward(msg.data) : motorR.backward(msg.data);
}
