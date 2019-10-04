/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 *
 *
 *  Servo       Arduino
 *  GND         GND
 *  5 volts     5 volts
 *  Signal      Pin 2
 *  
 *  
 *  Pot         Arduino
 *  1           3.3 volts
 *  2           A0
 *  3           AREF
 */



#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;
std_msgs::Float32 steer_msg;
ros::Publisher steer_pub("steer_sensor", &steer_msg);

Servo servo;

int val = 0;  // Potentiometer Value

float mapFloat(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void servo_cb( const geometry_msgs::Twist& cmd_msg){
  // Read in steering command on /cmd_vel
  float servo_cmd;
  
  //convert steering angle from rad to deg
  float steer_cmd = cmd_msg.angular.z * (180 / PI);
    
  // Change cmd_vel -1 to 1 to servo position 0-180
  if (steer_cmd >= 0){
    servo_cmd = mapFloat(steer_cmd, 0, 60, 90, 180);
  } else {
    servo_cmd = mapFloat(steer_cmd, 0, -60, 90, 0);
  }
  
  servo.write(servo_cmd);  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

//  Setup ROS subscriber to listen to Topic
//  <geometry_msgs::Twist>  Message DataType http://wiki.ros.org/common_msgs
//  When new message is published on Topic it will go to function
//  subscriber_name("ROS_topic_name", FUNCTION_CALLBACK)
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", servo_cb);


void setup(){
  Serial.begin(57600);
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(steer_pub);
  
  servo.attach(2); //attach it to pin 2
  pinMode(A0, INPUT);
}

void loop(){
 
  // read the input pin
  val = analogRead(A0); 
  //  Store new reading into ROS message
  steer_msg.data = val;
  //  Publish new ROS message on Topic
  steer_pub.publish( &steer_msg );
  
  nh.spinOnce();
  delay(1);
}
