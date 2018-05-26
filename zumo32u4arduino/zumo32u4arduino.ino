#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <LSM303.h>
#include <Zumo32U4.h>

long timer=0;              // Elapsed time since program started (milli second)
int vright = 0;            // Left Morter velocity (speed of motor)
int vleft = 0;             // Right Morter velocity (speed of motor)
int basespeed = 150;        // Base speed of Morter (Effective Range: 1 - 350)
long positionLeft  = 0;    // For encoder verification
long positionRight = 0;    // For encoder verification
long newLeft, newRight;    // Value of Encorder
std_msgs::String str_msg;  // Sensor value to be published

LSM303 compass;            // Magnetometer
L3G gyro;                  // Gyrometer
Zumo32U4Motors motors;     // Morter
Zumo32U4Encoders encoders; // Encoder
ros::NodeHandle nh;        // NodeHandler of ROS

void motorcontrol(const std_msgs::String& cmd_msg)
{

  // I : forward    ,  , : backward
  // L : turn right ,  J : turn left
  // S : stop

  String cmd = "";

  if (strlen(cmd_msg.data) != 0)
  {
    cmd = cmd_msg.data;
  }

  if (cmd == "i")
  {
    motors.setSpeeds(0, 0);
    delay(2);
    motors.setSpeeds(basespeed, basespeed);
    vleft = basespeed;vright = basespeed;
  }
  else if (cmd == ",")
  {
    motors.setSpeeds(0, 0);
    delay(2);
    motors.setSpeeds(-1 * basespeed, -1 * basespeed);
    vleft = -1 * basespeed;
    vright = -1 * basespeed;
  }
  else if (cmd == "l")
  {
    motors.setSpeeds(0, 0);
    delay(2);
    motors.setLeftSpeed(basespeed + 50);
    vleft = basespeed + 50;
    vright = 0;
  }
  else if (cmd == "j")
  {
    motors.setSpeeds(0, 0);
    delay(2);
    motors.setRightSpeed(basespeed + 50);
    vleft = 0;
    vright = basespeed + 50;
  }
  else if (cmd == "s")
  {
    motors.setSpeeds(0, 0);
    delay(2);
    vleft = 0;
    vright = 0;
  }

  int16_t newLeft = encoders.getCountsLeft();
  int16_t newRight = encoders.getCountsRight();

  if (!(encoders.checkErrorLeft()) && !(encoders.checkErrorRight())) {
    if (newLeft != positionLeft || newRight != positionRight) {
      positionLeft = newLeft;
      positionRight = newRight;
    }
  }

}

ros::Subscriber<std_msgs::String> sub("/command", motorcontrol);
ros::Publisher chatter("/sensorval", &str_msg);

void setup()
{
  //Serial.begin(9600);     // Debug Print

  Wire.begin();

  nh.initNode();           // Init ROS Node
  nh.advertise(chatter);   // Init ROS Publisher
  nh.subscribe(sub);       // Init ROS Subscriber

  compass.init();          // Init magnetometer
  compass.enableDefault();

  gyro.init();             // Init gyrometer
  gyro.enableDefault();
}

void loop()
{
  compass.read();   // Read magnetometer
  gyro.read();      // Read gyrometer
  timer = millis();
  String s = "";
  s += timer;       // [0]  Elapsed time since program started (milli second)
  s += ',';
  s += compass.a.x; // [1]  Accelerometer.x
  s += ',';
  s += compass.a.y; // [2]  Accelerometer.y
  s += ',';
  s += compass.a.z; // [3]  Accelerometer.z
  s += ',';
  s += compass.m.x; // [4]  Magnetometer.x
  s += ',';
  s += compass.m.y; // [5]  Magnetometer.y
  s += ',';
  s += compass.m.z; // [6]  Magnetometer.z
  s += ',';
  s += vleft;       // [7]  Left Morter velocity (speed of motor)
  s += ',';
  s += vright;      // [8]  Right Morter velocity (speed of motor)
  s += ',';
  s += newLeft;     // [9]  Left Morter odometry (Rotation angle of motor)
  s += ',';
  s += newRight;    // [10] Right Morter odometry (Rotation angle of motor)
  s += ',';
  s += gyro.g.x;    // [11] Gyrometer.x
  s += ',';
  s += gyro.g.y;    // [12] Gyrometer.y
  s += ',';
  s += gyro.g.z;    // [13] Gyrometer.z
  
  //Serial.println(s);  // Debug Print

  str_msg.data = s.c_str();
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1);
}


