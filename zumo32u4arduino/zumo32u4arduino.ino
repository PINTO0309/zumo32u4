#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <LSM303.h>
#include <Zumo32U4.h>

long timer=0;              // Elapsed time since program started (milli second)
int vright = 0;            // Left Morter velocity (speed of motor)
int vleft = 0;             // Right Morter velocity (speed of motor)
int basespeed = 50;        // Base speed of Morter (Effective Range: 1 - 350)
long positionLeft  = 0;    // For encoder verification
long positionRight = 0;    // For encoder verification
long newLeft, newRight;    // Value of Encorder
std_msgs::String str_msg;  // Sensor value to be published

LSM303 compass;            // Magnetometer
Zumo32U4Motors motors;     // Morter
Zumo32U4Encoders encoders; // Encoder
ros::NodeHandle nh;        // NodeHandler of ROS

void motorcontrol(const std_msgs::String& cmd_msg)
{

  // F : forward    ,  B : backward
  // R : turn right ,  L : turn left
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

ros::Subscriber<std_msgs::String> sub("/zumo32u4/command", motorcontrol);
ros::Publisher chatter("/zumo32u4/sensorval", &str_msg);

void setup()
{
  nh.initNode();               // Init ROS Node
  nh.advertise(chatter);       // ROS Publisher
  nh.subscribe(sub);           // ROS Subscriber

  compass.init();              // Init magnetometer
  compass.enableDefault();
}

void loop()
{

  compass.read();              // Read magnetometer
  timer = millis();
  
  str_msg.data = "";
  str_msg.data += timer;       // [0]  Elapsed time since program started (milli second)
  str_msg.data += ',';
  str_msg.data += compass.a.x; // [1]  Accelerometer.x
  str_msg.data += ',';
  str_msg.data += compass.a.y; // [2]  Accelerometer.y
  str_msg.data += ',';
  str_msg.data += compass.a.z; // [3]  Accelerometer.z
  str_msg.data += ',';
  str_msg.data += compass.m.x; // [4]  Magnetometer.x
  str_msg.data += ',';
  str_msg.data += compass.m.y; // [5]  Magnetometer.y
  str_msg.data += ',';
  str_msg.data += compass.m.z; // [6]  Magnetometer.z
  str_msg.data += ',';
  str_msg.data += vleft;       // [7]  Left Morter velocity (speed of motor)
  str_msg.data += ',';
  str_msg.data += vright;      // [8]  Right Morter velocity (speed of motor)
  str_msg.data += ',';
  str_msg.data += newLeft;     // [9]  Left Morter odometry (Rotation angle of motor)
  str_msg.data += ',';
  str_msg.data += newRight;    // [10] Right Morter odometry (Rotation angle of motor)

  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1);
}


