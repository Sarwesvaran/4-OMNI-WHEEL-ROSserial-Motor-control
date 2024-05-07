#include <ros.h>
#include <RMCS2303drive.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include "MapFloat.h"

RMCS2303 rmcs;  // creation of motor driver object
// slave ids to be set on the motor driver refer to the manual in the reference section
// byte slave_id1 = 1;
// byte slave_id2 = 2;
// byte slave_id3 = 3;
// byte slave_id4 = 4;

byte slave_id1 = 4;
byte slave_id2 = 1;
byte slave_id3 = 2;
byte slave_id4 = 3;

ros::NodeHandle nh;        // Node handle Object
geometry_msgs::Twist msg;  // msg variable of data type twist


// std_msgs::Int32 wheel1;  // for storing left encoder value
// std_msgs::Int32 wheel2;  // for storing right encoder value

// std_msgs::Int32 wheel3;  // for storing left encoder value
// std_msgs::Int32 wheel4;  // for storing right encoder value

std_msgs::Int32MultiArray motor_rev;

// Publisher object with topic names wheel1_ticks and wheel2_ticks for publishing Enc Values
// ros::Publisher wheel1_ticks("wheel1_ticks", &wheel1);
// ros::Publisher wheel2_ticks("wheel2_ticks", &wheel2);
// ros::Publisher wheel3_ticks("wheel3_ticks", &wheel3);
// ros::Publisher wheel4_ticks("wheel4_ticks", &wheel4);


ros::Publisher value_wheels("value_wheels", &motor_rev);


// Make sure to specify the correct values here
//*******************************************
double wheel_rad = 0.05, wheel_sep = 0.1725;  // wheel radius and wheel sepration in meters.
//******************************************
double w1 = 0, w2 = 0, w3 = 0, w4 = 0;
// double rad_wheel = 0.05, rad_bot = 0.25;
double rad_wheel = 0.05, rad_bot = 0.1725;  // new bot wheel rad after coupler change
double speed_ang;
double speed_lin_x;
double speed_lin_y;
double msg_value;
int var_reset;
// double leftPWM;
// double rightPWM;

double w1_pwm, w2_pwm, w3_pwm, w4_pwm;
String odom_R = " ";

void odom_resetCb(const std_msgs::String &msg) {
  Serial3.println("Odom loop done");
  
  odom_R = String(msg.data);
  Serial3.println(odom_R);

  if (odom_R == "1") {
    rmcs.SET_HOME(slave_id1);
    rmcs.SET_HOME(slave_id2);
    rmcs.SET_HOME(slave_id3);
    rmcs.SET_HOME(slave_id4);

    Serial3.println("Odom Reset done");
    odom_R = " ";
  }
}

void messageCb(const geometry_msgs::Twist &msg)  // cmd_vel callback function definition
{
  speed_lin_x = max(min(msg.linear.x, 1.0f), -1.0f);  // limits the linear x value from -1 to 1
  speed_lin_y = max(min(msg.linear.y, 1.0f), -1.0f);
  speed_ang = -1 * max(min(msg.angular.z, 1.0f), -1.0f);  // limits the angular z value from -1 to 1

  // Kinematic equation for finding the left and right velocities
  // w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  // w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));

  w1 = 20 * ((-0.707 * speed_lin_x) + (-0.707 * speed_lin_y) + (rad_bot * speed_ang));  // kinematic formula
  w2 = 20 * ((0.707 * speed_lin_x) + (-0.707 * speed_lin_y) + (rad_bot * speed_ang));
  w3 = 20 * ((0.707 * speed_lin_x) + (0.707 * speed_lin_y) + (rad_bot * speed_ang));
  w4 = 20 * ((-0.707 * speed_lin_x) + (0.707 * speed_lin_y) + (rad_bot * speed_ang));


  Serial3.print(w1);
  Serial3.print(" || ");
  Serial3.print(w2);
  Serial3.print(" || ");
  Serial3.print(w3);
  Serial3.print(" || ");
  Serial3.println(w4);


  if (w1 == 0 && w2 == 0 && w3 == 0 && w4 == 0) {
    w1_pwm = 0;
    w2_pwm = 0;
    w3_pwm = 0;
    w4_pwm = 0;

    rmcs.Brake_Motor(slave_id1, 0);
    rmcs.Brake_Motor(slave_id2, 0);
    rmcs.Brake_Motor(slave_id3, 0);
    rmcs.Brake_Motor(slave_id4, 0);  // if none of the above break the motors both in clockwise n anti-clockwise direction
    // rmcs.Brake_Motor(slave_id1, 1);
    // rmcs.Brake_Motor(slave_id2, 1);
    // rmcs.Brake_Motor(slave_id3, 1);
    // rmcs.Brake_Motor(slave_id4, 1);

  } else {
    w1_pwm = mapFloat(fabs(w1), 0.0, 14.14, 0, 4800);  // mapping the right wheel velocity with respect to Motor PWM values
    w2_pwm = mapFloat(fabs(w2), 0.0, 14.14, 0, 4800);  // mapping the right wheel velocity with respect to Motor PWM values
    w3_pwm = mapFloat(fabs(w3), 0.0, 14.14, 0, 4800);  // mapping the right wheel velocity with respect to Motor PWM values
    w4_pwm = mapFloat(fabs(w4), 0.0, 14.14, 0, 4800);  // mapping the right wheel velocity with respect to Motor PWM values

    Serial3.print(w1_pwm);
    Serial3.print(" | | ");
    Serial3.print(w2_pwm);
    Serial3.print(" | | ");
    Serial3.print(w3_pwm);
    Serial3.print(" | | ");
    Serial3.println(w4_pwm);

    rmcs.Speed(slave_id1, w1_pwm);
    rmcs.Speed(slave_id2, w2_pwm);
    rmcs.Speed(slave_id3, w3_pwm);
    rmcs.Speed(slave_id4, w4_pwm);


    rmcs.Enable_Digital_Mode(slave_id1, w1 > 0);
    rmcs.Enable_Digital_Mode(slave_id2, w2 > 0);  // forward condition
    rmcs.Enable_Digital_Mode(slave_id3, w3 > 0);
    rmcs.Enable_Digital_Mode(slave_id4, w4 > 0);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);  // creation of subscriber object sub for recieving the cmd_vel
ros::Subscriber<std_msgs::String> sub_1("odom_reset", &odom_resetCb);

void setup() {
  rmcs.Serial_selection(0);  // 0 -> for Harware serial tx1 rx1 of arduino mega
  rmcs.Serial0(9600);
  rmcs.begin(&Serial1, 9600);

  Serial3.begin(9600);

  nh.initNode();        // initialzing the node handle object
                        // nh.getHardware()->setBaud(57600);
  nh.subscribe(sub);    // subscribing to cmd vel with sub object
  nh.subscribe(sub_1);  // subscribing to cmd vel with sub object

  // nh.advertise(wheel1_ticks);  // advertise the wheel1_ticks topic
  // nh.advertise(wheel2_ticks);  // advertise the wheel1_ticks topic

  // nh.advertise(wheel3_ticks);  // advertise the wheel1_ticks topic
  // nh.advertise(wheel4_ticks);  // advertise the wheel1_ticks topic
  nh.advertise(value_wheels);  // advertise the wheel1_ticks topic
}

void loop() {
  // wheel1.data = rmcs.Position_Feedback(slave_id1);
  // wheel2.data = -rmcs.Position_Feedback(slave_id2);
  // wheel3.data = rmcs.Position_Feedback(slave_id3);
  // wheel4.data = -rmcs.Position_Feedback(slave_id4);
  int32_t encoder[4] = { rmcs.Position_Feedback(slave_id1), rmcs.Position_Feedback(slave_id2), rmcs.Position_Feedback(slave_id3), rmcs.Position_Feedback(slave_id4) };
  motor_rev.data = encoder;
  // motor_rev.data[0] = rmcs.Position_Feedback(slave_id1);
  // motor_rev.data[1] = rmcs.Position_Feedback(slave_id2);
  // motor_rev.data[2] = rmcs.Position_Feedback(slave_id3);
  // motor_rev.data[3] = rmcs.Position_Feedback(slave_id4);

  motor_rev.data_length = 4;

  // wheel1_ticks.publish(&wheel1);  // publish left enc values
  // wheel2_ticks.publish(&wheel2);  // publish right enc values
  // wheel3_ticks.publish(&wheel3);
  // wheel4_ticks.publish(&wheel4);

  value_wheels.publish(&motor_rev);

  // for (int i = 0; i < 4; i++) {
  //   Serial3.print(encoder[i]);
  //   Serial3.print(" | ");
  // }
  // Serial3.println();
  nh.spinOnce();

  delay(1);
}