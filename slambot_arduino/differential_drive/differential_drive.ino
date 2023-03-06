#include <Wire.h>
#include <MPU6050.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>

//Declare global variables
nav_msgs::Odometry odometry;
sensor_msgs::Imu imu;
sensor_msgs::JointState joint_states;
tf2_msgs::TFMessage transform;
geometry_msgs::Twist cmd_vel;
geometry_msgs::Vector3 acc, gyro;
ros::Publisher mpu_acc("imu/accelerometer", &acc);

const int MPU = 0x68; // MPU6050 I2C address

float g = 9.81;//value of gravitation acceleration

// define pins
//encoder
int ENC1A = 2;
int ENC1B = 3;  // Encoder pins for right motor
int ENC2A  = 18;
int ENC2B = 19; // Encoder pins for left motor

//L298N Motor Driver
int ENA = 5 ; //motor driver PWM pins
int ENB = 10; //motor driver PWM pins
int INT1 = 6; //direction control for left motor
int INT2 = 7; //direction control for left motor
int INT3 = 8; //direction control for right motor
int INT4 = 9; //direction control for right motor

//MPU6050
 

//Acceleration and Rotation rate values
float ax, ay, az; 
float gyrox, gyroy, gyroz;  

//Enoder count values
int pulses_left = 0;
int pulses_right = 0;

//Variables to determine each cycle to get robot odometry position
double pos_left = 0;
double pos_right = 0;
double pos_left_mm =0;
double pos_right_mm = 0;
double pos_old_left_mm = 0;
double pos_old_right_mm = 0;
double pos_left_diff = 0;
double pos_right_diff = 0;
double pos_average_diff = 0;
double pos_total = 0;

// tf variables to broadcast
double y=0;                                   // position in y direction
double x=0;                                   // position in x direction
double theta = 0;                             //yaw angle in radians

//Robot Values
float wheel_base =  ;//please enter wheel dist. value
float radius = 0.065 ; // Radius of wheel = 65mm
float encoderppr = 400;



//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(ENC1A) == digitalRead(ENC1B)){
    pos_left++;
    pulses_left++;
  }
  else{
    pos_left--;
    pulses_left--;
  } 
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(ENC2A) == digitalRead(ENC2B)){
    pos_right--;
    pulses_right--;
  }
  else{
    pos_right++;
    pulses_right++;
  }
}

//Calculate motor velocities inside this
void velocity_callback(const geometry_msgs::Twist& vel_msg)
{
  // based on twist.subscriber
  float linear_x = cmd_vel.linear.x;
  float linear_y = cmd_vel.linear.y;
  float angle = cmd_vel.angular.z;

  /*float vel_right = (angle * wheel_base) / 2 + linear;
  float vel_left = linear * 2 - vel_right;*/
  
  vel_msg.linear.x = linear_x;
  vel_msg.linear.y = linear_y;
  vel_msg.angular.z = angle;
}



//Calculate transform for base_footprint
void calculate_transform()
{
    if (speed_handle_left == speed_handle_right);
    else
    theta += ((pos_left_diff-pos_right_diff)/wheeltrack);

    //please complete the function
    transform.Quaternion.z = theta;
}

//Calculate odometry from encoder values
void calculate_odometry()
{
  // Determining position
    pos_left_mm = (pos_left/encoderppr) * 2*PI*radius;
    pos_right_mm = (pos_right/encoderppr) * 2*PI*radius;
    pos_left_diff = pos_left_mm - pos_old_left_mm;
    pos_right_diff = pos_right_mm - pos_old_right_mm;
    pos_old_left_mm = pos_left_mm;
    pos_old_right_mm = pos_right_mm;

    pos_average_diff = (pos_left_diff + pos_right_diff)/2;
    pos_total += pos_average_diff;

    if (speed_handle_left == speed_handle_right);
    else
    theta += ((pos_left_diff-pos_right_diff)/wheeltrack);    

    if (theta > PI)
    theta -= TWO_PI;
    if(theta < -(PI))
    theta += TWO_PI;

    y += pos_average_diff * sin(theta);
    x += pos_average_diff * cos(theta);

    odometry.pose.pose.position.x = x;
    odometry.pose.pose.position.y = y;
    odometry.pose.pose.position.z = 0.0;
}

//Calculate joint states from encoder values
void calculate_joint_states()
{
  /*string[] name, float64[] position, float64[] velocity, float64[] effort*/
  joint_states.name = revolute;
  joint_states.postion = theta;
  joint_states.velocity= cmd_vel.velocity.x;
  //joint_states.effort=;
  
}

//Calculate acceleration from IMU values
void calculate_imu()
{
  //mpu.read_acc();//get data from the accelerometer
  //mpu.read_gyro();//get data from the gyroscope

  //converts gyrovalues to radian per sec

  acc.x = ((float)mpu.ax*g)/16384;
  acc.y = ((float)mpu.ay*g)/16384;
  acc.z = ((float)mpu.az*g)/16384;

  //configuration of gyro values to rad/sec required
  gyro.x = mpu.gx;
  gyro.y = mpu.gy;
  gyro.z = mpu.gz;

  mpu_acc.publish(&acc);
  mpu_gyro.publish(&gyro);
}

//Initialise nodes, publishers, subscribers and serial monitor
void setup()
{
  Serial.begin(9600);                           //modified baudrate
  Serial.println("Starting...");
  
  //Initial Setup for Encoder
  pinMode(ENC1A,INPUT);
  pinMode(ENC1B,INPUT);
  pinMode(ENC2A,INPUT);
  pinMode(ENC2B,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1A),motor1_encoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A),motor2_encoder,RISING);

  pinMode(ENA, OUTPUT);   //please change these
  pinMode(ENB, OUTPUT);  //please change these
  
  /*
  //Initial Setup for MPU6050 I2C
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);
  */

  //ROS Publishers and subscribers
  node.initNode();
  node.advertise(odometry_publisher);
  node.advertise(imu_publisher);
  node.advertise(transform_publisher);
  node.advertise(joint_state_publisher);
  node.subscribe(twist_subscriber);
}

//Write your program logic
void loop()
{
  //Controlling speed (0 = off and 255 = max speed):
  /*Caution: The motors are 300 rpm please don't use PWM values greater than 75*/
  analogWrite(ENA, 50); //ENA pin
  analogWrite(ENB, 50); //ENB pin

  //Controlling spin direction of motors:
  digitalWrite(INT3, HIGH);
  digitalWrite(INT4, LOW);

  digitalWrite(INT1, HIGH);
  digitalWrite(INT2, LOW);
  delay(1000);

  digitalWrite(INT3, LOW);
  digitalWrite(INT4, HIGH);

  digitalWrite(INT1, LOW);
  digitalWrite(INT2, HIGH);
  delay(1000);
  
  node.spinOnce();
  delay(100);
}
