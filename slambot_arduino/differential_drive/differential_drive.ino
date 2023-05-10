#include <Wire.h>
#include <MPU6050.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>

#define looptime 100

//constants
float g = 9.81;//value of gravitation acceleration
float pi = 3.14159;//value of Pi

// define pins
//encoder
int ENC1A = 2;
int ENC1B = 3;  // Encoder pins for right motor
int ENC2A  = 18;
int ENC2B = 19; // Encoder pins for left motor

//L298N Motor Driver
int ENA = 5 ; //motor driver PWM pins for RIGHT motor
int ENB = 10; //motor driver PWM pins for LEFT motor
int INT_1 = 6; //direction control for RIGHT motor
int INT_2 = 7; //direction control for RIGHT motor
int INT_3 = 8; //direction control for LEFT motor
int INT_4 = 9; //direction control for LEFT motor

//Acceleration and Rotation rate values
int16_t ax, ay, az;
int16_t gx, gy, gz;

//Timer for mpu values
float timeStep = 0.01;
unsigned long timer = 0;

//Encoder count values
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

//Variables for Desired Traversal
double pos_left_desired = 0;
double pos_right_desired = 0;


// tf variables to broadcast
double y=0;                                   // position in y direction
double x=0;                                   // position in x direction
double theta = 0;                             //yaw angle in radians

//Robot Values
float wheel_base = 0.193;//please enter wheel dist. value
float radius = 0.065 ; // Radius of wheel = 65mm
float encoderppr = 400;

//Callback functions
float linear_x;
float linear_y;
float linear;
float angle;

//Motor Velocities
float vel_right;
float vel_left;

void velocity_callback(const geometry_msgs::Twist &vel_msg)
{
  // based on twist.subscriber
  linear_x = -1*vel_msg.linear.x;
  //linear_y = vel_msg.linear.y;
  angle = vel_msg.angular.z;

  //linear =sqrt(sq(linear_x) + sq(linear_y)); 

  vel_right = (angle * wheel_base) / 2 + linear_x;   //right motor velocity
  vel_left = linear_x * 2 - vel_right;               //left motor velocity
}

//Declare global variables
ros::NodeHandle node;                   //initialise ros node 
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odometry;
sensor_msgs::Imu imu_msg;
sensor_msgs::JointState joint_states;
tf2_msgs::TFMessage transform;
geometry_msgs::Twist cmd_vel;         
geometry_msgs::Vector3 acc, gyro;

//Delaring Publishers
ros::Publisher mpu_acc("imu/accelerometer", &acc);
ros::Publisher mpu_gyro("/imu/data", &gyro);
ros::Publisher Joint_States("/joint_state", &joint_states);
ros::Publisher odom("/odom", &odometry);

//Declaring Subsribers
ros::Subscriber<geometry_msgs::Twist> vel_msg("/cmd_vel", velocity_callback);


MPU6050 mpu;

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

//move left motor
void left_traverse(float vel_left)
{
  int encoder_count;
  encoder_count = (vel_left * looptime * encoderppr)/(2 * PI * radius);

  if(encoder_count < 0)
  {
    analogWrite(ENB, 40); 
    digitalWrite(INT_3, HIGH);
    digitalWrite(INT_4, LOW);

  }
    else if(encoder_count > 0)
  {
    analogWrite(ENB, 40); 
    digitalWrite(INT_3, LOW);
    digitalWrite(INT_4, HIGH);
  }
  else if(encoder_count == 0)
  {
    analogWrite(ENB, 40); 
    digitalWrite(INT_3, LOW);
    digitalWrite(INT_4, LOW);
  }

}

//move right motor
void right_traverse(float vel_right)
{
  int encoder_count;
  encoder_count = (vel_right * looptime * encoderppr)/(2 * PI * radius);

  if(encoder_count < 0)
  {
    analogWrite(ENA, 40); 
    digitalWrite(INT_1, HIGH);
    digitalWrite(INT_2, LOW);

  }
    else if(encoder_count > 0)
  {
    analogWrite(ENA, 40); 
    digitalWrite(INT_1, LOW);
    digitalWrite(INT_2, HIGH);
  }
  else if(encoder_count == 0)
  {
    analogWrite(ENA, 40); 
    digitalWrite(INT_1, LOW);
    digitalWrite(INT_2, LOW);
  }

}

//Calculate transform for base_footprint
void calculate_transform()
{
  //theta += ((pos_left_diff-pos_right_diff)/wheel_base);
  //please complete the function
  t.transform.rotation.z = theta;
}

//Calculate and Publishes odometry from encoder values
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

    theta += ((pos_left_diff-pos_right_diff)/wheel_base);    

    if (theta > PI)
    theta -= TWO_PI;
    if(theta < -(PI))
    theta += TWO_PI;

    y += pos_average_diff * sin(theta);
    x += pos_average_diff * cos(theta);

    odometry.pose.pose.position.x = x;
    odometry.pose.pose.position.y = y;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation.x = 0.0;
    odometry.pose.pose.orientation.y = 0.0;
    odometry.pose.pose.orientation.z = theta;
    odometry.pose.pose.orientation.w = 0.0;

    odom.publish(&odometry);

}


//Calculate joint states from encoder values
sensor_msgs::JointState calculate_joint_states()
{
  /*string[] name, float64[] position, float64[] velocity, float64[] effort*/
  *joint_states.name = "left_motor", "right_motor";
  *joint_states.position = pos_left_mm, pos_right_mm;
  *joint_states.velocity = vel_left, vel_right;
  //*joint_states.position = theta; //CORRRECTION
  //*joint_states.velocity = (vel_right + vel_left)/2;  //CORRECTION
  //joint_states.effort=;
  return joint_states;
  Joint_States.publish(&joint_states);
}

//Calculate acceleration from IMU values
sensor_msgs::Imu calculate_imu()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  /*
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  //mpu.read_acc();//get data from the accelerometer
  //pu.read_gyro();//get data from the gyroscope\
  */

  //converts accel values to m per sec^2
  acc.x = ((float)ax*g)/16384;
  acc.y = ((float)ay*g)/16384;
  acc.z = ((float)az*g)/16384;

  //convert of gyro values to rad/sec^2 
  gyro.x = ((float)gx*pi)/180;
  gyro.y = ((float)gy*pi)/180;
  gyro.z = ((float)gz*pi)/180;

  mpu_acc.publish(&acc);
  mpu_gyro.publish(&gyro);
}

//Initialise nodes, publishers, subscribers and serial monitor
void setup()
{
  Serial.begin(57600);                           //modified baudrate
  Serial.println("Starting...");

  //Initializing IMU
  mpu.initialize();
  mpu.CalibrateGyro(6);
  
  //Initial Setup for Encoder
  pinMode(ENC1A,INPUT);
  pinMode(ENC1B,INPUT);
  pinMode(ENC2A,INPUT);
  pinMode(ENC2B,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1A),encoderLeftMotor,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A),encoderRightMotor,RISING);

  pinMode(ENA, OUTPUT);   //please change these
  pinMode(ENB, OUTPUT);  //please change these

  //Initial Setup for MPU6050 I2C in Arduino C
  /*
  //Initial Setup for MPU6050 I2C in AVRC
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
  node.subscribe(vel_msg);
  node.advertise(odom);
  node.advertise(mpu_gyro);
  node.advertise(mpu_acc);
  node.advertise(Joint_States);
}

//Write your program logic
void loop()
{
  //Controlling speed (0 = off and 255 = max speed):
  /*Caution: The motors are 300 rpm please don't use PWM values greater than 75*/
  /*
  analogWrite(ENA, 50); //ENA pin
  analogWrite(ENB, 50); //ENB pin

  //Controlling spin direction of motors:
  digitalWrite(INT_4, HIGH);
  digitalWrite(INT_3, LOW);

  digitalWrite(INT_2, HIGH);
  digitalWrite(INT_1, LOW);
  */
 
  calculate_imu();
  calculate_transform();
  calculate_odometry();
  calculate_joint_states();
  left_traverse(vel_left);
  right_traverse(vel_right);

  //publishOdometry(LOOPTIME);

  /*
  delay(1000);

  digitalWrite(INT_3, LOW);
  digitalWrite(INT_4, HIGH);

  digitalWrite(INT_1, LOW);
  digitalWrite(INT_2, HIGH);
  delay(1000);
  */
  node.spinOnce();
}
