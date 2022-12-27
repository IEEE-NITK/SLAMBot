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

int motor_right_pin1 = 2, motor_right_pin1 = 3, motor_left_pin1 = 4, motor_left_pin1 = 5;

//Calculate motor velocities inside this
void velocity_callback(const geometry_msgs::Twist& vel_msg)
{
  
}

//Declare global objects
ros::NodeHandle  node;
ros::Publisher odometry_publisher("odom", &odometry);
ros::Publisher imu_publisher("imu", &imu);
ros::Publisher joint_state_publisher("joint_states", &joint_states);
ros::Publisher transform_publisher("tf", &transform);
ros::Subscriber<geometry_msgs::Twist> twist_subscriber("cmd_vel", velocity_callback);

//Calculate transform for base_footprint
void calculate_transform()
{
  
}

//Calculate odometry from encoder values
void calculate_odometry()
{
  
}

//Calculate joint states from encoder values
void calculate_joint_states()
{
  
}

//Calculate acceleration from IMU values
void calculate_imu()
{
  
}

//Initialise nodes, publishers, subscribers and serial monitor
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting...");
  
  pinMode(motor_right_pin1, OUTPUT);
  pinMode(motor_right_pin2, OUTPUT);
  pinMode(motor_left_pin1, OUTPUT);
  pinMode(motor_left_pin2, OUTPUT);

  pinMode(9, OUTPUT); 
  pinMode(10, OUTPUT);
  
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
  analogWrite(9, 100); //ENA pin
  analogWrite(10, 200); //ENB pin

  //Controlling spin direction of motors:
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  delay(1000);

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  delay(1000);
  
  node.spinOnce();
  delay(100);
}
