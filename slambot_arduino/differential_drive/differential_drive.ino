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
int motor_right, motor_left;

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
  node.spinOnce();
  delay(100);
}
