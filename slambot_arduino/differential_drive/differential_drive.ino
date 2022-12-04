#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

//Declare global objects and variables
ros::NodeHandle  node;

class SLAMbot
{
  private:
  geometry_msgs::Twist velocity;
  nav_msgs::Odometry odom;
  std_msgs::String string_message;
  float v_l, v_r, w;
  ros::Subscriber<geometry_msgs::Twist, SLAMbot> velocity_sub;
  ros::Publisher odometry_pub;

  public:
  //Declare constructor
  SLAMbot() : velocity_sub("cmd_vel", &SLAMbot::velocity_callback, this), odometry_pub("odom", &odom)
  {}

  //Declare functions
  void init(ros::NodeHandle& node)
  {
    node.subscribe(velocity_sub);
    node.advertise(odometry_pub);
  }
  void CalculateOdom()
  {
    
  }

  //Declare callbacks
  void velocity_callback(const geometry_msgs::Twist& msg){
    velocity = msg;
  }
};

SLAMbot bot;

//Initialise nodes, publishers and subscribers
void setup()
{
  node.initNode();
  bot.init(node);
}

//Write your program logic
void loop()
{
  node.spinOnce();
  delay(1);
}
