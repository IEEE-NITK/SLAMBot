#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/JointState.h>

using namespace std;

class TurtleBot3
{
    private:
    string name;

    public:
    nav_msgs::Odometry odom;
    sensor_msgs::Imu imu;
    sensor_msgs::LaserScan laser;
    sensor_msgs::JointState joints;

    //Give name to your bot just because xD
    TurtleBot3(string name)
    {
        this->name = name;
    }

    //Declare functions
    double distance_from(geometry_msgs::Point point)
    {
        return sqrt(pow(odom.pose.pose.position.x-point.x, 2) + 
                    pow(odom.pose.pose.position.y-point.y, 2) + 
                    pow(odom.pose.pose.position.z-point.z, 2));
    }

    //Define callback functions for sensors
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        odom = *msg;
    }
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        imu = *msg;
    }
    void joint_callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        joints = *msg;
    }
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        laser = *msg;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot3_basics"); //Initialise the node
    ros::NodeHandle node; //Declare node object
    ros::Rate rate(10); //Declare loop frequency
    TurtleBot3 robot("turtle"); //Declare object for feedbacks
    geometry_msgs::Twist velocity; //Declare velocity variable

    //Declare subscribers
    ros::Subscriber odometry_subscriber = node.subscribe<nav_msgs::Odometry>
                    ("/odom", 10, &TurtleBot3::odometry_callback, &robot);
    ros::Subscriber imu_subscriber = node.subscribe<sensor_msgs::Imu>
                    ("/imu", 10, &TurtleBot3::imu_callback, &robot);
    ros::Subscriber joint_state_subscriber = node.subscribe<sensor_msgs::JointState>
                    ("/joint_states", 10, &TurtleBot3::joint_callback, &robot);
    ros::Subscriber laser_subscriber = node.subscribe<sensor_msgs::LaserScan>
                    ("/scan", 10, &TurtleBot3::laser_callback, &robot);
    
    //Declare publishers
    ros::Publisher velocity_publisher = node.advertise<geometry_msgs::Twist>
                    ("/cmd_vel", 10);

    geometry_msgs::Point goal;
    goal.x = 0;
    goal.y = 0;
    goal.z = 0;

    while(ros::ok())
    {   
        while(robot.distance_from(goal)<0.1)
        {
            velocity.linear.x = 1.0;
            velocity.linear.y = 0.0;
            velocity.angular.z = 1.0;
            
            velocity_publisher.publish(velocity);
            ros::spinOnce();
            rate.sleep();
        }
    }
    return 0;
}