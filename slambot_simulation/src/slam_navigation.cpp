#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>

class SlamBot
{
    public:
        geometry_msgs::Pose pose;
        ros::NodeHandle node;

        SlamBot(int argc, char **argv, std::string topic_name)
        {
            ros::init(argc, argv, topic_name);
        }

        void odom_callback(const nav_msgs::Odometry::ConstPtr& data)
        {
            
        }

        void movebase_client()
        {
            ros::Subscriber odom_sub = this->node.subscribe<nav_msgs::Odometry>
                ("/odom", 10, &SlamBot::odom_callback, this);
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client("move_base", true);
            move_base_msgs::MoveBaseGoal goal;

        }
};

int main(int argc, char **argv)
{
    SlamBot robot(argc, argv, "slambot_motion");
    robot.movebase_client();
    
    return 0;
}