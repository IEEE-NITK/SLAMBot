#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

class SlamBot
{
    public:
        ros::NodeHandle node;
        geometry_msgs::Pose2D pose;

        SlamBot(int argc, char **argv, std::string topic_name)
        {
            ros::init(argc, argv, topic_name);
        }

        void odom_callback(const nav_msgs::Odometry::ConstPtr& data)
        {
            tf::Quaternion q(data->pose.pose.orientation.x, 
                            data->pose.pose.orientation.y, 
                            data->pose.pose.orientation.z, 
                            data->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;

            m.getRPY(roll, pitch, yaw);
            
            this->pose.x = data->pose.pose.position.x;
            this->pose.y = data->pose.pose.position.y;
            this->pose.theta = yaw;
        }

        void movebase_client(std::vector<std::array<float, 2>> goals)
        {
            ros::Subscriber odom_sub = this->node.subscribe<nav_msgs::Odometry>
                ("/odom", 10, &SlamBot::odom_callback, this);
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client("move_base", true);
            move_base_msgs::MoveBaseGoal goal;
            
            while(!action_client.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            int num_goals = goals.size();
            for(int i=0; i<num_goals; i++)
            {
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = goals[i][0];
            goal.target_pose.pose.position.y = goals[i][1];
            goal.target_pose.pose.orientation.w = 1.0;
            
            ROS_INFO("Sending goal");
            action_client.sendGoal(goal);

            action_client.waitForResult();

            if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Goal Point Reached!");
            else
                ROS_INFO("Could Not Reach Goal Point");
            }
        }
};

int main(int argc, char **argv)
{
    SlamBot robot(argc, argv, "slambot_motion");
    std::vector<std::array<float, 2>> goals = {{6.7, 4.5}, {3.35}, {-6.2, 3.1}, {1.2, 0.0}};
    
    robot.movebase_client(goals);
    
    return 0;
}