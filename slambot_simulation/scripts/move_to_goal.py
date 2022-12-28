#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

from tf.transformations import euler_from_quaternion

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

global pose
pose = [0, 0, 0]

def odom_callback(data):
    global pose
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


def movebase_client():

    global pose
   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
   # For storing goal locations
    goal_x = [6.7, 1.0, -6.2, 1.2]
    goal_y = [4.5, 3.35, 3.1, 0.0]

    len_x = len(goal_x)
    i = 0

    print("\nIEEE - ROS Simulation of Mobile Robot \n--------------------------------------------\n\nINITIAL POSITION - [-4.100000, 2.400000]")
    while i<len_x: 
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = goal_x[i]
        goal.target_pose.pose.position.y = goal_y[i]
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = 1


        # Sends the goal to the action server.
        client.send_goal(goal)

        while math.sqrt((goal_x[i] - pose[0])**2 + (goal_y[i] - pose[1])**2)>0.2:
           continue 

        print("TARGET %d [%f , %f]- REACHED"%((i+1),goal_x[i],goal_y[i]))
        i+=1

    print("TASK COMPLETED!")

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        rospy.Subscriber('/odom', Odometry, odom_callback)
        movebase_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

