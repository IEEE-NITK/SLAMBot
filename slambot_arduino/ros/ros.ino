#include <ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>


//initializing all the variables
//timers for sub-main loop
#define LOOPTIME 100                //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter
unsigned long lastMilli = 0;


const int PIN_ENCOD_A_MOTOR_LEFT = 3;             //A channel for encoder of left motor 
const int PIN_ENCOD_A_MOTOR_RIGHT = 2;              //A channel for encoder of right motor   
const byte M[4] = {5, 6, 7, 8};

//Variables to determine each cycle to get robot odometry position
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

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
noCommLoops = 0;  
x_pos_req=msg.linear.x
y_pos_req=msg.linear.y
theta_req=msg.angular.z
}

ros::NodeHandle nh;     //initialise ros node nh
char odom[] = "/odom";
char base_link[] = "/base_link";
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
nav_msgs::Odometry odom_msg;                                            //create an "odom_msg" ROS message
ros::Publisher odom_pub("odometry", &odom_msg);                         //create a publisher to ROS topic "odometry" using the "odom_msg" type
tf::TransformBroadcaster broadcaster; 


void setup() {
  nh.getHardware()->setBaud(500000);         //set baud for ROS serial communication
  nh.initNode();                            //init ROS node
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(odom_pub);                  //prepare to publish speed in ROS topic
  broadcaster.init(nh);



  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_LEFT), encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCOD_A_MOTOR_RIGHT), encoderRightMotor, RISING);
}

void loop() {
  nh.spinOnce();
  if((millis()-lastMilli) >= LOOPTIME){                                                                           // enter timed loop
    lastMilli = millis();
    
    if (abs(pulses_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pulses_left/encoderppr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate left wheel speed in m/s
      pulses_left = 0;                                                                 // reset pulses to 0 for next loop
    }
    
    if (abs(pulses_right) < 5){                                                         //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pulses_right/encoderppr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate right wheel speed in m/s
    pulses_right=0;
    }





    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      speed_req_left=0;
      }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      //Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }


// Determining position
    pos_left_mm = (pos_left/encoderppr) * 2*PI*radius;
    pos_right_mm = (pos_right/encoderppr) * 2 * PI * radius;
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

   publishOdometry(LOOPTIME);   //Publish odometry on ROS topic
 }
}

void publishOdometry(double time) {
  geometry_msgs::TransformStamped t;
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = x;       //robot position in X direction in mm
  t.transform.translation.y = y;       // robot position in Y direction in mm
  t.transform.translation.z = 0;
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = nh.now();      //timestamp for odometry data
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);
  odom_msg.twist.twist.linear.x = speed_act_left;    //left wheel speed (in m/s)
  odom_msg.twist.twist.linear.y = speed_act_right;   //right wheel speed (in m/s)
  odom_msg.twist.twist.angular.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  odom_pub.publish(&odom_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}





//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)){
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
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)){
    pos_right--;
    pulses_right--;
  }
  else{
    pos_right++;
    pulses_right++;
  }
}


















  

  
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication 
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message
  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  
  if (angular_speed_req == 0){
  speed_handle_left = speed_req;                 //Calculate the required speed for the left motor to comply with commanded linear and angular speeds  speed_req_right = speed_req + angular_speed_req*(wheeltrack/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
  speed_handle_right = speed_req;                //Calculate the required speed for the left motor to comply with commanded linear and angular speeds  speed_req_right = speed_req + angular_speed_req*(wheeltrack/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
  }
  else if( speed_req > 0 && angular_speed_req >0){
    speed_handle_left = speed_req + 0.1;
    speed_handle_right = speed_req - 0.1;
  }
  else if(speed_req > 0 && angular_speed_req <0){
    speed_handle_left = speed_req - 0.1;
    speed_handle_right = speed_req + 0.1;
  }
  else if( speed_req ==0 && angular_speed_req > 0 ){
    speed_handle_left = speed_req + 0.1;
    speed_handle_right = -(speed_req + 0.1);
  }
  else if( speed_req ==0 && angular_speed_req < 0 ){
    speed_handle_left = -(speed_req + 0.1);
    speed_handle_right = speed_req + 0.1;
  }
  else if( speed_req < 0 && angular_speed_req >0){
    speed_handle_left = (speed_req + 0.1);
    speed_handle_right =  (speed_req - 0.1);
  }
  else if(speed_req < 0 && angular_speed_req <0){
    speed_handle_left = -(speed_req - 0.1);
    speed_handle_right = -(speed_req + 0.1);
  }
}
