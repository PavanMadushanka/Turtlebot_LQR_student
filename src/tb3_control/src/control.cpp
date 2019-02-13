/*
Used for teaching controller design for Turtlebot3
Lantao Liu
ISE, Indiana University
*/

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "lqr_dubin.h"  //Here we are including the supporting class function for LQR calculation

using Eigen::MatrixXd;
using namespace Eigen;
using namespace std;

// Variables for LQR
double dt = 0.1; //Timestep
double roh = 10;  //Roh value for LQR calculation
int N = 100;  // Horizon
Matrix3d Q = MatrixXd::Identity(3,3);  //Q matrix is initialized as Identity matrix
MatrixXd R = MatrixXd::Identity(2,2) * roh;  //R matrix is initialized as Identity matrix x Roh.


// global vars
tf::Point Odom_pos; //odometry position (x, y, z)
double Odom_yaw;  //odometry orientation (yaw)
double Odom_v, Odom_w;  //odometry linear and angular speeds
double Odom_x, Odom_y; //To get the position of the robot

// ROS Topic Publishers
ros::Publisher cmd_vel_pub;
ros::Publisher marker_pub;

// ROS Topic Subscribers
ros::Subscriber odom_sub;

//This marker can be used to create a path
static visualization_msgs::Marker path;
// Define your own function to create a path here




/*
 * Callback function for odometry msg, used for odom subscriber
 */
void odomCallback(const nav_msgs::Odometry odom_msg)
{
 /* upon "hearing" odom msg, retrieve its position and orientation (yaw) information.
  * tf utility functions can be found here: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
  * odom data structure definition is here: https://mirror.umd.edu/roswiki/doc/diamondback/api/nav_msgs/html/msg/Odometry.html
  */
  tf::pointMsgToTF(odom_msg.pose.pose.position, Odom_pos);
  Odom_yaw = tf::getYaw(odom_msg.pose.pose.orientation);

  //update observed linear and angular speeds (real speeds published from simulation)
  Odom_v = odom_msg.twist.twist.linear.x;
  Odom_w = odom_msg.twist.twist.angular.z;

  Odom_x = Odom_pos.x();
  Odom_y = Odom_pos.y();
 }


/*
 * main function
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "control");  //ROS node registration
  ros::NodeHandle n("~");

 //Topic to publish control commands to the turtle bot model
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
 //Topic to publish marker information to rviz.
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1); 
  
 //This is listening to odometry data topic published from turtlebot model
  odom_sub    = n.subscribe("odom", 10, odomCallback);

  //ROS loop rate is matched with the timesteps we are using in LQR calculation
  ros::Rate loop_rate(1/dt);

  //we use geometry_msgs::twist to specify linear and angular speeds (v, w) which also denote our control inputs to pass to turtlebot
  geometry_msgs::Twist tw_msg;

  int frame_count = 0; //This keeps a count on the frame number
  int index = 0; //This keeps count on the timesteps passed in each horizon considered.

  // LQR Tuning parameters
  // Here we are changing the values of Q matrix to tune our LQR controller to behave as desired.
  Q(0,0) = 100;
  Q(1,1) = 100;
  Q(2,2) = 100;


  MatrixXd Goal = MatrixXd::Zero(4,1);
  MatrixXd Start = MatrixXd::Zero(4,1);
  
 //Defines the start point
  Start(0,0) = Odom_x; // X position
  Start(1,0) = Odom_y; // Y position
  Start(2,0) = Odom_yaw; // Orientation
  Start(3,0) = 0; // Starting speed

 //Defines Goal point
  Goal(0,0) = -5; // X position
  Goal(1,0) = 5;  // Y position
  Goal(2,0) = atan2((Goal(1,0)-Odom_y),(Goal(0,0)-Odom_x));  // Set the Goal orientation parallel to the path from Start to Goal
  Goal(3,0) = 0.1;  // Set speed at the Goal

  //Initiating LQR_dubin object
  LQR_dubin dubin(Q,R,dt,N);
  dubin.LQR_init(Goal); //For each change of Goal this function should be called to change A and B matrices accordingly.

  while (ros::ok())
  {
   // You can add a code here to update Start and Goal points according to the movement of the vehicle along your path.
   // Remember to call the init function of LQR object for each Goal you define.
    
    Vector2d Ut = dubin.Update_Ut(index, Goal, Odom_x, Odom_y, Odom_yaw); //Calculates the control output values using LQR class

    float speed = Ut[0];  // taking speed
    float angular = Ut[1];  // taking angular speed

   // Passing these values to Twist object
    tw_msg.angular.z = angular;
    tw_msg.linear.x = speed;

    cmd_vel_pub.publish(tw_msg); // Publishing the control data to turtlebot
    index++;

    frame_count ++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
