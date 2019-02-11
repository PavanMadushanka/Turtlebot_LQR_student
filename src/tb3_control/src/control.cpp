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
#include "lqr_dubin.h"

using Eigen::MatrixXd;
using namespace Eigen;
using namespace std;

// Variables for LQR
double dt = 0.1;
double roh = 10;
int N = 100;
Matrix3d Q = MatrixXd::Identity(3,3);
MatrixXd R = MatrixXd::Identity(2,2) * roh;


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

static visualization_msgs::Marker path;

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

  ros::init(argc, argv, "control");
  ros::NodeHandle n("~");
  tf::TransformListener m_listener;
  tf::StampedTransform transform;

  srand((unsigned int)time(NULL));
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  odom_sub    = n.subscribe("odom", 10, odomCallback);

  ros::Rate loop_rate(1/dt); // ros spins 10 frames per second

  //we use geometry_msgs::twist to specify linear and angular speeds (v, w) which also denote our control inputs to pass to turtlebot
  geometry_msgs::Twist tw_msg;

  int frame_count = 0;
  int index = 0;
  int index_c = 0;

  // LQR Tuning parameters
  Q(0,0) = 100;
  Q(1,1) = 100;
  Q(2,2) = 100;


  MatrixXd Goal = MatrixXd::Zero(4,1);
  MatrixXd Start = MatrixXd::Zero(4,1);
  int point_count = 1;
  Start(0,0) = Odom_x;
  Start(1,0) = Odom_y;
  Start(2,0) = Odom_yaw;
  Start(3,0) = 0;

  Goal(0,0) = -5;
  Goal(1,0) = 5;
  Goal(2,0) = atan2((Goal(1,0)-Odom_y),(Goal(0,0)-Odom_x));
  Goal(3,0) = 0.1;

  //Initiating LQR_dubin object
  LQR_dubin dubin(Q,R,dt,N);
  dubin.LQR_init(Goal);

  while (ros::ok())
  {
    Vector2d Ut = dubin.Update_Ut(index, Goal, Odom_x, Odom_y, Odom_yaw);

    float velocity = Ut[0];
    float angular = Ut[1];

    tw_msg.angular.z = angular;
    tw_msg.linear.x = velocity;

    cmd_vel_pub.publish(tw_msg);
    index++;

    frame_count ++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
