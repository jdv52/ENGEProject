#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
using namespace std;

#define PI 3.141592;

// Robot physical constants
#define TICKS_PER_REV 1;
#define WHEEL_RAD 1;
#define WHEEL_BASE 1;
#define TICKS_PER_METER 1;

ros::Publisher odom_pub;
nav_msgs::Odometry initial_pose;
nav_msgs::Odometry current_pose;

// Accumulators to keep track of linear distance traveled by each wheel
double left_distance = 0;
double right_distance = 0;

// Callback function for the right encoder ticks message. Calculates the distance traveled given
// the number of encoder ticks measured.
void calc_distance_right(const std_msgs::Int16& count)
{
  static int right_count = 0;
  if (count.data != 0 && right_count != 0)
  {
    int right_ticks = count.data - right_count;
    if (right_ticks > 10000)
      right_ticks = 0 - (65535 - right_ticks);
    else if (right_ticks < -1000)
      right_ticks = 65546 - right_ticks;
    right_distance = right_ticks / TICKS_PER_METER;
  }
  right_count = count.data;
}

// Callback function for the left encoder ticks message. Calculates the distance traveled given
// the number of encoder ticks measured.
void calc_distance_left(const std_msgs::Int16& count)
{
  static int left_count = 0;
  if (count.data != 0 && left_count != 0)
  {
    int left_ticks = count.data - left_count;
    if (left_ticks > 10000)
      left_ticks = 0 - (65535 - left_ticks);
    else if (left_ticks < -1000)
      left_ticks = 65546 - left_ticks;
    left_distance = left_ticks / TICKS_PER_METER;
  }
  left_count = count.data;
}

// This function publishes odometry messages based on calculations from encoder measurements
void update_odom()
{

  // Equations taken from kinematic model of differential drive robot
  double distance = (right_distance + left_distance) / 2;
  double arc_angle = (right_distance - left_distance) / WHEEL_BASE;
  double estimate_angle = arc_angle / 2 + initial_pose.pose.pose.orientation.z;

  // Calculate estimate of new position and orientation
  current_pose.pose.pose.position.x = initial_pose.pose.pose.position.x + distance * cos(estimate_angle);
  current_pose.pose.pose.position.y = initial_pose.pose.pose.position.y + distance * sin(estimate_angle);
  current_pose.pose.pose.orientation.z = initial_pose.pose.pose.orientation.z + arc_angle;

  if (isnan(current_pose.pose.pose.position.x) ||
      isnan(current_pose.pose.pose.position.y) ||
      isnan(current_pose.pose.pose.position.z))
  {
    current_pose.pose.pose.position.x = initial_pose.pose.pose.position.x;
    current_pose.pose.pose.position.y = initial_pose.pose.pose.position.y;
    current_pose.pose.pose.position.z = initial_pose.pose.pose.position.z;
  }
  
  // Calculate linear and angular velocity based on time elapsed since last reading
  current_pose.header.stamp = ros::Time::now();
  long dt = current_pose.header.stamp.toSec() - initial_pose.header.stamp.toSec();
  current_pose.twist.twist.linear.x = distance / dt;
  current_pose.twist.twist.angular.z = arc_angle / dt;

  // Store current pose for next read
  initial_pose.pose.pose.position.x = current_pose.pose.pose.position.x;
  initial_pose.pose.pose.position.y = current_pose.pose.pose.position.y;
  initial_pose.pose.pose.orientation.z = current_pose.pose.pose.orientation.z;
  initial_pose.header.stamp = current_pose.header.stamp;

  odom_pub.publish(current_pose);
}

// Main function. Runs main loop while ROS is running.
int main(int argc, char **argv) {

  // Set initial pose
  current_pose.header.frame_id = "odom";
  current_pose.pose.pose.position.z = 0;
  current_pose.pose.pose.orientation.x = 0;
  current_pose.pose.pose.orientation.y = 0;
  current_pose.twist.twist.linear.x = 0;
  current_pose.twist.twist.linear.y = 0;
  current_pose.twist.twist.angular.x = 0;
  current_pose.twist.twist.angular.y = 0;
  current_pose.twist.twist.angular.z = 0;
  initial_pose.pose.pose.position.x = 0;
  initial_pose.pose.pose.position.y = 0;
  initial_pose.pose.pose.orientation.z = 0.00000000001;

  ros::init(argc, argv, "odom_pub");
  ros::NodeHandle node;
  ros::Subscriber right_encoder_sub = node.subscribe("right_ticks", 100, calc_distance_right, ros::TransportHints().tcpNoDelay());
  ros::Subscriber left_encoder_sub = node.subscribe("left_ticks", 100, calc_distance_left, ros::TransportHints().tcpNoDelay());
  odom_pub = node.advertise<nav_msgs::Odometry>("odom_data", 100);
  ros::Rate loop_rate(30);

  while(ros::ok()) {
    update_odom();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
