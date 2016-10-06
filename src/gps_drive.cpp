#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/twist_marker_server/cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool publishSpeed(geometry_msgs::Twist base_cmd)
  {
    //publish the assembled command
    cmd_vel_pub_.publish(base_cmd);
    return true;
  }

};

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   // store the pose of the robot
   double odom_p_x = msg->pose.pose.position.x;
   double odom_p_y = msg->pose.pose.position.y;
   double odom_p_z = msg->pose.pose.position.z;
   double odom_o_x = msg->pose.pose.orientation.x;
   double odom_o_y = msg->pose.pose.orientation.y;
   double odom_o_z = msg->pose.pose.orientation.z;
   double odom_o_w = msg->pose.pose.orientation.w;
   // store the linear velocities and angular velocity of the robot
   double odom_l_x = msg->twist.twist.linear.x;
   double odom_l_y = msg->twist.twist.linear.y;
   double odom_a_z = msg->twist.twist.angular.z;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPt& filtered_gps)
{
  double gps_lat = filtered_gps->latitude;
  double gps_lon = filtered_gps->longitude;
  double gps_alt = filtered_gps->altitude;
}

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle sub_odo;
  ros::NodeHandle pub_cmd;
  // Create a RobotDriver object to store the publisher
  RobotDriver driver(pub_cmd);
  ros::rate r(30);
  while (ros::ok())
  {
     //! We will subscribe to the "/odometry/filtered" topic to get robot pose data
     ros::Subscriber robot_pose = sub_odo.subscribe("/odometry/filtered", 1, poseCallback);
     ros::Subscriber robot_gps = sub_odo.subscribe("/gps/filtered", 1, gpsCallback);
     
     ros::spinOnce();
     driver.publishSpeed();
     r.sleep();
  }
  return 0;
}
