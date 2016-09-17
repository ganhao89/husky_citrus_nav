#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
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
   double odom_x = msg->pose.pose.position.x;
   double odom_y = msg->pose.pose.position.y;
   double odom_z = msg->pose.pose.position.z;
   double odom_quat
}

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle sub;
  ros::NodeHandle pub;
  // Create a RobotDriver object to store the publisher
  RobotDriver driver(pub);
  
  //! We will subscribe to the "/odometry/filtered" topic to get robot pose data
  ros::Subscriber robot_pose = nh.subscribe("/odometry/filtered",10, poseCallback);
  ros::spin();
  return 0;
}
