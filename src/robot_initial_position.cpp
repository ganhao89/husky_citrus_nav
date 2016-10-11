#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>


double latitude_  = 0;
double longitude_ = 0;
double altitude_  = 0;
int count_ = 0;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_initial_position");

  ros::NodeHandle nh;
  // Create a Publisher obj to publish averaged GPS data
  ros::Publisher initial_pos =  nh.advertise<sensor_msgs::NavSatFix.h>("initGPS", 1);
  // Create a Subscriber obj to receive gps/fix data
  ros::Subscriber gps_pos = nh.subscribe("gps/fix", 1, gpsCallback);
  // Set the loop rate
  ros::Rate loop_rate(1);
  // Enter a loop to receive 10 GPS data and integrate them
  while (ros::ok() && count < 10)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  // Create a GPS message obj to store GPS data for publishing
  sensor_msgs::NavSatFix gps_msg;
  gps_msg.latitude  = latitude_/count_;
  gps_msg.longitude = longitude_/count_;
  gps_msg.altitude  = altitude_/count_;
  // Publish the averaged GPS data once
  initial_pos.publish(gps_msg);

  return 0;
}

// Receving GPS data and integrating them.
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  double gps_lat = msg->latitude;
  double gps_lon = msg->longitude;
  double gps_alt = msg->altitude;

  latitude_  = latitude_ + gps_lat;
  longitude_ = longitude_ + gps_lon;
  altitude_  = altitde_ + gps_alt;
  ++count_;
}
