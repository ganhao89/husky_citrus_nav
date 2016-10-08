#include "husky_citrus_nav/gps_drive.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_driver");

  RobotLocalization::GPSDrive drive;

  drive.run();

  return 0;
}
