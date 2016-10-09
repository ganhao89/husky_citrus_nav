#ifndef HUSKY_CITRUS_NAV_GPS_DRIVE
#define HUSKY_CITRUS_NAV_GPS_DRIVE

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>

namespace RobotLocalization
{

class GPSDrive
{
  public:
    //! Constructor
    GPSDrive();
    
   //! Destructor
   ~GPSDrive();
  
   //! Main run loop
    void run();
    
  private:
    //! Computes the wheel speeds
    void computeCmd(); 

    //! Callback for odometry/filtered data
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    //! Callback for gps/filtered data
    gpsCallback(const sensor_msgs::NavSatFix::ConstPt& filtered_gps)

    //! Callback for waypoint data 
    void waypointCallback(const geometry_msgs::PointStamped::ConstPt& waypoint)
    
    //! Variables for current location
    double x_current_;
    double y_current_;
    //! Variables for waypoint location
    double x_waypoint_;
    double y_waypoint_;
    //! Current heading
    double heading_current_;
    //! Expected heading
    double heading_waypoint_;
    //! robot wheel speed 
    geometry_msgs::Twist base_cmd;
    //! pid control parameters;
    double Kp_;
    double Kd_;
    double Ki_;

  
};

} // namespace RobotLocalization

#endif // HUSKY_CITRUS_NAV_GPS_DRIVE
