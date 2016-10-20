#ifndef HUSKY_CITRUS_NAV_GPS_DRIVE
#define HUSKY_CITRUS_NAV_GPS_DRIVE

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include "robot_localization/navsat_conversions.h"
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

    //! Callback for getting initial GPS location
    initgpsCallback(const sensor_msgs::NavSatFix::ConstPt& init_gps)

    //! Callback for waypoint data 
    void waypointCallback(const geometry_msgs::PointStamped::ConstPt& waypoint)
    
    //! Variables for current location
    double utm_x_current_;
    double utm_y_current_;
    //! Variables for waypoint location
    double utm_x_waypoint_;
    double utm_y_waypoint_;
    //! Current heading
    double tracking_;
    //! Expected heading
    double bearing_;
    //! robot wheel speed 
    geometry_msgs::Twist base_cmd;
    //! set PID controller parameters
    double dist_d_;
    double dist_i_;
    double theta_d_;
    double theta_i_;
    double dist_pre_;
    double theta_pre_;
    double Kp_;
    double Kd_;
    double Ki_;
    //! initial GPS location
    double init_utm_y_;
    double init_utm_x_;
    //! x and y displacement in GPS initial position
    utm_dx_;
    utm_dy_;
    //! counter
    int count_;
    //! wheel speed twist
    geometry_msgs::Twist base_cmd_;
    
};

} // namespace RobotLocalization

#endif // HUSKY_CITRUS_NAV_GPS_DRIVE
