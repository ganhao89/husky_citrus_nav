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
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& filtered_gps);

    //! Callback for getting initial GPS location
    void initgpsCallback(const sensor_msgs::NavSatFix::ConstPtr& init_gps);

    //! Callback for waypoint data 
    void waypointCallback(const geometry_msgs::PointStamped::ConstPtr& waypoint);
    
    //! member function to compute bearing
    double computeBearing(double x0, double y0, double x1, double y1);

    //! member function to evalute the localization accuracy
    double evalutePosition(bool evaluate);
    //! member function for triggering the canon camer to acquire images
    void imgAcquire();
    //! Variables for current location
    double utm_x_current_;
    double utm_y_current_;
    //! Variables for waypoint location
    double utm_x_waypoint_;
    double utm_y_waypoint_;
    //! waypoint status: false->not received; true->received
    bool waypoint_status_;
    //! status indicating if the robot has arrived the waypoint
    bool waypoint_arrived_;
    //! Current heading
    double tracking_;
    //! Expected heading
    double bearing_;
    //! robot wheel speed 
    geometry_msgs::Twist base_cmd_;
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
    double utm_dx_;
    double utm_dy_;
    //! counter
    int count_;
    //! evaluate the position accuracy if TRUE
    bool evalute_;
    
};

} // namespace RobotLocalization

#endif // HUSKY_CITRUS_NAV_GPS_DRIVE
