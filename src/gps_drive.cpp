#include "husky_citrus_nav/gps_drive.h"
#include <tf/transform_datatypes.h>
#include "robot_localization/navsat_conversions.h"
#include <cmath>

namespace RobotLocalization
{
  GPSDrive::GPSDrive():
    utm_x_current_(0.0),
    utm_y_current_(0.0),
    //! Variables for waypoint location
    utm_x_waypoint_(0.0),
    utm_y_waypoint_(0.0),
    //! Current heading
    tracking_(0.0),
    //! Expected heading
    bearing_(0.0),
    //! pid control parameters
    Kp_(1.0),
    Kd_(0.5),
    Ki_(0.3),
    dist_d_(0.0),
    dist_i_(0.0),
    theta_d_(0.0),
    theta_i_(0.0),
    dist_pre_(0.0),
    theta_pre_(0.0),
    //! initial GPS location
    init_utm_y_(0.0),
    init_utm_x_(0.0),
    //! x and y displacement in GPS initial position
    utm_dx_(0.0),
    utm_dy_(0.0),
    //! counter
    count_(0)
    //! base command
    //base_cmd_.linear.x(0.0),
    //base_cmd_.linear.y(0.0),
    //base_cmd_.angular.z(0.0)
    {
    }
  GPSDrive::~GPSDrive()
  {
  }

  void GPSDrive::run()
  {
    ros::Time::init();

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    nh_priv.param("Kp", Kp_, 1.0);
    nh_priv.param("Kd", Kd_, 0.5);
    nh_priv.param("Ki", Ki_, 0.3);
    
    ros::Subscriber robot_init_gps = nh.subscribe("/initGPS", 1, &GPSDrive::initgpsCallback, this);
  
    ros::Rate r(30);
    while (ros::ok())
    {
      std::cout<<"entered the while loop"<<std::endl;
      //! We will subscribe to the "/odometry/filtered" topic to get robot pose data
      ros::Subscriber robot_init_gps = nh.subscribe("/initGPS", 1, &GPSDrive::initgpsCallback, this);
      ros::Subscriber robot_pose = nh.subscribe("/odometry/filtered", 1, &GPSDrive::poseCallback, this);
      ros::Subscriber robot_gps = nh.subscribe("/gps/filtered", 1, &GPSDrive::gpsCallback, this);
      ros::Subscriber robot_waypoint=nh.subscribe("/waypoint", 1, &GPSDrive::waypointCallback, this);
      ros::spin();
      ros::Publisher robot_driver=nh.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 1);
      computeCmd();
      robot_driver.publish(base_cmd_);
      r.sleep();
    }  
  }  
  
  void GPSDrive::computeCmd()
  {
    double x0 = utm_x_current_;
    double y0 = utm_y_current_;
    double x1 = utm_x_waypoint_;
    double y1 = utm_y_waypoint_;
    bearing_ = computeBearing(x0,y0,x1,y1);
    double dist = sqrt(pow((x1-x0),2)+pow((y1-y0),2));
    double theta = bearing_ - tracking_;
    dist_d_ = dist-dist_pre_;
    theta_d_ = theta - theta_pre_;
    double x_speed = (Kp_*dist + Kd_*dist_d_ + Ki_*dist_i_)/10;
    if (x_speed > 0.8)
    {
       x_speed = 0.8;
    }
    double z_angular = (Kp_*theta + Kd_*theta_d_ + Ki_*theta_i_)/10;
    if (z_angular>0.8)
    {
       z_angular=0.8;
    }
    if (z_angular<-0.8)
    {
       z_angular = -0.8;
    }
    dist_pre_ = dist;
    theta_pre_ = theta;
    dist_i_ = dist_i_+dist;
    theta_i_ = theta_i_+theta;
    base_cmd_.linear.x = x_speed;
    base_cmd_.linear.y = 0.0;
    base_cmd_.angular.z = z_angular;
    
  }
  
  void GPSDrive::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // store the pose of the robot
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    tracking_ = roll; 
    std::cout<<"odometry received"<<std::endl; 
  }

  void GPSDrive::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& filtered_gps)
  {
    double gps_lat = filtered_gps->latitude;
    double gps_lon = filtered_gps->longitude;
    std::cout<<gps_lat<<"     "<<gps_lon<<std::endl;
    double utmX=0.0;
    double utmY=0.0;
    std::string utm_zone_tmp;
    NavsatConversions::LLtoUTM(gps_lat,gps_lon,utmY,utmX,utm_zone_tmp);
    std::cout<<utmX<<"     "<<utmY<<std::endl;
    if (count_ < 1)
    {
      utm_dx_ = utmX - init_utm_x_;
      utm_dy_ = utmY - init_utm_y_;
      count_=count_+1;
    }

    utm_x_current_ = utmX - utm_dx_;
    utm_y_current_ = utmY - utm_dy_;
    std::cout<<utm_x_current_<<std::endl;
    std::cout<<utm_y_current_<<std::endl;
    std::cout<<"gps received"<<std::endl;
  }

  void GPSDrive::initgpsCallback(const sensor_msgs::NavSatFix::ConstPtr& init_gps)
  {
    double init_gps_lat = init_gps->latitude;
    double init_gps_lon = init_gps->longitude;
    std::string utm_zone_tmp;
    double utmX=0.0;
    double utmY=0.0;
    NavsatConversions::LLtoUTM(init_gps_lat,init_gps_lon,utmY,utmX,utm_zone_tmp);
    init_utm_y_ = utmY;
    init_utm_x_ = utmX;
    std::cout<<init_utm_y_<<"\n"<<init_utm_x_<<std::endl;
    std::cout<<"initial GPS received"<<std::endl;
  }

  void GPSDrive::waypointCallback(const geometry_msgs::PointStamped::ConstPtr& waypoint)
  {
    // Get the waypoint in the UTM frame
    // NOTE: you must select the UTM frame option for the Click publish plugin in Mapviz
    utm_x_waypoint_ = waypoint->point.x;
    utm_y_waypoint_ = waypoint->point.y;
  }
  
  double GPSDrive::computeBearing(double x0, double y0, double x1, double y1)
  {
   static const double TWOPI = 6.2831853071795865;
   static const double RAD2DEG = 57.2957795130823209;
   // if (a1 = b1 and a2 = b2) throw an error 
   double theta = atan2(x1 - x0, y1 - y0);
   if (theta < 0.0)
       theta += TWOPI;
   return RAD2DEG * theta;
  }

}
  

