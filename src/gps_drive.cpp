#include "husky_citrus_nav/gps_drive.h"
#include <tf/transform_datatypes.h>

namespace RobotLocalization
{
  GPSDrive::GPSDrive():
    x_current_(0.0),
    y_current_(0.0),
    //! Variables for waypoint location
    x_waypoint_(0.0),
    y_waypoint_(0.0),
    //! Current heading
    tracking_(0.0),
    //! Expected heading
    bearing_(0.0),
    //! pid control parameters
    Kp_(1.0),
    Kd_(0.5),
    Ki_(0.3),
    //! initial GPS location
    init_gps_lat_(0.0),
    init_gps_lon_(0.0),
    init_gps_alt_(0.0),
    //! x and y displacement in GPS initial position
    dx_(0.0),
    dy_(0.0),
    //! counter
    counter_(0);

  GPSDrive::~GPSDrive()
  {
  }

  void GPSDrive::run()
  {
    //init the ROS node
    //ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    nh_priv.param("Kp", Kp_, 1.0);
    nh_priv.param("Kd", Kd_, 0.5);
    nh_priv.param("Ki", Ki_, 0.3);
    
    ros::Subscriber robot_init_gps = nh.subscribe("/initgps", 1, &GPSDrive::initgpsCallback, this);
    ros::spinOnce();
    ros::rate r(30);
    while (ros::ok())
    {
      //! We will subscribe to the "/odometry/filtered" topic to get robot pose data
      ros::Subscriber robot_pose = nh.subscribe("/odometry/filtered", 1, &GPSDrive::poseCallback, this);
      ros::Subscriber robot_gps = nh.subscribe("/gps/filtered", 1, &GPSDrive::gpsCallback, this);
      ros::Subscriber robot_waypoint=nh.subscribe("/waypoint", 1, &GPSDrive::waypointCallback, this);
      ros::spinOnce();
      ros::Publisher robot_driver=nh.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 1);
      robot_driver.publish(base_cmd);
      r.sleep();
    }  
  }  
  
  void GPSDrive::computeCmd()
  {
    bearing_ = 
  }
  
  void GPSDrive::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // store the pose of the robot
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //double odom_p_x = msg->pose.pose.position.x;
    //double odom_p_y = msg->pose.pose.position.y;
    //double odom_p_z = msg->pose.pose.position.z;
    //double odom_o_x = msg->pose.pose.orientation.x;
    //double odom_o_y = msg->pose.pose.orientation.y;
    //double odom_o_z = msg->pose.pose.orientation.z;
    //double odom_o_w = msg->pose.pose.orientation.w;

    // store the linear velocities and angular velocity of the robot

    //double odom_l_x = msg->twist.twist.linear.x;
    //double odom_l_y = msg->twist.twist.linear.y;
    //double odom_a_z = msg->twist.twist.angular.z;

    tracking_ = roll;
    
  }

  void GPSDrive::gpsCallback(const sensor_msgs::NavSatFix::ConstPt& filtered_gps)
  {
    double gps_lat = filtered_gps->latitude;
    double gps_lon = filtered_gps->longitude;
    double gps_alt = filtered_gps->altitude;
    if (count_ == 0)
    {
      dx_ = 
      dy_ = 
      ++count_;
    }
    dx=
    dy=
    x_current_ = ;
    y_current_ = ;
  }

  void GPSDrive::initgpsCallback(const sensor_msgs::NavSatFix::ConstPt& init_gps)
  {
    init_gps_lat_ = init_gps->latitude;
    init_gps_lon_ = init_gps->longitude;
    init_gps_alt_ = init_gps->altitude;
  }

  void GPSDrive::waypointCallback(const geometry_msgs::PointStamped::ConstPt& waypoint)
  {
    double way_x_lon = waypoint->point.x;
    double way_y_lat = waypoint->point.y;
  }

}
  

