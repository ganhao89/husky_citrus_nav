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
    heading_current_(0.0),
    //! Expected heading
    heading_waypoint_(0.0),
    //! pid control parameters
    Kp_(1.0),
    Kd_(0.5),
    Ki_(0.3);

  GPSDrive::~GPSDrive()
  {
  }

  void GPSDrive::run()
  {
    //init the ROS node
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    nh_priv.param("Kp", Kp_, 1.0);
    nh_priv.param("Kd", Kd_, 0.5);
    nh_priv.param("Ki", Ki_, 0.3);


    ros::rate r(30);
    while (ros::ok())
    {
      //! We will subscribe to the "/odometry/filtered" topic to get robot pose data
      ros::Subscriber robot_pose = nh.subscribe("/odometry/filtered", 1, &GPSDrive::poseCallback, this);
      ros::Subscriber robot_gps = nh.subscribe("/gps/filtered", 1, &GPSDrive::gpsCallback, this);
      ros::Subscriber robot_waypoint=nh.subscribe("/waypoint", 1, GPSDrive::waypointCallback, this);
      ros::spinOnce();
      ros::Publisher robot_driver=nh.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 1);
      robot_driver.publish(base_cmd);
      r.sleep();
    }  
  }  
  
  void GPSDrive::computeCmd()
  {
    
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

    heading_current_ = roll;
    
  }

  void GPSDrive::gpsCallback(const sensor_msgs::NavSatFix::ConstPt& filtered_gps)
  {
    double gps_lat = filtered_gps->latitude;
    double gps_lon = filtered_gps->longitude;
    double gps_alt = filtered_gps->altitude;
    
    x_current_ = gps_lon;
    y_current_ = gps_lat;
  }

  void GPSDrive::waypointCallback(const geometry_msgs::PointStamped::ConstPt& waypoint)
  {
    double way_x_lon = waypoint->point.x;
    double way_y_lat = waypoint->point.y;
  }

}
  

