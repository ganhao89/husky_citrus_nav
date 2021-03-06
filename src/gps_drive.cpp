#include "husky_citrus_nav/gps_drive.h"
#include <tf/transform_datatypes.h>
#include "robot_localization/navsat_conversions.h"
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <string>
namespace RobotLocalization
{
  GPSDrive::GPSDrive():
    utm_x_current_(0.0),
    utm_y_current_(0.0),
    //! Variables for waypoint location
    utm_x_waypoint_(0.0),
    utm_y_waypoint_(0.0),
    utm_x_waypoint_pre_(0.0),
    utm_y_waypoint_pre_(0.0),
    waypoint_status_(false),
    waypoint_arrived_(false),
    waypoint_changed_(false),
    //! Current heading
    tracking_(0.0),
    //! Expected heading
    bearing_(0.0),
    //! pid control parameters
    Kp_(0.5),
    Kd_(0.1),
    Ki_(0.01),
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
    count_(0),
    //! evaluate position error if true
    evalute_(true)
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
    
    nh_priv.param("Kp", Kp_, 0.5);
    nh_priv.param("Kd", Kd_, 0.1);
    nh_priv.param("Ki", Ki_, 0.01);
    nh_priv.param("evalute",evalute_, true);
    
    //! Wait for the initial GPS positon to arrive
    //ros::Subscriber robot_init_gps = nh.subscribe("/initGPS", 1, &GPSDrive::initgpsCallback, this);
    sensor_msgs::NavSatFix::ConstPtr initGPS_msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/initGPS", ros::Duration(1000));
    //! Convert the initial GPS coordinates to UTM coordinates
    double init_gps_lat = initGPS_msg->latitude;
    double init_gps_lon = initGPS_msg->longitude;
    std::string utm_zone_tmp;
    double utmX=0.0;
    double utmY=0.0;
    NavsatConversions::LLtoUTM(init_gps_lat,init_gps_lon,utmY,utmX,utm_zone_tmp);
    init_utm_y_ = utmY;
    init_utm_x_ = utmX;
    std::cout<<"init_utm_x = "<<init_utm_x_<<std::endl;
    std::cout<<"init_utm_y = "<<init_utm_y_<<std::endl;
    std::cout<<"initial GPS received"<<std::endl;
    std::cout<<init_gps_lat<<"    "<<init_gps_lon<<"\n"<<std::endl;

    //! We will subscribe to the "/odometry/filtered" topic to get robot pose data
    ros::Subscriber robot_pose = nh.subscribe("/odometry/filtered/global", 1, &GPSDrive::poseCallback, this);
    ros::Subscriber robot_gps = nh.subscribe("/gps/filtered", 1, &GPSDrive::gpsCallback, this);
    ros::Subscriber robot_waypoint=nh.subscribe("/waypoint", 1, &GPSDrive::waypointCallback, this);
    ros::Publisher robot_driver=nh.advertise<geometry_msgs::Twist>("/twist_marker_server/cmd_vel", 1);
    
    ros::Rate r(10);
    while (ros::ok())
    {
      ros::spinOnce(); 
      computeCmd();
      robot_driver.publish(base_cmd_);
      //evaluatePosition();
      imgAcquire();
      r.sleep();
    }  
  }  
  
  void GPSDrive::computeCmd()
  {
    double x0 = utm_x_current_;
    double y0 = utm_y_current_;
    double x1 = utm_x_waypoint_;
    double y1 = utm_y_waypoint_;
    if (utm_x_waypoint_pre_ != x1||utm_y_waypoint_pre_ != y1){
	waypoint_changed_=true;
    }
    utm_x_waypoint_pre_ = x1;
    utm_y_waypoint_pre_ = y1;
    if (x1==0 || y1 ==0){
       x1=x0;
       y1=y0;
    }
    bearing_ = computeBearing(x0,y0,x1,y1);
    std::cout<<"bearing = "<<bearing_<<std::endl;
    std::cout<<"tracking= "<<tracking_<<std::endl;
    double dist = sqrt(pow((x1-x0),2)+pow((y1-y0),2));
    double theta = tracking_ - bearing_;
    if (theta<-180.0){
	theta = theta+360.0;
    }else if (theta>180.0){ 
	theta = theta-360.0;
    }
    std::cout<<"theta = "<<theta<<std::endl;
    dist_d_ = dist-dist_pre_;
    theta_d_ = theta - theta_pre_;
    double x_speed = (Kp_*dist)/10;
    if (x_speed > 0.8)
    {
       x_speed = 0.8;
    }
    if (x_speed < 0.15)
    { 
       x_speed = 0.15;
    }
    double z_angular = (Kp_*theta)/90.0;
    if (z_angular>0.8)
    {
       z_angular=0.8;
    }
    if (z_angular<-0.8)
    {
       z_angular = -0.8;
    }
    std::cout<<"distance = "<<dist<<std::endl;
    if (dist<0.2){
	x_speed=0.0;
	z_angular=0.0;
	if (waypoint_status_ == true)
	{
	  waypoint_arrived_ = true;
	}
    }else{
	  waypoint_arrived_= false;
	 }

    dist_pre_ = dist;
    theta_pre_ = theta;
    dist_i_ = dist_i_+dist;
    theta_i_ = theta_i_+theta;
    base_cmd_.linear.x = x_speed;
    base_cmd_.linear.y = 0.0;
    base_cmd_.angular.z = z_angular;
    std::cout<<"base_cmd_.x = " << x_speed<<std::endl;
    std::cout<<"base_cmd.angular = "<<z_angular<<std::endl;
    
  }
   

  void GPSDrive::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // store the pose of the robot
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout<<"yaw = "<<yaw<<std::endl;
    yaw=yaw/3.141592653*180.0;
    if (yaw<0.0){
      yaw=-yaw;} else{yaw=360.0-yaw;}
    tracking_ = yaw; 
  }

  void GPSDrive::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& filtered_gps)
  {
   bool good_gps = (!std::isnan(filtered_gps->altitude) &&
                    !std::isnan(filtered_gps->latitude) &&
                    !std::isnan(filtered_gps->longitude));
   
   if(good_gps)
   {
      double gps_lat = filtered_gps->latitude;
      double gps_lon = filtered_gps->longitude;
      std::cout<<gps_lat<<"     "<<gps_lon<<std::endl;
      double utmX=0.0;
      double utmY=0.0;
      std::string utm_zone_tmp;
      NavsatConversions::LLtoUTM(gps_lat,gps_lon,utmY,utmX,utm_zone_tmp);
      std::cout<<utmX<<"     "<<utmY<<std::endl;
      /*if (count_ < 1)
      {
        utm_dx_ = utmX - init_utm_x_;
        utm_dy_ = utmY - init_utm_y_;
        count_=count_+1;
      }*/

      utm_x_current_ = utmX - utm_dx_;
      utm_y_current_ = utmY - utm_dy_;
      utm_x_current_ = utm_x_current_- init_utm_x_;
      utm_y_current_ = utm_y_current_- init_utm_y_;
      std::cout<<"gps received"<<std::endl;   
   }
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
     bool good_waypoint = (!std::isnan(waypoint->point.x) &&
                    !std::isnan(waypoint->point.y));
    if (good_waypoint){
      utm_x_waypoint_ = waypoint->point.x;
      utm_y_waypoint_ = waypoint->point.y;
    }
    if (utm_x_waypoint_ !=0 || utm_y_waypoint_ !=0){waypoint_status_=true;}
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

  double GPSDrive::evaluatePosition()
  {
    double error=0.0;
    int counter=0;
    if (evalute_==true && waypoint_arrived_==true)
    {
      sensor_msgs::NavSatFix::ConstPtr fixGPS_msg=ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/gps/fix", ros::Duration(2));
      sensor_msgs::NavSatFix::ConstPtr filteredGPS_msg=ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/gps/filtered", ros::Duration(10));
      
      double gps_lat_ave = 0.0;
      double gps_lon_ave = 0.0;
      
      while(counter<50)
      {
        fixGPS_msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/gps/fix", ros::Duration(2));
        double gpsfix_lat = fixGPS_msg->latitude;
        double gpsfix_lon = fixGPS_msg->longitude;
	gps_lat_ave += gpsfix_lat;
	gps_lon_ave += gpsfix_lon;
	counter++;
	sleep(1);
      }
      gps_lat_ave=gps_lat_ave/50.0;
      gps_lon_ave=gps_lon_ave/50.0;
      filteredGPS_msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/gps/filtered", ros::Duration(10));
      double gpsfiltered_lat = filteredGPS_msg->latitude;
      double gpsfiltered_lon = filteredGPS_msg->longitude;
      std::string utm_zone_tmp;
      double utmX_ave=0.0;
      double utmY_ave=0.0;
      double utmX_filtered=0.0;
      double utmY_filtered=0.0;
      NavsatConversions::LLtoUTM(gps_lat_ave,gps_lon_ave,utmY_ave,utmX_ave,utm_zone_tmp);
      NavsatConversions::LLtoUTM(gpsfiltered_lat,gpsfiltered_lon,utmY_filtered,utmX_filtered,utm_zone_tmp);
      error= sqrt(pow((utmY_ave-utmY_filtered),2)+pow((utmX_ave-utmX_filtered),2));
      std::cout<<"\n"<<"\n"<<"\n"<<std::endl;
    }
    std::cout<<"the evaluated error is = "<<error<<std::endl;
    return error;
  }

  void GPSDrive::imgAcquire()
  {
    if (evalute_==true && waypoint_arrived_==true&&waypoint_changed_==true)
    {
    	sensor_msgs::NavSatFix::ConstPtr filteredGPS_msg=ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/gps/filtered", ros::Duration(10));
    	std::stringstream gps_lat;
    	std::stringstream gps_lon;
    	gps_lat<<filteredGPS_msg->latitude;
    	gps_lon<<filteredGPS_msg->longitude;
    	std::string filename = "Desktop/gphoto2test/pics/"+gps_lat.str()+"+"+gps_lon.str();
    	std::string gphoto_capture = "sshpass -p clearpath ssh administrator@precag  gphoto2 --capture-image-and-download --filename="+filename+".jpg";
        system(gphoto_capture.c_str());
        waypoint_changed_=false;
    }
  }

}
  

