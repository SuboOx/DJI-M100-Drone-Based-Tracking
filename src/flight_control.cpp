#include "flight_control.h"

#include <fstream>
#include <iomanip>

#include "dji_sdk/dji_sdk.h"
#include "mission_planner.hpp"
#include "visual_tracker.hpp"
#include "kcftracker.hpp"
#include "gimbal.h"

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;


ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;

Mission square_mission;

ofstream logfile;


int main(int argc, char** argv)
{	
  ros::init(argc, argv, "flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node. We get status of our drone from this part.
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

  // Publish the control signal
  // ENU refers to East-North-Up, a type of coordinate frame
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
  
  MissionPlanner mission_planner;
  
  /**Open log file and start loging**/
  logfile.open("/home/ubuntu/log.txt",ios::app);
  logfile<<"^^Record Begins!^^"<<endl;
  
  /**Obtain control and set local position**/
  obtain_control();
  //if (!set_local_position())
  //{
    //ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    //return 1;
  //}
  
  GimbalInit();
	
	/**Creating video based tracking object and starting tracking**/
	IplImage *Img;
	ImageTracker ic;
	ImageReader ir;
	FpsCounter fpscounter;
	ir.ReaderInit();
  
	/**Taking off**/
    ROS_INFO("M100 taking off!");
    //bool takeoff_result = M100monitoredTakeoff();
	bool takeoff_result = true;
	/**Executing mission**/
  if(takeoff_result)
  {	ROS_INFO("Executing Mission!");
	  	while(ros::ok()){
		
		//Read image from gimbal and track the target
		fpscounter.Start();	
		ir.ReadImage();//frame won't be read from camera while using VNC
		Img = ir.getImage();
		ic.runKCF(Img);
		
		//Calculate control signals
		mission_planner.setRect(ic.getRect());
		mission_planner.Controller();
		
		//Execute mission in the body frame.		
		sensor_msgs::Joy controlVelYawRate;
		uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_BODY   | /*depends on which frame we will use*/
                DJISDK::STABLE_ENABLE);
		
		//Sending control signals
		controlVelYawRate.axes.push_back(mission_planner.getVelx());//x
		controlVelYawRate.axes.push_back(mission_planner.getVely());//y
		controlVelYawRate.axes.push_back(mission_planner.getVelz());//fly higher
		controlVelYawRate.axes.push_back(mission_planner.getVelyaw());//yaw
		controlVelYawRate.axes.push_back(flag);
		ctrlBrakePub.publish(controlVelYawRate);
		
		ros::spinOnce();
		fpscounter.End();
		}
  }
  
  
  logfile<<"--Record Ends!--"<<endl;

  return 0;
}


/* Very simple calculation of local NED offset between two pairs of GPS
coordinates. Accurate when distances are small.
*/

//void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         //sensor_msgs::NavSatFix& target,
                         //sensor_msgs::NavSatFix& origin)
//{
  //double deltaLon = target.longitude - origin.longitude;
  //double deltaLat = target.latitude - origin.latitude;

  //deltaNed.y = deltaLat * deg2rad * C_EARTH;
  //deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  //deltaNed.z = target.altitude - origin.altitude;
//}


//geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
//{
  //geometry_msgs::Vector3 ans;

  //tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  //R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  //return ans;
//}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

/**Callback functions, executed every time the subscriber updates. **/
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;
  
  logfile<<"###lng:"<<setprecision(15)<<current_gps.longitude<<endl;
  logfile<<"***lat:"<<setprecision(15)<<current_gps.latitude<<endl;
  logfile<<"&&&alt:"<<setprecision(15)<<current_gps.altitude<<endl;
  logfile<<"%%%time:"<<elapsed_time<<endl;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

/**Monitored Landing and taking off for M100.**/
bool M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude < 0.5)
  {
    ROS_ERROR("Takeoff failed."); //Bug may occur here! GPS is not accurate.
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Takeoff successed!");
    ros::spinOnce();
  }

  return true;
}

bool M100monitoredLand(){
	dji_sdk::DroneTaskControl droneTaskControl;
	droneTaskControl.request.task = 6; //hotkey 6 refers to landing
	drone_task_service.call(droneTaskControl);
	if(!droneTaskControl.response.result) {
		ROS_WARN("Landing failed!");
	return false;
  }
  ROS_INFO("Landing successed!");
  return true;
}

//function set_local_position() is to set home position that is used to execute auto homeback mission.
bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}
