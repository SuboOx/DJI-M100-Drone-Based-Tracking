#ifndef _MISSION_PLANNER_HPP_
#define _MISSION_PLANNER_HPP_

#include <iostream>
#include <opencv2/core/core.hpp>

class MissionPlanner{
	//Data read from KCF tracker
	cv::Rect rect;
	//Cmd to publish to the drone
	double vel_x,vel_y,vel_z,vel_yaw;
	
public:
	MissionPlanner(){vel_x = 0;vel_y = 0;vel_z = 0;vel_yaw = 0;};
	void Controller();
	void setRect(cv::Rect x){rect = x;}
	
	//Data publish pipelines
	double getVelx(){
		return vel_x;
	}
	double getVely(){
		return vel_y;
	}
	double getVelz(){
		return vel_z;
	}
	double getVelyaw(){
		return vel_yaw;
	}
	
protected:
	
};



#endif
