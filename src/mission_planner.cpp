#include "mission_planner.hpp"
#include <stdlib.h>

using namespace std;

#define IMAGE_H 480
#define IMAGE_W 640

#define DEAD_X 40
#define DEAD_Y 20

//image width: 640 pixels
//image height: 480 pixels
//NOTE: THE INITIAL VALUE OF X_MID AND Y_MID IS 0. CHECK THIS OUT!!!!

double PID_Controller(double P, double err){
	if (abs(P*err) > 1.5){
		return 1.5;
	}
	return P*err;
}

//Write Controller here

void MissionPlanner:: Controller(){
	double x_mid,y_mid;
	double x_err,y_err;
	
	if (rect.x == 0 && rect.y == 0){
		vel_x = 0;
		vel_y = 0;
		return;
	}
	
	x_mid = (double)(2*rect.x+rect.width)/2.0;
	y_mid = (double)(2*rect.y+rect.height)/2.0;
	
	x_err = x_mid - IMAGE_W/2;
	y_err = y_mid - IMAGE_H/2;
	
	//NEED TO IMPLEMENT DEAD ZONE -- viberate intensively now
	//cout<<"x:"<<rect.x<<" y:"<<rect.y<<" width:"<<rect.width<<" height:"<<rect.height<<endl;
	vel_y = -1*PID_Controller(0.004,x_err);
	vel_x = -1*PID_Controller(0.002,y_err);
	
	
	// This piece of code is extremely ugly, rewrite it!
	if (abs(x_err)<DEAD_X){
		vel_y = 0;
		//cout<<"balala nengliang"<<endl;
	}
	if (abs(y_err)<DEAD_Y){
		vel_x = 0;
		//cout<<"balala nengliang"<<endl;
	}
	cout<<"x_push:"<<vel_x<<" y_push:"<<vel_y<<endl;
	
	return;
}

