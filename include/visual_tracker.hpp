#ifndef _VISUAL_TRACKER_HPP_
#define _VISUAL_TRACKER_HPP_

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>
#include <string.h>
#include <time.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "kcftracker.hpp"

#define IMAGE_W 1280
#define IMAGE_H 720
#define FRAME_SIZE              IMAGE_W*IMAGE_H*3

using namespace std;
using namespace cv;

struct sRGB{
	int r;
	int g;
	int b;
};

static const string RGB_WINDOW = "Image window";
typedef unsigned char BYTE;

void onMouse(int event, int x, int y, int, void*);

//Image converter is to convert image form from ROS form to OpenCV form
class ImageTracker
{
	char fps_string[10];
	float dist_val[5] ;
	
	Rect result;
	
	//To save image
	int save_count = 0;
	char buf[24];
	int save_count_s = 0;
  
public:
  ImageTracker(){
	//Open the CV window to show image
    cv::namedWindow(RGB_WINDOW);
    sprintf(buf,"./tmp/%d_",rand());
  }

  ~ImageTracker()
  {
    cv::destroyWindow(RGB_WINDOW);
  }

  void runKCF(IplImage *Img);
  
  Rect getRect(){return result;}
	
};

//ImageReader will read the image from the gimbal camera and publish it to a topic
class ImageReader{
	int ret;
	
	IplImage *pRawImg;
	IplImage *pImg;
	unsigned char *pData;
	
	
public:	
	ImageReader(){
		
		pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H),IPL_DEPTH_8U,3);
		pImg = cvCreateImage(cvSize(640, 480),IPL_DEPTH_8U,3);
		pData  = new unsigned char[1280 * 720 * 3];
		
		//time=ros::Time::now();
		//image_pub = transport.advertise("/camera/rgb/image_color", 1);		
		}	
	~ImageReader(){
	}
	
	void ReaderInit();
	void ReadImage();
	
	sRGB yuvTorgb(int Y, int U, int V);
	unsigned char * NV12ToRGB(unsigned char * src, unsigned char * rgb, int width, int height);
	bool YUV420_To_BGR24(unsigned char *puc_y, unsigned char *puc_u, unsigned char *puc_v, unsigned char *puc_rgb, int width_y, int height_y);
	IplImage* YUV420_To_IplImage(unsigned char* pYUV420, int width, int height);
	IplImage* getImage();
	
protected:
};

class FpsCounter{
	public:
	void Start();
	void End();
};



#endif
