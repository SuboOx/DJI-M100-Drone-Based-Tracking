/* Note: 1. We will need to detect and renew ROI automaticly, currently we are using mouse to mark ROI.
 * 2. The movement part need to be renewed, currently it just publish to god knows what.
 * 3. It seems that ImageConverter is copied from ROS tutorials.
 * 4. manifold_cam_init(MODE) has to be set as GET_BUFFER_MODE.
 */

#include "kcftracker.hpp"
#include "djicam.h"
#include "visual_tracker.hpp"

#define SAVE_INTERVAL 3

using namespace std;
using namespace cv;

unsigned char buffer[FRAME_SIZE] = {0};
unsigned int frame_size = 0;
unsigned int nframe = 0;

Rect selectRect;
Point origin;
Mat rgbimage;


bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;

//Set the params of KCF tracker
bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

double fps_t,fps;


void FpsCounter::Start(){
			fps_t = (double) getTickCount();
	}
void FpsCounter::End(){
			fps_t = ((double)getTickCount() - fps_t) / getTickFrequency();
	fps = 1.0/fps_t;
		}

//Mouse event here
 void onMouse(int event, int x, int y, int, void*)
{
    if (select_flag)
    {
        selectRect.x = MIN(origin.x, x);        
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);   
        selectRect.height = abs(y - origin.y);
        selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        bBeginKCF = false;  
        select_flag = true; 
        origin = cv::Point(x, y);       
        selectRect = cv::Rect(x, y, 0, 0);  
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {	
		select_flag = false;
		
		if ((origin.x == x && origin.y == y)){
			cout<<"Illegal action detected, a null rectangle has been selected"<<endl;
			cout<<"Try to move your mouse to select a rectangle"<<endl;
		}
		else{
        bRenewROI = true;
	}
    }
}


void ImageTracker::runKCF(IplImage *Img)
  {
	
	rgbimage = Img;
	
    cv::setMouseCallback(RGB_WINDOW,onMouse, 0);
	//This is to init the tracker.
	//For one obj, only exec once.
    if(bRenewROI)
    {
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
    }
    
	//The tarcking process will start to exec here
    if(bBeginKCF)
    {
        result = tracker.update(rgbimage);
        //Draw the ROI rect here.
        cv::rectangle(rgbimage, result, cv::Scalar( 0, 255, 255 ), 1, 8 );
    }
    else
		//Draw the rect that mouse selected.
        cv::rectangle(rgbimage, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);
	
	sprintf(fps_string,"%.2f",fps);
	string fpsString("FPS:");
	fpsString += fps_string;
	
	putText(rgbimage,"KCF Tracker",Point(1,20),FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),1,LINE_MAX);
	putText(rgbimage,fpsString,Point(1,40),FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),1,LINE_MAX);
    cv::imshow(RGB_WINDOW, rgbimage);
    
    if (save_count == SAVE_INTERVAL){
    
    char num[5];
    sprintf(num,"%d",save_count_s);
    
    string nameString(".png");
    
    nameString = num+nameString;
    nameString = buf+nameString;
    save_count = 0;
    cv::imwrite(nameString, rgbimage);
		
	}
	
	save_count++;
    save_count_s++;
    
    cv::waitKey(1);
  }

sRGB ImageReader::yuvTorgb(int Y, int U, int V)
{
	sRGB rgb;
	rgb.r = (int)(Y + 1.4075 * (V-128));
	rgb.g = (int)(Y - 0.3455 * (U-128) - 0.7169*(V-128));
	rgb.b = (int)(Y + 1.779 * (U-128));
	rgb.r =(rgb.r<0? 0: rgb.r>255? 255 : rgb.r);
	rgb.g =(rgb.g<0? 0: rgb.g>255? 255 : rgb.g);
	rgb.b =(rgb.b<0? 0: rgb.b>255? 255 : rgb.b);
	return rgb;
}

unsigned char* ImageReader::NV12ToRGB(unsigned char * src, unsigned char * rgb, int width, int height){
	int numOfPixel = width * height;
	int positionOfU = numOfPixel;
	int startY,step,startU,Y,U,V,index,nTmp;
	sRGB tmp;

	for(int i=0; i<height; i++){
		startY = i*width;
		step = i/2*width;
		startU = positionOfU + step;
		for(int j = 0; j < width; j++){
			Y = startY + j;
			if(j%2 == 0)
				nTmp = j;
			else
				nTmp = j - 1;
			U = startU + nTmp;
			V = U + 1;
			index = Y*3;
			tmp = yuvTorgb((int)src[Y], (int)src[U], (int)src[V]);
			rgb[index+0] = (char)tmp.b;
			rgb[index+1] = (char)tmp.g;
			rgb[index+2] = (char)tmp.r;
		}
	}
	return rgb;
}

bool ImageReader::YUV420_To_BGR24(unsigned char *puc_y, unsigned char *puc_u, unsigned char *puc_v, unsigned char *puc_rgb, int width_y, int height_y)
{
	if (!puc_y || !puc_u || !puc_v || !puc_rgb)
	{
		return false;
	}
	int baseSize = width_y * height_y;
	int rgbSize = baseSize * 3;

	BYTE* rgbData = new BYTE[rgbSize];
	memset(rgbData, 0, rgbSize);

	int temp = 0;

	BYTE* rData = rgbData; 
	BYTE* gData = rgbData + baseSize;
	BYTE* bData = gData + baseSize;

	int uvIndex =0, yIndex =0;


	for(int y=0; y < height_y; y++)
	{
		for(int x=0; x < width_y; x++)
		{
			uvIndex = (y>>1) * (width_y>>1) + (x>>1);
			yIndex = y * width_y + x;

			temp = (int)(puc_y[yIndex] + (puc_v[uvIndex] - 128) * 1.4022);
			rData[yIndex] = temp<0 ? 0 : (temp > 255 ? 255 : temp);

			temp = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * (-0.3456) +
					(puc_v[uvIndex] - 128) * (-0.7145));
			gData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);

			temp = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * 1.771);
			bData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);
		}
	}

	int widthStep = width_y*3;
	for (int y = 0; y < height_y; y++)
	{
		for (int x = 0; x < width_y; x++)
		{
			puc_rgb[y * widthStep + x * 3 + 2] = rData[y * width_y + x]; //R
			puc_rgb[y * widthStep + x * 3 + 1] = gData[y * width_y + x]; //G
			puc_rgb[y * widthStep + x * 3 + 0] = bData[y * width_y + x]; //B
		}
	}

	if (!puc_rgb)
	{
		return false;
	}
	delete [] rgbData;
	return true;
}

IplImage* ImageReader::YUV420_To_IplImage(unsigned char* pYUV420, int width, int height)
{
	if (!pYUV420)
	{
		return NULL;
	}

	int baseSize = width*height;
	int imgSize = baseSize*3;
	BYTE* pRGB24 = new BYTE[imgSize];
	memset(pRGB24, 0, imgSize);

	BYTE* yData = pYUV420; 
	BYTE* uData = pYUV420 + baseSize; 
	BYTE* vData = uData + (baseSize>>2); 

	if(YUV420_To_BGR24(yData, uData, vData, pRGB24, width, height) == false || !pRGB24)
	{
		return NULL;
	}

	IplImage *image = cvCreateImage(cvSize(width, height), 8,3);
	memcpy(image->imageData, pRGB24, imgSize);

	if (!image)
	{
		return NULL;
	}

	delete [] pRGB24;
	return image;
}

void ImageReader::ReadImage(){
		//cout<<"Reading image"<<endl;
		ret = manifold_cam_read(buffer, &nframe, 1);//Problem
		//cout<<"Image read"<<endl;
		if(ret != -1)
		{
			NV12ToRGB(buffer,pData,1280,720);
			memcpy(pRawImg->imageData,pData,FRAME_SIZE);
			cvResize(pRawImg,pImg,CV_INTER_LINEAR);
		}
		
}

void ImageReader::ReaderInit(){
	cout<<"manifold_cam initing"<<endl;
	ret = manifold_cam_init(GETBUFFER_MODE);
	cout<<"manifold_cam inited"<<endl;
	if(ret == -1)
		cout<<"manifold init error"<<endl;
}

IplImage* ImageReader::getImage(){
	return pImg;
}

