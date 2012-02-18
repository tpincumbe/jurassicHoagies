//--------------------------------------------------------------------
//
// Team: Jurassic Pork
//
// Members:
//	David House
//  John Turner
//  Shashank Chamoli
//  Tim Pincumbe
//
// Note: Portions of our code have been copied & modified from the 
// MiGIO-2 template provided by Dr. Stillman
//
//--------------------------------------------------------------------
#pragma once
#include <iostream>
#include <sstream>
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "Subsystem.h"
#include "Logger.h"
using namespace cv;
using namespace std;

class ColorBlobDetector
{
public:
	ColorBlobDetector(void);
	~ColorBlobDetector(void);

	void trace();
	void resetPath(string tracking);
	void showImages();
private:
	
	// slices matrcies that hold H,S and V
	vector<Mat> slices;
	// create matrices to hold image
	Mat camImage;		// raw image from webcam
	Mat hsvImage;		// HSV image
	Mat blurImage;		// blur image
	Mat hue;			// hue channel
	Mat hue1;			// Hue upper bound
	Mat hue2;			// Hue lower bound
	Mat hue3;			// hue color filtering
	Mat final;			// Final display image
	Mat sat;			// Sat channel
	Mat sat1;			// Sat upper bound
	Mat sat2;			// sat lower bound
	Mat sat3;			// sat color filtering
	Mat val;			// Val channel
	Mat val1;			// Val upper bound
	Mat val2;			// Val lower bound
	Mat val3;			// Val color filtering
	Mat erd;			// Erosion Image
	Mat dia;			// dialate image
	Mat HnS;			// sat and hue channel
	Mat HSV;			// sat, hue, and val channel
	int HuethresH, 
	HuethresL,
	SatthresL,
	SatthresH ,
	ValthresL,
	ValthresH ,
	erosionCount ,
	blurSize;

	int pathPoller;
	Logger *mlf;		//Use to output path to a file
	vector<KeyPoint> keyPoints;
	vector<KeyPoint> path;
	SimpleBlobDetector blobDetector;

	string m_tracking;
};