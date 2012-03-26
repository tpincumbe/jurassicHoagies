/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// HUE 124 - 145
// SAT 141 - 224
// 


/**
* This Code has been modified from the original to work with the Jurassic Pork Project.
*/
#include "ColorBlobDetector.h"
VideoCapture camera(0); // video source for webcam
ColorBlobDetector::ColorBlobDetector(void){
	camera.set(CV_CAP_PROP_FRAME_WIDTH,640);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT,480);

	// Theshold Settings
	//HuethresL = 30, HuethresH =71;  //Green
	//HuethresL =116, HuethresH = 160; //Red
	HuethresL = 112, HuethresH = 145; //Hue thresholds (red)
	SatthresL = 130, SatthresH = 255; //Saturation thresholds (red)
	ValthresL = 0, ValthresH = 0;
	blurSize = 1;

	// Initialize blob detector
	SimpleBlobDetector::Params params;
    params.minThreshold = 40;
    params.maxThreshold = 60;
    params.thresholdStep = 5;
    params.minArea = 10; 
    params.minConvexity = .4f;
    params.minInertiaRatio = .1f;
    params.maxArea = 400;
    params.maxConvexity = 2;
    params.filterByColor = false;
    params.filterByCircularity = false;
	params.filterByArea = true;
    SimpleBlobDetector bd( params );
    bd.create("SimpleBlob");
	blobDetector = bd;
	pathPoller=0;	
	mlf = new Logger("trace.txt");
}

ColorBlobDetector::~ColorBlobDetector(void){
	cvDestroyAllWindows();
}

/**
* This function will capture an image from a camera find a red colored blob and track how it moves
* Every 5 frames the tracker will find the current position and put a green dot in that position
*/
void ColorBlobDetector::trace(){
	Mat cross = getStructuringElement(MORPH_CROSS, Size(5,5));

	//Start blob detections and tracking
		camera >> camImage;
		// blur image
		blur(camImage, blurImage, Size(2,2));
		// convert raw image to hsv
		cvtColor (camImage, hsvImage, CV_RGB2HSV);
		blur(hsvImage, hsvImage, Size(3,3));
		// split image to H,S and V images
		split(hsvImage,slices);
		slices[0].copyTo(hue); // get the hue channel
		slices[1].copyTo(sat); // get the sat channel
		slices[2].copyTo(val); // get the val channel

		threshold (hue, hue1, HuethresL,255, CV_THRESH_BINARY); // get lower bound for hue
		threshold (hue, hue2, HuethresH,255, CV_THRESH_BINARY_INV); // get upper bound for hue
		hue3 = hue1 & hue2; // multiply to get color range

		// apply thresshold for Sat channel
		threshold (sat, sat1, SatthresL,255, CV_THRESH_BINARY); // get lower bound for sat
		threshold (sat, sat2, SatthresH,255, CV_THRESH_BINARY_INV); // get upper bound for sat
		sat3 = sat1 & sat2; // multiply 2 matrix to get the color range

		// apply thresshold for Val channel
		threshold (val,val1,SatthresL,255, CV_THRESH_BINARY); // get lower bound
		threshold (val, val2,SatthresH,255, CV_THRESH_BINARY_INV); // get upper bound
		val3 = val1 & val2; // multiply 2 matrix to get the color range

		// combine sat and hue filter together
		HnS = sat3 & hue3;

		// erode and dialation to reduce noise
		erode(HnS,erd,cross,Point(-1,-1),erosionCount); // do erode
		dilate(HnS,dia,cross,Point(-1,-1),erosionCount);// do dialate

		// combine sat, hue, and val filters together
		HSV = sat3 & hue3 & val3;

		// erode and dialation to reduce noise
		erode(HSV,erd,cross,Point(-1,-1),erosionCount); // do erode
		dilate(HSV,dia,cross,Point(-1,-1),erosionCount);// do dialate

		//Just to show we can access the keyPoints array directly
		//You can also investigate the properties of the features to determine their _size_
		camImage.copyTo(final);
        blobDetector.detect(HSV, keyPoints);

		//This will add points to a vector to keep track of the path the blob took
		if (++pathPoller > 5){ //How many ticks before the path is updated with a new point 
			pathPoller = 0;
			if (keyPoints.size() > 0){
				path.push_back(keyPoints[0]);
				mlf->out("%f,%f",keyPoints[0].pt.x,keyPoints[0].pt.y); //This will output the current position to a file to be read later
			}
		}

		//Draw expected path
		if ( m_tracking.compare("square") == 0 )
		{
			rectangle(final, Point(190, 170), Point(360, 340), Scalar(0, 0, 255), 10); //square path
		}
		else if ( m_tracking.compare("triangle") == 0 )
		{
			//Draw triangluar path
			line(final, Point(190, 340), Point(190, 170), Scalar(0, 255, 255), 10);
			line(final, Point(190, 170), Point(360, 255), Scalar(0, 255, 255), 10);
			line(final, Point(360, 255), Point(190, 340), Scalar(0, 255, 255), 10);
		}
		

		//Draws circle around the blob that has been detected
		for(unsigned int i=0; i<keyPoints.size(); i++){
			circle(final, keyPoints[i].pt, 20, cvScalar(255,0,0), 10);
		}

		//Draw points from path
		for(unsigned int i = 0; i < path.size(); i++){
			circle(final, path[i].pt, 1, cvScalar(0,255,0), 5);
		}
		//Alternatively, there exists a simple function:
        //drawKeypoints(camImage, keyPoints, final, CV_RGB(0,255,0), 4);

		showImages();		
        cvWaitKey(1);
}

void ColorBlobDetector::resetPath(string tracking)
{
	m_tracking = tracking;
	path.clear();
	destroyAllWindows();
	showImages();
}

void ColorBlobDetector::showImages(){
	// show images
	//imshow("HNS", HnS);
	//imshow("HSV", HSV);
	//imshow("Hue color", hue3);
	//imshow("Sat color", sat3);
	imshow("Final", final);
}