#include <iostream>
#include <stdlib.h>
#include "LocsCalc.h"

LocsCalc::LocsCalc() : webCam(0) {
	// Initialize Camera
	webCam.set(CV_CAP_PROP_FRAME_WIDTH,640);
	webCam.set(CV_CAP_PROP_FRAME_HEIGHT,480);
	
	// Initialize Blob Detector
	params.minThreshold = 0;
	params.maxThreshold = 255;
	params.thresholdStep = 5;
	//params.minDistBetweenBlobs = 10000;
	//params.minArea = 50;
	//params.minConvexity = .4f;
	//params.minInertiaRatio = .1f;
	//params.maxArea = 500;
	//params.maxConvexity = 2;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByArea = false;
	blobDetector = SimpleBlobDetector(params);
	blobDetector.create("SimpleBlob");
}

LocsCalc::~LocsCalc() {
	cvDestroyAllWindows();
}

void LocsCalc::grabBackground() {
	webCam >> background;
}

void LocsCalc::grabObstacles() {
	webCam >> camImage;
	absdiff(background, camImage, obstacles);
}

vector<string> LocsCalc::detect(int project) {
	vector<string> msg;

	// get new image from webcam
	webCam >> camImage;
	
	// convert raw image to hsv
	cvtColor (camImage, hsvImage, CV_RGB2HSV);
	
	// blur images
	blur(hsvImage, whsv, Size(WHITEB,WHITEB));
	blur(hsvImage, phsv, Size(PINKB,PINKB));
	blur(hsvImage, ahsv, Size(APPLEB,APPLEB));
	
	// split image to H,S and V images
	split(whsv,wslices);
	split(phsv,pslices);
	split(ahsv,aslices);
	
	wslices[0].copyTo(whue); // get the hue channel
	wslices[1].copyTo(wsat); // get the sat channel
	wslices[2].copyTo(wval); // get the V channel
	pslices[0].copyTo(phue); // get the hue channel
	pslices[1].copyTo(psat); // get the sat channel
	pslices[2].copyTo(pval); // get the V channel
	aslices[0].copyTo(ahue); // get the hue channel
	aslices[1].copyTo(asat); // get the sat channel
	aslices[2].copyTo(aval); // get the V channel
	
	//apply threshold HUE upper/lower for color range
	threshold (whue,lb,WHITEHL,255, CV_THRESH_BINARY); // get lower bound
	threshold (whue,hb,WHITEHH,255, CV_THRESH_BINARY_INV); // get upper bound
	whue = lb & hb; // multiply 2 matrix to get the color range
	threshold (phue,lb,PINKHL,255, CV_THRESH_BINARY);
	threshold (phue,hb,PINKHH,255, CV_THRESH_BINARY_INV);
	phue = lb & hb;
	threshold (ahue,lb,APPLEHL,255, CV_THRESH_BINARY);
	threshold (ahue,hb,APPLEHH,255, CV_THRESH_BINARY_INV);
	ahue = lb & hb;
	
	// apply threshold for Sat channel
	threshold (wsat,lb,WHITESL,255, CV_THRESH_BINARY);
	threshold (wsat,hb,WHITESH,255, CV_THRESH_BINARY_INV);
	wsat = lb & hb;
	threshold (psat,lb,PINKSL,255, CV_THRESH_BINARY);
	threshold (psat,hb,PINKSH,255, CV_THRESH_BINARY_INV);
	psat = lb & hb;
	threshold (asat,lb,APPLESL,255, CV_THRESH_BINARY);
	threshold (asat,hb,APPLESH,255, CV_THRESH_BINARY_INV);
	asat = lb & hb;
	
	// apply thresshold for Val channel
	threshold (wval,lb,WHITEVL,255, CV_THRESH_BINARY);
	threshold (wval,hb,WHITEVH,255, CV_THRESH_BINARY_INV);
	wval = lb & hb;
	threshold (pval,lb,PINKVL,255, CV_THRESH_BINARY);
	threshold (pval,hb,PINKVH,255, CV_THRESH_BINARY_INV);
	pval = lb & hb;
	threshold (aval,lb,APPLEVL,255, CV_THRESH_BINARY);
	threshold (aval,hb,APPLEVH,255, CV_THRESH_BINARY_INV);
	aval = lb & hb;
	
	// combine sat, val and hue filter together
	whsv = whue & wsat & wval;
	phsv = phue & psat & pval;
	ahsv = ahue & asat & aval;
	
	// find white-colored blobs
	blobDetector.detect(whsv, keyPoints);
	float size = 0;
	
	// locate largest blob
	for(int i=0; i<keyPoints.size(); i++) {
		if (keyPoints[i].size > size && keyPoints[i].pt.x != 0) {
			xpleorear = keyPoints[i].pt.x;
			ypleorear = 480 - keyPoints[i].pt.y;
			size = keyPoints[i].size;
		}
		
		permKeyPoints.push_back(keyPoints[i]);
	}
	
	// circle pleo rear on camera
	circle(camImage, Point2f(xpleorear, 480 - ypleorear), 20, cvScalar(0,0,255));
	
	// find pink-colored blobs
	blobDetector.detect(phsv, keyPoints);
	size = 0;
	
	for(int i=0; i<keyPoints.size(); i++) {
		if (keyPoints[i].size > size && keyPoints[i].pt.x != 0) {
			xpleofront = keyPoints[i].pt.x;
			ypleofront = 480 - keyPoints[i].pt.y;
			size = keyPoints[i].size;
		}
		
		permKeyPoints.push_back(keyPoints[i]);
	}
	
	// circle pleo front on camera
	circle(camImage, Point2f(xpleofront, 480 - ypleofront), 20, cvScalar(0,0,255));
	
	// find apple-colored blobs
	blobDetector.detect(ahsv, keyPoints);
	size = 0;
	
	for(int i=0; i<keyPoints.size(); i++) {
		if (keyPoints[i].size > size && keyPoints[i].pt.x != 0) {
			xfruit = keyPoints[i].pt.x;
			yfruit = 480 - keyPoints[i].pt.y;
			size = keyPoints[i].size;
		}
		
		permKeyPoints.push_back(keyPoints[i]);
	}
	
	// circle apple on camera
	circle(camImage, Point2f(xfruit, 480 - yfruit), 20, cvScalar(0,0,255));
	
	// display locations in video feed
	showImages();
	
	// calculate the pleo's orientation
	float orient = calcOrient(xpleofront, ypleofront, xpleorear, ypleorear);
	
	float pleo[3] = {(xpleofront + xpleorear) / 2, (ypleofront + ypleorear) / 2, orient};
	float fruit[2] = {xfruit, yfruit};
	
	if (project <= 2) {
		// pass relevant information to route planner
		msg = rp.performAction(pleo, fruit);
	}
	
	cvWaitKey(1);
	return msg;
}

float LocsCalc::calcOrient(float xpf, float ypf, float xpr, float ypr) {
	// if the front is above the back, return a value between 0 and 180
	if (ypf > ypr)
		return atan2(ypf-ypr,xpf-xpr) * 180 / M_PI;
	// otherwise, return a value between 180 and 360
	else
		return atan2(ypf-ypr,xpf-xpr) * 180 / M_PI + 360;
}

void LocsCalc::resetPath() {
	cvDestroyAllWindows();
}

void LocsCalc::showImages(){
	imshow("Webcam Orignal", camImage);
//	imshow("HSV",whsv);
//	imshow("HSV2",phsv);
//	imshow("HSV3",ahsv);
}
