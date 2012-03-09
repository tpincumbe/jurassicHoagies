#include <iostream>
#include <stdlib.h>
#include "LocsCalc.h"

LocsCalc::LocsCalc() {
	
}

LocsCalc::~LocsCalc() {
	cvDestroyAllWindows();
}

void LocsCalc::detect() {
	VideoCapture webCam(0); // video source for webcam
	
	webCam.set(CV_CAP_PROP_FRAME_WIDTH,640);
	webCam.set(CV_CAP_PROP_FRAME_HEIGHT,480);

	SimpleBlobDetector::Params params;
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
	SimpleBlobDetector blobDetector( params );
	blobDetector.create("SimpleBlob");

	// get new image over and over from webcam
	webCam >> camImage;
	
	// conver raw image to hsv
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
	threshold (whue,whl,WHITEHL,255, CV_THRESH_BINARY); // get lower bound
	threshold (whue,whh,WHITEHH,255, CV_THRESH_BINARY_INV); // get upper bound
	wh = whl &whh; // multiply 2 matrix to get the color range
	threshold (phue,phl,PINKHL,255, CV_THRESH_BINARY); // get lower bound
	threshold (phue,phh,PINKHH,255, CV_THRESH_BINARY_INV); // get upper bound
	ph = phl &phh; // multiply 2 matrix to get the color range
	threshold (ahue,ahl,APPLEHL,255, CV_THRESH_BINARY); // get lower bound
	threshold (ahue,ahh,APPLEHH,255, CV_THRESH_BINARY_INV); // get upper bound
	ah = ahl &ahh; // multiply 2 matrix to get the color range

	// apply thresshold for Sat channel
	threshold (wsat,wsl,WHITESL,255, CV_THRESH_BINARY); // get lower bound
	threshold (wsat,wsh,WHITESH,255, CV_THRESH_BINARY_INV); // get upper bound
	ws = wsl &wsh; // multiply 2 matrix to get the color range
	threshold (psat,psl,PINKSL,255, CV_THRESH_BINARY); // get lower bound
	threshold (psat,psh,PINKSH,255, CV_THRESH_BINARY_INV); // get upper bound
	ps = psl &psh; // multiply 2 matrix to get the color range
	threshold (asat,asl,APPLESL,255, CV_THRESH_BINARY); // get lower bound
	threshold (asat,ash,APPLESH,255, CV_THRESH_BINARY_INV); // get upper bound
	as = asl &ash; // multiply 2 matrix to get the color range

	// apply thresshold for Val channel
	threshold (wval,wvl,WHITEVL,255, CV_THRESH_BINARY); // get lower bound
	threshold (wval,wvh,WHITEVH,255, CV_THRESH_BINARY_INV); // get upper bound
	wv = wvl &wvh; // multiply 2 matrix to get the color range
	threshold (pval,pvl,PINKVL,255, CV_THRESH_BINARY); // get lower bound
	threshold (pval,pvh,PINKVH,255, CV_THRESH_BINARY_INV); // get upper bound
	pv = pvl &pvh; // multiply 2 matrix to get the color range
	threshold (aval,avl,APPLEVL,255, CV_THRESH_BINARY); // get lower bound
	threshold (aval,avh,APPLEVH,255, CV_THRESH_BINARY_INV); // get upper bound
	av = avl &avh; // multiply 2 matrix to get the color range

	// combine sat, val and hue filter together
	wHSV = wh & ws & wv;
	pHSV = ph & ps & pv;
	aHSV = ah & as & av;

	blobDetector.detect(wHSV, keyPoints);
	float size = 0;
	for(int i=0; i<keyPoints.size(); i++) {
		if (keyPoints[i].size > size && keyPoints[i].pt.x != 0) {
			xpleorear = keyPoints[i].pt.x;
			ypleorear = 480 - keyPoints[i].pt.y;
			size = keyPoints[i].size;
		}
		//printf("rear size: %f \n", keyPoints[i].size);
		permKeyPoints.push_back(keyPoints[i]);
	}
	circle(camImage, Point2f(xpleorear, 480 - ypleorear), 20, cvScalar(0,0,255));

	blobDetector.detect(pHSV, keyPoints);
	size = 0;
	//printf("%d",keyPoints.size());
	for(int i=0; i<keyPoints.size(); i++) {
		//printf(" X: %f Y: %f Point Size: %f", keyPoints[i].pt.x, keyPoints[i].pt.y, keyPoints[i].size);
		if (keyPoints[i].size > size && keyPoints[i].pt.x != 0) {
			xpleofront = keyPoints[i].pt.x;
			ypleofront = 480 - keyPoints[i].pt.y;
			size = keyPoints[i].size;
		}
		//printf("front size: %f \n", keyPoints[i].size);
		permKeyPoints.push_back(keyPoints[i]);
	}
	//printf(" X: %f Y: %f Size: %f\n", xpleofront, ypleofront, size);
	circle(camImage, Point2f(xpleofront, 480 - ypleofront), 20, cvScalar(0,0,255));

	blobDetector.detect(aHSV, keyPoints);
	size = 0;
	//printf("%d",keyPoints.size());
	for(int i=0; i<keyPoints.size(); i++) {
		//printf(" Point Size: %f", keyPoints[i].size);
		if (keyPoints[i].size > size && keyPoints[i].pt.x != 0) {
			xfruit = keyPoints[i].pt.x;
			yfruit = 480 - keyPoints[i].pt.y;
			size = keyPoints[i].size;
		}
		//printf("apple size: %f \n", keyPoints[i].size);
		permKeyPoints.push_back(keyPoints[i]);
	}
	//printf("\n");
	circle(camImage, Point2f(xfruit, 480 - yfruit), 20, cvScalar(0,0,255));
	
	showImages();

	float orient = calcOrient(xpleofront, ypleofront, xpleorear, ypleorear);

	//printf("Orientation: %f\n", orient);

	float pleo[3] = {(xpleofront + xpleorear) / 2, (ypleofront + ypleorear) / 2, orient};
	float fruit[2] = {xfruit, yfruit};

	//printf("%f", temp);
	//system("pause");

	//printf("%f", temp);
	//system("pause");
	
	rp.performAction(pleo, fruit);

	cvWaitKey(1);
}

float LocsCalc::calcOrient(float xpf, float ypf, float xpr, float ypr) {
	if (ypf > ypr)
		return atan2(ypf-ypr,xpf-xpr) * 180 / M_PI;
	else
		return atan2(ypf-ypr,xpf-xpr) * 180 / M_PI + 360;
}

void LocsCalc::resetPath() {
	cvDestroyAllWindows();
}

void LocsCalc::showImages(){
	imshow("Webcam Orignal", camImage);
//	imshow("HSV",wHSV);
//	imshow("HSV2",pHSV);
//	imshow("HSV3",aHSV);
}