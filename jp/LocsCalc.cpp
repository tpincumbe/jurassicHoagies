#include <iostream>
#include <stdlib.h>
#include "LocsCalc.h"
#include "Search.h"



float pixelsPerGrid2 = 4.318*4;



LocsCalc::LocsCalc() : webCam(0) {
	// Initialize Camera
	webCam.set(CV_CAP_PROP_FRAME_WIDTH,640);
	webCam.set(CV_CAP_PROP_FRAME_HEIGHT,480);
	
	// Initialize Blob Detector
	/*params.minThreshold = 0;
	params.maxThreshold = 255;
	params.thresholdStep = 5;*/
	//params.minDistBetweenBlobs = 10000;
	params.minArea = 20;
	//params.minConvexity = .4f;
	//params.minInertiaRatio = .1f;
	params.maxArea = 5000;
	//params.maxConvexity = 2;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByArea = true;
	blobDetector = SimpleBlobDetector(params);
	blobDetector.create("SimpleBlob");
}

LocsCalc::~LocsCalc() {
	cvDestroyAllWindows();
}

void LocsCalc::grabBackground() {
	for (int i = 0; i < 10; i++){
		webCam >> camImage;
	}
	camImage.copyTo(background);
}

void LocsCalc::grabObstacles() {
	for (int i = 0; i < 10; i++){
		webCam >> camImage;
	}
	absdiff(background, camImage, obstacles);
	camImage.copyTo(fullBackground);
	
	int r;
	int g;
	int b;

	
	IplImage img = obstacles;

	for (int i = 0; i < obstacles.size().height; i ++) {
		vector<int> newRow;
		for (int j = 0; j < obstacles.size().width; j ++ ) {



			/*Vec3f& elem = obstacles.at<Vec3f>(i, j);
			b = elem[0];
			g = elem[1];
			r = elem[2];

			if (i==0 && j==0)
				cout << "r: " << r << " g: " << g << " b: " << b << endl;*/

			
			b = ((uchar*)(img.imageData + img.widthStep*i))[j*3];
			g = ((uchar*)(img.imageData + img.widthStep*i))[j*3+1];
			r = ((uchar*)(img.imageData + img.widthStep*i))[j*3+2];
			

			/*r = obstacles.data[i*2 * obstacles.size().width + j];
			g = obstacles.data[((i*2)+1) * obstacles.size().width + j];
			b=0;*/
			//b = obstacles.data[((i*2)+2) * obstacles.size().width + j];


			/*r = obstacles.data[i * obstacles.size().width + j * 3];
			g = obstacles.data[i * obstacles.size().width + j * 3 + 1];
			b = obstacles.data[i * obstacles.size().width + j * 3 + 2];*/
			if ((r | g | b) > 31) {
				newRow.push_back(1);
			} else {
				newRow.push_back(0);
			}
		}
		obstacleGrid.push_back(newRow);		// obstacleGrid will be sent as input to Grid() in Search.cpp
	}
	
	Logger *loggr = new Logger("output_file.txt");

	cout << "Obstacle grid:" << endl;
	for (int i=0; i<obstacleGrid.size(); i++) {
		string s;
		stringstream ss;
		for (int j=0; j<obstacleGrid.at(0).size(); j++) {
			ss << obstacleGrid.at(i).at(j);
		}
		s = ss.str();
		char* sc = &s.at(0);
		loggr->out(sc);
	}

}

vector<vector<int>> LocsCalc::getMap() {
	return obstacleGrid;
}

void LocsCalc::detect(int project, SystemQueue *msq) {

	vector<string> msg;
	Mat test;

	bool afterFirstRun = false;
	bool foundPleoFront = false;
	bool foundPleoBack = false;
	bool foundFruit = false;

	vector<vector<int>> pixelPath;
	int indexOnPath = 0;

	while(1){

		if (!foundPleoFront || !foundPleoBack || !foundFruit) {
			cout << "looking for pleo and fruit..." << endl;
		}

		// get new image from webcam
		webCam >> camImage;

		if (project >= 3) {
			// find image sans background + obstacles
			absdiff(fullBackground, camImage, noBack);
			blur(noBack, noBack, Size(4,4));
		}
		
		// convert raw image to hsv
		cvtColor (camImage, hsvImage, CV_RGB2HSV);
		
		// blur images
		blur(hsvImage, whsv, Size(WHITEB,WHITEB));
		blur(hsvImage, phsv, Size(PINKB,PINKB));
		blur(hsvImage, ahsv, Size(APPLEB,APPLEB));
		
		// split image into H, S, and V images
		split(whsv,slices);
		slices[0].copyTo(whue); // get the hue channel
		slices[1].copyTo(wsat); // get the sat channel
		slices[2].copyTo(wval); // get the val channel
		
		// split image into H, S, and V images
		split(phsv,slices);
		slices[0].copyTo(phue); // get the hue channel
		slices[1].copyTo(psat); // get the sat channel
		slices[2].copyTo(pval); // get the val channel
		
		// split image into H, S, and V images
		split(ahsv,slices);
		slices[0].copyTo(ahue); // get the hue channel
		slices[1].copyTo(asat); // get the sat channel
		slices[2].copyTo(aval); // get the val channel
		
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

		if (project >= 3) {
			// split image into R, G, and B images
			split(noBack,slices);
			
			/* combine slices such that the new image is as
			   bright or brighter than the brightest channel */
			slices[0].copyTo(noBack);
			noBack |= slices[1];
			noBack |= slices[2];
			
			// apply threshold
			threshold (noBack,noBack,31,255, CV_THRESH_BINARY);
			
			// combine hsv images with subtracted background
			/*imshow("white",whsv);
			imshow("pink",phsv);
			imshow("apple",ahsv);*/
			whsv &= noBack;
			phsv &= noBack;
			ahsv &= noBack;
		}

		// find white-colored blobs
		blobDetector.detect(whsv, keyPoints);
		float size = 0;
		
		if (keyPoints.size() > 0) {
			if (!foundPleoBack)
				cout << " * found pleo back" << endl;
			foundPleoBack = true;
		}
		
		//cout << "white key points size: " << keyPoints.size() << endl;
		// locate largest blob
		for(unsigned int i=0; i<keyPoints.size(); i++) {
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
		
		if (keyPoints.size() > 0) {
			if (!foundPleoFront)
				cout << " * found pleo front" << endl;
			foundPleoFront = true;
		}

		//cout << "pink key points size: " << keyPoints.size() << endl;
		for(unsigned int i=0; i<keyPoints.size(); i++) {
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
		//cout << "apple key points size: " << keyPoints.size() << endl;

		if (keyPoints.size() > 0) {
			if (!foundFruit)
				cout << " * found fruit" << endl;
			foundFruit = true;
		}

		for(unsigned int i=0; i<keyPoints.size(); i++) {
			if (keyPoints[i].size > size && keyPoints[i].pt.x != 0) {
				xfruit = keyPoints[i].pt.x;
				yfruit = 480 - keyPoints[i].pt.y;
				size = keyPoints[i].size;
			}
			
			permKeyPoints.push_back(keyPoints[i]);
		}
		
		// circle apple on camera
		circle(camImage, Point2f(xfruit, 480 - yfruit), 20, cvScalar(0,0,255));
		
		// calculate the pleo's orientation
		float orient = calcOrient(xpleofront, ypleofront, xpleorear, ypleorear);
		
		float pleo[3] = {(xpleofront + xpleorear) / 2, (ypleofront + ypleorear) / 2, orient};
		float fruit[2] = {xfruit, yfruit};

		if (project <= 2) {
			// pass relevant information to route planner
			float defaultThresh = 15;
			msg = rp.performAction(pleo, fruit, &defaultThresh);
			if (0 == msg[1].compare("normalize")){
				resetPath();
				break;
			}
		} else if (project >= 3 && foundPleoFront && foundPleoBack && foundFruit) {

			if (!afterFirstRun) {
				vector<int> pleoLoc; pleoLoc.push_back(pleo[0]); pleoLoc.push_back(pleo[1]);
				vector<int> fruitLoc; fruitLoc.push_back(fruit[0]); fruitLoc.push_back(fruit[1]);
				pixelPath = search.findPath(obstacleGrid, pleoLoc, fruitLoc);
				indexOnPath = 0;

				cout << "indexOnPath = " << indexOnPath << endl;

				afterFirstRun = true;
			}
			
			for (int i=0; i<640; i+=pixelsPerGrid2) {
				line(camImage, Point(i,0), Point(i,480), RGB(255,0,255), 1);
			}
			for (int j=0; j<480; j+=pixelsPerGrid2) {
				line(camImage, Point(0,j), Point(640,j), RGB(255,0,255), 1);
			}
			for (int i=0; i<pixelPath.size(); i++) {
				cv::Scalar theColor = Scalar(0,255,0);
				if (i < indexOnPath)
					theColor = Scalar(255,0,0);
				circle(camImage,Point(pixelPath.at(i).at(0),480-pixelPath.at(i).at(1)),3,theColor);
			}

			//cout << "indexOnPath = " << indexOnPath << " checkPoint=(" << pixelPath[indexOnPath][0] << ", " << pixelPath[indexOnPath][1] << ")" << endl;

			float checkPoint[] = {pixelPath[indexOnPath][0], pixelPath[indexOnPath][1]};
			float thresh = 35;	// TODO: good value // checkPoint along path
			if (indexOnPath == pixelPath.size()-1) {	// checkPoint is the fruit
				thresh = 15;
			}

			msg = rp.performAction(pleo, checkPoint, &thresh);

			msq->PushMessage("pleo", msg);

			if (0 == msg[1].compare("normalize")) {	// reached current checkpoint
				if (++indexOnPath >= pixelPath.size())	break;
			}
		} else {
			msq->PushMessage("pleo", msg);
		}


		// display locations in video feed
		showImages();

		/*if (0 == msg[1].compare("normalize")){
			resetPath();
			break;
		}*/

	}
}

float LocsCalc::calcOrient(float xpf, float ypf, float xpr, float ypr) {
	// if the front is above the back, return a value between 0 and 180
	if (ypf > ypr)
		return static_cast<float>(atan2(ypf-ypr,xpf-xpr) * 180 / M_PI);
	// otherwise, return a value between 180 and 360
	else
		return static_cast<float>(atan2(ypf-ypr,xpf-xpr) * 180 / M_PI + 360);
}

void LocsCalc::resetPath() {
	cvDestroyAllWindows();
}

void LocsCalc::showImages() {
	
	/*cv::Scalar green = cvScalar(0,255,0);
	circle(camImage, Point(100, 10), 5, green, 1);*/

	imshow("Webcam Original", camImage);
//	imshow("Background", background);

	imshow("Obstacles", obstacles);
	//imshow("fullBackground", fullBackground);
	imshow("noBack",noBack);
//	imshow("HSV",whsv);
//	imshow("HSV2",phsv);
//	imshow("HSV3",ahsv);
	
	cvWaitKey(1);
}