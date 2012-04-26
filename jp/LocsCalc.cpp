#include <iostream>
#include <stdlib.h>
#include "LocsCalc.h"
#include "Search.h"



float pixelsPerGrid2 = 16;

vector<vector<int>> emptyGrid;


LocsCalc::LocsCalc() : webCam(0) {
	// Initialize Camera
	webCam.set(CV_CAP_PROP_FRAME_WIDTH,640);
	webCam.set(CV_CAP_PROP_FRAME_HEIGHT,480);
	
	// Initialize Blob Detector
	params.minArea = 5;
	params.maxArea = 200;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByArea = true;
	blobDetector = SimpleBlobDetector(params);
	blobDetector.create("SimpleBlob");

	params.minArea = 100;
	params.maxArea = 4000;
	rovBD = SimpleBlobDetector(params);
	rovBD.create("SimpleBlob");

	params2.minArea = 50;
	params2.maxArea = 5000;
	params2.filterByColor = false;
	params2.filterByCircularity = false;
	params2.filterByInertia = false;
	params2.filterByConvexity = false;
	params2.filterByArea = true;
	blobDetector2 = SimpleBlobDetector(params2);
	blobDetector2.create("SimpleBlob");
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

			/*b = ((uchar*)(img.imageData + img.widthStep*i))[j*3];
			g = ((uchar*)(img.imageData + img.widthStep*i))[j*3+1];
			r = ((uchar*)(img.imageData + img.widthStep*i))[j*3+2];*/

			/*if ((r | g | b) > 62) {
				newRow.push_back(1);				
			} else {*/
				newRow.push_back(0);
			//}
		}
		obstacleGrid.push_back(newRow);		// obstacleGrid will be sent as input to Grid() in Search.cpp
	}

	emptyGrid = obstacleGrid;

	Mat obsCopy;
	cvtColor(obstacles, obsCopy, CV_RGB2GRAY);
	obstacles = obsCopy;
	threshold (obstacles,obstacles,30,255, CV_THRESH_BINARY);
	
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
	bool foundRovio = false;

	int buildingIndex;
	int numBuildings;
	//bool[] isBuilding;

	vector<vector<int>> pixelPath;
	int indexOnPath = 0;

	vector<KeyPoint> buildings;
	
	int sendMsg = 0;
	while(1) {

		if (!foundPleoFront || !foundPleoBack || !foundRovio) {
			cout << "looking for pleo and rovio..." << endl;
		}

		// get new image from webcam
		webCam >> camImage;

		if (project >= 3) {
			// find image sans background + obstacles
			absdiff(fullBackground, camImage, noBack);
			blur(noBack, noBack, Size(4,4));
		}

		noBack.copyTo(rov);
		cvtColor(rov, rov, CV_RGB2GRAY);
		threshold(rov, rov, 30, 255, CV_THRESH_BINARY);
		
		// convert raw image to hsv
		cvtColor (camImage, hsvImage, CV_RGB2HSV);
		
		// blur images
		blur(hsvImage, whsv, Size(WHITEB,WHITEB));
		blur(hsvImage, phsv, Size(PINKB,PINKB));
		
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
		
		//apply threshold HUE upper/lower for color range
		threshold (whue,lb,WHITEHL,255, CV_THRESH_BINARY); // get lower bound
		threshold (whue,hb,WHITEHH,255, CV_THRESH_BINARY_INV); // get upper bound
		whue = lb & hb; // multiply 2 matrix to get the color range
		threshold (phue,lb,PINKHL,255, CV_THRESH_BINARY);
		threshold (phue,hb,PINKHH,255, CV_THRESH_BINARY_INV);
		phue = lb & hb;
		
		// apply threshold for Sat channel
		threshold (wsat,lb,WHITESL,255, CV_THRESH_BINARY);
		threshold (wsat,hb,WHITESH,255, CV_THRESH_BINARY_INV);
		wsat = lb & hb;
		threshold (psat,lb,PINKSL,255, CV_THRESH_BINARY);
		threshold (psat,hb,PINKSH,255, CV_THRESH_BINARY_INV);
		psat = lb & hb;
		
		// apply thresshold for Val channel
		threshold (wval,lb,WHITEVL,255, CV_THRESH_BINARY);
		threshold (wval,hb,WHITEVH,255, CV_THRESH_BINARY_INV);
		wval = lb & hb;
		threshold (pval,lb,PINKVL,255, CV_THRESH_BINARY);
		threshold (pval,hb,PINKVH,255, CV_THRESH_BINARY_INV);
		pval = lb & hb;
		
		// combine sat, val and hue filter together
		whsv = whue & wsat & wval;
		phsv = phue & psat & pval;

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
			imshow("pink",phsv);*/
			whsv &= noBack;
			phsv &= noBack;
		}

		// find white-colored blobs
		blobDetector.detect(whsv, keyPoints);
		float size = 0;
		
		if (foundPleoFront) {
		
			//cout << "white key points size: " << keyPoints.size() << endl;
			// locate largest blob
			for(unsigned int i=0; i<keyPoints.size(); i++) {
				if (keyPoints[i].size > size && keyPoints[i].pt.x != 0) {

					float tmpX = keyPoints[i].pt.x;
					float tmpY = 480 - keyPoints[i].pt.y;
					if (sqrt((tmpX-xpleofront)*(tmpX-xpleofront)+(tmpY-ypleofront)*(tmpY-ypleofront)) < 15) {
						xpleorear = keyPoints[i].pt.x;
						ypleorear = 480 - keyPoints[i].pt.y;
						size = keyPoints[i].size;
						
						if (!foundPleoBack)
							cout << " * found pleo back" << endl;
						foundPleoBack = true;

						permKeyPoints.push_back(keyPoints[i]);

						break;
					}

				}
			}
		}
		
		// circle pleo rear on camera
		circle(camImage, Point2f(xpleorear, 480 - ypleorear), 20, cvScalar(255,0,0));
		
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
		


		rovBD.detect(rov, keyPoints);
		size = 0;
		if (keyPoints.size() > 0){
			if (!foundRovio)
				cout << " * found rovio" << endl;
			foundRovio = true;
		}

		for (unsigned int i=0; i<keyPoints.size(); i++){
			if (keyPoints[i].size > size && keyPoints[i].pt.x != 0){
				xrov = keyPoints[i].pt.x;
				yrov = 480 - keyPoints[i].pt.y;

				//if (fabs(xrov-xpleorear) < 50 || fabs(yrov-ypleorear) < 50){
				//      cout << "xfabs: " << fabs(xrov-xpleorear) << endl;
				//      cout << "yfabs: " << fabs(yrov-ypleorear) << endl;
				//      cout << "found pleo again" << endl;
				//      continue; //found pleo again.
				//}
				size = keyPoints[i].size;
			}

			permKeyPoints.push_back(keyPoints[i]);
		}

		//circle rovio on camera
		circle(camImage, Point2f(xrov, 480 - yrov), 20, cvScalar(0,255,0));


		
		// calculate the pleo's orientation
		float orient = calcOrient(xpleofront, ypleofront, xpleorear, ypleorear);
		
		float pleo[3] = {(xpleofront + xpleorear) / 2, (ypleofront + ypleorear) / 2, orient};

		/*if (project <= 2) {
		//	// pass relevant information to route planner
		//	float defaultThresh = 15;
		//	msg = rp.performAction(pleo, fruit, &defaultThresh);
		//	if (0 == msg[1].compare("normalize")){
		//		resetPath();
		//		break;
		//	}
		//} else*/ if (project >= 3 && foundPleoFront && foundPleoBack && foundRovio) {

			if (!afterFirstRun) {

				blobDetector2.detect(obstacles, buildings);

				vector<int> end;
				buildingIndex = 0;
				numBuildings = buildings.size();

				for (int i=0; i<buildings.size(); i++) {
					if (i==0) {
						end.push_back((int)buildings[0].pt.x);
						end.push_back(480 - (int)buildings[0].pt.y);
					}
					circle(obstacles, Point2f(buildings.at(i).pt.x, buildings.at(i).pt.y), 5, cvScalar(0,0,255));
				}

				vector<int> pleoLoc; pleoLoc.push_back(pleo[0]); pleoLoc.push_back(pleo[1]);
				

				herp derp TODO: not @ 0,0 but @ rovioLoc in grid row/col terms

				obstacleGrid = emptyGrid;
				obstacleGrid[0][0] = 1;

				pixelPath = search.findPath(obstacleGrid, pleoLoc, end);
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

			float checkPoint[] = {pixelPath[indexOnPath][0] + (pixelsPerGrid2/2), pixelPath[indexOnPath][1] - (pixelsPerGrid2/2)};
			float thresh = 35;	// TODO: good value // checkPoint along path
			if (indexOnPath == pixelPath.size()-1) {	// checkPoint is the fruit
				thresh = 15;
			}

			msg = rp.performAction(pleo, checkPoint, &thresh, pixelPath.size()-indexOnPath-1);

			if (sendMsg > 4){
				cout << "MSG: " << msg[1] << endl;
				msq->PushMessage("pleo", msg);
				sendMsg = 0;
			}

			if (0 == msg[1].compare("normalize") && afterFirstRun) {	// reached current checkpoint
				if (++indexOnPath >= pixelPath.size()) {
					buildingIndex++;

					if (buildingIndex >= numBuildings)
						break;
					
					vector<int> end;

					end.push_back((int)buildings[buildingIndex].pt.x);
					end.push_back(480 - (int)buildings[buildingIndex].pt.y);

					vector<int> pleoLoc; pleoLoc.push_back(pleo[0]); pleoLoc.push_back(pleo[1]);
					pixelPath = search.findPath(obstacleGrid, pleoLoc, end);
					indexOnPath = 0;
				}
			}
		} else {
			if (sendMsg > 4 && msg.size() > 1){
				cout << "MSG: " << msg[1] << endl;
				msq->PushMessage("pleo", msg);
				sendMsg = 0;
			}
		}


		// display locations in video feed
		showImages();

		/*if (0 == msg[1].compare("normalize")){
			resetPath();
			break;
		}*/
		sendMsg++;
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