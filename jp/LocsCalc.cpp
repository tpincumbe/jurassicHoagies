#include <iostream>
#include <stdlib.h>
#include "LocsCalc.h"

float xpleofront;
float ypleofront;
float xpleorear;
float ypleorear;
float xfruit;
float yfruit;

int whitehl = 0;
int whitehh = 100;
int whitesl = 0;
int whitesh = 40;
int whitevl = 200;
int whitevh = 255;
int whiteb = 15;

int pinkhl = 110;
int pinkhh = 150;
int pinksl = 0;
int pinksh = 100;
int pinkvl = 170;
int pinkvh = 255;
int pinkb = 9;

int applehl = 100;
int applehh = 180;
int applesl = 100;
int applesh = 255;
int applevl = 0;
int applevh = 238;
int appleb = 3;

/*int whitehl = 80;
int whitehh = 100;
int whitesl = 0;
int whitesh = 40;
int whitevl = 200;
int whitevh = 255;
int whiteb = 10;

int pinkhl = 110;
int pinkhh = 150;
int pinksl = 0;
int pinksh = 100;
int pinkvl = 170;
int pinkvh = 255;
int pinkb = 9;

int applehl = 100;
int applehh = 180;
int applesl = 100;
int applesh = 255;
int applevl = 0;
int applevh = 238;
int appleb = 3;*/

int main(int argc, char **argv) {
	//xpleofront = 100;
	//ypleofront = 100;
	//xpleorear = 96;
	//ypleorear = 102;

	VideoCapture webCam(0); // video source for webcam
	
	webCam.set(CV_CAP_PROP_FRAME_WIDTH,640);
	webCam.set(CV_CAP_PROP_FRAME_HEIGHT,480);
	
	// slices matrcies that hold H,S and V
	vector<Mat> wslices;
	vector<Mat> pslices;
	vector<Mat> aslices;
	
	// Cross Element for Erosion/Dilation
	Mat cross = getStructuringElement(MORPH_CROSS, Size(5,5));
	
	// create matrices to hold image
	Mat camImage;		// raw image from webcam
	Mat wb;				// blur image
	Mat pb;				// blur image
	Mat ab;				// blur image
	Mat hsvImage;		// hsv image
	Mat whsv;			// hsv image
	Mat phsv;			// hsv image
	Mat ahsv;			// hsv image
	Mat whue;			// hue channel
	Mat phue;			// hue channel
	Mat ahue;			// hue channel
	Mat whl;			// Hue lower bound
	Mat whh;			// Hue upper bound
	Mat phl;			// Hue lower bound
	Mat phh;			// Hue upper bound
	Mat ahl;			// Hue lower bound
	Mat ahh;			// Hue upper bound
	Mat wh;				// hue color filtering
	Mat ph;				// hue color filtering
	Mat ah;				// hue color filtering
	Mat wsat;			// Sat channel
	Mat psat;			// Sat channel
	Mat asat;			// Sat channel
	Mat wsl;			// sat lower bound
	Mat wsh;			// Sat upper bound
	Mat psl;			// sat lower bound
	Mat psh;			// Sat upper bound
	Mat asl;			// sat lower bound
	Mat ash;			// Sat upper bound
	Mat ws;				// sat color filtering
	Mat ps;				// sat color filtering
	Mat as;				// sat color filtering
	Mat wval;			// Val channel
	Mat pval;			// Val channel
	Mat aval;			// Val channel
	Mat wvl;			// Val lower bound
	Mat wvh;			// Val upper bound
	Mat pvl;			// Val lower bound
	Mat pvh;			// Val upper bound
	Mat avl;			// Val lower bound
	Mat avh;			// Val upper bound
	Mat wv;				// Val color filtering
	Mat pv;				// Val color filtering
	Mat av;				// Val color filtering
	Mat wHSV;			// HSV color fiter detected
	Mat pHSV;			// HSV color fiter detected
	Mat aHSV;			// HSV color fiter detected

	SimpleBlobDetector::Params params;
	params.minThreshold = 40;
	params.maxThreshold = 60;
	params.thresholdStep = 5;
	params.minDistBetweenBlobs = 10000;
	params.minArea = 100;
	//params.minConvexity = .4f;
	//params.minInertiaRatio = .1f;
	params.maxArea = 10000;
	//params.maxConvexity = 2;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	SimpleBlobDetector blobDetector( params );
	blobDetector.create("SimpleBlob");
	
	vector<KeyPoint> permKeyPoints;
	vector<KeyPoint> keyPoints;

	while(1) {
		// get new image over and over from webcam
		webCam >> camImage;

		// blur image
		//blur(camImage, blurImage, Size(11,11));
		blur(camImage, wb, Size(whiteb,whiteb));
		blur(camImage, pb, Size(pinkb,whiteb));
		blur(camImage, ab, Size(appleb,whiteb));
		
		// conver raw image to hsv
		cvtColor (camImage, hsvImage, CV_RGB2HSV);

		// blur images
		blur(hsvImage, whsv, Size(whiteb,whiteb));
		blur(hsvImage, phsv, Size(pinkb,pinkb));
		blur(hsvImage, ahsv, Size(appleb,appleb));

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
		threshold (whue,whl,whitehl,255, CV_THRESH_BINARY); // get lower bound
		threshold (whue,whh,whitehh,255, CV_THRESH_BINARY_INV); // get upper bound
		wh = whl &whh; // multiply 2 matrix to get the color range
		threshold (phue,phl,pinkhl,255, CV_THRESH_BINARY); // get lower bound
		threshold (phue,phh,pinkhh,255, CV_THRESH_BINARY_INV); // get upper bound
		ph = phl &phh; // multiply 2 matrix to get the color range
		threshold (ahue,ahl,applehl,255, CV_THRESH_BINARY); // get lower bound
		threshold (ahue,ahh,applehh,255, CV_THRESH_BINARY_INV); // get upper bound
		ah = ahl &ahh; // multiply 2 matrix to get the color range

		// apply thresshold for Sat channel
		threshold (wsat,wsl,whitesl,255, CV_THRESH_BINARY); // get lower bound
		threshold (wsat,wsh,whitesh,255, CV_THRESH_BINARY_INV); // get upper bound
		ws = wsl &wsh; // multiply 2 matrix to get the color range
		threshold (psat,psl,pinksl,255, CV_THRESH_BINARY); // get lower bound
		threshold (psat,psh,pinksh,255, CV_THRESH_BINARY_INV); // get upper bound
		ps = psl &psh; // multiply 2 matrix to get the color range
		threshold (asat,asl,applesl,255, CV_THRESH_BINARY); // get lower bound
		threshold (asat,ash,applesh,255, CV_THRESH_BINARY_INV); // get upper bound
		as = asl &ash; // multiply 2 matrix to get the color range

		// apply thresshold for Val channel
		threshold (wval,wvl,whitevl,255, CV_THRESH_BINARY); // get lower bound
		threshold (wval,wvh,whitevh,255, CV_THRESH_BINARY_INV); // get upper bound
		wv = wvl &wvh; // multiply 2 matrix to get the color range
		threshold (pval,pvl,pinkvl,255, CV_THRESH_BINARY); // get lower bound
		threshold (pval,pvh,pinkvh,255, CV_THRESH_BINARY_INV); // get upper bound
		pv = pvl &pvh; // multiply 2 matrix to get the color range
		threshold (aval,avl,applevl,255, CV_THRESH_BINARY); // get lower bound
		threshold (aval,avh,applevh,255, CV_THRESH_BINARY_INV); // get upper bound
		av = avl &avh; // multiply 2 matrix to get the color range

		// combine sat, val and hue filter together
		wHSV = wh & ws & wv;
		pHSV = ph & ps & pv;
		aHSV = ah & as & av;

		blobDetector.detect(wHSV, keyPoints);
		for(int i=0; i<keyPoints.size(); i++) {
			xpleorear = keyPoints[i].pt.x;
			ypleorear = 480 - keyPoints[i].pt.y;
			circle(camImage, keyPoints[i].pt, 20, cvScalar(255,255,255), 10);
			permKeyPoints.push_back(keyPoints[i]);
		}

		blobDetector.detect(pHSV, keyPoints);
		for(int i=0; i<keyPoints.size(); i++) {
			xpleofront = keyPoints[i].pt.x;
			ypleofront = 480 - keyPoints[i].pt.y;
			circle(camImage, keyPoints[i].pt, 20, cvScalar(255,255,255), 10);
			permKeyPoints.push_back(keyPoints[i]);
		}

		blobDetector.detect(aHSV, keyPoints);
		for(int i=0; i<keyPoints.size(); i++) {
			xfruit = keyPoints[i].pt.x;
			yfruit = 480 - keyPoints[i].pt.y;
			circle(camImage, keyPoints[i].pt, 20, cvScalar(255,255,255), 10);
			permKeyPoints.push_back(keyPoints[i]);
		}
		
		imshow("Webcam Orignal", camImage);
		imshow("HSV",wHSV);
		imshow("HSV2",pHSV);
		imshow("HSV3",aHSV);
		cvWaitKey(5);

		float orient = calcOrient(xpleofront, ypleofront, xpleorear, ypleorear);

		float pleo[3] = {(xpleofront + xpleorear) / 2, (ypleofront + ypleorear) / 2, orient};
		float fruit[2] = {xfruit, yfruit};

		getAction(pleo, fruit);
		//printf("%f", temp);
		//system("pause");
	}

	//printf("%f", temp);
	//system("pause");
}

float calcOrient(float xpf, float ypf, float xpr, float ypr) {
	if (ypf > ypr)
		return atan2(ypf-ypr,xpf-xpr) * 180 / M_PI;
	else
		return atan2(ypf-ypr,xpf-xpr) * 180 / M_PI + 360;
}