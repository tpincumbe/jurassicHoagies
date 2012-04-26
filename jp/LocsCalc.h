#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "RoutePlanner.h"
#include "Search.h"
#include <vector>
using namespace cv;

#define WHITEHL 0
#define WHITEHH 100
#define WHITESL 0
#define WHITESH 40
#define WHITEVL 200
#define WHITEVH 255
#define WHITEB 15

#define PINKHL 110
#define PINKHH 150
#define PINKSL -1
#define PINKSH 100
#define PINKVL 140
#define PINKVH 255
#define PINKB 9

#define APPLEHL 100
#define APPLEHH 180
#define APPLESL 100
#define APPLESH 255
#define APPLEVL 130
#define APPLEVH 238
#define APPLEB 12

class LocsCalc
{
public:
	float xpleofront;
	float ypleofront;
	float xpleorear;
	float ypleorear;
	float xfruit;
	float yfruit;
	float xrov, yrov;
	
	void grabBackground();
	void grabObstacles();
	vector<vector<int>> getMap();
	void detect(int project, SystemQueue *msq);
	LocsCalc();
	~LocsCalc();

	void resetPath();

private:
	float calcOrient(float,float,float,float);
	void showImages();

	RoutePlanner rp;
	Search search;

	// camera
	VideoCapture webCam;

	// blob detector
	SimpleBlobDetector::Params params;
	SimpleBlobDetector blobDetector;
	SimpleBlobDetector rovBD;
	
	SimpleBlobDetector::Params params2;
	SimpleBlobDetector blobDetector2;

	// matrix that holds seperate color channels
	vector<Mat> slices;

	// raw image from webcam
	Mat camImage;

	// background subtraction matrices
	Mat background;		// background image
	Mat obstacles;		// obstacles
	Mat rov;
	Mat fullBackground;	// background and obstacles
	Mat noBack;			// image - background and obstacles
	vector<vector<int>> obstacleGrid;	// indicates location of obstacles
	
	// matrices for blob detection
	Mat hsvImage;		// hsv image
	Mat whsv;			// hsv image
	Mat phsv;			// hsv image
	Mat ahsv;			// hsv image
	Mat whue;			// hue channel
	Mat phue;			// hue channel
	Mat ahue;			// hue channel
	Mat wsat;			// Sat channel
	Mat psat;			// Sat channel
	Mat asat;			// Sat channel
	Mat wval;			// Val channel
	Mat pval;			// Val channel
	Mat aval;			// Val channel
	Mat lb;				// h/s/v lower bound
	Mat hb;				// h/s/v upper bound

	vector<KeyPoint> permKeyPoints;
	vector<KeyPoint> keyPoints;
};