#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "RoutePlanner.h"
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

	void detect();
	LocsCalc();
	~LocsCalc();

	void resetPath();

private:
	float calcOrient(float,float,float,float);
	void showImages();

	RoutePlanner rp;

	// slices matrcies that hold H,S and V
	vector<Mat> wslices;
	vector<Mat> pslices;
	vector<Mat> aslices;
	
	// create matrices to hold image
	Mat camImage;		// raw image from webcam
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

	vector<KeyPoint> permKeyPoints;
	vector<KeyPoint> keyPoints;
};