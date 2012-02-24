#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
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

	float* detect(bool ffruit);

private:
	float calcOrient(float,float,float,float);
};