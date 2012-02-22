#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
using namespace cv;

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