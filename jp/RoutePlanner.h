#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdio.h>
#include "Subsystem.h"

#define ANGLE_ERROR 2.5f
#define TURN_THRESHOLD 40.00f
#define REACH_ERROR 15.0f

#define STOP 0
#define WALK_FORWARD 1
#define VEER_LEFT 2
#define VEER_RIGHT 3
#define SHARP_TURN_LEFT 4
#define SHARP_TURN_RIGHT 5


class RoutePlanner : public Subsystem
{
public:
<<<<<<< HEAD
	void Tick(size_t tick);
	void Execute(string behavior, string argument);
	void Shutdown();
	string MonitorMessage();
=======
>>>>>>> 5c72f1734b64dd740cfa9fd0328f64c84150df91
	void performAction(float* p, float* t);
private:
	int getAction(float[], float[]);
};