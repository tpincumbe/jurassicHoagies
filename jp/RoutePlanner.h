#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdio.h>
#include "Subsystem.h"

#define ANGLE_ERROR 2.5f
#define TURN_THRESHOLD 40.00f

#define STOP 0
#define WALK_FORWARD 1
#define VEER_LEFT 2
#define VEER_RIGHT 3
#define SHARP_TURN_LEFT 4
#define SHARP_TURN_RIGHT 5


class RoutePlanner : public Subsystem
{
public:
	float REACH_ERROR;
	void Tick(size_t tick);
	void Execute(string behavior, string argument);
	void Shutdown();
	string MonitorMessage();
	vector<string> performAction(float* p, float* t, float* thresh, int cpsLeft);
private:
	int getAction(float[], float[], float* thresh, int cpsLeft);
};