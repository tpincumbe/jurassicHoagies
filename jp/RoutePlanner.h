#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdio.h>

#define ANGLE_ERROR 2.5f
#define TURN_THRESHOLD 40.00f
#define REACH_ERROR 15.0f

#define STOP 0
#define WALK_FORWARD 1
#define VEER_LEFT 2
#define VEER_RIGHT 3
#define SHARP_TURN_LEFT 4
#define SHARP_TURN_RIGHT 5

int getAction(float[], float[]);