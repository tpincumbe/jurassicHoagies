/*Route planner for Project 2.*/
#include "RoutePlanner.h"
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Eigen"

/*Takes in the current position of the pleo and the current position of the object
pleo_pos = {pleo_x, pleo_y, pleo_ori};
target_pos = {target_x, target_y};

returns a string
*/

using namespace Eigen;
using namespace std;

int getAction(float* pleo_pos, float* target_pos)
{
	float pleo_x, pleo_y, pleo_ori, target_x, target_y;
	float destination_vector[2];
	float destination_unit[2];
	float angle_diff;
	float destination_mag;
	float pleo_unit_x, pleo_unit_y;
	float target_angle;

	/*Populate variables before they disappear*/
	pleo_x = pleo_pos[0];
	pleo_y = pleo_pos[1];
	pleo_ori = pleo_pos[2];

	target_x = target_pos[0];
	target_y = target_pos[1];

	/*Calculate the unit vector for the pleo's direction*/
	pleo_unit_x = cos(pleo_ori*0.01745);
	pleo_unit_y = sin(pleo_ori*0.01745);
	
	cout << "pleo_ori: " << pleo_ori << endl;
	cout << "pleo_x: " << pleo_unit_x << endl;
	cout << "pleo_y: " << pleo_unit_y << endl;

	/*Calculate the destination vector (From pleo's position to the target*/
	destination_vector[0] = target_x - pleo_x;
	destination_vector[1] = target_y - pleo_y;
	
	/*Calculate the magnitude of the vector (i.e. the displacement needed to travel*/
	destination_mag = sqrt(pow(destination_vector[0], 2) + pow(destination_vector[1], 2));

	/*If the pleo has already reached the target, stop the robot*/
	if(destination_mag < REACH_ERROR)
    {
		printf("Reached Destination. Stopping Pleo\n");
		return STOP;
	}

	destination_unit[0] = destination_vector[0]/destination_mag;
	destination_unit[1] = destination_vector[1]/destination_mag;

	// cross and dot product of destination x pleo vectors
	Vector3d v(pleo_unit_x,pleo_unit_y,0);
	Vector3d w(destination_unit[0],destination_unit[1],0);

	Vector3d cp = w.cross(v);
	double dp = w.dot(v);
	int dir = 0;
	int mult = 1;
	string s;
	double angle = acos(dp)*180/3.14159;

	if (angle <= 2.5)
		mult = 0;	// go straight
	else if (angle >= 80)
		mult = 2;	// hard turn

	if (cp.z() > 0) {
		dir = 1*mult;	// right
		s = "right";
	}
	else {
		dir = -1*mult;	// left
		s = "left";
	}
	if (mult == 2)
		s += " sharply";

	cout << "Turn " << angle << " degrees " << s << " Distance Away: " << destination_mag << endl;

	/*Decide what to do next*/

	switch (dir) {
	case -2:
		cout << "Turn SHARPLY to the LEFT";
		return SHARP_TURN_LEFT;
	case -1:
		cout << "Turn to the LEFT";
		return SHARP_TURN_LEFT;
	case 0:
		cout << "Walk FORWARD";
		return SHARP_TURN_LEFT;
	case 1:
		cout << "Turn to the RIGHT";
		return SHARP_TURN_LEFT;
	case 2:
		cout << "Turn SHARPLY to the RIGHT";
		return SHARP_TURN_LEFT;
	}

}