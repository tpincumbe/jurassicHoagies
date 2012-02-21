/*Route planner for Project 2.*/
#include "RoutePlanner.h"


/*Takes in the current position of the pleo and the current position of the object
pleo_pos = {pleo_x, pleo_y, pleo_ori};
target_pos = {target_x, target_y};


returns a string
*/
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
	pleo_unit_x = acos(pleo_ori);
	pleo_unit_y = asin(pleo_ori);

	/*Calculate the destination vector (From pleo's position to the target*/
	destination_vector[0] = target_x - pleo_y;
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

	target_angle = acos(destination_unit[0]); /*or: asin(destination_unit[1]);*/
	
	/*Calculate the angle difference*/
	angle_diff = target_angle - pleo_ori;
	printf("Distance: %f, Turn Angle: %s\n", destination_mag, angle_diff);


	/*Decide what to do next*/

	/*If the angle difference is less than the angle threshold*/
	if(angle_diff < (-1.0 * ANGLE_ERROR))
	{
		if(abs(angle_diff) > TURN_THRESHOLD)
        {
			/*Sharp Turn Right (possibly in place)*/
			printf("TURN sharply to the RIGHT\n");
			return SHARP_TURN_RIGHT;
		}
		
		/*Veer Right*/
		printf("VEER to the RIGHT\n");
		return VEER_RIGHT;
        
	} else if(angle_diff > ANGLE_ERROR) /*If the angle difference is more than the angle threshold*/
        {
			if(abs(angle_diff) > TURN_THRESHOLD)
				{
					/*Sharp Turn Left (possibly in place)*/
					printf("TURN sharply to the LEFT\n");
					return SHARP_TURN_LEFT;
                }
                /*Veer Left*/
				printf("VEER to the LEFT\n");
				return VEER_LEFT;
	} else { /*If the difference is within the angle error, keep true*/
		/*Walk forward*/
		printf("Walk FORWARD\n");
		return WALK_FORWARD;
	}

}