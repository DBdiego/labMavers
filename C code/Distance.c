/*
 * Distance.c
 *
 *  Created on: Mar 21, 2019
 *      Author: Gervase
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/* Function that returns the distance and required rotation when given a goal (x-,y- coordinate)
 *
 */
double distance_func(int x_goal, int y_goal, double dist_func[])
{
	int x_frame, y_frame, x_center, y_center, img_height, img_width, h_fov, v_fov;
	double altitude, tan_gamma, distance, dist_to_frame, required_rotation, p;
	altitude      = 1;      // This will be taken from the states (z-coordinate)
	img_height    = 260;    // Will most likely be defined (or simply read from the image) elsewhere
	img_width     = 512;    //  "
	h_fov         = 120;    // Need to be measured
	v_fov         = 40;     //  "

    tan_gamma     = tan((M_PI/2) - v_fov*(M_PI/180));
	dist_to_frame = altitude * tan_gamma;

	x_frame  = img_width  - x_goal;
	x_center = x_goal - (img_width/2);

	_Bool lost = 0;

	if(lost == 0) {
		required_rotation = atan((x_center * tan((h_fov/2)*(M_PI/180)))/(img_width/2));
		y_frame  = img_height - y_goal;
		y_center = y_goal - (img_height/2);
		p = y_frame/y_center * altitude * tan_gamma;
		distance = (dist_to_frame + p)/cos(required_rotation);
		dist_func[0] = required_rotation;
		dist_func[1] = distance;
	}
	else {
		required_rotation = 0;
		distance = 0;
		dist_func[0] = required_rotation;
		dist_func[1] = distance;
	}

}

double main()
{
	int x, y;
	double dist_func[2];

	printf("Enter x_goal, y_goal:\n"); // Just for the purpose of testing
	scanf("%d%d", &x, &y);             // Just for the purpose of testing

	distance_func(x, y, dist_func);

	printf("required_rotation = %f\n", dist_func[0]);
	printf("Distance = %f\n", dist_func[1]);

	return 0;
}

