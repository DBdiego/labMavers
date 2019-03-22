/*
 * Distance.c
 *
 *  Created on: Mar 21, 2019
 *      Author: Gervase
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
//#include <pprz algebra int.h>

double main()
{
   int x_goal, y_goal, x_frame, y_frame, x_center, y_center, img_height, img_width, h_fov, v_fov;
   double altitude, tan_gamma, distance, dist_to_frame, required_rotation, p;

   printf("Enter x_goal, y_goal:\n"); // Just for the purpose of testing
   scanf("%d%d", &x_goal, &y_goal);  // Just for the purpose of testing

   altitude      = 1;
   img_height    = 260;
   img_width     = 512;
   h_fov         = 120;
   v_fov         = 40;
   tan_gamma     = tan((M_PI/2) - v_fov*(M_PI/180));
   dist_to_frame = altitude * tan_gamma;

   x_frame  = img_width  - x_goal;
   x_center = x_goal - (img_width/2);
   printf("x_center = %d\n", x_center);
   _Bool lost = 0;

   if(lost == 0) {
   required_rotation = atan((x_center * tan((h_fov/2)*(M_PI/180)))/(img_width/2));
   y_frame  = img_height - y_goal;
   y_center = y_goal - (img_height/2);
   p = y_frame/y_center * altitude * tan_gamma;
   distance = (dist_to_frame + p)/cos(required_rotation);

   }
   else {
   distance = 0;
   required_rotation = 0;

   }

   printf("required_rotation = %f\n", required_rotation);

   return 0;
}

