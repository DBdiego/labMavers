/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>

#define NAV_C // needed to get the nav funcitons like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif


#ifndef H_FOV
#define H_FOV 110
#endif

#ifndef V_FOV
#define V_FOV 50
#endif


uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t increase_nav_heading(float incrementDegrees);
uint8_t chooseRandomIncrementAvoidance(void);
void distance_func(uint16_t goals[], double dist_func[]);
float dist_to_wp(uint8_t waypoint);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA,
  CENTER_TARGET,
  UPDATE_WP
};

// define settings
float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 0.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
int confidence = 1;

static int conf_goals[3][2];            // this is a static array of goal coordinates
static int loop_counter = 0;
int conf_index;

int16_t target_img_coors[2];
//double dist_func[2];

struct img_struct  {
    int16_t w;
    int16_t h;
};

struct img_struct img;


float target_yaw;
static int rotating;



const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free



// Getting info from cv_detect_color_object.c
#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t x_goal, int16_t y_goal,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
  target_img_coors[0] = x_goal;
  target_img_coors[1] = y_goal;
};




// ============================= GROUP FUNCTIONS =================================
/* Function that returns the distance and required rotation when given a goal (x-,y- coordinate)
 *
 */
void distance_func(uint16_t goals[2], double dist_func[])
{
  float x_frame, y_frame, x_center, y_center; //img_height, img_width, h_fov, v_fov;
  double altitude, tan_gamma, distance, dist_to_frame, required_rotation, p, pitch;

  altitude  = stateGetPositionEnu_f()->z;      // This will be taken from the states (z-coordinate)
  pitch     = stateGetNedToBodyEulers_f()->theta;
  //img_height    = img.h;    // Will most likely be defined (or simply read from the image) elsewhere
  //img_width     = img.w;    //  "
  //h_fov         = 120;    // Need to be measured
  //v_fov         = 40;     //  "

  //printf("pitch bitch: %f \n", pitch);

  tan_gamma     = tan((M_PI/2) - 0.5*V_FOV*(M_PI/180)- pitch);

  dist_to_frame = altitude * tan_gamma;

  x_frame  = img.w  - goals[0];
  x_center = goals[0] - ((float)img.w/2);


  //_Bool lost = 0;

  //if(lost == 0) {
  	//printf("x-frame, x-center: %d, %d \n", x_center, x_frame);
    required_rotation = atan((x_center * tan((H_FOV/2.0)*(M_PI/180)))/(img.w/2));
    y_frame  = (float)goals[1];
    y_center = ((float)img.h/2) - (float)goals[1];
    p = (y_frame/y_center)* altitude * tan_gamma;
    distance = (dist_to_frame + p)/cosf(required_rotation);
    dist_func[0] = required_rotation;
    dist_func[1] = distance;
  /*}
  else {
    required_rotation = 0;
    distance = 0;
    dist_func[0] = required_rotation;
    dist_func[1] = distance;
  }
  */
}
/* This function simply calculates the distance between two points.
 *
 */
float pythagoras(int point1[], int point2[]){
	float distance;
	distance = sqrt((point1[0]-point2[0])*(point1[0]-point2[0]) + (point1[1]-point2[1])*(point1[1]-point2[1]));
	return distance;
}


/* This function compares three adjacent target coordinates and outputs a confidence boolean
 * ie. it checks if a target coordinate is noise or not.
 */
int confidence_func(int conf_goals[3][2], int confidence, uint16_t final_target[]){
	int limit, point1[2], point2[2], point3[2];
	float distance_12, distance_13, distance_23;

	limit = 100;

	int x_values[3], y_values[3]; // Defining x_values- and y_value-arrays

	//This for loop places all x-coordinates in one array and y-coordinates in another
	for (int i = 0; i < 3; i++){
		x_values[i] = conf_goals[i][0];
		y_values[i] = conf_goals[i][1];
	}

	point1[0] = x_values[0];
	point1[1] = y_values[0];

	point2[0] = x_values[1];
	point2[1] = y_values[1];

	point3[0] = x_values[2];
	point3[1] = y_values[2];

	distance_12 = pythagoras(point1, point2);

	/* This 'if' statement compares the locations of the 3 sets of goal coordinates
	 * and gives a confidence and selects a point.
	 */
	if (distance_12 < limit){
		confidence = 1;
		final_target[0]    = x_values[0];
		final_target[1]    = y_values[0];
	} else{
		distance_13 = pythagoras(point1, point3);
		distance_23 = pythagoras(point2, point3);
		if (distance_13 > limit && distance_23 < limit){
			confidence = 1;
			final_target[0] = x_values[1];
			final_target[1] = y_values[1];
		}else if (distance_13 < limit && distance_23 > limit){
			confidence = 1;
			final_target[0] = x_values[0];
			final_target[1] = y_values[0];
		}else if (distance_13 > limit && distance_23 > limit){
			confidence = 0;
		}
	}
	confidence = 1;
	return confidence;
}
// ============================================= END GROUP FUNCTIONS ==============================================











/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}




/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  /*
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }
  */
  if (!autopilot_in_flight()){
    return;
  }


  img.w = front_camera.output_size.h;
  img.h = front_camera.output_size.w;

  //______________________Added this______________________
  int final_target[2];
  int confidence = 0;
  double dist_func[2];

  conf_index = loop_counter%3;
  conf_goals[conf_index][0] = target_img_coors[0];
  conf_goals[conf_index][1] = target_img_coors[1];

  confidence = confidence_func(conf_goals, confidence, final_target);
  /*
  printf("conf_index: %d", conf_index);
  printf("[%d, %d]\n", conf_goals[0][0], conf_goals[0][1]);
  printf("[%d, %d]\n", conf_goals[1][0], conf_goals[1][1]);
  printf("[%d, %d]\n", conf_goals[2][0], conf_goals[2][1]);
  printf("== [%d, %d]\n", final_target[0],final_target[1]);
  */


//  printf("->%d\n", confidence);
  if (confidence){
	  distance_func(final_target, dist_func);
  }
  //printf("rotation, distance: [%f, %f]\n", dist_func[0],dist_func[1]);
  //_______________________________________________________

  loop_counter += 1;

  // compute current color thresholds
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  // -------------------- STATES -----------------------
  float current_heading = stateGetNedToBodyEulers_f()->psi;
  float yaw_diff = target_yaw-current_heading;
  float center_heading_change;
  float margin = 0.5 * M_PI/180;
  float dist_new;
  FLOAT_ANGLE_NORMALIZE(current_heading);



  // ------------------------------- STATES -------------------------------
  switch (navigation_state){
    //1
    case SAFE:
      printf("SAFE FLIGHT: ");

      printf("dist_to_wp = %f\n", dist_to_wp(WP_GOAL));

      if (dist_to_wp(WP_GOAL) < 0.05f){
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      else if (confidence == 0)
      {
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      else {
        navigation_state = SAFE;
      };

      break;
    //2
    case SEARCH_FOR_SAFE_HEADING:
      printf("SEARCH_HEAD: ");
      if (rotating == 0){
        chooseRandomIncrementAvoidance();
        FLOAT_ANGLE_NORMALIZE(current_heading);
        target_yaw = current_heading + 0.5;
        FLOAT_ANGLE_NORMALIZE(target_yaw);

        increase_nav_heading(oag_heading_rate);
        yaw_diff = target_yaw-current_heading;
        rotating = 1;

      }else{
    	  yaw_diff = fmod((target_yaw-current_heading + 2*3.141592), 2*3.141592);
    	  printf("target, current, diff: %f, %f, %f \n",DegOfRad(target_yaw), DegOfRad(current_heading), DegOfRad(yaw_diff));
		  float y_func = 0.0077 * (yaw_diff*yaw_diff) + 0.8462 * sqrtf(yaw_diff*yaw_diff);

		  if (abs(DegOfRad(yaw_diff)) < 25){
			  increase_nav_heading(oag_heading_rate * y_func * sqrtf(yaw_diff*yaw_diff)/yaw_diff);

		  } else {
			  increase_nav_heading(oag_heading_rate * sqrtf(yaw_diff*yaw_diff)/yaw_diff);
		  };
      }


      if (yaw_diff > -margin && yaw_diff < margin){
        rotating = 0;

        printf("Target REACHED \n");


        // Check if enough pixels in FOV
        if (floor_count < floor_count_threshold) {
          printf("Too Little Green \n");
          float heading_inc;
//          if (avoidance_heading_direction >= 0){
//            heading_inc = 2.09;
//          } else {
//            heading_inc = -2.09;
//          };
          heading_inc = 0.6;

          //increase_nav_heading(heading_inc);
            FLOAT_ANGLE_NORMALIZE(current_heading);
            target_yaw = current_heading + heading_inc;
            FLOAT_ANGLE_NORMALIZE(target_yaw);
            yaw_diff = target_yaw-current_heading;
            increase_nav_heading(heading_inc);
            target_yaw =  current_heading + heading_inc;
            rotating = 1;


        }else{

            if (confidence == 1){
              navigation_state = CENTER_TARGET;
            } else {
              navigation_state = SEARCH_FOR_SAFE_HEADING;
            }
        }


      }

    break;
    case CENTER_TARGET:
      printf("CENTER_TARGET: ");

      if (rotating == 0){
    	  center_heading_change = dist_func[0];
    	  FLOAT_ANGLE_NORMALIZE(center_heading_change);
    	  FLOAT_ANGLE_NORMALIZE(current_heading);
		  target_yaw = current_heading + center_heading_change;
		  FLOAT_ANGLE_NORMALIZE(target_yaw);

		  increase_nav_heading(oag_heading_rate);
		  yaw_diff = target_yaw-current_heading;
		  rotating = 1;

      }else{
		  FLOAT_ANGLE_NORMALIZE(current_heading);
		  FLOAT_ANGLE_NORMALIZE(target_yaw);
    	  yaw_diff = target_yaw-current_heading;
    	  FLOAT_ANGLE_NORMALIZE(yaw_diff);
    	  printf("target, current, diff: %f, %f, %f \n",DegOfRad(target_yaw), DegOfRad(current_heading), DegOfRad(yaw_diff));


		  float y_func = 0.0077 * (yaw_diff*yaw_diff) + 0.8462 * sqrtf(yaw_diff*yaw_diff);
		  if (abs(DegOfRad(yaw_diff)) < 25){
			  increase_nav_heading(oag_heading_rate * y_func * sqrtf(yaw_diff*yaw_diff)/yaw_diff);
		  } else {
			  increase_nav_heading(oag_heading_rate * sqrtf(yaw_diff*yaw_diff)/yaw_diff);
		  }

      }

      if (yaw_diff > -margin && yaw_diff < margin){
    	  rotating = 0;
    	  printf("Target REACHED\n");
    	  navigation_state = UPDATE_WP;
		}

	  break;


    //4
    case UPDATE_WP:
      printf("UPDATE_WP: ");
      printf(" Distance: %f", dist_func[1]);
      // stop
      if (dist_func[1] < 1) {
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      } else {
    	printf("Updating WP_TRAJECTORY\n");
        moveWaypointForward(WP_TRAJECTORY, dist_func[1]);
        dist_new = dist_func[1];
        while (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY)) && dist_new > 1){
          dist_new -= 0.3;
          moveWaypointForward(WP_TRAJECTORY, dist_new);
        }
        if (dist_new < 1) {
            navigation_state = SEARCH_FOR_SAFE_HEADING;
        } else {
          printf("GOAL FOUND\n");
          moveWaypointForward(WP_GOAL, (dist_new-0.3));
          navigation_state = SAFE;
        }
      }
      break;
    default:
      break;
  }
  return;
}



/*
 * RANDOM NUMBER INCREMENT
 */
uint8_t chooseRandomIncrementAvoidance()
{
  int numdeg = 30;
  float numrad = numdeg*(3.14/180.f);
  avoidance_heading_direction = numrad;

  return false;
};





/*
 * CHANGES HEADING
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementRad)
{
/*
  if (incrementRad > M_PI){
    incrementRad = 2*M_PI - incrementRad;
  }else if (incrementRad < -M_PI){
    incrementRad = 2*M_PI + incrementRad;
  }
  */

  float new_heading = stateGetNedToBodyEulers_f()->psi + incrementRad;//RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  //VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}








































/*
 * USED IN moveWaypointForward
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * USED IN moveWaypointForward
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}


/*
 * Calculates distance of vector to waypoint
 */
float dist_to_wp(uint8_t waypoint)
{
  float diff_x = (waypoint_get_x(waypoint) - stateGetPositionEnu_f()->x);
  float diff_y = (waypoint_get_y(waypoint) - stateGetPositionEnu_f()->y);
  float diff_T = sqrtf(diff_x*diff_x + diff_y*diff_y);
  return diff_T;
}





