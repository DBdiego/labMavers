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
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif


#ifndef H_FOV
#define H_FOV 120
#endif

#ifndef V_FOV
#define V_FOV 40
#endif

uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
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

// Added this
static int conf_goals[3][2];            // this is a static array of goal coordinates
static int loop_counter = 0;
int conf_index;

int16_t target_img_coors[2];
double dist_func[2];

struct img_struct  {
		int16_t w;
		int16_t h;
};

struct img_struct img;



const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free
/*
// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;

static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

*/



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
void distance_func(uint16_t goals[], double dist_func[])
{
	int x_frame, y_frame, x_center, y_center; //img_height, img_width, h_fov, v_fov;
	double altitude, tan_gamma, distance, dist_to_frame, required_rotation, p, pitch;

	altitude  = stateGetPositionEnu_f()->z;      // This will be taken from the states (z-coordinate)
	pitch     = stateGetNedToBodyEulers_f()->theta;
	//img_height    = img.h;    // Will most likely be defined (or simply read from the image) elsewhere
	//img_width     = img.w;    //  "
	//h_fov         = 120;    // Need to be measured
	//v_fov         = 40;     //  "

	printf("pitch bitch: %f \n", pitch);

    tan_gamma     = tan((M_PI/2) - V_FOV*(M_PI/180)- pitch);
	dist_to_frame = altitude * tan_gamma;

	x_frame  = img.w  - goals[0];
	x_center = goals[0] - (img.w/2);

	_Bool lost = 0;

	if(lost == 0) {
		required_rotation = atan((x_center * tan((H_FOV/2)*(M_PI/180)))/(img.w/2));
		y_frame  = goals[1];
		y_center = (img.h/2) - goals[1];
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
int confidence_func(int conf_goals[3][2], int confidence, int final_target[]){
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
		printf("Confident 1\n");
	} else{
		distance_13 = pythagoras(point1, point3);
		distance_23 = pythagoras(point2, point3);
		if (distance_13 > limit && distance_23 < limit){
			confidence = 1;
			final_target[0] = x_values[1];
			final_target[1] = y_values[1];
			printf("Confident 2\n");
		}else if (distance_13 < limit && distance_23 > limit){
			confidence = 1;
			final_target[0] = x_values[0];
			final_target[1] = y_values[0];
			printf("Confident 3\n");
		}else if (distance_13 > limit && distance_23 > limit){
			confidence = 0;
			printf("Not confident \n");
		}
	}
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
  //AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}




/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }


  img.w = front_camera.output_size.h;
  img.h = front_camera.output_size.w;




  //______________________Added this______________________
  int final_target[2];
  int confidence = 0;
  conf_index = loop_counter%3;
  conf_goals[conf_index][0] = target_img_coors[0];
  conf_goals[conf_index][1] = target_img_coors[1];

  confidence = confidence_func(conf_goals, confidence, final_target);
/*
  printf("%d\n", conf_index);
  printf("[%d, %d]\n", conf_goals[0][0], conf_goals[0][1]);
  printf("[%d, %d]\n", conf_goals[1][0], conf_goals[1][1]);
  printf("[%d, %d]\n", conf_goals[2][0], conf_goals[2][1]);
  printf("== [%d, %d]\n", final_target[0],final_target[1]);
*/

  printf("->%d\n", confidence);
  if (confidence){
	  distance_func(final_target, dist_func);
  }
  printf("== [%f, %f]\n", dist_func[0],dist_func[1]);
  //_______________________________________________________




  loop_counter ++;
  // compute current color thresholds
  int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  VERBOSE_PRINT("target coors image: [%d,%d] \n required rotation: %f\n distance: %fm\n\n", target_img_coors[0], target_img_coors[1], dist_func[0], dist_func[1]);
  //VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  //VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  //VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);


  // update our safe confidence using color threshold
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);

  //test_func(stateGetPositionEnu_f()->z);

  /*
  filter_green(struct image_t *img,
		  	   bool draw,
			   uint8_t lum_min,
			   uint8_t lum_max,
			   uint8_t cb_min,
			   uint8_t cb_max,
			   uint8_t cr_min,
			   uint8_t cr_max,
			   uint8_t img_filt[img->w][img->h]);
  */

  // -------------------- STATES -----------------------
  switch (navigation_state){
    //1
    case SAFE:
      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        guidance_h_set_guided_body_vel(speed_sp, 0);
      }

      break;

    //2
    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_guided_body_vel(0, 0);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;

    //3
    case SEARCH_FOR_SAFE_HEADING:
      guidance_h_set_guided_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2){
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = SAFE;
      }
      break;

    //4
    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_guided_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_guided_heading_rate(avoidance_heading_direction * RadOfDeg(15));

      navigation_state = REENTER_ARENA;

      break;

    //5
    case REENTER_ARENA:
      // force floor center to opposite side of turn to head back into arena
      if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SAFE;
      }
      break;
    default:
      break;
  }
  return;
}



/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else {
    avoidance_heading_direction = -1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  return false;
};




















