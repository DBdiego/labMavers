/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

// ------------------------- DEFINES ----------------------------------------

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif


#ifndef max_num_areas
#define max_num_areas 5
#endif

#ifndef num_pixels_above
#define num_pixels_above 3
#endif

#ifndef max_dist_cent
#define max_dist_cent 0.3  //ration of img_height representing the maximum distance the target destination can be from its centroid
#endif


// ------------------------- VARIABLE DECLARATION/INITIALISATION ---------------------------------

// Filter Settings
// Orange filter:
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;
// Green filter:
uint8_t cod_lum_min11 = 0;
uint8_t cod_lum_max11 = 0;
uint8_t cod_cb_min11 = 0;
uint8_t cod_cb_max11 = 0;
uint8_t cod_cr_min11 = 0;
uint8_t cod_cr_max11 = 0;

// Should be the same as Green
// for GUIDED mode
uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;
static int alf = 0;
// ------------------------- STRUCTURES ------------------------------------------
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
  int goal_x; // You can add shit here that you want to send to the guidance function, such as goal in x...
  int goal_y; // ...and y
};
struct color_object_t global_filters[2];

// ------------------------- FUNCTION DECLARATION ---------------------------------

uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max);
void filter_green(struct image_t *img, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint8_t img_filt[img->h][img->w]);//), uint16_t *max_y, uint16_t *max_x);
void determine_target(int size_x, int size_y, int binary_image[size_y][size_x], int goals[], _Bool showoff);
void array_printer(int size_x, int size_y, int input_array[size_y][size_x]);

/*
 * Object detector:
 * Main function that calculates the colour count of orange, detects green floor and determines centroids
 * Local variables are copied to color_object_t structure to give to color_object_detector_periodic()
 * @param img - input image to process
 * @param filter - which detection filter to process (just keep this as is)...
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
	uint8_t lum_min, lum_max;
	uint8_t cb_min, cb_max;
	uint8_t cr_min, cr_max;
	uint8_t lum_min11, lum_max11;
	uint8_t cb_min11, cb_max11;
	uint8_t cr_min11, cr_max11;
	bool draw;

	switch (filter){
	case 1:
	  lum_min = cod_lum_min1;
	  lum_max = cod_lum_max1;
	  cb_min = cod_cb_min1;
	  cb_max = cod_cb_max1;
	  cr_min = cod_cr_min1;
	  cr_max = cod_cr_max1;

	  lum_min11 = cod_lum_min11;
	  lum_max11 = cod_lum_max11;
	  cb_min11 = cod_cb_min11;
	  cb_max11 = cod_cb_max11;
	  cr_min11 = cod_cr_min11;
	  cr_max11 = cod_cr_max11;
	  draw = cod_draw1;
	  break;
	case 2:
	  lum_min = cod_lum_min2;
	  lum_max = cod_lum_max2;
	  cb_min = cod_cb_min2;
	  cb_max = cod_cb_max2;
	  cr_min = cod_cr_min2;
	  cr_max = cod_cr_max2;
	  draw = cod_draw2;
	  break;
	default:
	  return img;
	};

	int32_t x_c, y_c;

	// Initialise local variables for filter_green(), determine_target()
	// Mind the types! (uint8_t cannot be used instead of int)!
	int size_x = img->w;
	int size_y = img->h;
	int binary_image[img->w][img->h];
	int goals[2];
	_Bool showoff = 0;

	//for (int i = 0; i < img->w; i++){
	//	for (int j = 0; j < img->h; j++){
	//		binary_image[i][j] = 0;
	//	}
	//}



	// Applying filter and centroid functions:
	filter_green(img, draw, lum_min11, lum_max11, cb_min11, cb_max11, cr_min11, cr_max11, &binary_image);

	if (alf == 20){
		for (int i = 0; i < img->w; i++){
			for (int j = 0; j < img->h; j++){
				printf("%d ",binary_image[i][j]);
			}
		printf("\n");
		}
	}
	alf++;
	determine_target(size_x, size_y, binary_image, goals, showoff);
	// Orange detector... (I'm just leaving this in for now)
	uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max);
	VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
	VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
		hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

	// Lock mutex and write to global_filters (color_object_t structure):
	pthread_mutex_lock(&mutex);
	global_filters[filter-1].color_count = count;
	global_filters[filter-1].x_c = x_c;
	global_filters[filter-1].y_c = y_c;
	global_filters[filter-1].updated = true;
	global_filters[filter-1].goal_x = goals[0];
	global_filters[filter-1].goal_y = goals[1];
	pthread_mutex_unlock(&mutex);

	return img;
}

struct image_t *object_detector1(struct image_t *img);
struct image_t *object_detector1(struct image_t *img)
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img);
struct image_t *object_detector2(struct image_t *img)
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
  cod_lum_min11 = COLOR_OBJECT_DETECTOR_LUM_MIN11;
  cod_lum_max11 = COLOR_OBJECT_DETECTOR_LUM_MAX11;
  cod_cb_min11 = COLOR_OBJECT_DETECTOR_CB_MIN11;
  cod_cb_max11 = COLOR_OBJECT_DETECTOR_CB_MAX11;
  cod_cr_min11 = COLOR_OBJECT_DETECTOR_CR_MIN11;
  cod_cr_max11 = COLOR_OBJECT_DETECTOR_CR_MAX11;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2);
#endif
}

/*
 * find_object_centroid
 * @param img - input image to process
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param draw - whether or not to draw on image
 * @return number of pixels found
 */
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt ++;
        tot_x += x;
        tot_y += y;
        //if (draw){
        //  *yp = 255;  // make pixel brighter in image
        //}
      }
    }
  }
  if (cnt > 0) {
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }
  return cnt;
}

void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  // Lock mutex and copy global_filters to local_filters:
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  // USE THE FIELDS FOR <pixel_width> AND <pixel_height> IN THE <VISUAL_DETECTION> MESSAGE TO COMMUNICATE THE GOALS:
  // Please look closely at:
  // orange_avoider.c line 70
  // conf/abi.xml line 185
  // var/include/abi_messages.h line 77
  // Note: actually, pixel_width as described in abi.xml is actually a int16_t, but I'm assigning it an int, which is not a problem
  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        local_filters[0].goal_x, local_filters[0].goal_y, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        0, 0, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }
}



/*
 * Determines the x and y targe given a binary 2D array which represents the filtered image
 * Note: This function assumes that the picture is right side up!
 * @param int size_x - width of binary_image
 * @param int size_y - height of binary_image
 * @param binary_image[][] - 2D array representing binary filtered image from camera
 * @param goals[] - Empty array to be filled by function
 * @param _Bool showoff - if 1, will print the binary filtered image array
 */
void determine_target(int size_x, int size_y, int binary_image[size_y][size_x], int goals[], _Bool showoff)
{
	// Define vars:
	int Ax_tot, Ay_tot, count;
	int centroid_coord[max_num_areas][2];
	float percentages[max_num_areas];
	float combined_scores[max_num_areas];

	int im_width  = size_x;
	int im_height = size_y;

	int max_idx   = (im_width)/max_num_areas;
	int total_pixels = im_width * im_height;
	int total_area_pixels = (total_pixels)/max_num_areas;
	float max_score = 0;
	int max_score_location = 0;


	if (showoff){
		printf("\n image specs: width=%d, height=%d , %d , %d, %d\n\n", im_width, im_height, max_idx, total_pixels, total_area_pixels);
	}


	// Going over multiple vertical areas in which frame is divided
	for (int area_ind = 0; area_ind < max_num_areas; ++area_ind){

		Ax_tot = 0;
		Ay_tot = 0;
		count  = 0;

		// Going through all x-indices of those areas
		for (int cent_idy = 0; cent_idy < im_height; ++cent_idy){

			// Going through all y-indices of those areas
			for (int cent_idx = (area_ind * max_idx); cent_idx < ((area_ind+1) * max_idx); ++cent_idx){

				// x_bar = sum(A*x)/sum(A) --> Ax_tot / count
				Ax_tot += (binary_image[cent_idy][cent_idx] * cent_idx);
				Ay_tot += (binary_image[cent_idy][cent_idx] * cent_idy);

				count  += binary_image[cent_idy][cent_idx];
			}
		}


		// Centroid location
		if (count > 0){
			centroid_coord[area_ind][0] = Ax_tot*1.0f/count;
			centroid_coord[area_ind][1] = Ay_tot*1.0f/count;
		}else{
			centroid_coord[area_ind][0] = 0;
			centroid_coord[area_ind][1] = im_height-1;
		}



		// Percentage of positive green in image
		percentages[area_ind] = (1.0*count)/total_area_pixels;


		// Computing combined score
		combined_scores[area_ind] = percentages[area_ind] * (im_height - centroid_coord[area_ind][1]);


		if (combined_scores[area_ind] > max_score){
			max_score = combined_scores[area_ind];
			max_score_location = area_ind;
		}

		if (showoff){
			//// SHOW OFF MODE /// (from python)
			// Changing the location of the centroid to a 5 (for feedback purposes only)
			binary_image[centroid_coord[area_ind][1]][centroid_coord[area_ind][0]] = 5;

			printf("Cent_xy_%d = [%d, %d]; %f - %f \n", area_ind, centroid_coord[area_ind][0],
					centroid_coord[area_ind][1], (percentages[area_ind]*100),combined_scores[area_ind]);
		}

	}


	if (showoff){
		printf("\n");
		printf("selection: area=%d, score=%f \n\n", max_score_location, max_score);
	}


	// Selection of suggested destination x-coordinate (on image)
	int x_goal = centroid_coord[max_score_location][0];

	// Selection of suggested destination y-coordinate (on image)
	int walker_sum;
	int running_best;
	int y_goal = 0;

	for (int idy = centroid_coord[max_score_location][1]; idy > num_pixels_above; idy-=num_pixels_above){
		walker_sum = 0;
		for (int id_walker = idy-1; id_walker>(idy-1-num_pixels_above);--id_walker){
			walker_sum += binary_image[id_walker][x_goal];
			running_best = id_walker;
		}

		if ((walker_sum == num_pixels_above) && ((centroid_coord[max_score_location][1]- running_best) < max_dist_cent*im_height)){
			y_goal = running_best;
		}
	}

	// Changing feedback variable wiht found target destionation coordinates (image frame coors)
	goals[0] = x_goal;
	goals[1] = y_goal;


	if (showoff){
		// /// SHOW OFF MODE /// (from python)
		// Changing the location of the destination to a 55 (for feedback purposes only)
		binary_image[y_goal][x_goal] = 55;

		// Printing resulting binary array
		array_printer(im_width, im_height, binary_image);
	}
}

// Function to print a 2D array
void array_printer(int size_x, int size_y, int input_array[size_y][size_x])
{
	printf("[%d, %d]\n", size_x, size_y);
	for (int idy = 0; idy < size_y; ++idy){
		for (int idx = 0; idx < size_x; ++idx){
			printf("%d ", input_array[idy][idx]);
		}
		printf("\n");
	}
}



/*
 * Filter image img and modify binary img_filt s.t. filtered pixels location will be = 1
 * Note: img_filt will be the transpose structure of img!!!
 * @param img - input image to process
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param draw - whether or not to draw on image
 * @param img_filt - 2D binary array of size of image pixels
 */
void filter_green(struct image_t *img, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
							  uint8_t img_filt[img->w][img->h])//, uint16_t *max_y, uint16_t *max_x)
{
  uint8_t *buffer = img->buf;

  // Go through all the pixels
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      // First changing from UYVY to YUV:
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }
      // Then checking filter:
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        if (img_filt[img->w - x][y] == 0){
        	img_filt[img->w - x][y] = 1;
        }
        *yp = 255;  // make pixel brighter in image

        //if (x > *max_x){
        //	//printf("%d\n", max_x);
        //	*max_y = y;
        //	*max_x = x;
        //}
      } else{
    	  img_filt[img->w - x][y] = 0;
      }
    }
  }
  //printf("%d, %d\n", max_x, max_y);
}




