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

#ifndef MAX_NUM_AREAS
#define MAX_NUM_AREAS 5
#endif

#ifndef MIN_GREEN_PIX_AREA
#define MIN_GREEN_PIX_AREA 100
#endif

#ifndef NUM_PIX_ABOVE
#define NUM_PIX_ABOVE 3
#endif

#ifndef MAX_DIST_CENT
#define MAX_DIST_CENT 0.3  //ration of img_height representing the maximum distance the target destination can be from its centroid
#endif



// Orange filter Settings
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


uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

static int alf = 0;

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
  uint16_t goalx;
  uint16_t goaly;
};
struct color_object_t global_filters[2];
uint16_t goals[2];

// Function
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint16_t goals[]);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
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

  // Filter and find centroid
  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, goals);
  alf++;


  VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].updated = true;
  global_filters[filter-1].goalx = goals[0];
  global_filters[filter-1].goaly = goals[1];

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
int16_t img_size[2];
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint16_t goals[])
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

  // NEWLY DEFINED :)
  uint16_t area_width = (img->h)/(float)MAX_NUM_AREAS;
  uint32_t area_count;
  uint32_t Ax;
  uint32_t Ay;
  uint16_t centroid_coords[MAX_NUM_AREAS][2];
  uint8_t max_score_location = 0;
  uint16_t x_goal = 0;
  uint16_t y_goal = 0;
  int binary_img[img->h][img->w];
  img_size[0] = img->w;
  img_size[1] = img->h;
  int NW = 1;

  float percentages[MAX_NUM_AREAS];
  float current_score;
  float max_score = 0.0;

  for (int i = 0; i < img->h; i++){
	for (int j = 0; j < img->w; j++){
		binary_img[i][j] = 0;
	}
  }


  // Go through all sub-areas
  for (uint16_t area_ind=0; area_ind < MAX_NUM_AREAS; area_ind++){

	  area_count = 0;
	  Ax = 0;
	  Ay = 0;
	  current_score = 0.0;

	  // Go through all the y-pixels of sub-area
	  for (uint16_t y = (area_ind * area_width); y < ((area_ind+1) * area_width); y++) {
		  // Go through all the x-pixels of sub-area
		  for (uint16_t x = 0; x < img->w; x++) {

			  // Converting to YUV color scheme
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
			  };


			  // COLOR FILTER - Check if the color is inside the specified values
			  if ( (*yp >= lum_min) && (*yp <= lum_max) &&
				   (*up >= cb_min ) && (*up <= cb_max ) &&
				   (*vp >= cr_min ) && (*vp <= cr_max )) {

				  binary_img[y][x] = NW;
				  cnt ++;
				  tot_x += x;
				  tot_y += y;

				  area_count ++;

				  // x_bar = sum(A*x)/sum(A) --> Ax_tot / count
				  Ax += x ;
				  Ay += y ;


				  if (draw){
					  *yp = 255;  // make pixel brighter in image
				  };

			  };
		  };
	  };


      // Centroid of sub-area (if enough green detected)
      if (area_count > MIN_GREEN_PIX_AREA){
    	  centroid_coords[area_ind][0] = Ax/(1.0 * area_count) ;
    	  centroid_coords[area_ind][1] = Ay/(1.0 * area_count) ;
    	  percentages[area_ind] = (1.0 * area_count)/(area_width * img->w);
      } else{
    	  centroid_coords[area_ind][0] = 0;
    	  centroid_coords[area_ind][1] = 0;
    	  percentages[area_ind] = 0.0;
      };

      current_score = percentages[area_ind] * (centroid_coords[area_ind][0]);

      // Finding highest score of all areas
      if (current_score > max_score){
    	  max_score = current_score;
    	  max_score_location = area_ind;
      };


      if (alf == 1){
    		for (int i = 0; i < img->h; i++){
    			for (int j = 0; j < img->w; j++){
    				printf("%d ",binary_img[i][j]);
    			}
    	    printf("\n");
    	    }
      };

  };


  // --------- TARGET COORDINATES ---------

  // Selection of suggested destination x-coordinate (on image)
  uint8_t walker_sum;
  uint16_t running_best;

  x_goal = centroid_coords[max_score_location][0]; //(temporary value)
  for (uint16_t idx = centroid_coords[max_score_location][0]; idx < img->w; idx+=NUM_PIX_ABOVE){
	  walker_sum = 0;
	  for (int id_walker = idx+1; id_walker<(idx+1+NUM_PIX_ABOVE);id_walker++){
		  walker_sum += binary_img[y_goal][id_walker];
		  running_best = id_walker;
	  };

	  if ((walker_sum == NUM_PIX_ABOVE) && ((running_best - centroid_coords[max_score_location][0]) < MAX_DIST_CENT * img->w)){
		  x_goal = running_best;
	  };
  };

  y_goal = centroid_coords[max_score_location][1];


  // !!!! KEEP IN MIND, THE IMAGE IS STILL ROTATED !AT THIS POINT !!!!
  // thus x_goal is actually y_goal and vice-versa
  goals[0] = y_goal;
  goals[1] = x_goal;

  /*
  if (alf%40 == 0){
	  printf("%d, %d\n", y_goal, x_goal);
	  printf("%f, %f\n", y_goal/(520*1.0), x_goal/(240*1.0));
  }
  */



  // OLD CENTROID CALC
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
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        0, 0, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }

  if(local_filters[1].updated){
	  AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
			  local_filters[1].goalx, local_filters[1].goaly, local_filters[1].color_count, 1);

	  local_filters[1].updated = false;
  }
}


/*
 cod_lum_min2
 cod_lum_max2
 cod_cb_min2
 cod_cb_max2
 cod_cr_min2
 cod_cr_max2
 */










