/*
 * Titties.c
 *
 *  Created on: Mar 21, 2019
 *      Author: parallels
 */


#include <stdio.h>

#ifndef max_num_areas
#define max_num_areas 5
#endif

#ifndef num_pixels_above
#define num_pixels_above 3
#endif

#ifndef max_dist_cent
#define max_dist_cent 0.3  //ration of img_height representing the maximum distance the target destination can be from its centroid
#endif


int binary_image[35][35] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
							{0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
							{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
							{0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
							{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}};

/*
int matrix[max_x][max_y];

for (int idx = 0; max_x; idx ++){
	for (int ydx = 0; max_y; ydx) {
		if (int ydx >= 0.5*max_y){
			matrix[idx][idy] = 1;
		} else{
			matrix[idx][idy] = 0;
		}
	}
}
*/





// Declaring variables
int Ax_tot, Ay_tot, count;
int centroid_coord[max_num_areas][2];
float percentages[max_num_areas];
float combined_scores[max_num_areas];



// Prototyping functions
void array_printer(int size_x, int size_y, int input_array[size_x][size_y]);


int main() {

	// Defining parameters regarding the image size
	int im_width  = sizeof(binary_image[0])/sizeof(binary_image[0][0]);
	int im_height = sizeof(binary_image)/sizeof(binary_image[0]);
	int max_idx   = (1.0*im_width)/max_num_areas;
	int total_pixels = im_width * im_height;
	int total_area_pixels = (1.0*total_pixels)/max_num_areas;
	float max_score = 0;
	int max_score_location = 0;



	printf("\nimage specs: width=%d, height=%d \n\n", im_width, im_height);



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
			};
		};


		// Centroid location
		centroid_coord[area_ind][0] = Ax_tot/count;
		centroid_coord[area_ind][1] = Ay_tot/count;

		// Percentage of positive green in image
		percentages[area_ind] = (1.0*count)/total_area_pixels;

		// Computing combined score
		combined_scores[area_ind] = percentages[area_ind] * (im_height - centroid_coord[area_ind][1]);

		if (combined_scores[area_ind] > max_score){
			max_score = combined_scores[area_ind];
			max_score_location = area_ind;
		};

		//// SHOW OFF MODE /// (from python)
		// Changing the location of the centroid to a 5 (for feedback purposes only)
		binary_image[centroid_coord[area_ind][1]][centroid_coord[area_ind][0]] = 5;

		printf("Cent_xy_%d = [%d, %d]; %f \% -> %f\n", area_ind, centroid_coord[area_ind][0],
				centroid_coord[area_ind][1], (percentages[area_ind]*100),combined_scores[area_ind]);
	};

	printf("\n");

	// Selection of suggested destination x-coordinate (on image)
	printf("selection: area=%d, score=%f \n\n", max_score_location, max_score);
	int x_goal = centroid_coord[max_score_location][0];

	// Selection of suggested destination y-coordinate (on image)
	int walker_sum;
	int running_best;
	int y_goal=0;
	for (int idy = centroid_coord[max_score_location][1]; idy > 0; idy-=num_pixels_above){

		walker_sum = 0;

		for (int id_walker = idy-1; id_walker>(idy-1-num_pixels_above);--id_walker){
			walker_sum += binary_image[id_walker][x_goal];
			running_best = id_walker;
			printf("idy:%d , idw:%d -> %d \n", idy,id_walker,walker_sum);
		};

		if ((walker_sum == num_pixels_above) && ((centroid_coord[max_score_location][1]- running_best) < max_dist_cent*im_height)){
			y_goal = running_best;
		};
	};


	// /// SHOW OFF MODE /// (from python)
	// Changing the location of the destination to a 55 (for feedback purposes only)
	binary_image[y_goal][x_goal] = 55;


	// Printing resulting binary array
	array_printer(im_width, im_height, binary_image);


	return 0;
};


void array_printer(int size_x, int size_y, int input_array[size_x][size_y])
{
	for (int idy = 0; idy < size_y; ++idy){
		for (int idx = 0; idx < size_x; ++idx){
			printf("%d ", input_array[idy][idx]);
		};
		printf("--> %d \n", idy);
	};
};





