#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <cmath>

using namespace std;

int main()
{
	// declare the height, width and array
	int height = 9, width = 45, **number_array = 0;
	float **sector_averages = 0;
	int v_sectors = 3, h_sectors = 15;
	int sector_height, sector_width;

	// declare array setup void function
	void array_setup(int h, int w, int **array);
	void sector_averager (int hor_sectors, int vert_sectors, int sector_h, 
						  int sector_w, int **input_array, float **output_array);

	// determine sector height
	sector_height = height/v_sectors;
	sector_width = width/h_sectors;

	/* 
	 * make pointer array which will be filled with in the array setup function
	 */
	number_array = new int *[height];
	for(int i = 0; i<height; ++i){
		number_array[i] = new int[width];
	}

	/* 
	 * make pointer array which will be filled with the averages of the sectors 
	 */
	sector_averages = new float *[sector_height];
	for(int j = 0; j<v_sectors; j++){
		sector_averages[j] = new float[h_sectors];
	}

	array_setup(height, width, number_array);
	sector_averager (h_sectors, v_sectors, sector_height, sector_width, 
					number_array, sector_averages);
}


/*
 * This fuction fills the array defined in the main function with a values from 1 to
 * the total amount of indices in the array from left to right.
 *
 * for example:
 *
 *  1  2  3  4  5
 *  6  7  8  9  10
 *  11 12 13 14 15
 *  16 17 18 19 20
 *
 * This matrix will be used by the sector averager which can be usefull for the 
 * green trace module.
 */
void array_setup (int h, int w, int **array)
{	
	srand((unsigned)time(NULL));

	for (int i = 0; i<h; i++){
		for (int j = 0; j < w; j++){

			array[i][j] = (i*w)+(j+1);;

		}
	}
}


/*
 * This piece of code selectes certain parts of the array and averages the values.
 * These values are then put in an array/list which can be used for control.
 */
void sector_averager (int hor_sectors, int vert_sectors, int sector_h, int sector_w, 
					  int **input_array, float **output_array)
{
	float sum = 0;

	for(int i = 0; i < vert_sectors; i++){
		for(int j = 0; j<hor_sectors; j++){
			for(int k = 0; k<(sector_h); k++){
				for(int l = 0; l<(sector_w); l++){

					sum += input_array[k+i*sector_h][l+j*sector_w];

				}
			}

		output_array[i][j] = sum/(sector_h*sector_w);
		sum = 0;

		};
	}
}
