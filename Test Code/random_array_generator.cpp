#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <cmath>

using namespace std;
int main()
{
	// declare the height, width and array
	int height = 45, width = 45, **number_array = 0;//, *number_list = 0;
	int **sector_averages = 0, **sector_averages_2 = 0;
	int v_sectors = 15, h_sectors = 15;
	int sector_height, sector_width, sector_end = 3, x_end;
	int safe;
	int number_list[height*width*2];

	// declare array setup void function
	void array_setup(int h, int w, int **array);
	void sector_averager (int hor_sectors, int vert_sectors, int sector_h, 
						  int sector_w, int **input_array, int **output_array);
    void sector_averager_2 (int image_width, int image_height, int sector_h, int sector_w, int **input_array, int **output_array);
    void sector_averager_3 (int *input_image, int **output_array, int x_end, int sector_h, int sector_w, int image_height, int image_width);
    void list_setup (int h, int w, int *list);

	// determine sector height
	sector_height = height/v_sectors;
	sector_width = width/h_sectors;
	x_end = sector_end*sector_width;

	/* 
	 * make pointer array which will be filled with in the array setup function
	 */
	number_array = new int *[height];
	for(int i = 0; i<height; ++i){
		number_array[i] = new int[width];
	}

	//number_list = new int *[height*width*2];

	/* 
	 * make pointer array which will be filled with the averages of the sectors 
	 */
	sector_averages = new int *[v_sectors];
	for(int j = 0; j<v_sectors; ++j){
		sector_averages[j] = new int[h_sectors];
	}

	sector_averages_2 = new int *[v_sectors];
	for(int j = 0; j<v_sectors; ++j){
		sector_averages_2[j] = new int[sector_end];
	}

	//array_setup(height, width, number_array);

	//sector_averager_2(width, height, sector_height, sector_width, number_array, sector_averages);

	list_setup(height, width, number_list);

	sector_averager_3 (number_list, sector_averages_2, x_end, sector_height, sector_width, height, width);



	printf("%s\n","The calculations are done");

	return 0;
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
	srand((unsigned)time(0));

	for (int i = 0; i<h; ++i){
		for (int j = 0; j < w; ++j){

			array[i][j] = (rand()%2)*255; //(i*w)+(j+1);
		}
	}
}

void list_setup (int h, int w, int *list)
{
	for(int i = 0; i <h*w*2; i+=2) {
		list[i] = 0;
		list[i+1] = i;
	}
}

void sector_averager_3 (int *input_image, int **output_array, int x_end, int sector_h, int sector_w, int image_height, int image_width) {
	int sum = 0;
	int s = 0;

	for(int i = 0; i < image_height; ++i){
		for(int j = s*sector_w*2; j <x_end*2; j+=2) {
			sum += input_image[i*image_width*2+j+1];
			printf("sum = %d,i = %d ,j = %d, s = %d\n",sum,i,j,s);
			if(j+2 == (s+1)*sector_w*2) {
				printf("break\n");
				break;
			}
		}
		if((i+1)%sector_h == 0) {
			printf("%d\n",sum/(sector_h*sector_w));
			printf("%d\n",(i+1)/sector_h+s*15);
			output_array[(i+1)/sector_h-1][s] = sum/(sector_h*sector_w);
			sum = 0;
		}
		if(i == image_height-1 && s < (x_end/sector_w) - 1) {
			i = -1;
			++s;
		}
	}
}
/*
 * This piece of code selectes certain parts of the array and averages the values.
 * These values are then put in an array/list which can be used for control.
 
void sector_averager (int hor_sectors, int vert_sectors, int sector_h, int sector_w, 
					  int **input_array, int **output_array)
{
	int sum = 0;

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
*/

/*
 * This piece of code selectes certain parts of the array and averages the values.
 * These values are then put in an array/list which can be used for control.
 */
void sector_averager_2 (int image_width, int image_height, int sector_h, int sector_w, int **input_array, int **output_array)
{
	int sum = 0;
	int s = 0;
	int binary_threshold = 130;

	for(int i = 0; i < image_height; ++i){
		for(int j = s*sector_w ; j < image_width ; ++j) {
			sum += input_array[i][j];
		    if(j == ((s+1)*sector_w-1)) {
				break;
		    }
		}
		if((i+1)%sector_h == 0){
			if (sum/(sector_h*sector_w) > binary_threshold)
      		{
        		output_array[(i+1)/sector_h-1][s] = 1;
      		}
      		else{
        		output_array[(i+1)/sector_h-1][s] = 0;
      		}
			//output_array[(i+1)/sector_h-1][s] = sum/(sector_h*sector_w);
			sum = 0;
		}
		if(i == image_height-1 && s < (image_width/sector_w - 1)) {
			i = -1;
			s += 1;
		}
	}
}
