/*
 * features.c
 *
 *  Created on: Aug 31, 2018
 *      Author: iho
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "config.h"


float avg_x,avg_y,avg_z, max_x, max_y, max_z, sma;

uint8_t features_init(int32_t ds[BATCH_SIZE][3]){

	uint32_t sum_x = 0, sum_y = 0, sum_z = 0;

	max_x = 0.0;
	max_y = 0.0;
	max_z = 0.0;

	for (int i=0;i<BATCH_SIZE;i++){
     sum_x += abs(ds[i][2]);
     sum_y += abs(ds[i][1]);
     sum_z += abs(ds[i][0]);

    if (abs(ds[i][2]) > max_x ) max_x = abs(ds[i][2]);
    if (abs(ds[i][1]) > max_y ) max_y = abs(ds[i][1]);
    if (abs(ds[i][0]) > max_z ) max_z = abs(ds[i][0]);

	}

	avg_x = (float)(sum_x / (BATCH_SIZE)); //BATCH_SIZE = 512*4 in text!!! without ( and ) it calculates (sum_x/4)*512 !!!!
	avg_y = (float)(sum_y / (BATCH_SIZE));
	avg_z = (float)(sum_z / (BATCH_SIZE));

	sma = (float)((sum_x+sum_y+sum_z) / (BATCH_SIZE));

	return FEATURES_NUM;

}


float f_avg_x(){
	return avg_x;
}

float f_avg_y(){
	return avg_y;
}

float f_avg_z(){
	return avg_z;
}

float f_max_x(){
	return max_x;
}

float f_max_y(){
	return max_y;
}

float f_max_z(){
	return max_z;
}

float f_sma(){
	return sma;
}

float f_mean(void *data, int dataSize){
	return 0;
}

float f_sum(void *data, int dataSize){
	float res = 0.0;
	for (int i=0;i<dataSize;i++){
		res++;
	//	res += ((float [7][dataSize])data)[0][i];
	}
	return res;
}
