/*
 * features.h
 *
 *  Created on: Aug 31, 2018
 *      Author: iho
 */

#ifndef MAIN_FEATURES_H_
#define MAIN_FEATURES_H_


//prototypes for feature calculation functions

uint8_t features_init(int32_t [BATCH_SIZE][3] );

float f_avg_x(),f_avg_y(),f_avg_z(), f_mean(), f_sum(),f_max_x(),f_max_y(),f_max_z(),f_sma();

//array of feature calculation functions

float (* features_calc[FEATURES_NUM])() = {f_avg_x,f_avg_y,f_avg_z,f_max_x,f_max_y,f_max_z,f_sma};

#endif /* MAIN_CONFIG_H_ */


