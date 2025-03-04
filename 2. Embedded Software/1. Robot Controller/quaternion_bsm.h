/*
 * quaternion_bsm.h
 *
 *  Created on: May 26, 2022
 *      Author: bsm6656
 */

#ifndef INC_QUATERNION_BSM_H_
#define INC_QUATERNION_BSM_H_

#include "mpu9250_spi_bsm.h"
#include "stm32f4xx_hal.h"
#include <math.h>


extern float GyroMeasError;
extern float GyroMeasDrift;
extern float beta;
extern float zeta;

extern void madgwickFilterMARG(float ax, float ay, float az, float gx, float gy, float gz,
		float mx, float my, float mz, float*q, float freq);
extern void madgwickFilter(float ax, float ay, float az, float gx, float gy, float gz,
		float mx, float my, float mz, float*q, float freq);

#endif /* INC_QUATERNION_BSM_H_ */
