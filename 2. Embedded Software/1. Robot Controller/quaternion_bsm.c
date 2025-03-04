/*
 * quaternion_bsm.c
 *
 *  Created on: May 26, 2022
 *      Author: bsm6656
 */

#include "quaternion_bsm.h"

float GyroMeasError = M_PI * 50.0f / 180.0f;
float GyroMeasDrift = M_PI * 0.2f / 180.0f;
float beta;
float zeta;

// reference direction of flux in earth frame
float b_x = 1;
float b_z = 0;


// estimate gyroscope biases error
float w_bx = 0;
float w_by = 0;
float w_bz = 0;


void madgwickFilterMARG(float ax, float ay, float az, float gx, float gy, float gz,
		float mx, float my, float mz, float*q, float freq){

	float q0 = q[0];
	float q1 = q[1];
	float q2 = q[2];
	float q3 = q[3];


	float hx, hy, hz;						// Computed flux in the earth frame
	float q0_dot_w, q1_dot_w, q2_dot_w, q3_dot_w;	// Quaternion rate from gyroscope
	float q0_hat_dot, q1_hat_dot, q2_hat_dot, q3_hat_dot;	// Quaternion rate from gyroscope
	float wx_e, wy_e, wz_e;						// Gyro error
	float w_x, w_y, w_z;					// bias removed gyro



	float fg1, fg2, fg3;					// Error(quaternion prediction - sensor read)
	float fg1_d0, fg1_d1, fg1_d2, fg1_d3;	// gradients
	float fg2_d0, fg2_d1, fg2_d2, fg2_d3;
	float fg3_d0, fg3_d1, fg3_d2, fg3_d3;

	float fm1, fm2, fm3;
	float fm1_d0, fm1_d1, fm1_d2, fm1_d3;
	float fm2_d0, fm2_d1, fm2_d2, fm2_d3;
	float fm3_d0, fm3_d1, fm3_d2, fm3_d3;


	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;

	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;

	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;





	float norm;
	float dt = 1.0f / freq;
	beta = sqrtf(3.0f / 4.0f) * GyroMeasError;
	zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;


	// Normalize accelerometer & magnetometer data
	norm = sqrtf(ax*ax + ay*ay + az*az);
	if(norm == 0.0f) return;
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	norm = sqrtf(mx*mx + my*my + mz*mz);
	if(norm == 0.0f) return;
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Convert scale of the gyro from deg/s to rad/s

	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;



	// Convert reference axis to the sensor axis

	fg1 = 2.0f * (q1q3 - q0q2) - ax;
	fg2 = 2.0f * (q0q1 + q2q3) - ay;
	fg3 = 2.0f * (0.5 - q1q1 - q2q2) - az;


	fm1 = 2.0f * (b_x * (0.5 - q2q2 - q3q3) + b_z * (q1q3 - q0q2)) - mx;
	fm2 = 2.0f * (b_x * (q1q2 - q0q3) + b_z * (q2q3 + q0q1)) - my;
	fm3 = 2.0f * (b_x * (q1q3 + q0q2) + b_z * (0.5 - q1q1 - q2q2)) - mz;


	// Gradients of above fcns.
	fg1_d0 = - 2.0f * q2;
	fg1_d1 = 2.0f * q3;
	fg1_d2 = - 2.0f * q0;
	fg1_d3 = 2.0f * q1;

	fg2_d0 = fg1_d3;
	fg2_d1 = - fg1_d2;
	fg2_d2 = fg1_d1;
	fg2_d3 = - fg1_d0;

	fg3_d0 = 0;
	fg3_d1 = - 4.0f * q1;
	fg3_d2 = - 4.0f * q2;
	fg3_d3 = 0;


	fm1_d0 = fg1_d0 *b_z;
	fm1_d1 = fg1_d1 * b_z;
	fm1_d2 = fg3_d2 * b_x + fg1_d2 * b_z;
	fm1_d3 = - 4.0f * q3 * b_x + fg1_d3 * b_z;

	fm2_d0 = - fg1_d1 * b_x + fg1_d3 * b_z;
	fm2_d1 = fg2_d3 * b_x + fg2_d1 * b_z;
	fm2_d2 = fg1_d3 * b_x + fm1_d1;
	fm2_d3 = fg1_d2 * b_x - fm1_d0;

	fm3_d0 = fg2_d3 *b_x;
	fm3_d1 = fg1_d1 * b_x + fg3_d1 * b_z;
	fm3_d2 = fg2_d1 * b_x + fg3_d2 * b_z;
	fm3_d3 = fg1_d3 * b_x;


	// gradient descent

	q0_hat_dot = fg1_d0 * fg1 + fg2_d0 * fg2 + fg3_d0 * fg3 + fm1_d0 * fm1 + fm2_d0 * fm2 + fm3_d0 * fm3;		// use magnetometer data or not
	q1_hat_dot = fg1_d1 * fg1 + fg2_d1 * fg2 + fg3_d1 * fg3	+ fm1_d1 * fm1 + fm2_d1 * fm2 + fm3_d1 * fm3;
	q2_hat_dot = fg1_d2 * fg1 + fg2_d2 * fg2 + fg3_d2 * fg3	+ fm1_d2 * fm1 + fm2_d2 * fm2 + fm3_d2 * fm3;
	q3_hat_dot = fg1_d3 * fg1 + fg2_d3 * fg2 + fg3_d3 * fg3	+ fm1_d3 * fm1 + fm2_d3 * fm2 + fm3_d3 * fm3;

	norm = sqrtf(q0_hat_dot*q0_hat_dot + q1_hat_dot*q1_hat_dot + q2_hat_dot*q2_hat_dot + q3_hat_dot*q3_hat_dot);
	norm = 1.0f / norm;
	q0_hat_dot *= norm;
	q1_hat_dot *= norm;
	q2_hat_dot *= norm;
	q3_hat_dot *= norm;


	// Compute gyro error
	wx_e = 2.0f * (q0 * q1_hat_dot - q1 * q0_hat_dot - q2 * q3_hat_dot + q3 * q2_hat_dot);
	wy_e = 2.0f * (q0 * q2_hat_dot + q1 * q3_hat_dot - q2 * q0_hat_dot - q3 * q1_hat_dot);
	wz_e = 2.0f * (q0 * q3_hat_dot - q1 * q2_hat_dot + q2 * q1_hat_dot - q3 * q0_hat_dot);


	// gyro bias compensation
	w_bx += wx_e * dt *zeta;
	w_by += wy_e * dt *zeta;
	w_bz += wz_e * dt *zeta;

	w_x = gx - w_bx;
	w_y = gy - w_by;
	w_z = gz - w_bz;


	// rate of change of quaternion using gyro read & Numetical integration

	q0_dot_w = 0.5f * (-q1*w_x - q2*w_y - q3*w_z ) - beta * q0_hat_dot;
	q1_dot_w = 0.5f * (q0*w_x + q2*w_z - q3*w_y ) - beta * q1_hat_dot;
	q2_dot_w = 0.5f * (q0*w_y - q1*w_z + q3*w_x ) - beta * q2_hat_dot;
	q3_dot_w = 0.5f * (q0*w_z + q1*w_y - q2*w_x ) - beta * q3_hat_dot;



	q0 += q0_dot_w * dt;
	q1 += q1_dot_w * dt;
	q2 += q2_dot_w * dt;
	q3 += q3_dot_w * dt;

	norm = sqrtf(q0q0 + q1q1 + q2q2 + q3q3);
	norm = 1.0f / norm;

	q0 *= norm;
	q1 *= norm;
	q2 *= norm;
	q3 *= norm;


	q0q0 = q0*q0;
	q0q1 = q0*q1;
	q0q2 = q0*q2;
	q0q3 = q0*q3;

	q1q1 = q1*q1;
	q1q2 = q1*q2;
	q1q3 = q1*q3;

	q2q2 = q2*q2;
	q2q3 = q2*q3;
	q3q3 = q3*q3;


	// Set Earth magnetic field Compensated by sensor mag data
	hx = 2.0f * (mx * (0.5 - q2q2 - q3q3) +  my * (q1q2 - q0q3) +  mz * (q1q3 + q0q2));
	hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1));
	hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2));

	b_x = sqrtf(hx*hx + hy*hy);
	b_z = hz;

	q[0] = q0;
	q[1] = q1;
	q[2] = q2;
	q[3] = q3;


}



void madgwickFilter(float ax, float ay, float az, float gx, float gy, float gz,
		float mx, float my, float mz, float*q, float freq){

	float q0=q[0];
	float q1=q[1];
	float q2=q[2];
	float q3=q[3];


	float e0, e1, e2, e3;
	float q0_dot, q1_dot, q2_dot, q3_dot;


	float fg1, fg2, fg3;					// Error(quaternion prediction - sensor read)
	float fg1_d0, fg1_d1, fg1_d2, fg1_d3;	// gradients
	float fg2_d0, fg2_d1, fg2_d2, fg2_d3;
	float fg3_d0, fg3_d1, fg3_d2, fg3_d3;



	float norm;
	float dt = 1.0f / freq;
	beta = sqrtf(3.0f / 4.0f) * GyroMeasError;

	// Normalize accelerometer & magnetometer data
	norm = sqrtf(ax*ax + ay*ay + az*az);
	if(norm == 0.0f) return;
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;



	// Convert scale of the gyro from deg/s to rad/s

	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;




	// Convert reference axis to the sensor axis

	fg1 = 2.0f * (q1*q3 - q0*q2) - ax;
	fg2 = 2.0f * (q0*q1 + q2*q3) - ay;
	fg3 = 2.0f * (0.5 - q1*q1 - q2*q2) - az;



	// Gradients of above fcns.
	fg1_d0 = - 2.0f * q2;
	fg1_d1 = 2.0f * q3;
	fg1_d2 = - 2.0f * q0;
	fg1_d3 = 2.0f * q1;

	fg2_d0 = 2.0f * q1;
	fg2_d1 = 2.0f * q0;
	fg2_d2 = 2.0f * q3;
	fg2_d3 = 2.0f * q2;

	fg3_d0 = 0;
	fg3_d1 = - 4.0f * q1;
	fg3_d2 = - 4.0f * q2;
	fg3_d3 = 0;




	// gradient descent

	e0 = fg1_d0 * fg1 + fg2_d0 * fg2 + fg3_d0 * fg3;

	e1 = fg1_d1 * fg1 + fg2_d1 * fg2 + fg3_d1 * fg3;

	e2 = fg1_d2 * fg1 + fg2_d2 * fg2 + fg3_d2 * fg3;

	e3 = fg1_d3 * fg1 + fg2_d3 * fg2 + fg3_d3 * fg3;


	norm = sqrtf(e0*e0 + e1*e1 + e2*e2 + e3*e3);
	norm = 1.0f / norm;
	e0 *= norm;
	e1 *= norm;
	e2 *= norm;
	e3 *= norm;


	// rate of change of quaternion using gyro read & Numetical integration

	q0_dot = 0.5f * (-q1*gx - q2*gy - q3*gz ) - beta * e0;
	q1_dot = 0.5f * (q0*gx + q2*gz - q3*gy ) - beta * e1;
	q2_dot = 0.5f * (q0*gy - q1*gz + q3*gx ) - beta * e2;
	q3_dot = 0.5f * (q0*gz + q1*gy - q2*gx ) - beta * e3;

	q0 += q0_dot * dt;
	q1 += q1_dot * dt;
	q2 += q2_dot * dt;
	q3 += q3_dot * dt;

	norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	norm = 1.0f / norm;

	q[0] = q0 * norm;
	q[1] = q1 * norm;
	q[2] = q2 * norm;
	q[3] = q3 * norm;

}

