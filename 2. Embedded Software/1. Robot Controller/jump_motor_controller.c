/*
 * jump_motor_controller.c
 *
 *  Created on: 2023. 3. 16.
 *      Author: BSM
 */
#include "jump_motor_controller.h"
#include <math.h>


struct JUMP_MOTOR_ jump_motor;

/*
 * Module: 		MOTOR
 * Description: Init Jump Motor with TIMER and INFORMATION
 * Input:  		TIM_HandleTypeDef* htim_motor;
 * 		   		TIM_HandleTypeDef* htim_encoder;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */

void Jump_Motor_Init(TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_encoder)
{
	jump_motor.htim_motor = htim_motor;
	jump_motor.htim_encoder = htim_encoder;

}

/*
 * Module:		MOTOR
 * Description: One Step Control for Jumping Motor
 * Input:  		uint8_t user_cmd
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Jumping_Motor_Control_step(uint8_t user_cmd)
{
	float target_angle;
	float theta_out;
	float error_theta;

	Jumping_Motor_Enc_read(); //update (jump_motor.jump_motor_position)
	theta_out = jump_motor.jump_motor_position / 97.064;

	switch (user_cmd)
	{
		case JUMPER_STATE_E_STORE:
		{
			target_angle = jump_motor_e_store_position;
			error_theta = (target_angle  - theta_out) / rad2deg;
			break;
		}
		case JUMPER_STATE_TRIG:
		{
			target_angle = jump_motor_e_store_position + energy_release_delta;
			error_theta = (target_angle  - theta_out) / rad2deg;
			break;
		}
		case JUMPER_STATE_INIT:
		{
			target_angle = jump_motor_e_store_position + energy_release_delta + jump_motor_init_delta;
			error_theta = ((int)(target_angle - theta_out) % (360)) / rad2deg;
			break;
		}
		default:
		{
			target_angle = 0;
			error_theta = (target_angle  - theta_out)  / rad2deg;
			break;
		}
	}
	//error_theta = target_angle / rad2deg - theta_out;

	if(error_theta > jump_motor_voltage_sat)
	{
		error_theta = jump_motor_voltage_sat;
	}
	else if(error_theta < - jump_motor_voltage_sat)
	{
		error_theta = - jump_motor_voltage_sat;
	}
	Jumper_Motor_PWM_run(error_theta);
}


/*
 * Module:		MOTOR
 * Description: PWM Control for Jumping Motor
 * Input:  		float jump_motor_norm_input
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Jumper_Motor_PWM_run(float jump_motor_norm_input)
{
	uint32_t pwm_ccr;
	if(jump_motor_norm_input > 0){
		pwm_ccr = (uint32_t)jump_motor.htim_motor->Instance->ARR * jump_motor_norm_input;
		jump_motor.htim_motor->Instance->CCR1 = jump_motor.htim_motor->Instance->ARR - pwm_ccr;
		jump_motor.htim_motor->Instance->CCR2 = jump_motor.htim_motor->Instance->ARR;
	}
	else if(jump_motor_norm_input < 0){
		pwm_ccr = (uint32_t)jump_motor.htim_motor->Instance->ARR * (- jump_motor_norm_input);
		jump_motor.htim_motor->Instance->CCR1 = jump_motor.htim_motor->Instance->ARR;
		jump_motor.htim_motor->Instance->CCR2 = jump_motor.htim_motor->Instance->ARR - pwm_ccr;
	}
	else {
		jump_motor.htim_motor->Instance->CCR1 = jump_motor.htim_motor->Instance->ARR;
		jump_motor.htim_motor->Instance->CCR2 = jump_motor.htim_motor->Instance->ARR;
	}
}

/*
 * Module:		ENCODER
 * Description: Init the Encoder of Jumping Motor
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Jumping_Motor_Enc_init()
{
	jump_motor.jump_enc_init = jump_motor.htim_encoder->Instance->CNT;

}

/*
 * Module:		ENCODER
 * Description: Read Encoder Value of Jumping Motor
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Jumping_Motor_Enc_read()
{
	//Q: Why this function?
	jump_motor.jump_motor_position= (jump_motor.htim_encoder->Instance->CNT + jump_motor.htim_encoder->Init.Period - jump_motor.jump_enc_init) % jump_motor.htim_encoder->Init.Period;
}


