/*
 * motor_drive.c
 *
 *  [HR8833]
 *  	[	On / Off mode	]
 *  	|	in1	|	in2	|	out1	|	out2	|		Function		|
 *  	|	0	|	0	|		Z	|		Z	|	Coast / Fast decay	|
 *  	|	0	|	1	|		L	|		H	|		Reverse			|
 *   	|	1	|	0	|		H	|		L	|		Forward			|
 *  	|	1	|	1	|		L	|		L	|	Brake / Slow decay	|
 *  	[	PWM mode	]
 *  	|	in1	|	in2	|			Function			|		Speed	|
 *  	|	PWM	|	0	|	Forward PWM / Fast decay	|	High duty	|
 *  	|	1	|	PWM	|	Forward PWM / Slow decay	|  	Low duty	|
 *   	|	0	|	PWM	|	Reverse PWM / Fast decay	|	High duty	|
 *  	|	PWM	|	1	|	Reverse PWM / Slow decay	|  	Low duty	|
 *
 *
 *  Created on: 2022. 8. 10.
 *      Author: BSM
 */
#include "motor_drive.h"

int16_t jumper_motor_error = 0;
int16_t jumper_motor_error_sum = 0;
int16_t jumper_motor_error_diff = 0;

int16_t hang_motor_error = 0;
int16_t hang_motor_error_sum = 0;
int16_t hang_motor_error_diff = 0;

int16_t yaw_motor_error = 0;
int16_t yaw_motor_error_sum = 0;
int16_t yaw_motor_error_diff = 0;

/*----------------------------------------------------------*/
/****	Motor System ID	****/
/*
 * Module: 		MOTOR SYSTEM ID
 * Description: Step Input of Motor Sys ID
 * Input:  		TIM_HandleTypeDef* htim_motor;
 * 		   		uint16_t enc_init;
 *				uint16_t input_magnitude;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Motor_Sys_ID_Step_Input(TIM_HandleTypeDef* htim_motor, uint16_t enc_init, uint16_t input_magnitude)
{
	uint16_t pwm_ccr;

	pwm_ccr = input_magnitude;
	htim_motor->Instance->CCR1 = htim_motor->Instance->ARR - pwm_ccr;
	htim_motor->Instance->CCR2 = htim_motor->Instance->ARR;
}

/*
 * Module: 		MOTOR SYSTEM ID
 * Description: Step Input of Hang Motor Sys ID
 * Input:  		TIM_HandleTypeDef* htim;
 * 		   		uint16_t adc_angle;
 *				uint16_t input_magnitude;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Hang_Motor_Sys_ID_Step_Input(TIM_HandleTypeDef* htim, uint16_t adc_angle, int32_t input_magnitude)
{

	uint16_t pwm_ccr;
	if((adc_angle < 100) || (adc_angle > 4000))
	{
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
	else
	{
		if(input_magnitude >= 0)
		{
			pwm_ccr = input_magnitude;
			htim->Instance->CCR1 = htim->Instance->ARR - pwm_ccr;
			htim->Instance->CCR2 = htim->Instance->ARR;
		}
		else
		{
			pwm_ccr = (0 - input_magnitude);
			htim->Instance->CCR1 = htim->Instance->ARR;
			htim->Instance->CCR2 = htim->Instance->ARR - pwm_ccr;
		}
	}
}

/*---------------------------------------------------------------------------*/
/****	Jump Motor	****/
/*
 * Module: 		JUMP MOTOR
 * Description: Position Control of Jump Motor for Gliding Function
 * Input:  		TIM_HandleTypeDef* htim_motor;
 * 		   		TIM_HandleTypeDef* htim_encoder;
 *				uint16_t energy_stored_state;
 *				uint16_t enc_init;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Jump_Motor_Gliding_Position_Control(TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_encoder, uint16_t energy_stored_state, uint16_t enc_init)
{
	uint16_t Kp = 10;
	uint16_t Ki = 0;
	uint16_t Kd = 0;
	int32_t pid_gain = 0;
	uint16_t pwm_ccr;

	uint16_t current_enc_data;
	uint16_t trigger_state = 0;

	trigger_state = energy_stored_state + 100;

	current_enc_data= (htim_encoder->Instance->CNT + htim_encoder->Init.Period - enc_init) % htim_encoder->Init.Period;

	PID_Errors(&jumper_motor_error, &jumper_motor_error_sum, &jumper_motor_error_diff, trigger_state, current_enc_data);

	pid_gain = Kp * (jumper_motor_error / 1) + Ki * (jumper_motor_error_sum / 100) + Kd * (jumper_motor_error_diff * 100);

	if(pid_gain >= 0)
	{
		pwm_ccr = (uint16_t)pid_gain;
		htim_motor->Instance->CCR1 = htim_motor->Instance->ARR - pwm_ccr;
		htim_motor->Instance->CCR2 = htim_motor->Instance->ARR;
	}
	else
	{
		pwm_ccr =  (uint16_t)(0 - pid_gain);
		htim_motor->Instance->CCR1 = htim_motor->Instance->ARR;
		htim_motor->Instance->CCR2 = htim_motor->Instance->ARR - pwm_ccr;
	}

}

/*
 * Module: 		JUMP MOTOR
 * Description: Position Control of Jump Motor for Landing Function
 * Input:  		TIM_HandleTypeDef* htim_motor;
 * 		   		TIM_HandleTypeDef* htim_encoder;
 *				uint16_t energy_stored_state;
 *				uint16_t enc_init;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Jump_Motor_Landing_Position_Control(TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_encoder, uint16_t energy_stored_state, uint16_t enc_init)
{
	uint16_t Kp = 10;
	uint16_t Ki = 0;
	uint16_t Kd = 0;
	int32_t pid_gain = 0;
	uint16_t pwm_ccr;

	uint16_t current_enc_data;
	uint16_t rest_state = 0;

	rest_state = (0 + energy_stored_state + 500) % htim_encoder->Init.Period;

	current_enc_data= (((htim_encoder->Instance->CNT + htim_encoder->Init.Period - enc_init) % htim_encoder->Init.Period) + energy_stored_state)  % htim_encoder->Init.Period;

	PID_Errors(&jumper_motor_error, &jumper_motor_error_sum, &jumper_motor_error_diff, rest_state, current_enc_data);

	pid_gain = Kp * (jumper_motor_error / 1) + Ki * (jumper_motor_error_sum / 100) + Kd * (jumper_motor_error_diff * 100);

	if(pid_gain >= 0)
	{
		pwm_ccr = (uint16_t)pid_gain;
		htim_motor->Instance->CCR1 = htim_motor->Instance->ARR - pwm_ccr;
		htim_motor->Instance->CCR2 = htim_motor->Instance->ARR;
	}
	else
	{
		pwm_ccr =  (uint16_t)(0 - pid_gain);
		htim_motor->Instance->CCR1 = htim_motor->Instance->ARR;
		htim_motor->Instance->CCR2 = htim_motor->Instance->ARR - pwm_ccr;
	}
}

/*
 * Module: 		JUMP MOTOR
 * Description: Position Control of Jump Motor for Trigger Function
 * Input:  		TIM_HandleTypeDef* htim_motor;
 * 		   		TIM_HandleTypeDef* htim_encoder;
 *				uint16_t energy_stored_state;
 *				uint16_t enc_init;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Jump_Motor_Trigger_Position_Control(TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_encoder, uint16_t energy_stored_state, uint16_t enc_init)
{
	uint16_t Kp = 10;
	uint16_t Ki = 0;
	uint16_t Kd = 0;
	int32_t pid_gain = 0;
	uint16_t pwm_ccr;

	uint16_t current_enc_data;
	uint16_t trigger_state = 0;

	trigger_state = energy_stored_state + 100;

	current_enc_data= (htim_encoder->Instance->CNT + htim_encoder->Init.Period - enc_init) % htim_encoder->Init.Period;

	PID_Errors(&jumper_motor_error, &jumper_motor_error_sum, &jumper_motor_error_diff, trigger_state, current_enc_data);

	pid_gain = Kp * (jumper_motor_error / 1) + Ki * (jumper_motor_error_sum / 100) + Kd * (jumper_motor_error_diff * 100);

	if(pid_gain >= 0)
	{
		pwm_ccr = (uint16_t)pid_gain;
		htim_motor->Instance->CCR1 = htim_motor->Instance->ARR - pwm_ccr;
		htim_motor->Instance->CCR2 = htim_motor->Instance->ARR;
	}
	else
	{
		pwm_ccr =  (uint16_t)(0 - pid_gain);
		htim_motor->Instance->CCR1 = htim_motor->Instance->ARR;
		htim_motor->Instance->CCR2 = htim_motor->Instance->ARR - pwm_ccr;
	}

}

/*
 * Module: 		JUMP MOTOR
 * Description: Position Control of Jump Motor for Energy Store Function
 * Input:  		TIM_HandleTypeDef* htim_motor;
 * 		   		TIM_HandleTypeDef* htim_encoder;
 *				uint16_t energy_stored_state;
 *				uint16_t enc_init;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Jump_Motor_Energy_Store_Position_Control(TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_encoder, uint16_t energy_stored_state, uint16_t enc_init)
{
	uint16_t Kp = 10;
	uint16_t Ki = 0;
	uint16_t Kd = 0;
	int32_t pid_gain = 0;
	uint16_t pwm_ccr;

	uint16_t current_enc_data;

	current_enc_data= (htim_encoder->Instance->CNT + htim_encoder->Init.Period - enc_init) % htim_encoder->Init.Period;

	PID_Errors(&jumper_motor_error, &jumper_motor_error_sum, &jumper_motor_error_diff, energy_stored_state, current_enc_data);

	pid_gain = Kp * (jumper_motor_error / 1) + Ki * (jumper_motor_error_sum / 100) + Kd * (jumper_motor_error_diff * 100);

	if(pid_gain >= 0)
	{
		pwm_ccr = (uint16_t)pid_gain;
		htim_motor->Instance->CCR1 = htim_motor->Instance->ARR - pwm_ccr;
		htim_motor->Instance->CCR2 = htim_motor->Instance->ARR;
	}
	else
	{
		pwm_ccr =  (uint16_t)(0 - pid_gain);
		htim_motor->Instance->CCR1 = htim_motor->Instance->ARR;
		htim_motor->Instance->CCR2 = htim_motor->Instance->ARR - pwm_ccr;
	}


}

/*
 * Module: 		JUMP MOTOR--Encoder
 * Description: Shift Encoder Value
 * Input:  		TIM_HandleTypeDef* htim;
 * 		   		TIM_HandleTypeDef* htim_encoder;
 *				uint16_t energy_stored_state;
 *				uint16_t encoder_init;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
int Jump_Motor_Enc_Shifted(TIM_HandleTypeDef* htim, uint16_t encoder_init)
{
	return (htim->Instance->CNT + htim->Init.Period - encoder_init) % htim->Init.Period;// - encoder_init;
}

/*
 * Module: 		JUMP MOTOR--Encoder
 * Description: Read Encoder Value
 * Input:  		TIM_HandleTypeDef* htim;
 * 		   		uint16_t* encoder_raw_cnt;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Jump_Motor_Enc_read_raw(TIM_HandleTypeDef* htim, uint16_t* encoder_raw_cnt)
{
	*encoder_raw_cnt = htim->Instance->CNT;// - encoder_init;

}

/*
 * Module: 		JUMP MOTOR
 * Description: PWM Control for Jump Motor
 * Input:  		TIM_HandleTypeDef* htim;
 * 		   		uint8_t input;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Jump_Motor_PWM_run(TIM_HandleTypeDef* htim, uint8_t input)
{
	uint32_t pwm_ccr;
	if(input > 128){
		pwm_ccr = 999*(input-128)/128;
		htim->Instance->CCR1 = htim->Instance->ARR - pwm_ccr;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
	else if(input < 127){
		pwm_ccr = 65535*(128-input)/128;
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR - pwm_ccr;
	}
	else {
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
}
/*
void Jump_Motor_PWM_Backward(){

}
*/
/*
 * Module: 		JUMP MOTOR
 * Description: PWM STOP Control for Jump Motor
 * Input:  		TIM_HandleTypeDef* htim;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Jump_Motor_PWM_Stop(TIM_HandleTypeDef* htim)
{
	htim->Instance->CCR1 = htim->Instance->ARR;
	htim->Instance->CCR2 = htim->Instance->ARR;
}

/*---------------------------------------------------------------------------*/
/****	Yaw_Motor	****/
/*
 * Module: 		YAW MOTOR
 * Description: Control of YAW Motor
 * Input:  		TIM_HandleTypeDef* htim;
 * 		   		uint16_t adc_angle;
 *				uint16_t target;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Yaw_Motor_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target)
{
	uint16_t Kp = 5;
	uint16_t Ki = 0;
	uint16_t Kd = 0;
	int32_t pid_gain = 0;
	uint16_t pwm_ccr;

	PID_Errors(&yaw_motor_error, &yaw_motor_error_sum, &yaw_motor_error_diff, target, adc_angle);

	if(adc_angle < 100 || adc_angle > 3996)
	{
		pid_gain = 0;
	}
	//else
	//Q: Why Comment Out else?
	{
		pid_gain = (Kp * (yaw_motor_error / 1) + Ki * (yaw_motor_error_sum / 100) + Kd * (yaw_motor_error_diff * 100));
	}
	if(pid_gain >= 0)
	{
		pwm_ccr = (uint16_t) (pid_gain);
		if(pwm_ccr >= htim->Instance->ARR){pwm_ccr = htim->Instance->ARR;} //TODO: ADD MAXIMUM CONDITION
		htim->Instance->CCR1 = htim->Instance->ARR - pwm_ccr;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
	else
	{
		pwm_ccr =  (uint16_t) (0 - pid_gain);
		if(pwm_ccr >= htim->Instance->ARR){pwm_ccr = htim->Instance->ARR;} //TODO: ADD MAXIMUM CONDITION
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR - pwm_ccr;
	}
}

/*
 * Module: 		YAW MOTOR
 * Description: PWM Control of YAW Motor
 * Input:  		TIM_HandleTypeDef* htim;
 * 		   		uint8_t input;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Yaw_Motor_PWM_run(TIM_HandleTypeDef* htim, uint8_t input)
{
	uint32_t pwm_ccr;
	if(input > 128){
		pwm_ccr = 9000*(input-128)/128;
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR - pwm_ccr;
	}
	else if(input < 127){
		pwm_ccr = 9000*(128-input)/128;
		htim->Instance->CCR1 = htim->Instance->ARR - pwm_ccr;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
	else {
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
}

/*
void Yaw_Motor_PWM_Backward(){

}
*/

/*
 * Module: 		YAW MOTOR
 * Description: PWM STOP Control of YAW Motor
 * Input:  		TIM_HandleTypeDef* htim;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Yaw_Motor_PWM_Stop(TIM_HandleTypeDef* htim)
{
	htim->Instance->CCR1 = htim->Instance->ARR;
	htim->Instance->CCR2 = htim->Instance->ARR;
}

/*---------------------------------------------------------------------------*/
/****	Hang_Motor	****/
/*
 * Module: 		HANG MOTOR
 * Description: Position Control of YAW Motor for Landing in Gliding Function
 * Input:  		TIM_HandleTypeDef* htim;
 *				uint16_t adc_angle;
 *				uint16_t target;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Hang_Motor_Landing_In_Gliding_Position_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target)
{
	uint16_t Kp = 10;
	uint16_t Ki = 0;
	uint16_t Kd = 0;
	int32_t pid_gain = 0;
	uint16_t pwm_ccr;


	PID_Errors(&hang_motor_error, &hang_motor_error_sum, &hang_motor_error_diff, target, adc_angle);

	if(adc_angle < 100 || adc_angle > 3996)
	{
		pid_gain = 0;
		pid_gain = Kp * (hang_motor_error / 1) + Ki * (hang_motor_error_sum / 100) + Kd * (hang_motor_error_diff * 100);
		pid_gain = 0 - pid_gain;
	}
	else
	{
		pid_gain = Kp * (hang_motor_error / 1) + Ki * (hang_motor_error_sum / 100) + Kd * (hang_motor_error_diff * 100);
	}

	if(pid_gain >= 0)
	{
		pwm_ccr = (uint16_t) pid_gain / 65536;
		htim->Instance->CCR1 = htim->Instance->ARR - pwm_ccr;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
	else
	{
		pwm_ccr =  (uint16_t)(0 - pid_gain) / 65536;
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR - pwm_ccr;
	}
}

/*
 * Module: 		HANG MOTOR
 * Description: Position Control of YAW Motor for Gliding Function
 * Input:  		TIM_HandleTypeDef* htim;
 *				uint16_t adc_angle;
 *				uint16_t target;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Hang_Motor_Gliding_Position_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target)
{
	uint16_t Kp = 5;
	uint16_t Ki = 0;
	uint16_t Kd = 0;
	int32_t pid_gain = 0;
	uint16_t pwm_ccr;

	PID_Errors(&yaw_motor_error, &yaw_motor_error_sum, &yaw_motor_error_diff, target, adc_angle);

	if(adc_angle < 100 || adc_angle > 3996)
	{
		pid_gain = 0;
	}
	//else
	//Q: Why Comment Out else?
	{
		pid_gain = (Kp * (yaw_motor_error / 1) + Ki * (yaw_motor_error_sum / 100) + Kd * (yaw_motor_error_diff * 100));
	}
	if(pid_gain >= 0)
	{
		pwm_ccr = (uint16_t) (pid_gain);
		if(pwm_ccr >= htim->Instance->ARR){pwm_ccr = htim->Instance->ARR;} //TODO: ADD MAXIMUM CONDITION
		htim->Instance->CCR1 = htim->Instance->ARR - pwm_ccr;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
	else
	{
		pwm_ccr =  (uint16_t) (0 - pid_gain);
		if(pwm_ccr >= htim->Instance->ARR){pwm_ccr = htim->Instance->ARR;} //TODO: ADD MAXIMUM CONDITION
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR - pwm_ccr;
	}
}

/*
 * Module: 		HANG MOTOR
 * Description: Position Control of YAW Motor for Jumping in Gliding Function
 * Input:  		TIM_HandleTypeDef* htim;
 *				uint16_t adc_angle;
 *				uint16_t target;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Hang_Motor_Jumping_In_Gliding_Position_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target)
{
	uint16_t Kp = 10;
	uint16_t Ki = 0;
	uint16_t Kd = 0;
	int32_t pid_gain = 0;
	uint16_t pwm_ccr;


	PID_Errors(&hang_motor_error, &hang_motor_error_sum, &hang_motor_error_diff, target, adc_angle);

	if(adc_angle < 100 || adc_angle > 3996)
	{
		pid_gain = 0;
		pid_gain = Kp * (hang_motor_error / 1) + Ki * (hang_motor_error_sum / 100) + Kd * (hang_motor_error_diff * 100);
		pid_gain = 0 - pid_gain;
	}
	else
	{
		pid_gain = Kp * (hang_motor_error / 1) + Ki * (hang_motor_error_sum / 100) + Kd * (hang_motor_error_diff * 100);
	}

	if(pid_gain >= 0)
	{
		pwm_ccr = (uint16_t) pid_gain;
		if(pwm_ccr >= htim->Instance->ARR){pwm_ccr = htim->Instance->ARR;}
		htim->Instance->CCR1 = htim->Instance->ARR - pwm_ccr;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
	else
	{
		pwm_ccr =  (uint16_t)(0 - pid_gain);
		if(pwm_ccr >= htim->Instance->ARR){pwm_ccr = htim->Instance->ARR;}
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR - pwm_ccr;
	}
}

/*
 * Module: 		HANG MOTOR
 * Description: Position Control of YAW Motor for Landing Function
 * Input:  		TIM_HandleTypeDef* htim;
 *				uint16_t adc_angle;
 *				uint16_t target;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Hang_Motor_Landing_Position_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target)
{
	uint16_t Kp = 10;
	uint16_t Ki = 0;
	uint16_t Kd = 0;
	int32_t pid_gain = 0;
	uint16_t pwm_ccr;


	PID_Errors(&hang_motor_error, &hang_motor_error_sum, &hang_motor_error_diff, target, adc_angle);

	if(adc_angle < 100 || adc_angle > 3996)
	{
		pid_gain = 0;
		pid_gain = Kp * (hang_motor_error / 1) + Ki * (hang_motor_error_sum / 100) + Kd * (hang_motor_error_diff * 100);
		pid_gain = 0 - pid_gain;
	}
	else
	{
		pid_gain = Kp * (hang_motor_error / 1) + Ki * (hang_motor_error_sum / 100) + Kd * (hang_motor_error_diff * 100);
	}

	if(pid_gain >= 0)
	{
		pwm_ccr = (uint16_t) pid_gain;
		htim->Instance->CCR1 = htim->Instance->ARR - pwm_ccr;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
	else
	{
		pwm_ccr =  (uint16_t)(0 - pid_gain);
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR - pwm_ccr;
	}
}

/*
 * Module: 		HANG MOTOR
 * Description: Position Control of YAW Motor for Jumping Function
 * Input:  		TIM_HandleTypeDef* htim;
 *				uint16_t adc_angle;
 *				uint16_t target;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Hang_Motor_Jumping_Position_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target)
{
	uint16_t Kp = 10;
	uint16_t Ki = 0;
	uint16_t Kd = 0;
	int32_t pid_gain = 0;
	uint16_t pwm_ccr;


	PID_Errors(&hang_motor_error, &hang_motor_error_sum, &hang_motor_error_diff, target, adc_angle);

	if(adc_angle < 100 || adc_angle > 3996)
	{
		pid_gain = 0;
		pid_gain = Kp * (hang_motor_error / 1) + Ki * (hang_motor_error_sum / 100) + Kd * (hang_motor_error_diff * 100);
		pid_gain = 0 - pid_gain;
	}
	else
	{
		pid_gain = Kp * (hang_motor_error / 1) + Ki * (hang_motor_error_sum / 100) + Kd * (hang_motor_error_diff * 100);
	}

	if(pid_gain >= 0)
	{
		pwm_ccr = (uint16_t) pid_gain;
		htim->Instance->CCR1 = htim->Instance->ARR - pwm_ccr;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
	else
	{
		pwm_ccr =  (uint16_t)(0 - pid_gain);
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR - pwm_ccr;
	}
}

/*
 * Module: 		HANG MOTOR
 * Description: PWM Control of HANG Motor
 * Input:  		TIM_HandleTypeDef* htim;
 * 		   		uint8_t input;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Hang_Motor_PWM_run(TIM_HandleTypeDef* htim, uint8_t input)
{
	uint32_t pwm_ccr;
	if(input > 128){
		pwm_ccr = 9000*(input-128)/128;
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR - pwm_ccr;
	}
	else if(input < 127){
		pwm_ccr = 9000*(128-input)/128;
		htim->Instance->CCR1 = htim->Instance->ARR - pwm_ccr;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
	else {
		htim->Instance->CCR1 = htim->Instance->ARR;
		htim->Instance->CCR2 = htim->Instance->ARR;
	}
}
/*
void Hang_Motor_PWM_Backward(){

}
*/
/*
 * Module: 		HANG MOTOR
 * Description: PWM STOP Control of HANG Motor
 * Input:  		TIM_HandleTypeDef* htim;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Hang_Motor_PWM_Stop(TIM_HandleTypeDef* htim)
{
	htim->Instance->CCR1 = htim->Instance->ARR;
	htim->Instance->CCR2 = htim->Instance->ARR;
}

/*---------------------------------------------------------------------------*/
/****	Fold_Motor	****/
/*
 * Module: 		FOLD MOTOR
 * Description: FOLDING Control of FOLD Motor
 * Input:  		uint8_t direction;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Fold_Motor_Folding_Control(uint8_t direction)
{
	if(direction == 1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	}
	else if(direction == 2){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	}
}

/*
 * Module: 		FOLD MOTOR
 * Description: Forward Control of FOLD Motor
 * Input:  		uuint8_t input;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Fold_Motor_run_Forward(uint8_t input)
{
	if(input == 1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	}
	else if(input == 2){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	}
}

void Fold_Motor_run_Backward(){

}

/*
 * Module: 		FOLD MOTOR
 * Description: STOP Control of FOLD Motor
 * Input:  		uuint8_t input;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Fold_Motor_run_Stop()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
}

/*---------------------------------------------------------------------------*/
/****	Dep_Motor	****/
/*
 * Module: 		FOLD MOTOR
 * Description: FOLDING Control of DEP Motor
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Dep_Motor_Folding_Control()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

/*
 * Module: 		FOLD MOTOR
 * Description: TODO
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Dep_Motor_Trigger()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

/*
 * Module: 		DEP MOTOR
 * Description: Forward Control of DEP MOTOR
 * Input:  		uint8_t input;
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Dep_Motor_run_Forward(uint8_t input)
{
	if(input == 1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	}
	else if(input == 2){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	}
}

/*
 * Module: 		DEP MOTOR
 * Description: Backward Control of DEP MOTOR
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Dep_Motor_run_Backward(){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

/*
 * Module: 		DEP MOTOR
 * Description: STOP Control of DEP MOTOR
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void Dep_Motor_run_Stop()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}


/*---------------------------------------------------------------------------*/
/****	Control_Fcns	****/
/*
 * Module: 		DEP MOTOR
 * Description: TODO
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/15/2023
 */
void PID_Errors(int16_t* error, int16_t* error_sum, int16_t* error_diff, uint16_t target, uint16_t current)
{
	*error_diff = ((int16_t)target - (int16_t)current) - *error;
	*error = (int16_t)target - (int16_t)current;
	*error_sum += *error;

}
