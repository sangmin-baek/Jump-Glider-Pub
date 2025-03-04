/*
 * motor_drive.h
 *
 *  Created on: Aug 10, 2022
 *      Author: BSM
 */

#ifndef INC_MOTOR_DRIVE_H_
#define INC_MOTOR_DRIVE_H_

#include "stm32f4xx_hal.h"
#include "main.h"

extern void Motor_Sys_ID_Step_Input(TIM_HandleTypeDef* htim_motor, uint16_t enc_init, uint16_t input_magnitude);
extern void Hang_Motor_Sys_ID_Step_Input(TIM_HandleTypeDef* htim, uint16_t adc_angle, int32_t input_magnitude);


extern void Jump_Motor_Gliding_Position_Control(TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_encoder, uint16_t energy_stored_state, uint16_t enc_init);
extern void Jump_Motor_Landing_Position_Control(TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_encoder, uint16_t energy_stored_state, uint16_t enc_init);
extern void Jump_Motor_Trigger_Position_Control(TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_encoder, uint16_t energy_stored_state, uint16_t enc_init);
extern void Jump_Motor_Energy_Store_Position_Control(TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_encoder, uint16_t energy_stored_state, uint16_t enc_init);

extern int Jump_Motor_Enc_Shifted(TIM_HandleTypeDef* htim, uint16_t encoder_init);
extern void Jump_Motor_Enc_read_raw(TIM_HandleTypeDef* htim, uint16_t* encoder_raw_cnt);

extern void Jump_Motor_PWM_run(TIM_HandleTypeDef* htim, uint8_t input);
extern void Jump_Motor_PWM_Stop(TIM_HandleTypeDef* htim);




extern void Yaw_Motor_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target);
extern void Yaw_Motor_PWM_run(TIM_HandleTypeDef* htim, uint8_t input);
extern void Yaw_Motor_PWM_Stop(TIM_HandleTypeDef* htim);



extern void Hang_Motor_Landing_In_Gliding_Position_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target);
extern void Hang_Motor_Gliding_Position_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target);
extern void Hang_Motor_Jumping_In_Gliding_Position_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target);

extern void Hang_Motor_Landing_Position_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target);
extern void Hang_Motor_Jumping_Position_Control(TIM_HandleTypeDef* htim, uint16_t adc_angle, uint16_t target);
extern void Hang_Motor_PWM_run(TIM_HandleTypeDef* htim, uint8_t input);
extern void Hang_Motor_PWM_Stop(TIM_HandleTypeDef* htim);



extern void Fold_Motor_Folding_Control(uint8_t direction);

extern void Fold_Motor_run_Forward(uint8_t input);
extern void Fold_Motor_run_Stop();

extern void Dep_Motor_Folding_Control();
extern void Dep_Motor_Trigger();
extern void Dep_Motor_run_Forward(uint8_t input);
extern void Dep_Motor_run_Stop();


extern void PID_Errors(int16_t* error, int16_t* error_sum, int16_t* error_diff, uint16_t target, uint16_t current);

#endif /* INC_MOTOR_DRIVE_H_ */
