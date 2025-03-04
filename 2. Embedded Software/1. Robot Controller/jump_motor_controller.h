/*
 * jump_motor_controller.h
 *
 *  Created on: 2023. 3. 16.
 *      Author: BSM
 */

#ifndef INC_JUMP_MOTOR_CONTROLLER_H_
#define INC_JUMP_MOTOR_CONTROLLER_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define jump_motor_smp_tim          (0.001)

#define jump_motor_k_t              (0.006)
#define jump_motor_l_a            (0.00011)
#define jump_motor_r_a                (5.1)
#define jump_motor_gear_eff           (0.6)
#define gain_load                  (0.0305)

#define rad2deg                   (57.2958)

#define D_N0                            (0)
#define D_N1                       (0.1711)
#define D_N2                       (0.0071)

#define D_D0                            (1)
#define D_D1                      (-0.8970)
#define D_D2                   (4.0471E-11)

#define jump_motor_voltage_sat          (1)

#define jump_motor_e_store_position   (200)
#define energy_release_delta           (30)
#define jump_motor_init_delta         (135)


#define jump_motor_gear_ratio         (4315)
#define jump_motor_vmax                 (6)


#define cmd_jump_e_store                (1)
#define cmd_jump_trig                   (2)
#define cmd_jump_init                   (3)

struct JUMP_MOTOR_{

	TIM_HandleTypeDef* htim_motor;
	TIM_HandleTypeDef* htim_encoder;

	uint16_t jump_enc_init;
	uint16_t jump_motor_position;
	uint16_t jump_motor_speed;

};

extern struct JUMP_MOTOR_ jump_motor;


extern void Jumping_Motor_Control_step(uint8_t jump_motor_norm_input);

extern void Jumper_Motor_PWM_run(float jump_motor_norm_input);
extern void Jumping_Motor_Enc_read();
extern void Jumping_Motor_Enc_init();
extern void Jump_Motor_Init(TIM_HandleTypeDef* htim_motor, TIM_HandleTypeDef* htim_encoder);

#endif /* INC_JUMP_MOTOR_CONTROLLER_H_ */
