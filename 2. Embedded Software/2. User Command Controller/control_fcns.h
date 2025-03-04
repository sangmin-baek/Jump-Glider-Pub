/*
 * control_fcns.h
 *
 *  Created on: Sep 19, 2022
 *      Author: bsm6656
 */

#ifndef INC_CONTROL_FCNS_H_
#define INC_CONTROL_FCNS_H_

#include "stm32f4xx_hal.h"
#include "main.h"
// PARAMETER

struct LOWPASS_FILTER_{
	float alpha;
	uint8_t	flag;
	uint16_t out_last[3];
};

extern uint8_t dip_prev[3];
extern uint8_t tact_prev[3];

// EXTERN FUNCTION
extern int 	Initialize_robot(UART_HandleTypeDef *huart);
extern void LowPassFilter_Init(void);
extern void LowPassFilter_Filtering(uint8_t* user_input_raw, uint32_t*  ADC_Filt_data);
extern void Read_State(uint8_t * state_reg, uint16_t* ADC_data);
extern int 	Jumper_init(UART_HandleTypeDef* huart, uint8_t* state_reg, uint16_t* ADC_data);
void Check_SW();
void ADC_LowPassFilter_Filtering(uint16_t* ADC_Data, uint16_t*  ADC_Filt_data);
void Update_Data_State(uint16_t* ADC_Data, uint16_t*  ADC_Filt_data);
void Read_Filtered_State(uint8_t * state_reg, uint16_t*  ADC_Filt_data);

#endif /* INC_CONTROL_FCNS_H_ */
