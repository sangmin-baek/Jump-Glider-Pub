/*
 * robot_control_fcns.h
 *
 *  Created on: 2022. 9. 19.
 *      Author: BSM
 */

#ifndef INC_ROBOT_CONTROL_FCNS_H_
#define INC_ROBOT_CONTROL_FCNS_H_

#include "stm32f4xx_hal.h"


extern void Convert_RX_to_Control(uint8_t* rx_data, uint8_t* cntr_mode, uint8_t* ADC_Joy, uint8_t* ADC_pot, uint8_t* Tact_input);

#endif /* INC_ROBOT_CONTROL_FCNS_H_ */
