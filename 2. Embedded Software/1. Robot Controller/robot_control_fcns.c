/*
 * robot_control_fcns.c
 *
 *  Created on: 2022. 9. 19.
 *      Author: BSM
 */

#include "robot_control_fcns.h"

void Convert_RX_to_Control(uint8_t* rx_data, uint8_t* cntr_mode, uint8_t* ADC_Joy, uint8_t* ADC_pot, uint8_t* Tact_input)
{
	*cntr_mode = rx_data[0];
	*ADC_Joy = rx_data[1];
	*(ADC_Joy + 1) = rx_data[2];
	*ADC_pot = rx_data[3];
	*Tact_input = rx_data[4];
}

void Encoder_init()
{

}

void Encoder_setting_jumper_cntr()
{

}
