/*
 * control_fcns.c
 *
 *  Created on: Sep 19, 2022
 *      Author: bsm6656
 */
#include "control_fcns.h"
#include "main.h"


#define SET_UP		0xff
#define NON_SET_UP	0x00
#define JUMP_STEP    (32767u)
#define BUFFER_SIZE  (10u)

uint8_t dip_prev[3];
uint8_t tact_prev[3];

struct LOWPASS_FILTER_ lowpass_filter;

int Initialize_robot(UART_HandleTypeDef *huart)
{
	uint8_t start_code = 0xFF;
	uint8_t recieve_code = 0;;

	while(recieve_code == 0)
	{
		HAL_UART_Transmit(huart, &start_code, 1, 100);
		HAL_UART_Receive(huart, &recieve_code, 1, 100);
	}

		if(recieve_code == 1)
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

			return 0;

		}
		else if(recieve_code == 2)
		{
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

			return 0;
		}
		else if(recieve_code == 3)
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

			while(HAL_GPIO_ReadPin(SW_DIP_1_Input_GPIO_Port, SW_DIP_1_Input_Pin))
			{
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
				HAL_Delay(500);
			}
			return 1;

		}
		else return 0;



}

/*
 * Module:		FILTER
 * Description: LOW PASS FILTER Initialization
 * Input:  		/
 * Output: 		/
 * Author: 		HMS
 * Date: 		5/15/2023
 */
void LowPassFilter_Init(void)
{
	lowpass_filter.flag = 0;
	lowpass_filter.alpha = 0.2;
	lowpass_filter.out_last[0] = 0;
	lowpass_filter.out_last[1] = 0;
	lowpass_filter.out_last[2] = 0;

}

/*
 * Module:		FILTER
 * Description: Update the switch value and ADC value
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/16/2023
 */
void Update_Data_State(uint16_t* ADC_Data, uint16_t*  ADC_Filt_data)
{
	Check_SW();
	ADC_LowPassFilter_Filtering(ADC_Data, ADC_Filt_data);
}

/*
 * Module:		FILTER
 * Description: CHECK SWITCH OUTPUT VALUE
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/16/2023
 */
void Check_SW()
{
	uint8_t i = 0;

	dip_prev[0] = dip_prev[1];
	dip_prev[1] = dip_prev[2];
	*(dip_prev + 2) = NON_SET_UP << 4;
	*(dip_prev + 2) |= (!HAL_GPIO_ReadPin(SW_DIP_1_Input_GPIO_Port, SW_DIP_1_Input_Pin)) << 3;
	*(dip_prev + 2) |= (!HAL_GPIO_ReadPin(SW_DIP_2_Input_GPIO_Port, SW_DIP_2_Input_Pin)) << 2;
	*(dip_prev + 2) |= (!HAL_GPIO_ReadPin(SW_DIP_3_Input_GPIO_Port, SW_DIP_3_Input_Pin)) << 1;
	*(dip_prev + 2) |= (!HAL_GPIO_ReadPin(SW_DIP_4_Input_GPIO_Port, SW_DIP_4_Input_Pin));

	tact_prev[0] = tact_prev[1];
	tact_prev[1] = tact_prev[2];
	*(tact_prev + 2) = 0;
	*(tact_prev + 2) |= (!HAL_GPIO_ReadPin(SW1_EXTI9_GPIO_Port, SW1_EXTI9_Pin)) << 4;
	*(tact_prev + 2) |= (!HAL_GPIO_ReadPin(SW2_EXTI8_GPIO_Port, SW2_EXTI8_Pin)) << 3;
	*(tact_prev + 2) |= (!HAL_GPIO_ReadPin(SW3_EXTI7_GPIO_Port, SW3_EXTI7_Pin)) << 2;
	*(tact_prev + 2) |= (!HAL_GPIO_ReadPin(SW4_EXTI6_GPIO_Port, SW4_EXTI6_Pin)) << 1;
	*(tact_prev + 2) |= (!HAL_GPIO_ReadPin(SW5_EXTI5_GPIO_Port, SW5_EXTI5_Pin));

}
/*
 * Module:		FILTER
 * Description: LOW PASS FILTERING FOR ADC VALUE
 * Input:  		uint16_t* ADC_data;
 * 				uint16_t*  ADC_Filt_data;
 * Output: 		/
 * Author: 		HMS
 * Date: 		5/16/2023
 */
void ADC_LowPassFilter_Filtering(uint16_t* ADC_Data, uint16_t*  ADC_Filt_data)
{
	if(lowpass_filter.flag == 0)
	{
		lowpass_filter.flag = 1;
		*(lowpass_filter.out_last) = *ADC_Data;
		*(lowpass_filter.out_last+1) = *(ADC_Data + 1);
		*(lowpass_filter.out_last+2) = *(ADC_Data + 2);

	}
	*ADC_Filt_data = *(lowpass_filter.out_last) + lowpass_filter.alpha*(*ADC_Data-*(lowpass_filter.out_last));
	*(ADC_Filt_data+1) = *(lowpass_filter.out_last+1) + lowpass_filter.alpha*(*(ADC_Data + 1)-*(lowpass_filter.out_last+1));
	*(ADC_Filt_data+2) = *(lowpass_filter.out_last+2) + lowpass_filter.alpha*(*(ADC_Data + 2)-*(lowpass_filter.out_last+2));

	*lowpass_filter.out_last = *ADC_Filt_data;
	*(lowpass_filter.out_last+1) = *(ADC_Filt_data+1);
	*(lowpass_filter.out_last+2) = *(ADC_Filt_data+2);

}



void Read_Filtered_State(uint8_t * state_reg, uint16_t*  ADC_Filt_data)
{
	*state_reg = (dip_prev[1] == dip_prev[2]) ? ((dip_prev[0] == dip_prev[1]) ? (dip_prev[2]):(*state_reg) ): (*state_reg);

	*(state_reg + 4) = (tact_prev[1] == tact_prev[2]) ? ((tact_prev[0] == tact_prev[1]) ? (tact_prev[2]):(*(state_reg + 4)) ): (*(state_reg + 4));

	*(state_reg + 1) = *(ADC_Filt_data + 1) / 16;
	*(state_reg + 2) = *ADC_Filt_data / 16;
	*(state_reg + 3) = (*(ADC_Filt_data + 2) / 16);
}

void Read_State(uint8_t * state_reg, uint16_t* ADC_data)
{
	/* Reading 1st byte : DIP state */
		/*
		 * 	DIP 1 : Initialize
		 * 	DIP 2 : Jumping / Jump-Gliding Mode
		 * 	DIP 3 : Manual / Auto / ML Mode
		 * 	DIP 4 : Save file
		 */
		*state_reg = NON_SET_UP << 4;
		*state_reg |= (!HAL_GPIO_ReadPin(SW_DIP_1_Input_GPIO_Port, SW_DIP_1_Input_Pin)) << 3;
		*state_reg |= (!HAL_GPIO_ReadPin(SW_DIP_2_Input_GPIO_Port, SW_DIP_2_Input_Pin)) << 2;
		*state_reg |= (!HAL_GPIO_ReadPin(SW_DIP_3_Input_GPIO_Port, SW_DIP_3_Input_Pin)) << 1;
		*state_reg |= (!HAL_GPIO_ReadPin(SW_DIP_4_Input_GPIO_Port, SW_DIP_4_Input_Pin));



		/* Reading 2nd byte : Joy-x state */
		/*
		 *	Joy-x : Yaw motor
		 */
		//*(state_reg + 1) = 255 - (*ADC_data / 16);
		*(state_reg + 1) = *(ADC_data + 1) / 16;



		/* Reading 3rd byte : Joy-y state */
		/*
		 * 	Joy-y : Pitch motor
		 */
		//*(state_reg + 2) = 255 - (*(ADC_data + 1) / 16);
		*(state_reg + 2) = *ADC_data / 16;



		/* Reading 4th byte : Pot state */
		/*
		 * 	Pot : Jumper motor
		 */
		//*(state_reg + 3) = 255 - (*(ADC_data + 2) / 16);
		*(state_reg + 3) = (*(ADC_data + 2) / 16);



		/* Reading 5th byte : Tack_SW state */
		/*
		 *  Tact 1 : E store
		 *  Tact 2 : E trigger
		 *  Tact 3 : Landing
		 *  Tact 4 : Deploy
		 *  Tact 5 : Fold
		 */
		*(state_reg + 4) = 0;
		*(state_reg + 4) |= (!HAL_GPIO_ReadPin(SW1_EXTI9_GPIO_Port, SW1_EXTI9_Pin)) << 4;
		*(state_reg + 4) |= (!HAL_GPIO_ReadPin(SW2_EXTI8_GPIO_Port, SW2_EXTI8_Pin)) << 3;
		*(state_reg + 4) |= (!HAL_GPIO_ReadPin(SW3_EXTI7_GPIO_Port, SW3_EXTI7_Pin)) << 2;
		*(state_reg + 4) |= (!HAL_GPIO_ReadPin(SW4_EXTI6_GPIO_Port, SW4_EXTI6_Pin)) << 1;
		*(state_reg + 4) |= (!HAL_GPIO_ReadPin(SW5_EXTI5_GPIO_Port, SW5_EXTI5_Pin));


}

int Jumper_init(UART_HandleTypeDef* huart, uint8_t* state_reg, uint16_t* ADC_data)
{
	uint8_t tx_data = 0;
	uint8_t rx_code = 0;

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);


	while(!HAL_GPIO_ReadPin(SW_DIP_1_Input_GPIO_Port, SW_DIP_1_Input_Pin))
	{
		Read_State(state_reg, ADC_data);
		HAL_UART_Transmit(huart, state_reg, sizeof(state_reg), 100);
		HAL_Delay(10);
	}

	state_reg[0] |= (SET_UP << 4);

	while(rx_code == 0)
	{
		HAL_UART_Transmit(huart, state_reg, sizeof(state_reg), 100);
		HAL_UART_Receive(huart, &rx_code, 1, 100);
	}

	if(rx_code == 0xFF)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		return 2;
	}

}
