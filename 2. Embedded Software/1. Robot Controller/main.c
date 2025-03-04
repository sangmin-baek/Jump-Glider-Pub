/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

///*	C Standard libs	*///

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

///*	BSM Custom libs	*///

#include "mpu9250_spi_bsm.h"	//	for MPU-9250	// Scale of the a, g, m need to be tuned
#include "file_handle_bsm.h"	//	for uSD SDIO
#include "motor_drive.h"		//	for Motor drivers
#include "robot_control_fcns.h"		// robot control flow related
#include "jump_motor_controller.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


// Robot Init related Macros //
#define RF_CONNECTING		0x00
#define RF_SET				0x01
#define SENSOR_CONNECTING	0x02
#define SENSOR_SET			0x03
#define ENCODER_SETTING		0x04
#define ROBOT_READY			0x05

#define ROBOT_RUNNING		0x0F

#define USD_DATA_SAVED		0xAB

#define ENERGY_STORING		0x01
#define ROBOT_JUMPING		0x02
#define WING_DEPLOY			0x04
#define LANDING				0x08
#define GLIDING				0x0F
#define REFOLDING			0x1F






/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

//*****	Robot Init related Variables *****//

uint8_t cntr_status = 0;
uint8_t imu_setup = 0;
uint8_t usd_setup = 0;
uint8_t sensors_set = 0;
uint8_t robot_state = 0;

//***** ADC DMA read *****//

uint16_t ADC_raw[2];	// using 12 bit ADC
float angle_conv_coef_a_hj = 1;	// for adc data conversion to the deg
float angle_conv_coef_b_hj = 0;
float angle_conv_coef_a_yaw = 1;
float angle_conv_coef_b_yaw = 0;



//***** IMU related *****//

uint8_t IMU_CALIBRATE = 0;	// IMU calibration flag
int16_t destination[3];		// Not sure need to check (seems like related with reading magnetometer data)



//***** UART related buffs *****//

uint8_t rx_data[10];
uint8_t rx_DMA[10];
uint8_t tx_data[10];
char sprintf_buff[255];	// for serial terminal printf
char sprintf_robot_data[255];



//***** Hand-held Controller CMDs *****//

uint8_t dip_cntr_mode;
uint8_t ADC_Joy[2];
uint8_t ADC_pot;
uint8_t Tact_input;



//***** Robot Data  *****//

struct ROBOT_DATA{
	uint16_t angle_joint;	// deg*10
	uint16_t angle_yaw;		// deg*10

	uint16_t IMU_data[3];	// rpy
	uint16_t IMU_rpy_rate[3];
	uint16_t IMU_rpy_acc[3];
	uint16_t IMU_xyz_rate[3];
	uint16_t IMU_xyz_acc[3];

	uint16_t Encoder_jump_motor;

	uint16_t Motor_input_Jump;
	uint16_t Motor_input_HJ;
} robot_data;



//***** Temp variables *****//

uint16_t i = 0;
uint32_t time_cnt = 0;
uint32_t time_prev = 0;

uint16_t enc_cnt = 0;
uint16_t enc_psc = 0;
uint16_t enc_init = 0;
uint16_t enc_val = 0;

uint8_t timer_flag = 0;



//***** Robot Control variables *****//

uint8_t jump_or_jumpglide = 0;	// jump only = 1, jump gliding = 0;
uint8_t robot_running_state = 0;
uint16_t jumping_motor_energy_stored_enc_data = 3000;

uint8_t jumper_cmd = 0;
uint8_t glider_cmt = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void CHECK_RF_Connection(void);
void CHECK_IMU_Connection(void);
void CHECK_uSD_Connection(void);
void START_Peripherals(void);
void PRE_Processing(void);
void WHILE_Processing(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * Module:		RF
 * Description: CHECK RF Connection
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/16/2023
 */
void CHECK_RF_Connection(void)
{
	while(cntr_status == RF_CONNECTING)
	{
		HAL_UART_Receive(&huart3, rx_data, 1, 100);

		if(rx_data[0] == 0x0F)
		{
			cntr_status = RF_SET;
		}
		else
		{
			tx_data[0] = rx_data[0];
			HAL_UART_Transmit(&huart3, tx_data, 1, 100);
		}

		HAL_Delay(10);
	}

	while(cntr_status == RF_SET)
	{
		HAL_UART_Receive(&huart3, rx_data, 1, 100);
		if(rx_data[0] == 0x0F)
		{
			tx_data[0] = 0x0F;
			HAL_UART_Transmit(&huart3, tx_data, 1, 100);
			cntr_status = SENSOR_CONNECTING;
		}
		else;
		HAL_Delay(10);
	}
}

/*
 * Module:		IMU
 * Description: CHECK IMU Connection
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/16/2023
 */
void CHECK_IMU_Connection(void)
/*{
	init_MPU9250(&hspi1,IMU_CS_GPIO_Out_GPIO_Port, IMU_CS_GPIO_Out_Pin);

	uint8_t MPU_ADD = 0;
	uint8_t AK_ADD = 0;

	while(MPU_ADD != MPU9250_WHOAMI_DEFAULT_VALUE || AK_ADD != AK8963_WHOAMI_DEFAULT_VALUE)
		//while(MPU_ADD != MPU9250_WHOAMI_DEFAULT_VALUE)
	{
		calibrate_MPU9250();
		setup_MPU9250();
		readBytes_SPI(WHO_AM_I_MPU9250,1,&MPU_ADD);
		readBytes_AK8963(AK8963_WHO_AM_I, 1, &AK_ADD);
		HAL_Delay(100);
	}

	if(MPU_ADD == MPU9250_WHOAMI_DEFAULT_VALUE && AK_ADD == AK8963_WHOAMI_DEFAULT_VALUE)
	{
		imu_setup = 1;
	}

	updateAccGyro();

	if(IMU_CALIBRATE)		// IMU_Calibration Process
	{
		sprintf(sprintf_buff,"IMU Ready\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)sprintf_buff, strlen(sprintf_buff), 10);
		HAL_Delay(500);

		sprintf(sprintf_buff,"Calibration Gyro, Don't Move \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)sprintf_buff, strlen(sprintf_buff), 10);
		calibrateGyro();
		sprintf(sprintf_buff,"GyroBias : %7.3f, %7.3f, %7.3f \r\n", gyroBias.x, gyroBias.y, gyroBias.z);
		HAL_UART_Transmit(&huart2, (uint8_t*)sprintf_buff, strlen(sprintf_buff), 10);
		HAL_Delay(500);

		sprintf(sprintf_buff,"Calibration Mag, Trace 8s \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)sprintf_buff, strlen(sprintf_buff), 10);
		calibrateMag();
		sprintf(sprintf_buff,"MagBias : %7.3f, %7.3f, %7.3f \r\n", magBias.x, magBias.y, magBias.z);
		HAL_UART_Transmit(&huart2, (uint8_t*)sprintf_buff, strlen(sprintf_buff), 10);
		sprintf(sprintf_buff,"MagScale : %7.3f, %7.3f, %7.3f \r\n", magScale.x, magScale.y, magScale.z);
		HAL_UART_Transmit(&huart2, (uint8_t*)sprintf_buff, strlen(sprintf_buff), 10);

		HAL_Delay(100000);
	}
	else applyCalibratedVal();

	for(i=0;i<1000;i++)
	{
		filteringAndGetRPY();
		HAL_Delay(1);
	}
	i=0;
}
*/
{
	imu_setup = 1;
}

/*
 * Module:		uSD
 * Description: CHECK uSD Connection
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/16/2023
 */
void CHECK_uSD_Connection(void)
{
/*	while(usd_setup != 1)
	{
		if(Mount_SD(SDPath) == FR_OK)
		{
			if(Create_File() == FR_OK)
			{
				if(Open_File() == FR_OK)
				{
					for(i = 0; i<100; i++){
						sprintf(sprintf_buff,"%d\n",i);
						Write_openedFile(sprintf_buff);
					}
					if(Close_File() == FR_OK)
					{
						usd_setup = 1;
					}
				}
			}
		}
	}*/
	usd_setup = 1;
	sensors_set = imu_setup & usd_setup;
	robot_state |= sensors_set;
}

/*
 * Module:		TIMER
 * Description: INIT TIMER AND START
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/16/2023
 */
void START_Peripherals(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_raw, 2);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim7);
}

/*
 * Module:		PROCESSING
 * Description: PRE PROCESSING
 * 				1. Sensor check
 * 				2. Jumper Encoder set
 * 				3. Robot run start
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/16/2023
 */
void PRE_Processing(void)
{
	i = 0;
	tx_data[0] = robot_state;

	HAL_UART_Receive_DMA(&huart3, rx_DMA, 5);

	while(cntr_status != ROBOT_RUNNING)
	{
		Convert_RX_to_Control(rx_DMA, &dip_cntr_mode, ADC_Joy, &ADC_pot, &Tact_input);

		i++;

		sprintf(sprintf_buff,
		        "[%d] rx : [%x][%x][%x][%x][%x] dip : %x,	ADC_Joy : %x, %x,	ADC_pot : %x,	Tact : %x \r\nTx : %d\nRobot_State: %x\n",
		        i, rx_DMA[0],rx_DMA[1],rx_DMA[2],rx_DMA[3],rx_DMA[4], dip_cntr_mode,
		        ADC_Joy[0], ADC_Joy[1], ADC_pot, Tact_input, tx_data[0], robot_state);

		HAL_UART_Transmit(&huart2, (uint8_t*)sprintf_buff, strlen(sprintf_buff), 10);


		switch (cntr_status) {
			case SENSOR_CONNECTING:
			{
				if(((dip_cntr_mode >> 3) & 1) == 1)
				{
					cntr_status = SENSOR_SET;
					tx_data[0] = robot_state;
				}else;
			}break;

			case SENSOR_SET:
			{
				if(((dip_cntr_mode >> 3) & 1) == 1)
				{
					cntr_status = ENCODER_SETTING;
					tx_data[0] = robot_state;
				}else;
			}break;

			case ENCODER_SETTING:
			{
				if(((dip_cntr_mode >> 3) & 1) == 0)
				{
					//Encoder_init();
					cntr_status = ROBOT_READY;
					robot_state |= (1 << 1);
					tx_data[0] = robot_state;

					Jumper_Motor_PWM_run(0);
					HAL_Delay(100);

					Jumping_Motor_Enc_init();
				}
				else
				{
					Jumper_Motor_PWM_run((float) (ADC_pot - 128) / 128.0f);
				}

			} break;

			case ROBOT_READY:
			{
				if((dip_cntr_mode & 1) == 1)
				{
					//if(Open_File() == FR_OK)						// Open uSD
					{
						robot_state |= (1 << 2);
						tx_data[0] = robot_state;
						cntr_status = ROBOT_RUNNING;

					}//else;
				}else;

			}break;

			default:
				break;
		}

		HAL_UART_Transmit(&huart3, tx_data, 1, 100);
		HAL_Delay(100);
	}
}

/*
 * Module:		PROCESSING
 * Description: WHILE PROCESSING
 * Input:  		/
 * Output: 		/
 * Author: 		BSM
 * Date: 		5/16/2023
 */
void WHILE_Processing(void)
{
	/*********  0. IMU Filtering  *********/
	filteringAndGetRPY();
	if(timer_flag)
	{
		//Write_openedFile(sprintf_robot_data);
		time_cnt++;
		/*********  2. Data Reading  *********/
		sprintf(sprintf_robot_data, "%lu	%.2f	%.2f	%.2f	%d	%d\r\n",
		        time_cnt, yaw*180/M_PI, pitch*180/M_PI, roll*180/M_PI, ADC_raw[0], ADC_raw[1]);

		/***********  1. Timer Cnt  ***********/

		if(time_cnt % 100 == 0)
		{
			Convert_RX_to_Control(rx_DMA, &dip_cntr_mode, ADC_Joy, &ADC_pot, &Tact_input);

			sprintf(sprintf_buff," [%lu] rx : [%x][%x][%x][%x][%x] , Keep getting it\r\n ADC : [%x][%x]\r\n",
				  time_cnt, rx_DMA[0],rx_DMA[1],rx_DMA[2],rx_DMA[3],rx_DMA[4], ADC_raw[0], ADC_raw[1], dip_cntr_mode,
				  ADC_Joy[0], ADC_Joy[1], ADC_pot, Tact_input, tx_data[0], robot_state);

			//HAL_UART_Transmit(&huart2, (uint8_t*)sprintf_buff, strlen(sprintf_buff), 10);
			//HAL_UART_Transmit(&huart3, tx_data, 1, 1);	// Robot State TX 10 HZ
		}

		if(time_cnt % 500 == 0)	// Robot LED Toggle 1 HZ
		{
			HAL_GPIO_TogglePin(LED_GPIO_Out_GPIO_Port, LED_GPIO_Out_Pin);
		}


		/**************************************/
		Jumping_Motor_Control_step(jumper_cmd);

		/********  3. Check DIP state  ********/
		if(((dip_cntr_mode >> 1) & 1) == 0)	// Automatic Motor run
		{
			if(((dip_cntr_mode >> 2) & 1) == 1)	// jumping only
			{
				jump_or_jumpglide = 1;

				switch (robot_running_state)
				{
					case ENERGY_STORING:
					{
						if(((Tact_input >> 3) & 1) == 1)
						{
							jumper_cmd = cmd_jump_trig;
							robot_running_state = ROBOT_JUMPING;
						} else;

					}break;

					case ROBOT_JUMPING:
					{


						if(((Tact_input >> 2) & 1) == 1)
						{
							jumper_cmd = cmd_jump_init;
							robot_running_state = LANDING;
						} else;

					}break;

					case LANDING:
					{


						if(((Tact_input >> 4) & 1) == 1)
						{
							jumper_cmd = cmd_jump_e_store;
							robot_running_state = ENERGY_STORING;
						} else;

					}break;

					default:	// First run
					{
						if(((Tact_input >> 4) & 1) == 1)
						{
							jumper_cmd = cmd_jump_e_store;

							robot_running_state = ENERGY_STORING;
						} else;
					}break;
				}
			}


			/*
			 * To-Do
			 *  PID code
			 */
			else 	// jump-gliding
			{
				jump_or_jumpglide = 0;

				if(((Tact_input >> 4) & 1) == 1)
				{
					jumper_cmd = cmd_jump_e_store;
					robot_running_state = ENERGY_STORING;
				} else;

				if(((Tact_input >> 3) & 1) == 1)
				{
					jumper_cmd = cmd_jump_trig;
					robot_running_state = ROBOT_JUMPING;
				} else;



				if(((Tact_input >> 2) & 1) == 1)
				{
					jumper_cmd = cmd_jump_init;
					robot_running_state = LANDING;
				} else;

				if(((Tact_input >> 1) & 1) == 1)
				{
					Dep_Motor_run_Forward(1);
				}
				else
				{
					Dep_Motor_run_Stop();
				}

				//Hang_Motor_Gliding_Position_Control(&htim4, ADC_raw[0], ADC_pot * 10 + 800); // To -Do
				Hang_Motor_Gliding_Position_Control(&htim4, ADC_raw[1], ADC_pot * 10 + 800); // 230702 adc0 broken
				Yaw_Motor_Control(&htim3, ADC_raw[1], (8 * ADC_Joy[0]) + 900); 	// To -Do
				if(((Tact_input >> 0) & 1) == 1)
				{
					if((time_cnt % 10) == 1)
					{
						Dep_Motor_Folding_Control();
					}
					else
					{
						Dep_Motor_run_Stop();
					}
					Fold_Motor_Folding_Control(((Tact_input >> 0) & 1) + ((Tact_input >> 1) & 1));


				}
				else
				{
					Fold_Motor_run_Stop();
				}
			}
		}
		else	// Manual Motor run
		{

		}
	}

	if((dip_cntr_mode & 1) == 0 && (dip_cntr_mode>>3 & 1) == 1)
	{
		//Motor All Stop
		sprintf(sprintf_robot_data, "EOF [%lu]\r\n", time_cnt);
		Write_openedFile(sprintf_robot_data);

		HAL_TIM_Base_Stop_IT(&htim7);
		if(Close_File() == FR_OK)
		{
			tx_data[0] = USD_DATA_SAVED;
		}
		while(1)
		{
			HAL_UART_Transmit(&huart3, tx_data, 1, 10);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_Delay(100);
		}
	}
	timer_flag = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */


	/******************************************************************************/
	/* [1. Check RF connection] */

	CHECK_RF_Connection();

	/******************************************************************************/
	/* [2-1. IMU connection] */

	CHECK_IMU_Connection();

	/******************************************************************************/
	/* [2-2. uSD connection] */

	CHECK_uSD_Connection();

	/******************************************************************************/
	/* [2-3. Start peripherals] */
	START_Peripherals();

	/* [2-4. Start peripherals] */
	Jump_Motor_Init(&htim2, &htim1);
	/******************************************************************************/
	/* [3. Pre-processing] */

	PRE_Processing();
	/******************************************************************************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		WHILE_Processing();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 34943;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 44;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Motor_Dep_A_GPIO_Out_Pin|Motor_Dep_B_GPIO_Out_Pin|Motor_Fold_A_GPIO_Out_Pin|Motor_Fold_B_GPIO_Out_Pin
                          |IMU_CS_GPIO_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Out_GPIO_Port, LED_GPIO_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Motor_Dep_A_GPIO_Out_Pin Motor_Dep_B_GPIO_Out_Pin Motor_Fold_A_GPIO_Out_Pin Motor_Fold_B_GPIO_Out_Pin */
  GPIO_InitStruct.Pin = Motor_Dep_A_GPIO_Out_Pin|Motor_Dep_B_GPIO_Out_Pin|Motor_Fold_A_GPIO_Out_Pin|Motor_Fold_B_GPIO_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_GPIO_Out_Pin */
  GPIO_InitStruct.Pin = IMU_CS_GPIO_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IMU_CS_GPIO_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_CD_Pin */
  GPIO_InitStruct.Pin = SDIO_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDIO_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GPIO_Out_Pin */
  GPIO_InitStruct.Pin = LED_GPIO_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Out_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//
//	if((time_cnt % 100 == 0) && (huart == &huart3))
////	if((huart == &huart3))
//	{
//		if(HAL_UART_Transmit(&huart3, tx_data, 1, 10) == HAL_OK)
//	//	if(HAL_UART_Transmit_DMA(&huart3, tx_data, 1) == HAL_OK)
//		{
//			sprintf(sprintf_buff,"Send Data : %d\n", tx_data[0]);
//			//HAL_UART_Transmit(&huart2, (uint8_t*)sprintf_buff, strlen(sprintf_buff), 10);		TEST 1201
//			//HAL_UART_Receive_DMA(&huart3, rx_DMA, 5);
//
//		}
//		else
//		{
//			Error_Handler();
//		}
//
//	}
//	else;
//
//	HAL_UART_Receive_DMA(&huart3, rx_DMA, 5);
//}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim7)
	{
		timer_flag = 1;
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
