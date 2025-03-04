/*
 * mpu9250_spi_bsm.c
 *
 *  Created on: May 26, 2022
 *      Author: bsm6656
 */
#include "mpu9250_spi_bsm.h"
#include <math.h>

struct _MPU9250 _mpu9250;
float aRes, gRes, mRes;
enum GyroRange _gyroRange;
enum AccelRange _accelRange;
enum DlpfBandwidth _bandwidth;
enum MagnBits _magBits;
enum MagnMode _magMode;
enum SampleRate _smpRate;

struct Vect3 a;
struct Vect3 g;
struct Vect3 m;
struct Vect3 magCalibration;
struct Vect3 magBias;
struct Vect3 magScale;
struct Vect3 gyroBias;
struct Vect3 accelBias;
uint16_t DataCnt=0;
uint16_t drdy, datacheck, cntcheck;
int16_t MPU9250Data[12];

float q[4];
float roll, pitch, yaw;
float freq = 1000;


//float mag_declination;


void init_MPU9250(SPI_HandleTypeDef *hspi, GPIO_TypeDef* csPort, uint16_t csPin){
	_mpu9250.hspi = hspi;
	_mpu9250.csPort = csPort;
	_mpu9250.csPin = csPin;

	magCalibration.x = 0;
	magCalibration.y = 0;
	magCalibration.z = 0;

	magBias.x = 0;
	magBias.y = 0;
	magBias.z = 0;

	magScale.x = 1;
	magScale.y = 1;
	magScale.z = 1;

	gyroBias.x = 0;
	gyroBias.y = 0;
	gyroBias.z = 0;

	accelBias.x = 0;
	accelBias.y = 0;
	accelBias.z = 0;

	q[0] = 1.0f;
	q[1] = 0.0f;
	q[2] = 0.0f;
	q[3] = 0.0f;

}

void filteringAndGetRPY(){
	updateAccGyro();
	madgwickFilterMARG(a.x, a.y, a.z, g.x, g.y, g.z, m.y, m.x, m.z, q, freq);
	quaternionToYPR_zyx();

}

void quaternionToYPR_zyx(){	// rotation order : yaw(z) -> pitch(y) -> roll(x)
	float r_11, r_12, r_21, r_13, r_31, r_23, r_32, r_33;
	/*
	 * R	 = [ cos(p)cos(y)							cos(p)sin(y)					-sin(p);
	 * 			sin(r)sin(p)cos(y)-cos(r)sin(y)		sin(r)sin(p)cos(y)+cos(r)cos(y)		cos(p)sin(r);
	 * 			cos(r)sin(p)cos(y)+sin(r)sin(y)		cos(r)sin(p)sin(y)-sin(r)cos(y)		cos(p)cos(r);]
	 *
	 * R(q)	 = [q0^2 + q1^2 - q2^2 - q3^2		2(q1q2 + q0q3)				2(q1q3 - q0q2);
	 * 			2(q1q2 - q0q3)				q0^2 - q1^2 + q2^2 - q3^2		2(q2q3 + q0q1);
	 * 			2(q0q2 + q1q3)					2(q2q3 - q0q1)			q0^2 - q1^2 - q2^2 + q3^2]
	 */
	r_11 = 2.0f * (q[0]*q[0] + q[1]*q[1]) - 1;
	r_12 = 2.0f * (q[1]*q[2] + q[0]*q[3]);
	r_21 = 2.0f * (q[1]*q[2] - q[0]*q[3]);

	r_13 = 2.0f * (q[1]*q[3] - q[0]*q[2]);
	r_31 = 2.0f * (q[1]*q[3] + q[0]*q[2]);

	r_23 = 2.0f * (q[2]*q[3] + q[0]*q[1]);
	r_32 = 2.0f * (q[2]*q[3] - q[0]*q[1]);
	r_33 = 2.0f * (q[0]*q[0] + q[3]*q[3]) - 1;

	roll = atan2f(r_23,r_33);
	pitch = - asinf(r_13);
	yaw = atan2f(r_12,r_11);

	yaw += magDeclination*DEG_TO_RAD;


	yaw = ((yaw > M_PI) ? (yaw - TWO_PI) : ((yaw < - M_PI) ? (yaw + TWO_PI) : yaw));
}

void applyCalibratedVal(){

//	gyroBias.x = 0.232;
//	gyroBias.y = 1.211;
//	gyroBias.z = 0.354;

	magBias.x = 3675.777;
	magBias.y = 1844.056;
	magBias.z = -50.604;

	magScale.x = 0.514;
	magScale.y = 1.159;
	magScale.z = 5.220;

}

void calibrateMag(){
	uint16_t i=0;
	uint16_t Samples = 1500;
	int32_t mag_bias_tmp[3] = {0, 0, 0};
	int32_t mag_scale_tmp[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767};
	int16_t mag_min[3] = {32767, 32767, 32767};
	int16_t mag_tmp[3] = {0, 0, 0};
	int16_t MPU9250Data[12];

	HAL_Delay(3000);

	/*
	char printf_buff[256];
	sprintf(printf_buff,"Draw multiple 8s \r\n");
	HAL_UART_Transmit(&huart2, (uint8_t)printf_buff, strlen(printf_buff), 20);
	 */
	for(i=0; i<Samples; i++){

		readMag_Call();
		HAL_Delay(15);

		readMPU9250Data(MPU9250Data);

		mag_tmp[0] = MPU9250Data[8];
		mag_tmp[1] = MPU9250Data[9];
		mag_tmp[2] = MPU9250Data[10];

		for(uint8_t j=0; j<3; j++){
			if(mag_tmp[j] > mag_max[j]){
				mag_max[j] = mag_tmp[j];
			}
			if(mag_tmp[j] < mag_min[j]){
				mag_min[j] = mag_tmp[j];
			}
		}
	}
	mag_bias_tmp[0] = (mag_max[0] + mag_min[0]) / 2;
	mag_bias_tmp[1] = (mag_max[1] + mag_min[1]) / 2;
	mag_bias_tmp[2] = (mag_max[2] + mag_min[2]) / 2;

	magBias.x = (float) mag_bias_tmp[0] * mRes * magCalibration.x;
	magBias.y = (float) mag_bias_tmp[1] * mRes * magCalibration.y;
	magBias.z = (float) mag_bias_tmp[2] * mRes * magCalibration.z;


	mag_scale_tmp[0] = (mag_max[0] - mag_min[0]) / 2;
	mag_scale_tmp[1] = (mag_max[1] - mag_min[1]) / 2;
	mag_scale_tmp[2] = (mag_max[2] - mag_min[2]) / 2;

	float avg_scale = mag_scale_tmp[0] + mag_scale_tmp[1] + mag_scale_tmp[2];
	avg_scale /= 3.0;

	magScale.x = avg_scale / ((float)mag_scale_tmp[0]);
	magScale.y = avg_scale / ((float)mag_scale_tmp[1]);
	magScale.z = avg_scale / ((float)mag_scale_tmp[2]);


}

void calibrateGyro(){
	uint8_t i=0;
	uint8_t Samples = 100;
	struct Vect3 gyroSum;
	gyroSum.x = 0;
	gyroSum.y = 0;
	gyroSum.z = 0;
	HAL_Delay(100);
	/*
	char printf_buff[256];
	sprintf(printf_buff,"Do not move\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t)printf_buff, strlen(printf_buff), 20);
	*/
	for(i=0; i<Samples; i++){
		updateAccGyro();
		gyroSum.x += g.x;
		gyroSum.y += g.y;
		gyroSum.z += g.z;
		HAL_Delay(20);
	}
	gyroSum.x = gyroSum.x / ((float)Samples);
	gyroSum.y = gyroSum.y / ((float)Samples);
	gyroSum.z = gyroSum.z / ((float)Samples);

	gyroBias = gyroSum;

}

void calibrate_MPU9250()
{
	int32_t gyro_bias[3] = {0};
	int32_t accel_bias[3] = {0};

	uint8_t MPU9250Data[12];
	uint16_t fifo_cnt = 0;
	uint16_t i = 0;
	uint16_t set_cnt = 0;


	// Reset device, reset all registers, clear all
	writeByte_SPI(PWR_MGMT_1, PWR_RESET);	// [0x6B] set [0x80] Toggle reset device
	HAL_Delay(100);

	// set clock source
	writeByte_SPI(PWR_MGMT_1, CLOCK_SEL_PLL); 	// [0x6B] set [0x01]  // Select clock source in PWR_MGMT_1 reg.
	writeByte_SPI(PWR_MGMT_2, SEN_ENABLE);		// [0x6C] set [0x00] // xyz accel & gyro on
	HAL_Delay(200);

	// Configure device for bias calculation
	writeByte_SPI(INT_ENABLE, 0x00);	// Disable all interrupts
	writeByte_SPI(FIFO_EN, 0x00);		// Disable FIFO
	writeByte_SPI(PWR_MGMT_1, 0x00);	// Turn on internal clock source
	writeByte_SPI(I2C_MST_CTRL, 0x00);	// Disable I2C master
	writeByte_SPI(USER_CTRL, 0x00);		// Disable FIFO and I2C master modes
	writeByte_SPI(USER_CTRL, 0x0C);		// Reset FIFO and DMP
	HAL_Delay(15);

	// Configure MPU9250 gyro & accelerometer for bias calculation
	writeByte_SPI(MPU_CONFIG, 0x01);	// Set DLP to 188 Hz
	writeByte_SPI(SMPLRT_DIV, 0x00);	// Set SMPRT to 1 kHz
	writeByte_SPI(GYRO_CONFIG, 0x00);	// Set gyro full-scale to 250 deg/s, maximum sensitivity
	writeByte_SPI(ACCEL_CONFIG, 0x00);	// Set accel full-scale to 2 g, maximum sensitivity

	uint16_t gyrosensitivity = 131;		// 2^16 LSB / (500 Deg/sec)	(+-250)
	uint16_t accelsensitivity = 16384;	// 2^16 LSB / (4 g)			(+- 2)


	for(i = 0; i < 1000; i++)
	{
		filteringAndGetRPY();
		HAL_Delay(2);
	}
	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte_SPI(USER_CTRL, 0x40);		// Enable FIFO
	writeByte_SPI(FIFO_EN, 0x78);	// Enable gyro & accel for FIFO (max size 512 bytes in MPU-9250)
	HAL_Delay(40);		// Data collection

	// Turn off FIFO
	writeByte_SPI(FIFO_EN, 0x00);
	readBytes_SPI(FIFO_COUNTH, 2, MPU9250Data);	// Read FIFO sample count
	fifo_cnt = ((uint16_t)MPU9250Data[0] << 8) | MPU9250Data[1];
	set_cnt = fifo_cnt / 12;		// Number of data sets

	for(i = 0; i < set_cnt; i++)
	{
		int16_t acc_tmp[3] = {0};
		int16_t gyro_tmp[3] = {0};

		readBytes_SPI(FIFO_R_W, 12, MPU9250Data);		// Read Sensor data

		acc_tmp[0] = (int16_t)(((int16_t)MPU9250Data[0] << 8) | MPU9250Data[1]);	// signed int
		acc_tmp[1] = (int16_t)(((int16_t)MPU9250Data[2] << 8) | MPU9250Data[3]);
		acc_tmp[2] = (int16_t)(((int16_t)MPU9250Data[4] << 8) | MPU9250Data[5]);

		gyro_tmp[0] = (int16_t)(((int16_t)MPU9250Data[6] << 8) | MPU9250Data[7]);	// signed int
		gyro_tmp[1] = (int16_t)(((int16_t)MPU9250Data[8] << 8) | MPU9250Data[9]);
		gyro_tmp[2] = (int16_t)(((int16_t)MPU9250Data[10] << 8) | MPU9250Data[11]);

		accel_bias[0] += (int32_t) acc_tmp[0];
		accel_bias[1] += (int32_t) acc_tmp[1];
		accel_bias[2] += (int32_t) acc_tmp[2];

		gyro_bias[0] += (int32_t) gyro_tmp[0];
		gyro_bias[1] += (int32_t) gyro_tmp[1];
		gyro_bias[2] += (int32_t) gyro_tmp[2];

	}
	accel_bias[0] /= (int32_t) set_cnt;
	accel_bias[1] /= (int32_t) set_cnt;
	accel_bias[2] /= (int32_t) set_cnt;

	gyro_bias[0] /= (int32_t) set_cnt;
	gyro_bias[1] /= (int32_t) set_cnt;
	gyro_bias[2] /= (int32_t) set_cnt;


	// Compensate gravity
	if(accel_bias[2] > 0L)
	{
		accel_bias[2] -= (int32_t) accelsensitivity;
	}
	else
	{
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	gyroBias.x = (float) gyro_bias[0] /(float) gyrosensitivity;
	gyroBias.y = (float) gyro_bias[1] /(float) gyrosensitivity;
	gyroBias.z = (float) gyro_bias[2] /(float) gyrosensitivity;


	accelBias.x = (float) accel_bias[0] /(float) accelsensitivity;
	accelBias.y = (float) accel_bias[1] /(float) accelsensitivity;
	accelBias.z = (float) accel_bias[2] /(float) accelsensitivity;
}

void updateAccGyro(){
	/*
	int16_t MPU9250Data[7];
	readMPU9250Data(MPU9250Data);
	a.x = (float)MPU9250Data[0] * aRes - accelBias.x;
	a.y = (float)MPU9250Data[1] * aRes - accelBias.y;
	a.z = (float)MPU9250Data[2] * aRes - accelBias.z;
	g.x = (float)MPU9250Data[4] * gRes - gyroBias.x;
	g.y = (float)MPU9250Data[5] * gRes - gyroBias.y;
	g.z = (float)MPU9250Data[6] * gRes - gyroBias.z;
	*/
	int16_t MPU9250Data[12];
	readMPU9250Data(MPU9250Data);
	a.x = (float)MPU9250Data[0] * aRes - accelBias.x;
	a.y = (float)MPU9250Data[1] * aRes - accelBias.y;
	a.z = (float)MPU9250Data[2] * aRes - accelBias.z;

	g.x = (float)MPU9250Data[4] * gRes - gyroBias.x;
	g.y = (float)MPU9250Data[5] * gRes - gyroBias.y;
	g.z = (float)MPU9250Data[6] * gRes - gyroBias.z;

	if(MPU9250Data[7] & 0x01){
		if(!(MPU9250Data[11] & 0x08)){
			m.x = (float)(MPU9250Data[8] * mRes * magCalibration.x - magBias.x) * magScale.x;
			m.y = (float)(MPU9250Data[9] * mRes * magCalibration.y - magBias.y) * magScale.y;
			m.z = (float)(MPU9250Data[10] * mRes * magCalibration.z - magBias.z) * magScale.z;
			cntcheck++;
		}
	}
	/*
	m.x = (float)(MPU9250Data[8] * mRes * magCalibration.x - magBias.x) * magScale.x;
	m.y = (float)(MPU9250Data[9] * mRes * magCalibration.y - magBias.y) * magScale.y;
	m.z = (float)(MPU9250Data[10] * mRes * magCalibration.z - magBias.z) * magScale.z;
	*/
}

void readMPU9250Data(int16_t* destination){
	/*
	uint8_t rawData[14];
	readBytes_SPI(ACCEL_XOUT_H, 14, rawData);
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];	// Acc_x
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];	// Acc_y
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];	// Acc_z
	destination[3] = ((int16_t)rawData[6] << 8) | rawData[7];	// Temperature
	destination[4] = ((int16_t)rawData[8] << 8) | rawData[9];	// Gy_x
	destination[5] = ((int16_t)rawData[10] << 8) | rawData[11];	// Gy_y
	destination[6] = ((int16_t)rawData[12] << 8) | rawData[13];	// Gy_z
	*/


	uint8_t rawData[22];
	//readMag_Call();

	if(DataCnt%1 == 0){
		readMag_Call();
	}
	DataCnt++;

	readBytes_SPI(ACCEL_XOUT_H, 15, rawData);
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];	// Acc_x
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];	// Acc_y
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];	// Acc_z

	destination[3] = ((int16_t)rawData[6] << 8) | rawData[7];	// Temperature

	destination[4] = ((int16_t)rawData[8] << 8) | rawData[9];	// Gy_x
	destination[5] = ((int16_t)rawData[10] << 8) | rawData[11];	// Gy_y
	destination[6] = ((int16_t)rawData[12] << 8) | rawData[13];	// Gy_z

	destination[7] = rawData[14];
	drdy = destination[7];
	if(destination[7] == 0x01){
		readBytes_SPI(ACCEL_XOUT_H+15, 7, rawData+15);
		destination[11] = rawData[21];
		datacheck = destination[11];

		destination[8] = ((int16_t)rawData[16] << 8) | rawData[15];	// M_x
		destination[9] = ((int16_t)rawData[18] << 8) | rawData[17];	// M_y
		destination[10] = ((int16_t)rawData[20] << 8) | rawData[19];	// M_z
		}
}
/*
void readMPU9250Data(int16_t* destination){
	/*
	uint8_t rawData[14];
	readBytes_SPI(ACCEL_XOUT_H, 14, rawData);
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];	// Acc_x
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];	// Acc_y
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];	// Acc_z
	destination[3] = ((int16_t)rawData[6] << 8) | rawData[7];	// Temperature
	destination[4] = ((int16_t)rawData[8] << 8) | rawData[9];	// Gy_x
	destination[5] = ((int16_t)rawData[10] << 8) | rawData[11];	// Gy_y
	destination[6] = ((int16_t)rawData[12] << 8) | rawData[13];	// Gy_z
	*


	uint8_t rawData[22];
	//readMag_Call();

	if(DataCnt%1 == 0){
		readMag_Call();
	}
	DataCnt++;

	readBytes_SPI(ACCEL_XOUT_H, 22, rawData);
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];	// Acc_x
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];	// Acc_y
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];	// Acc_z

	destination[3] = ((int16_t)rawData[6] << 8) | rawData[7];	// Temperature

	destination[4] = ((int16_t)rawData[8] << 8) | rawData[9];	// Gy_x
	destination[5] = ((int16_t)rawData[10] << 8) | rawData[11];	// Gy_y
	destination[6] = ((int16_t)rawData[12] << 8) | rawData[13];	// Gy_z

	destination[7] = rawData[14];
	destination[11] = rawData[21];

	destination[8] = ((int16_t)rawData[16] << 8) | rawData[15];	// M_x
	destination[9] = ((int16_t)rawData[18] << 8) | rawData[17];	// M_y
	destination[10] = ((int16_t)rawData[20] << 8) | rawData[19];	// M_z
}*/

void readMag_Call(){
	writeByte_SPI(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);
	writeByte_SPI(I2C_SLV0_REG, AK8963_ST1);
	writeByte_SPI(I2C_SLV0_CTRL, I2C_SLV0_EN | 8);

}
/*
void readMag_Call(){
	writeByte_SPI(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);
	writeByte_SPI(I2C_SLV0_REG, AK8963_XOUT_L);
	writeByte_SPI(I2C_SLV0_CTRL, I2C_SLV0_EN | 7);

}
*/
void updateMag(){
	int16_t MagData[3];
	readMagData(MagData);

	m.x = (float)(MagData[0] * mRes * magCalibration.x - magBias.x) * magScale.x;
	m.y = (float)(MagData[1] * mRes * magCalibration.y - magBias.y) * magScale.y;
	m.z = (float)(MagData[2] * mRes * magCalibration.z - magBias.z) * magScale.z;
}

void readMagData(int16_t* destination){
	uint8_t rawData[7];

	uint8_t AK_ST1_data;
	readBytes_AK8963(AK8963_ST1, 1, &AK_ST1_data);

	//if(ST1_data & 0x01){
		readBytes_AK8963(AK8963_XOUT_L, 7, rawData);

		uint8_t c = rawData[6];
		//if(!(c & 0x08)){
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		//}
	//}
}

void setup_MPU9250(){
	uint8_t add_mpu;// = {0};
	uint8_t add_ak;
	readBytes_SPI(WHO_AM_I_MPU9250, 1, &add_mpu);
	if(add_mpu == 0x71 || add_mpu == 0x70){
		set_params_MPU9250();
		HAL_Delay(100);
		readBytes_AK8963(AK8963_WHO_AM_I, 1, &add_ak);
		if(add_ak == 0x48){
			set_params_AK8963();

		}
		else{// Req. this field to check
			/*do{
				readBytes_AK8963(AK8963_WHO_AM_I, 1, &add_ak);
				HAL_Delay(100);
			}while(add_ak != 0x48);
			set_params_AK8963();*/

		}
	}
	else{// Req. this field to check
		/*do{
			readBytes_SPI(WHO_AM_I_MPU9250, 1, &add_mpu);
			HAL_Delay(100);
		}while(add_mpu != 0x71);

		set_params_MPU9250();
		HAL_Delay(100);
		readBytes_AK8963(AK8963_WHO_AM_I, 1, &add_ak);
		if(add_ak == 0x48){
			set_params_AK8963();

		}
		else{// Req. this field to check
			do{
				readBytes_AK8963(AK8963_WHO_AM_I, 1, &add_ak);
				HAL_Delay(100);
			}while(add_ak != 0x48);
			set_params_AK8963();
		}*/
	}
}

void set_params_AK8963(){
	uint8_t rawData[8];

	HAL_Delay(50);
	replaceBlock_AK8963(AK8963_CNTL, MGN_POWER_DN, 0, 4);	// AK [0x0A] set [0x00]  // power down AK
	HAL_Delay(50);
	replaceBlock_AK8963(AK8963_CNTL, MGN_FUSE_ROM, 0, 4);	// AK [0x0A] set [0x0F]  // Fuse ROM access mode
	HAL_Delay(50);
	readBytes_AK8963(AK8963_ASAX, 3, rawData);				// AK Read reg [0x10] ~ [0x12] // Sensitivity adjustment data for x,y,z are stored on fuse ROM on shipment

	magCalibration.x = (float)(rawData[0] - 128)/256.f + 1.f; //eqn. from register map document
	magCalibration.y = (float)(rawData[1] - 128)/256.f + 1.f;
	magCalibration.z = (float)(rawData[2] - 128)/256.f + 1.f;

	replaceBlock_AK8963(AK8963_CNTL, MGN_POWER_DN, 0, 4);	// AK [0x0A] set [0x00]  // power down AK
	HAL_Delay(50);

	uint8_t AK_cntr_bit=0;
	_magBits = MGN_16BITS;
	_magMode = MGN_CONT_MEAS2;
	AK_cntr_bit |= _magBits<<4;
	AK_cntr_bit |= _magMode;
	replaceBlock_AK8963(AK8963_CNTL, AK_cntr_bit, 0, 5);	// AK [0x0A] set resolution and measurement mode
	writeByte_SPI(PWR_MGMT_1, CLOCK_SEL_PLL); 				// [0x6B] set [0x01]  // Select clock source in PWR_MGMT_1 reg.
	HAL_Delay(50);
	//uint8_t buff;
	//readBytes_AK8963(AK8963_CNTL, 1, &buff);
	mRes = 10. * 4912. / 32760.0;		// for 16 Bit resolution [conversion to the milli Gauss]
	//mRes = 10. * 4912. / 8192.0;		// if 14 Bit

	//readBytes_AK8963(AK8963_ST1, 8, rawData);
	readMag_Call();
}

void set_params_MPU9250(){
	writeByte_SPI(PWR_MGMT_1, 0x00); 		// [0x6B] set [0x00]  // reset MPU9250
	writeByte_SPI(PWR_MGMT_1, CLOCK_SEL_PLL);	// [0x6B] set [0x01]  // Select clock source in PWR_MGMT_1 reg.
	writeByte_SPI(USER_CTRL, I2C_MST_EN);		// [0x6A] set [0x20]  // Set mpu9250 I2C master for other sensors
	writeByte_SPI(I2C_MST_CTRL, I2C_MST_CLK); 	// [0x24] set [0x0D]  // Set I2c Clk 400 kHz
	replaceBlock_AK8963(AK8963_CNTL, MGN_POWER_DN, 0, 4);	// AK [0x0A] set [0x00]  // power down AK
	writeByte_SPI(PWR_MGMT_1, PWR_RESET); 		// [0x6B] set [0x80]  // reset MPU9250
	HAL_Delay(100);


	replaceBlock_AK8963(AK8963_CNTL2, AK8963_RESET, 0, 1);
	writeByte_SPI(PWR_MGMT_1, CLOCK_SEL_PLL);	// [0x6B] set [0x01]  // Select clock source in PWR_MGMT_1 reg.
	setDlpfBandwidth(DLPF_BANDWIDTH_5HZ);
	writeByte_SPI(PWR_MGMT_2, SEN_ENABLE);		// [0x6C] set [0x00] // xyz accel & gyro on
	writeByte_SPI(SMPLRT_DIV, SR_1000HZ);		// [0x19] set [0] // { 1000Hz = 0 , 1000/(1+n) Hz }
	setGyroRange(GYRO_RANGE_2000DPS);			// { 250 DPS, 500 DPS, 1000 DPS, 2000 DPS }
	setAccelRange(ACCEL_RANGE_16G);				// { 2G, 4G, 8G, 16G }
	setDlpfBandwidth(DLPF_BANDWIDTH_184HZ);		// { 250Hz, 184Hz, 92Hz, 41Hz, 20Hz, 10Hz, 5Hz }
	writeByte_SPI(ACCEL_CONFIG2, 0x01);			//

	writeByte_SPI(INT_PIN_CFG, 0x20);			// [0x37] set [0x20]  // INT pin / Bypass enable configuration
	writeByte_SPI(INT_ENABLE, 0x00);			// [0x38] set [0x01]  // data ready interrupt
	writeByte_SPI(USER_CTRL, I2C_MST_EN/**/);		// [0x6A] set [0x20]  // Set mpu9250 I2C master for other sensors
	HAL_Delay(100);
	writeByte_SPI(I2C_MST_CTRL, I2C_MST_CLK); 	// [0x24] set [0x0D]  // Set I2c Clk 400 kHz


}

void enableDataReadyInterrupt(){
	writeByte_SPI(INT_PIN_CFG, 0x00);			// [0x37] set [0x00]  // INT pin / Bypass enable configuration
	writeByte_SPI(INT_ENABLE, 0x01);			// [0x38] set [0x01]  // data ready interrupt
}

void setSampleRate(enum SampleRate samplerate){
	writeByte_SPI(SMPLRT_DIV, samplerate);		// [0x19] set [9] // { 1000Hz = 0 , 1000/(1+n) Hz }
	_smpRate = samplerate;
}


void setGyroRange(enum GyroRange gyrorange){
	switch(gyrorange){
	case GYRO_RANGE_250DPS:
		gRes = 250.0f/32768.0f;
		break;
	case GYRO_RANGE_500DPS:
		gRes = 500.0f/32768.0f;
		break;
	case GYRO_RANGE_1000DPS:
		gRes = 1000.0f/32768.0f;
		break;
	case GYRO_RANGE_2000DPS:
		gRes = 2000.0f/32768.0f;
		break;
	}
	replaceBlock(GYRO_CONFIG, gyrorange, 3, 2); 	// [0x1B] reg. replace [4 ~ 3] bits // Setting sensing range
	_gyroRange = gyrorange;
}

void setAccelRange(enum AccelRange accelrange){
	switch(accelrange){
	case ACCEL_RANGE_2G:
		aRes = 2.0f/32768.0f;
		break;
	case ACCEL_RANGE_4G:
		aRes = 4.0f/32768.0f;
		break;
	case ACCEL_RANGE_8G:
		aRes = 8.0f/32768.0f;
		break;
	case ACCEL_RANGE_16G:
		aRes = 16.0f/32768.0f;
		break;
	}
	replaceBlock(ACCEL_CONFIG, accelrange, 3, 2); 	// [0x1C] reg. replace [4 ~ 3] bits // Setting sensing range
	_accelRange = accelrange;
}

void setDlpfBandwidth(enum DlpfBandwidth bandwidth){
	replaceBlock(ACCEL_CONFIG2, bandwidth, 0, 4);	// [0x1D] reg. replace [3 ~ 0] bits // Setting accel Dlpbandwidth
	replaceBlock(MPU_CONFIG, bandwidth, 0, 3);		// [0x1A] reg. replace [2 ~ 0] bits //
	_bandwidth = bandwidth;
}

/*void setMagneticDeclination(){

}*/

void readBytes_SPI(uint8_t address, uint8_t len, uint8_t* readdata){
	uint8_t RW_Add = 0;

	RW_Add = (address | SPI_READ);

	HAL_GPIO_WritePin(_mpu9250.csPort, _mpu9250.csPin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(_mpu9250.hspi, &RW_Add, 1, HAL_MAX_DELAY);

	HAL_SPI_Receive(_mpu9250.hspi, readdata, len, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(_mpu9250.csPort, _mpu9250.csPin, GPIO_PIN_SET);
}

void writeByte_SPI(uint8_t address, uint8_t data){
	uint8_t RW_Add = 0;

	RW_Add = (address);

	HAL_GPIO_WritePin(_mpu9250.csPort, _mpu9250.csPin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(_mpu9250.hspi, &RW_Add, 1, HAL_MAX_DELAY);

	HAL_SPI_Transmit(_mpu9250.hspi, &data, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(_mpu9250.csPort, _mpu9250.csPin, GPIO_PIN_SET);
}

void writeByte_AK8963(uint8_t subaddress, uint8_t data){
	writeByte_SPI(I2C_SLV0_ADDR, AK8963_I2C_ADDR);
	writeByte_SPI(I2C_SLV0_REG, subaddress);
	writeByte_SPI(I2C_SLV0_DO, data);
	writeByte_SPI(I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1);
}

void readBytes_AK8963(uint8_t subaddress, uint8_t len, uint8_t* readdata){
	writeByte_SPI(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);
	writeByte_SPI(I2C_SLV0_REG, subaddress);
	writeByte_SPI(I2C_SLV0_CTRL, I2C_SLV0_EN | len);
	HAL_Delay(10);
	readBytes_SPI(EXT_SENS_DATA_00, len, readdata);
}

void replaceBlock(uint8_t address, uint8_t block, uint8_t from, uint8_t size){
	// replace part of byte pointed by [address] to the [block], the  part is [size] and starts [from] bit //
	uint8_t data = 0;
	readBytes_SPI(address, 1, &data);

	data &= ~(((1 << size) - 1) << from);
	data |= block << from;

	writeByte_SPI(address, data);
}

void replaceBlock_AK8963(uint8_t subaddress, uint8_t block, uint8_t from, uint8_t size){
	uint8_t data = 0;
	readBytes_AK8963(subaddress, 1, &data);

	data &= ~(((1 << size) - 1) << from);
	data |= block << from;

	writeByte_AK8963(subaddress, data);

}
/*
void setGyroBias(struct Vect3 V){
	gyroBias = V;
}

void setAccBias(struct Vect3 V){
	accelBias = V;
}

void setMagBias(struct Vect3 V){
	magBias = V;
}

void setMagScale(struct Vect3 V){
	magScale = V;
}

void calibrationProcess(){

}

void applyCalibratedValue(){

}
*/
