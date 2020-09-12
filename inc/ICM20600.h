/*
 * ADXL377.h
 *
 *  Created on: Aug 19, 2020
 *      Author: Claes
 *      CUSTOM LIBRARY BY CLAES
 */

#ifndef INC_ICM20600_H_
#define INC_ICM20600_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32l431xx.h"
#include "stm32l4xx_hal_spi.h"



// CONFIG:
// Define this to use the DMA for data transfers:
//#define IMU_DMA

// Define your SPI bus and pins here:
extern SPI_HandleTypeDef hspi3;
#define ICM20600_SPI			hspi3
#define	ICM20600_CS_GPIO		IMU_CS_GPIO_Port
#define	ICM20600_CS_PIN		IMU_CS_Pin


typedef struct dataLog_t {
	float ax, ay, az, ax_prev;
	float gx, gy, gz;
	float gx_offset, gy_offset, gz_offset;
	float roll;
	float rollPrev;
	float upVector, sideVector, forwardVector;
	float totalGyro;
	float sideVectorChange;
} dataLog_t;

int map(int x, int in_min, int in_max, int out_min, int out_max); // Helper function

// Averaging filter settings for Low Power Accelerometer mode
typedef enum acc_averaging_sample_type_t {
  ACC_AVERAGE_4 = 0,
  ACC_AVERAGE_8,
  ACC_AVERAGE_16,
  ACC_AVERAGE_32,
} Acc_avg_sample_type_t;

// Averaging filter configuration for low-power gyroscope mode
typedef enum gyro_averaging_sample_type_t {
  GYRO_AVERAGE_1 = 0,
  GYRO_AVERAGE_2,
  GYRO_AVERAGE_4,
  GYRO_AVERAGE_8,
  GYRO_AVERAGE_16,
  GYRO_AVERAGE_32,
  GYRO_AVERAGE_64,
  GYRO_AVERAGE_128,
} Gyro_avg_sample_type_t;

// CLAES HACK:
// Gyroscope output data rate
typedef enum gyro_lownoise_odr_type_t
{
  GYRO_RATE_8K_BW_3281 = 0,
  GYRO_RATE_8K_BW_250,
  GYRO_RATE_1K_BW_176,
  GYRO_RATE_1K_BW_92,
  GYRO_RATE_1K_BW_41,
  GYRO_RATE_1K_BW_20,
  GYRO_RATE_1K_BW_10,
  GYRO_RATE_1K_BW_5,
} Gyro_LowNoise_ODR;
// CLAES HACK
// Accelerometer output data rate
typedef enum acc_lownoise_odr_type_t
{
  ACC_RATE_4K_BW_1046 = 0,
  ACC_RATE_1K_BW_420,
  ACC_RATE_1K_BW_218,
  ACC_RATE_1K_BW_99,
  ACC_RATE_1K_BW_44,
  ACC_RATE_1K_BW_21,
  ACC_RATE_1K_BW_10,
  ACC_RATE_1K_BW_5,
} Acc_LowNoise_ODR;

// CLAES HACK
// ICM20600 power mode
typedef enum icm20600_power_type_t
{
  ICM_SLEEP_MODE = 0,
  ICM_STANDYBY_MODE,
  ICM_ACC_LOW_POWER,
  ICM_ACC_LOW_NOISE,
  ICM_GYRO_LOW_POWER,
  ICM_GYRO_LOW_NOISE,
  ICM_6AXIS_LOW_POWER,
  ICM_6AXIS_LOW_NOISE,
} ICM20600_Power_Type_t;

typedef enum GyroRange_ {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
} GyroRange;

typedef enum AccelRange_ {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
} AccelRange;

typedef enum DLPFBandwidth_ {
	DLPF_BANDWIDTH_184HZ = 0,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
} DLPFBandwidth;


#define	LP_ACCEL_ODR_4HZ 		255
#define	LP_ACCEL_ODR_8HZ 		127
#define	LP_ACCEL_ODR_10HZ 	99
#define	LP_ACCEL_ODR_16HZ		63
#define	LP_ACCEL_ODR_31HZ		31
#define	LP_ACCEL_ODR_50HZ		19
#define	LP_ACCEL_ODR_63HZ		15
#define	LP_ACCEL_ODR_100HZ	9
#define	LP_ACCEL_ODR_125HZ	7
#define	LP_ACCEL_ODR_200HZ	4
#define	LP_ACCEL_ODR_250HZ	3
#define	LP_ACCEL_ODR_500HZ	0 // should be 1 according to datasheet?



uint8_t ICM20600_Init();
void ICM20600_setup_shotCounter_settings();
/* read the data, each argument should point to a array for x, y, and x */
void ICM20600_GetData();

void ICM20600_sleep(bool value);

void ICM20600_setPowerMode(ICM20600_Power_Type_t mode);

void ICM20600_setAccOutputDataRate(Acc_LowNoise_ODR odr);

void ICM20600_setGyroOutputDataRate(Gyro_LowNoise_ODR odr);

void ICM20600_setAccAverageSample(Acc_avg_sample_type_t sample);

void ICM20600_setGyroAverageSample(Gyro_avg_sample_type_t sample);

/* sets the sample rate divider to values other than default */
void ICM20600_SetSampleRateDivider(uint8_t srd);
/* sets the DLPF bandwidth to values other than default */
void ICM20600_SetDLPFBandwidth(DLPFBandwidth bandwidth);
/* sets the gyro full scale range to values other than default */
void ICM20600_SetGyroRange(GyroRange range);
/* sets the accelerometer full scale range to values other than default */
void ICM20600_SetAccelRange(AccelRange range);

void calculate_orientation();


#ifdef __cplusplus
}
#endif

#endif /* INC_ICM20600_H_ */
