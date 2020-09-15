/*
 * ICM20600.c
 *
 *  Created on: Aug 19, 2020
 *      Author: Claes Christian Jakobsen
 */

#include "ICM20600.h"

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

const float G = 9.807f; // Gravity constant
const float _d2r = 3.14159265359f / 180.0f; // Degrees to radians constant

//extern dataLog_t* IMU;

int16_t _axcounts, _aycounts, _azcounts;
int16_t _gxcounts, _gycounts, _gzcounts;
float _accelScale;
float _gyroScale;
AccelRange _accelRange;
GyroRange _gyroRange;

const uint8_t READWRITE_CMD = 0x80;
const uint8_t MULTIPLEBYTE_CMD = 0x40;
const uint8_t DUMMY_BYTE = 0x00;




// ICM20600 registers
const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t DLPF_184 = 0x01;
const uint8_t DLPF_92 = 0x02;
const uint8_t DLPF_41 = 0x03;
const uint8_t DLPF_20 = 0x04;
const uint8_t DLPF_10 = 0x05;
const uint8_t DLPF_5 = 0x06;
const uint8_t CONFIG = 0x1A;
const uint8_t SMPDIV = 0x19;
const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_WOM_EN = 0x40;
const uint8_t INT_RAW_RDY_EN = 0x01;
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_CYCLE = 0x20;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;
const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;
const uint8_t DIS_GYRO = 0x07;
const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D;
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t ACCEL_INTEL_EN = 0x80;
const uint8_t ACCEL_INTEL_MODE = 0x40;
const uint8_t LP_ACCEL_ODR = 0x1E;
const uint8_t WOM_THR = 0x1F;
const uint8_t WHO_AM_I = 0x75;
const uint8_t FIFO_EN = 0x23;
const uint8_t FIFO_TEMP = 0x80;
const uint8_t FIFO_GYRO = 0x70;
const uint8_t FIFO_ACCEL = 0x08;
const uint8_t FIFO_MAG = 0x01;
const uint8_t FIFO_COUNT = 0x72;
const uint8_t FIFO_READ = 0x74;

//extern float ax, ay, az, gx, gy, gz;

#define GET_DATA_SIZE 14

static uint8_t _accel_buffer[GET_DATA_SIZE];

void ICM20600_writeRegister(uint8_t subAddress, uint8_t data);


/* enables the data ready interrupt */
void ICM20600_enableDataReadyInterrupt() {

  // setting the interrupt:
	ICM20600_writeRegister(INT_PIN_CFG,INT_PULSE_50US); // setup interrupt, 50 us pulse

	ICM20600_writeRegister(INT_ENABLE,INT_RAW_RDY_EN);// set to data ready
}

/* disables the data ready interrupt */
void ICM20600_disableDataReadyInterrupt() {
  ICM20600_writeRegister(INT_ENABLE,INT_DISABLE);// disable interrupt
}

void ICM20600_sleep(bool value)
{
  if (value == true)
  {
    ICM20600_writeRegister(PWR_MGMNT_2, 0b00111111); // Disables all axes on acc and gyro individually, just in case. (Not necessary)
    ICM20600_writeRegister(PWR_MGMNT_1, 0b01111111); // Set sleep bit etc. in power register 1

  }
  else
  {
    //writeRegister(PWR_MGMNT_1,0x80); // Resets device
    ICM20600_writeRegister(PWR_MGMNT_1, 0b10000000); // Resets device
  }
}

static inline void ICM20600_Activate()
{
	//ICM20600_OnActivate();
	HAL_GPIO_WritePin(ICM20600_CS_GPIO, ICM20600_CS_PIN, GPIO_PIN_RESET);
}

static inline void ICM20600_Deactivate()
{
	HAL_GPIO_WritePin(ICM20600_CS_GPIO, ICM20600_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&ICM20600_SPI,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		return -1;
	}
	else
	{
	}
	return receivedbyte;
}

void ICM20600_SPI_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	ICM20600_Activate();
	SPIx_WriteRead(WriteAddr);
	while(NumByteToWrite>=0x01)
	{
		SPIx_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	ICM20600_Deactivate();
}

void ICM20600_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	ICM20600_Activate();
	uint8_t data = ReadAddr | READWRITE_CMD;
	HAL_SPI_Transmit(&ICM20600_SPI, &data, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&ICM20600_SPI, pBuffer, NumByteToRead, HAL_MAX_DELAY);
	ICM20600_Deactivate();
}

/* writes a byte to ICM20600 register given a register address and data */
void ICM20600_writeRegister(uint8_t subAddress, uint8_t data)
{
	ICM20600_SPI_Write(&data, subAddress, 1);
	HAL_Delay(1);
}

/* reads registers from ICM20600 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
	ICM20600_SPI_Read(dest, subAddress, count);
}


/* gets the ICM20600 WHO_AM_I register value, expected to be 0x71 */
static uint8_t whoAmI(){
	// read the WHO AM I register
	readRegisters(WHO_AM_I,1,_accel_buffer);

	// return the register value
	return _accel_buffer[0];
}


/* starts communication with the ICM20600 */
uint8_t ICM20600_Init()
{
	// reset the ICM20600
	ICM20600_writeRegister(PWR_MGMNT_1,PWR_RESET);
	HAL_Delay(1);

	// select clock source to gyro
	ICM20600_writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);

	// reset the ICM20600
	ICM20600_writeRegister(PWR_MGMNT_1,PWR_RESET);
	// wait for ICM20600 to come back up
	HAL_Delay(1);

	// select clock source to gyro
	ICM20600_writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL);

	uint8_t who = whoAmI();
	if(who != 0x11) 	// 0x11 for ICM20600!
		return 1;

	// enable accelerometer and gyro
	ICM20600_writeRegister(PWR_MGMNT_2,SEN_ENABLE);

	// setting accel range to 16G as default
	ICM20600_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G);
	  _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
	  _accelRange = ACCEL_RANGE_16G;

	// setting the gyro range to 2000DPS as default
	ICM20600_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS);
	  _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
	  _gyroRange = GYRO_RANGE_2000DPS;

	// setting bandwidth to 184Hz as default
	ICM20600_writeRegister(ACCEL_CONFIG2,DLPF_184);

	// setting gyro bandwidth to 184Hz
	ICM20600_writeRegister(CONFIG,DLPF_184);

	// setting the sample rate divider to 0 as default
	ICM20600_writeRegister(SMPDIV,0x00);

	// successful init, return 0
	return 0;
}

void ICM20600_setup_shotCounter_settings()
{
	// Overall power settings:
	ICM20600_setPowerMode(ICM_6AXIS_LOW_POWER);
	//ICM20600_SetSampleRateDivider(LP_ACCEL_ODR_125HZ); // Sample rate div only works in Low Power mode
	ICM20600_SetSampleRateDivider(LP_ACCEL_ODR_250HZ); // Sample rate div only works in Low Power mode

	// Acc config:
	ICM20600_SetAccelRange(ACCEL_FS_SEL_16G);
	ICM20600_setAccAverageSample(ACC_AVERAGE_4);
	ICM20600_setAccOutputDataRate(ACC_RATE_1K_BW_420); // Seems appropriate for ShotCounter

	// Gyro config:
	ICM20600_SetGyroRange(GYRO_FS_SEL_2000DPS);
	ICM20600_setGyroAverageSample(GYRO_AVERAGE_1);
	ICM20600_setGyroOutputDataRate(GYRO_RATE_1K_BW_176); // Seems appropriate for ShotCounter

	ICM20600_enableDataReadyInterrupt();
}

void ICM20600_setPowerMode(ICM20600_Power_Type_t mode)
{
    uint8_t data_pwr1;
    uint8_t data_pwr2 = 0x00;
    uint8_t data_gyro_lp;
    readRegisters(PWR_MGMNT_1, 1, _accel_buffer);
    data_pwr1 = _accel_buffer[0];
    data_pwr1 &= 0x8f;                  // 0b10001111
    readRegisters(LP_ACCEL_ODR, 1, _accel_buffer);
    data_gyro_lp = _accel_buffer[0];
    // When set to ‘1’ low-power gyroscope mode is enabled. Default setting is 0
    data_gyro_lp &= 0x7f;               // 0b01111111
    switch (mode) {
        case ICM_SLEEP_MODE:
            data_pwr1 |= 0x40;          // set 0b01000000
            break;

        case ICM_STANDYBY_MODE:
            data_pwr1 |= 0x10;          // set 0b00010000
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            break;

        case ICM_ACC_LOW_POWER:
            data_pwr1 |= 0x20;          // set bit5 0b00100000
            data_pwr2 = 0x07;           //0x00000111 disable gyro
            break;

        case ICM_ACC_LOW_NOISE:
            data_pwr1 |= 0x00;
            data_pwr2 = 0x07;           //0x00000111 disable gyro
            break;

        case ICM_GYRO_LOW_POWER:
            data_pwr1 |= 0x00;          // dont set bit5 0b00000000
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            data_gyro_lp |= 0x80;
            break;

        case ICM_GYRO_LOW_NOISE:
            data_pwr1 |= 0x00;
            data_pwr2 = 0x38;           // 0x00111000 disable acc
            break;

        case ICM_6AXIS_LOW_POWER:
            data_pwr1 |= 0x00;          // dont set bit5 0b00100000
            data_gyro_lp |= 0x80;
            break;

        case ICM_6AXIS_LOW_NOISE:
            data_pwr1 |= 0x00;
            break;

        default:
            break;
    }
    ICM20600_writeRegister(PWR_MGMNT_1,data_pwr1);
    ICM20600_writeRegister(PWR_MGMNT_2,data_pwr2);
    ICM20600_writeRegister(LP_ACCEL_ODR,data_gyro_lp);
}

/* sets the accelerometer full scale range to values other than default */
void ICM20600_SetAccelRange(AccelRange range)
{
	ICM20600_writeRegister(ACCEL_CONFIG, range);
}

/* sets the gyro full scale range to values other than default */
void ICM20600_SetGyroRange(GyroRange range)
{
	//writeRegister(GYRO_CONFIG, range);

	switch(range) {
	    case GYRO_RANGE_250DPS: {
	      // setting the gyro range to 250DPS
	      ICM20600_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS);
	      _gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
	      break;
	    }
	    case GYRO_RANGE_500DPS: {
	      // setting the gyro range to 500DPS
	      ICM20600_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS);
	      _gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
	      break;
	    }
	    case GYRO_RANGE_1000DPS: {
	      // setting the gyro range to 1000DPS
	    	ICM20600_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS);
	      _gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
	      break;
	    }
	    case GYRO_RANGE_2000DPS: {
	      // setting the gyro range to 2000DPS
	      ICM20600_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS);
	      _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
	      break;
	    }
	  }
	  _gyroRange = range;
}

// Adjusted from I2C FROM THIS LIBRARY: https://github.com/Seeed-Studio/Seeed_ICM20600_AK09918/blob/master/ICM20600.cpp
void ICM20600_setAccOutputDataRate(Acc_LowNoise_ODR odr) {
    uint8_t data;
    readRegisters(ACCEL_CONFIG2, 1, _accel_buffer);
    data = _accel_buffer[0];
    data &= 0xf0;  // 0b11110000

    switch (odr) {
        case ACC_RATE_4K_BW_1046:
            data |= 0x08;
            break;

        case ACC_RATE_1K_BW_420:
            data |= 0x07;
            break;

        case ACC_RATE_1K_BW_218:
            data |= 0x01;
            break;

        case ACC_RATE_1K_BW_99:
            data |= 0x02;
            break;

        case ACC_RATE_1K_BW_44:
            data |= 0x03;
            break;

        case ACC_RATE_1K_BW_21:
            data |= 0x04;
            break;

        case ACC_RATE_1K_BW_10:
            data |= 0x05;
            break;

        case ACC_RATE_1K_BW_5:
            data |= 0x06;
            break;

        default:
            break;
    }
    ICM20600_writeRegister(ACCEL_CONFIG2,data);
}


// Hack from I2C lib:
void ICM20600_setGyroOutputDataRate(Gyro_LowNoise_ODR odr) {
    uint8_t data;
    readRegisters(CONFIG, 1, _accel_buffer);
    data = _accel_buffer[0];
    data &= 0xf8;  // DLPF_CFG[2:0] 0b11111000

    switch (odr) {
        case GYRO_RATE_8K_BW_3281:
            data |= 0x07;
            break;
        case GYRO_RATE_8K_BW_250:
            data |= 0x00;
            break;
        case GYRO_RATE_1K_BW_176:
            data |= 0x01;
            break;
        case GYRO_RATE_1K_BW_92:
            data |= 0x02;
            break;
        case GYRO_RATE_1K_BW_41:
            data |= 0x03;
            break;
        case GYRO_RATE_1K_BW_20:
            data |= 0x04;
            break;
        case GYRO_RATE_1K_BW_10:
            data |= 0x05;
            break;
        case GYRO_RATE_1K_BW_5:
            data |= 0x06;
            break;
    }
    ICM20600_writeRegister(CONFIG, data);
}


// Hack from I2C lib
// (for low power mode only)
void ICM20600_setAccAverageSample(Acc_avg_sample_type_t sample) {
    uint8_t data = 0;
    readRegisters(ACCEL_CONFIG2, 1, _accel_buffer);
    data = _accel_buffer[0];

    data &= 0xcf; // & 0b11001111
    switch (sample) {
        case ACC_AVERAGE_4:
            data |= 0x00; // 0bxx00xxxx
            break;

        case ACC_AVERAGE_8:
            data |= 0x10; // 0bxx01xxxx
            break;

        case ACC_AVERAGE_16:
            data |= 0x20; // 0bxx10xxxx
            break;

        case ACC_AVERAGE_32:
            data |= 0x30; // 0bxx11xxxx
            break;

        default:
            break;
    }
    ICM20600_writeRegister(ACCEL_CONFIG2, data);
}

// Hack from I2C lib
// (for low power mode only)
void ICM20600_setGyroAverageSample(Gyro_avg_sample_type_t sample) {
    uint8_t data = 0;
    readRegisters(LP_ACCEL_ODR, 1, _accel_buffer);
    data = _accel_buffer[0];

    data &= 0x8f;           // 0b10001111
    switch (sample) {
        case GYRO_AVERAGE_1:
            data |= 0x00; // 0bx000xxxx
            break;

        case GYRO_AVERAGE_2:
            data |= 0x10; // 0bx001xxxx
            break;

        case GYRO_AVERAGE_4:
            data |= 0x20; // 0bx010xxxx
            break;

        case GYRO_AVERAGE_8:
            data |= 0x30; // 0bx011xxxx
            break;

        case GYRO_AVERAGE_16:
            data |= 0x40; // 0bx100xxxx
            break;

        case GYRO_AVERAGE_32:
            data |= 0x50; // 0bx101xxxx
            break;

        case GYRO_AVERAGE_64:
            data |= 0x60;
            break;

        case GYRO_AVERAGE_128:
            data |= 0x70;
            break;


        default:
            break;
    }
    ICM20600_writeRegister(LP_ACCEL_ODR, data);
}



/* sets the DLPF bandwidth to values other than default */
void ICM20600_SetDLPFBandwidth(DLPFBandwidth bandwidth)
{
	ICM20600_writeRegister(ACCEL_CONFIG2,bandwidth);
	ICM20600_writeRegister(CONFIG,bandwidth);
}

/* sets the sample rate divider to values other than default */
void ICM20600_SetSampleRateDivider(uint8_t srd)
{
	ICM20600_writeRegister(SMPDIV, srd);
}

extern dataLog_t IMU;
/* read the data, each argiment should point to a array for x, y, and x */
void ICM20600_GetData()
{
	//readRegisters(ACCEL_OUT, 14, _accel_buffer); 	// grab the data from the ICM20600

	uint8_t data = ACCEL_OUT | READWRITE_CMD;
	ICM20600_Activate();


#if IMU_DMA
	HAL_SPI_Transmit(&ICM20600_SPI, &data, 1, HAL_MAX_DELAY);
	//HAL_SPI_Transmit_DMA(&ICM20600_SPI, &data, 1);					// DMA
	HAL_SPI_Receive_DMA(&ICM20600_SPI, _accel_buffer, 14); // Start the receiving of IMU data



#else
	// NORMAL SPI RECEIVE MODE:
	HAL_SPI_Transmit(&ICM20600_SPI, &data, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&ICM20600_SPI, _accel_buffer, GET_DATA_SIZE, HAL_MAX_DELAY);

	// Combine into 16 bit values:
	_axcounts = (((int16_t)_accel_buffer[0]) << 8) | _accel_buffer[1];
	_aycounts = (((int16_t)_accel_buffer[2]) << 8) | _accel_buffer[3];
	_azcounts = (((int16_t)_accel_buffer[4]) << 8) | _accel_buffer[5];
	_gxcounts = (((int16_t)_accel_buffer[8]) << 8) | _accel_buffer[9];
	_gycounts = (((int16_t)_accel_buffer[10]) << 8) | _accel_buffer[11];
	_gzcounts = (((int16_t)_accel_buffer[12]) << 8) | _accel_buffer[13];

	// Convert to float:
	IMU.ax = (float)(_aycounts * _accelScale);
	IMU.ay = (float)(_axcounts * _accelScale);
	IMU.az = (float)(_azcounts * _accelScale * -1);
	IMU.gx = (float)(_gycounts * _gyroScale);
	IMU.gy = (float)(_gxcounts * _gyroScale);
	IMU.gz = (float)(_gzcounts * _gyroScale * -1);
	ICM20600_Deactivate();
#endif
}

#if IMU_DMA

void ICM20600_start_receiving_DMA()
{
	//printf("h\n");
	HAL_SPI_Receive_DMA(&ICM20600_SPI, _accel_buffer, 14); // Start the receiving of IMU data
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{

	if (hspi->Instance == SPI3) {

		ICM20600_Deactivate();

		// Combine into 16 bit values:
		_axcounts = (((int16_t) _accel_buffer[0]) << 8) | _accel_buffer[1];
		_aycounts = (((int16_t) _accel_buffer[2]) << 8) | _accel_buffer[3];
		_azcounts = (((int16_t) _accel_buffer[4]) << 8) | _accel_buffer[5];
		_gxcounts = (((int16_t) _accel_buffer[8]) << 8) | _accel_buffer[9];
		_gycounts = (((int16_t) _accel_buffer[10]) << 8) | _accel_buffer[11];
		_gzcounts = (((int16_t) _accel_buffer[12]) << 8) | _accel_buffer[13];

		// Convert to float:
		IMU.ax = (float) (_aycounts * _accelScale);
		IMU.ay = (float) (_axcounts * _accelScale);
		IMU.az = (float) (_azcounts * _accelScale * -1);
		IMU.gx = (float) (_gycounts * _gyroScale);
		IMU.gy = (float) (_gxcounts * _gyroScale);
		IMU.gz = (float) (_gzcounts * _gyroScale * -1);
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	//printf("e");
}
#endif
