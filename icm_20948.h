#pragma once

#include "main.h"

/* USER DEFINE */
extern SPI_HandleTypeDef hspi2;
#define IMU_HSPI hspi2
#define IMU_CS_PORT SPI2_CS_GPIO_Port
#define IMU_CS_PIN SPI2_CS_GPIO_Pin
#define GYRO_RANGE _gyro_range_1000dps
#define ACCEL_RANGE _accel_range_8g
/* END USER DEFINE */

// registers
#define REG_BANK_SEL 0x7F
#define WHO_AM_I 0x00
#define PWR_MGMT_1 0x06
#define ODR_ALIGN_EN 0x09
#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG 0x01
#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_CONFIG 0x14
#define USER_CTRL 0x03
#define XG_OFFS_USRH 0x03
#define XG_OFFS_USRL 0x04
#define YG_OFFS_USRH 0x05
#define YG_OFFS_USRL 0x06
#define ZG_OFFS_USRH 0x07
#define ZG_OFFS_USRL 0x08


// magnetometer and i2c master conf
#define I2C_MST_CLK 01
#define LP_CONFIG 0x05
#define I2C_MST_ODR_CONFIG 0x00
#define ODR_ALIGN_EN 0x09
#define I2C_SLV0_ADDR 0x03
#define I2C_SLV0_REG 0x04
#define I2C_SLV0_CTRL 0x05
#define I2C_SLV0_DO 0x06
#define AK09916_ADDRESS 0x0c

//magnetometer registers
#define MAG_CNTL2 0x31
#define MAG_CNTL3 0x32
#define MAG_DATA 0x11

#define ACCEL_XOUT_H 0x2D
#define ACCEL_XOUT_L 0x2E
#define ACCEL_YOUT_H 0x2F
#define ACCEL_YOUR_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32
#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34
#define GYRO_YOUT_H 0x35
#define GYRO_YOUR_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38
#define TEMP_OUT_H 0x39
#define TEMP_OUT_L 0x3A

//macros
#define DISABLE_I2C_SLAVE 0x01

//magnetometer bias removal
#define MAG_BIAS_X -130
#define MAG_BIAS_Y 87
#define MAG_BIAS_Z -170

typedef enum{
	_gyro_range_250dps,
	_gyro_range_500dps,
	_gyro_range_1000dps,
	_gyro_range_2000dps
} gyro_range;

typedef enum{
	_accel_range_2g,
	_accel_range_4g,
	_accel_range_8g,
	_accel_range_16g
} accel_range;

typedef struct{
	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
	int16_t x_mag;
	int16_t y_mag;
	int16_t z_mag;
} icm_20948_data;

void icm_20948_select_reg_bank(uint8_t user_bank);
void icm_20948_read_reg(uint8_t reg, uint8_t* data);
void icm_20948_write_reg(uint8_t reg, uint8_t data);
void icm_20948_remove_gyro_bias(void);
void icm_20948_init(void);
void icm_20948_get_address(uint8_t* icm_address);
void icm_20948_read_temp(int16_t* temp);
void icm_20948_read_data(icm_20948_data* icm_data);
