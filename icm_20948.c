#include "icm_20948.h"

static void icm_start_spi(void){
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}

static void icm_stop_spi(void){
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

static void icm_transmit_byte(uint8_t byte){
	HAL_SPI_Transmit(&IMU_HSPI, &byte, 1, 100);
}

static void icm_receive_byte(uint8_t* byte){
	HAL_SPI_Receive(&IMU_HSPI, byte, 1, 100);
}

void icm_20948_select_reg_bank(uint8_t user_bank){
	if(user_bank>3) return;
	icm_start_spi();
	user_bank = user_bank << 4;
	icm_transmit_byte(REG_BANK_SEL);
	icm_transmit_byte(user_bank);
	icm_stop_spi();
}

void icm_20948_read_reg(uint8_t reg, uint8_t* data){
	uint8_t reg_read_flag = reg|0x80;
	icm_start_spi();
	icm_transmit_byte(reg_read_flag);
	icm_receive_byte(data);
	icm_stop_spi();
}

void icm_20948_write_reg(uint8_t reg, uint8_t data){
	icm_start_spi();
	icm_transmit_byte(reg);
	icm_transmit_byte(data);
	icm_stop_spi();
}

void icm_20948_init(void){
	icm_20948_select_reg_bank(0);
	icm_20948_write_reg(PWR_MGMT_1, 0xc1);
	HAL_Delay(100);
	icm_20948_write_reg(PWR_MGMT_1, 0x01);

	icm_20948_select_reg_bank(2);
	// data sampled at the same time
	icm_20948_write_reg(ODR_ALIGN_EN, 0x01);

	// gyro config
	icm_20948_write_reg(GYRO_SMPLRT_DIV, 0x00);
	icm_20948_write_reg(GYRO_CONFIG, ((GYRO_RANGE << 1) | 0x01));

	// accel config
	icm_20948_write_reg(ACCEL_SMPLRT_DIV_1, 0x00);
	icm_20948_write_reg(ACCEL_SMPLRT_DIV_2, 0x00);
	icm_20948_write_reg(ACCEL_CONFIG, ((ACCEL_RANGE << 1) | 0x01));

	//disable i2c
	uint8_t user_ctrl_data;
	icm_20948_select_reg_bank(0);
	icm_20948_read_reg(USER_CTRL, &user_ctrl_data);
	icm_20948_write_reg(USER_CTRL, user_ctrl_data|0x10);
}

void icm_2048_get_address(uint8_t* icm_address){
	icm_20948_select_reg_bank(0);
	icm_20948_read_reg(WHO_AM_I, icm_address);
}

void icm_20948_read_temp(int16_t* temp){
	uint8_t temp_h;
	uint8_t temp_l;

	icm_20948_select_reg_bank(0);
	icm_20948_read_reg(TEMP_OUT_L, &temp_l);
	icm_20948_read_reg(TEMP_OUT_H, &temp_h);
	*temp = (temp_h << 8) | temp_l;
}

void icm_20948_read_data(icm_20948_data* icm_data){
	uint8_t data_rx[12];
	uint8_t temp_data = ACCEL_XOUT_H|0x80;

	icm_20948_select_reg_bank(0);
	icm_start_spi();
	HAL_SPI_Transmit(&IMU_HSPI, &temp_data, 1, 1000);
	HAL_SPI_Receive(&IMU_HSPI, data_rx, 12, 1000);
	icm_stop_spi();

	icm_data->x_accel = (data_rx[0] << 8) | data_rx[1];
	icm_data->y_accel = (data_rx[2] << 8) | data_rx[3];
	icm_data->z_accel = (data_rx[4] << 8) | data_rx[5];
	icm_data->x_gyro = (data_rx[6] << 8) | data_rx[7];
	icm_data->y_gyro = (data_rx[8] << 8) | data_rx[9];
	icm_data->z_gyro = (data_rx[10] << 8) | data_rx[11];

}
