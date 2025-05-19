#include "icm_20948.h"

static void icm_start_spi(void) {
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}

static void icm_stop_spi(void) {
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

static void icm_transmit_byte(uint8_t byte) {
	HAL_SPI_Transmit(&IMU_HSPI, &byte, 1, 100);
}

static void icm_receive_bytes(uint16_t no_bytes, uint8_t *byte) {
	HAL_SPI_Receive(&IMU_HSPI, byte, no_bytes, 100);
}

static void ak09916_write_reg(uint8_t reg, uint8_t data){
	icm_20948_select_reg_bank(3);
	icm_20948_write_reg(I2C_SLV0_ADDR, AK09916_ADDRESS);
	icm_20948_write_reg(I2C_SLV0_REG, reg);
	icm_20948_write_reg(I2C_SLV0_DO, data);
	HAL_Delay(50);
	icm_20948_write_reg(I2C_SLV0_CTRL, 0x80|0x01);
	HAL_Delay(50);
}

static void ak09916_read_reg(uint8_t reg, uint8_t len){
	icm_20948_select_reg_bank(3);

	icm_20948_write_reg(I2C_SLV0_ADDR, 0x80|AK09916_ADDRESS);
	icm_20948_write_reg(I2C_SLV0_REG, reg);
	HAL_Delay(50);
	icm_20948_write_reg(I2C_SLV0_CTRL, 0x80|len);
	HAL_Delay(50);
}

static void ak09916_mag_init(void){
	// enable magnetometer
	uint8_t user_ctrl_data;
	icm_20948_select_reg_bank(0);
	icm_20948_read_reg(USER_CTRL, &user_ctrl_data);
	icm_20948_write_reg(USER_CTRL, user_ctrl_data | 0x02);
	HAL_Delay(100);

	icm_20948_read_reg(USER_CTRL, &user_ctrl_data);
	icm_20948_write_reg(USER_CTRL, user_ctrl_data | 0x20);
	HAL_Delay(10);

	icm_20948_select_reg_bank(3);
	icm_20948_write_reg(I2C_MST_CLK, 0x07);
	HAL_Delay(10);
	icm_20948_select_reg_bank(0);
	icm_20948_write_reg(LP_CONFIG, 0x40);
	HAL_Delay(10);

	icm_20948_select_reg_bank(3);
	icm_20948_write_reg(I2C_MST_ODR_CONFIG, 0x03);
	HAL_Delay(50);

	//reset magnetometer
	ak09916_write_reg(MAG_CNTL3, 0x01);
	HAL_Delay(100);

	//continous measurment mode
	ak09916_write_reg(MAG_CNTL2, 0x08);
}


void icm_20948_select_reg_bank(uint8_t user_bank) {
	if (user_bank > 3)
		return;
	icm_start_spi();
	user_bank = user_bank << 4;
	icm_transmit_byte(REG_BANK_SEL);
	icm_transmit_byte(user_bank);
	icm_stop_spi();
}

void icm_20948_read_reg(uint8_t reg, uint8_t *data) {
	uint8_t reg_read_flag = reg | 0x80;
	icm_start_spi();
	icm_transmit_byte(reg_read_flag);
	icm_receive_bytes(1, data);
	icm_stop_spi();
}

void icm_20948_read_reg_burst(uint16_t no_regs, uint8_t reg, uint8_t* data){
	uint8_t reg_read_flag = reg | 0x80;
	icm_start_spi();
	icm_transmit_byte(reg_read_flag);
	icm_receive_bytes(no_regs, data);
	icm_stop_spi();
}

void icm_20948_write_reg(uint8_t reg, uint8_t data) {
	icm_start_spi();
	icm_transmit_byte(reg);
	icm_transmit_byte(data);
	icm_stop_spi();
}

void icm_20948_remove_gyro_bias(void){
	icm_20948_data icm_data;
	int32_t x_mes_sum = 0, y_mes_sum = 0, z_mes_sum = 0;
	for(int i=0; i<100; i++){
		icm_20948_read_data(&icm_data);
		x_mes_sum += icm_data.x_gyro;
		y_mes_sum += icm_data.y_gyro;
		z_mes_sum += icm_data.z_gyro;
	}
	int16_t x_bias, y_bias, z_bias;
	x_bias = -(int16_t)(x_mes_sum/400);
	y_bias = -(int16_t)(y_mes_sum/400);
	z_bias = -(int16_t)(z_mes_sum/400);
	icm_20948_select_reg_bank(2);
	icm_20948_write_reg(XG_OFFS_USRH, (uint8_t)(x_bias>>8));
	icm_20948_write_reg(XG_OFFS_USRL, (uint8_t)(x_bias));
	icm_20948_write_reg(YG_OFFS_USRH, (uint8_t)(y_bias>>8));
	icm_20948_write_reg(YG_OFFS_USRL, (uint8_t)(y_bias));
	icm_20948_write_reg(ZG_OFFS_USRH, (uint8_t)(z_bias>>8));
	icm_20948_write_reg(ZG_OFFS_USRL, (uint8_t)(z_bias));
}

void icm_20948_init(void) {
	icm_20948_select_reg_bank(0);
	icm_20948_write_reg(PWR_MGMT_1, 0xc1);
	HAL_Delay(100);
	icm_20948_write_reg(PWR_MGMT_1, 0x01);

	icm_20948_select_reg_bank(2);
	// data sampled at the same time
	icm_20948_write_reg(ODR_ALIGN_EN, 0x01);

	// gyro config
	icm_20948_remove_gyro_bias();
	icm_20948_write_reg(GYRO_SMPLRT_DIV, 0x00);
	icm_20948_write_reg(GYRO_CONFIG, ((GYRO_RANGE << 1) | 0x01));

	// accel config
	icm_20948_write_reg(ACCEL_SMPLRT_DIV_1, 0x00);
	icm_20948_write_reg(ACCEL_SMPLRT_DIV_2, 0x00);
	icm_20948_write_reg(ACCEL_CONFIG, ((ACCEL_RANGE << 1) | 0x01));

	//disable i2c and enable magnetometer
	uint8_t user_ctrl_data;
	icm_20948_select_reg_bank(0);
	icm_20948_read_reg(USER_CTRL, &user_ctrl_data);
	icm_20948_write_reg(USER_CTRL, user_ctrl_data | 0x10);
	HAL_Delay(100);
	ak09916_mag_init();
	ak09916_read_reg(MAG_DATA, 8);
}



void icm_20948_get_address(uint8_t *icm_address) {
	icm_20948_select_reg_bank(0);
	icm_20948_read_reg(WHO_AM_I, icm_address);
}

void icm_20948_read_temp(int16_t *temp) {
	uint16_t no_data_bytes = 2;
	uint8_t data_rx[no_data_bytes];
	icm_20948_select_reg_bank(0);
	icm_20948_read_reg_burst(no_data_bytes, (TEMP_OUT_H | 0x08), data_rx);
	*temp = (data_rx[0] << 8) | data_rx[1];
}

void icm_20948_read_data(icm_20948_data *icm_data) {
	uint16_t no_data_bytes = 22;
	uint8_t data_rx[no_data_bytes];

	icm_20948_select_reg_bank(0);
	icm_20948_read_reg_burst(no_data_bytes,(ACCEL_XOUT_H | 0x80), data_rx);

	icm_data->x_accel = (data_rx[0] << 8) | data_rx[1];
	icm_data->y_accel = (data_rx[2] << 8) | data_rx[3];
	icm_data->z_accel = (data_rx[4] << 8) | data_rx[5];

	icm_data->x_gyro = (data_rx[6] << 8) | data_rx[7];
	icm_data->y_gyro = (data_rx[8] << 8) | data_rx[9];
	icm_data->z_gyro = (data_rx[10] << 8) | data_rx[11];

	icm_data->x_mag = ((data_rx[15] << 8) | data_rx[14]) - MAG_BIAS_X;
	icm_data->y_mag = ((data_rx[17] << 8) | data_rx[16]) - MAG_BIAS_Y;
	icm_data->z_mag = ((data_rx[19] << 8) | data_rx[18]) - MAG_BIAS_Z;

}
