# icm_20948_stm32_library
Library for controlling icm_20948 IMU sensor over SPI

# Quick guide
## SPI Setup
* Clock polarity: HIGH
* Clock phase: 2
* Max clock frequency 7MHz
* Data is delivered MSB first
* Supports Single or Burst Read/Writes

## How to use library
* In header file you can change parameters in ```USER DEFINE``` section to adjust range of accel and gyro
* Call ```icm_20948_init``` to initialize device
* To check connection call ```icm_20948_get_address``` - should output 0xEA
* To read data over dma (preffered) call ```icm_20948_read_data_dma```
* To convert raw data in uint8_t array to ```icm_20948_data``` struct use ```icm_20948_spi_dma_callback```

### Example
```
volatile uint8_t timer_elapsed = 0;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == IMU_HSPI.Instance) {
    	icm_20948_spi_dma_callback();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == SPI2_DR_INT_Pin) {
    uint8_t data;
    icm_20948_read_exti(&data);
    if(data == 0x01 && timer_elapsed == 1){
    	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    	icm_20948_read_data_dma();
    	timer_elapsed = 0;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM10){
		timer_elapsed = 1;
	}
}
```