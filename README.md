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
* To read data call ```icm_20948_read_data```