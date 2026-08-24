/*
* icm20948.h
*
*  Created on: Dec 26, 2020
*      Author: mokhwasomssi
*/


#ifndef __ICM20948_H__
#define	__ICM20948_H__


#include "../../Core/Inc/spi.h"			// header from stm32cubemx code generate

/* User Configuration */
#define ICM20948_SPI					(&hspi3)

#define ICM20948_SPI_CS_PIN_PORT		GPIOA
#define ICM20948_SPI_CS_PIN_NUMBER		GPIO_PIN_15

#define ICM20948_SPI_EXTI_PIN_PORT		GPIOD
#define ICM20948_SPI_EXTI_PIN_NUMBER    GPIO_PIN_1


/* Defines */
#define READ							0x80
#define WRITE							0x00


/* Typedefs */
typedef enum
{
	ub_0 = 0 << 4,
	ub_1 = 1 << 4,
	ub_2 = 2 << 4,
	ub_3 = 3 << 4
} userbank;

typedef enum
{
	_250dps,
	_500dps,
	_1000dps,
	_2000dps
} gyro_full_scale;

typedef enum
{
	_2g,
	_4g,
	_8g,
	_16g
} accel_full_scale;

typedef struct
{
	float x;
	float y;
	float z;
} axises;

typedef enum
{
	power_down_mode = 0,
	single_measurement_mode = 1,
	continuous_measurement_10hz = 2,
	continuous_measurement_20hz = 4,
	continuous_measurement_50hz = 6,
	continuous_measurement_100hz = 8
} operation_mode;

/* Main Functions */

// sensor init function.
// if sensor id is wrong, it is stuck in while.
void icm20948_init();
void ak09916_init();

// 16 bits ADC value. raw data.
void icm20948_gyro_read(axises* data);	
void icm20948_accel_read(axises* data);
uint8_t ak09916_mag_read(axises* data);

// Convert 16 bits ADC value to their unit.
void icm20948_gyro_read_dps(axises* data); 
void icm20948_accel_read_g(axises* data);
uint8_t ak09916_mag_read_Gauss(axises* data);
uint8_t icm20948_read_imu_sync(axises* gyro, axises* accel);

//DMA read
void icm20948_read_imu_dma_start(uint8_t* tx_buf, uint8_t* rx_buf);
uint8_t icm20948_parse_dma(uint8_t* dma_rx_buf, axises* gyro, axises* accel);


/* Sub Functions */
uint8_t icm20948_who_am_i();
uint8_t ak09916_who_am_i();

void icm20948_device_reset();
void ak09916_soft_reset();

void icm20948_wakeup();
void icm20948_sleep();

void icm20948_spi_slave_enable();

void icm20948_i2c_master_reset();
void icm20948_i2c_master_enable();
void icm20948_i2c_master_clk_frq(uint8_t config); // 0 - 15

void icm20948_clock_source(uint8_t source);
void icm20948_odr_align_enable();


/**<x Gyro Bandwidth = 12100 Hz */
/**<7 Gyro Bandwidth = 360 Hz   */
/**<0 Gyro Bandwidth = 200 Hz   */
/**<1 Gyro Bandwidth = 150 Hz   */
/**<2 Gyro Bandwidth = 120 Hz   */
/**<3 Gyro Bandwidth = 51 Hz    */
/**<4 Gyro Bandwidth = 24 Hz    */
/**<5 Gyro Bandwidth = 12 Hz    */
/**<6 Gyro Bandwidth = 6 Hz     */

void icm20948_gyro_low_pass_filter(uint8_t config); // 0 - 7

/**<x Accel Bandwidth = 1210 Hz  */
/**<7 Accel Bandwidth = 470 Hz   */
/**<0 Accel Bandwidth = 246 Hz   */
/**<1 Accel Bandwidth = 246 Hz   */
/**<2 Accel Bandwidth = 111 Hz   */
/**<3 Accel Bandwidth = 50 Hz    */
/**<4 Accel Bandwidth = 24 Hz    */
/**<5 Accel Bandwidth = 12 Hz    */
/**<6 Accel Bandwidth = 6 Hz     */

void icm20948_accel_low_pass_filter(uint8_t config); // 0 - 7

void icm20948_gyro_dlpf_bypass();

// Output Data Rate = 1.125kHz / (1 + divider)
void icm20948_gyro_sample_rate_divider(uint8_t divider);
void icm20948_accel_sample_rate_divider(uint16_t divider);
void ak09916_operation_mode_setting(operation_mode mode);

// Calibration before select full scale.
void icm20948_gyro_calibration();
void icm20948_accel_calibration();
void icm20948_mag_calibration(float * bias_dest, float * scale_dest);

void icm20948_gyro_full_scale_select(gyro_full_scale full_scale);
void icm20948_accel_full_scale_select(accel_full_scale full_scale);

void ICM20948_enable_irq(uint8_t dataReadyEnable, uint8_t womEnable);

uint8_t ICM20948_DRDY();


/* ICM-20948 Registers */
#define ICM20948_ID						0xEA
#define REG_BANK_SEL					0x7F

// USER BANK 0
#define B0_WHO_AM_I						0x00		
#define B0_USER_CTRL					0x03
#define B0_LP_CONFIG					0x05
#define B0_PWR_MGMT_1					0x06
#define B0_PWR_MGMT_2					0x07
#define B0_INT_PIN_CFG					0x0F		
#define B0_INT_ENABLE					0x10
#define B0_INT_ENABLE_1					0x11
#define B0_INT_ENABLE_2					0x12
#define B0_INT_ENABLE_3					0x13
#define B0_I2C_MST_STATUS				0x17		
#define B0_INT_STATUS					0x19		
#define B0_INT_STATUS_1					0x1A
#define B0_INT_STATUS_2					0x1B
#define B0_INT_STATUS_3					0x1C
#define B0_DELAY_TIMEH					0x28
#define B0_DELAY_TIMEL					0x29
#define B0_ACCEL_XOUT_H					0x2D		
#define B0_ACCEL_XOUT_L					0x2E		
#define B0_ACCEL_YOUT_H					0x2F		
#define B0_ACCEL_YOUT_L					0x30		
#define B0_ACCEL_ZOUT_H					0x31		
#define B0_ACCEL_ZOUT_L					0x32	
#define B0_GYRO_XOUT_H					0x33	
#define B0_GYRO_XOUT_L					0x34
#define B0_GYRO_YOUT_H					0x35
#define B0_GYRO_YOUT_L					0x36
#define B0_GYRO_ZOUT_H					0x37
#define B0_GYRO_ZOUT_L					0x38
#define B0_TEMP_OUT_H					0x39		
#define B0_TEMP_OUT_L					0x3A
#define B0_EXT_SLV_SENS_DATA_00			0x3B
#define B0_EXT_SLV_SENS_DATA_01			0x3C
#define B0_EXT_SLV_SENS_DATA_02			0x3D
#define B0_EXT_SLV_SENS_DATA_03			0x3E
#define B0_EXT_SLV_SENS_DATA_04			0x3F
#define B0_EXT_SLV_SENS_DATA_05			0x40
#define B0_EXT_SLV_SENS_DATA_06			0x41
#define B0_EXT_SLV_SENS_DATA_07			0x42
#define B0_EXT_SLV_SENS_DATA_08			0x43
#define B0_EXT_SLV_SENS_DATA_09			0x44
#define B0_EXT_SLV_SENS_DATA_10			0x45
#define B0_EXT_SLV_SENS_DATA_11			0x46
#define B0_EXT_SLV_SENS_DATA_12			0x47
#define B0_EXT_SLV_SENS_DATA_13			0x48
#define B0_EXT_SLV_SENS_DATA_14			0x49
#define B0_EXT_SLV_SENS_DATA_15			0x4A
#define B0_EXT_SLV_SENS_DATA_16			0x4B
#define B0_EXT_SLV_SENS_DATA_17			0x4C
#define B0_EXT_SLV_SENS_DATA_18			0x4D
#define B0_EXT_SLV_SENS_DATA_19			0x4E
#define B0_EXT_SLV_SENS_DATA_20			0x4F
#define B0_EXT_SLV_SENS_DATA_21			0x50
#define B0_EXT_SLV_SENS_DATA_22			0x51
#define B0_EXT_SLV_SENS_DATA_23			0x52
#define B0_FIFO_EN_1					0x66	
#define B0_FIFO_EN_2					0x67
#define B0_FIFO_RST						0x68
#define B0_FIFO_MODE					0x69
#define B0_FIFO_COUNTH					0X70
#define B0_FIFO_COUNTL					0X71
#define B0_FIFO_R_W						0x72
#define B0_DATA_RDY_STATUS				0x74
#define B0_FIFO_CFG						0x76	

// USER BANK 1
#define B1_SELF_TEST_X_GYRO				0x02	
#define B1_SELF_TEST_Y_GYRO				0x03
#define B1_SELF_TEST_Z_GYRO				0x04
#define B1_SELF_TEST_X_ACCEL			0x0E	
#define B1_SELF_TEST_Y_ACCEL			0x0F
#define B1_SELF_TEST_Z_ACCEL			0x10
#define B1_XA_OFFS_H					0x14	
#define B1_XA_OFFS_L					0x15
#define B1_YA_OFFS_H					0x17
#define B1_YA_OFFS_L					0x18
#define B1_ZA_OFFS_H					0x1A
#define B1_ZA_OFFS_L					0x1B
#define B1_TIMEBASE_CORRECTION_PLL		0x28	

// USER BANK 2
#define B2_GYRO_SMPLRT_DIV				0x00	
#define B2_GYRO_CONFIG_1				0x01	
#define B2_GYRO_CONFIG_2				0x02
#define B2_XG_OFFS_USRH					0x03	
#define B2_XG_OFFS_USRL 				0x04
#define B2_YG_OFFS_USRH					0x05
#define B2_YG_OFFS_USRL					0x06
#define B2_ZG_OFFS_USRH					0x07
#define B2_ZG_OFFS_USRL					0x08
#define B2_ODR_ALIGN_EN					0x09	
#define B2_ACCEL_SMPLRT_DIV_1			0x10	
#define B2_ACCEL_SMPLRT_DIV_2			0x11		
#define B2_ACCEL_INTEL_CTRL				0x12		
#define B2_ACCEL_WOM_THR				0x13
#define B2_ACCEL_CONFIG					0x14
#define B2_ACCEL_CONFIG_2				0x15
#define B2_FSYNC_CONFIG					0x52
#define B2_TEMP_CONFIG					0x53
#define B2_MOD_CTRL_USR					0X54

// USER BANK 3
#define B3_I2C_MST_ODR_CONFIG			0x00
#define B3_I2C_MST_CTRL					0x01
#define B3_I2C_MST_DELAY_CTRL			0x02	
#define B3_I2C_SLV0_ADDR				0x03
#define B3_I2C_SLV0_REG					0x04		
#define B3_I2C_SLV0_CTRL				0x05
#define B3_I2C_SLV0_DO					0x06
#define B3_I2C_SLV1_ADDR				0x07		
#define B3_I2C_SLV1_REG					0x08		
#define B3_I2C_SLV1_CTRL				0x09
#define B3_I2C_SLV1_DO					0x0A
#define B3_I2C_SLV2_ADDR				0x0B		
#define B3_I2C_SLV2_REG					0x0C		
#define B3_I2C_SLV2_CTRL				0x0D
#define B3_I2C_SLV2_DO					0x0E
#define B3_I2C_SLV3_ADDR				0x0F		
#define B3_I2C_SLV3_REG					0x10		
#define B3_I2C_SLV3_CTRL				0x11
#define B3_I2C_SLV3_DO					0x12
#define B3_I2C_SLV4_ADDR				0x13	
#define B3_I2C_SLV4_REG					0x14		
#define B3_I2C_SLV4_CTRL				0x15
#define B3_I2C_SLV4_DO					0x16
#define B3_I2C_SLV4_DI					0x17
	

/* AK09916 Registers */
#define AK09916_ID						0x09
#define MAG_SLAVE_ADDR                  0x0C

#define MAG_WIA2						0x01
#define MAG_ST1							0x10
#define MAG_HXL							0x11
#define MAG_HXH							0x12
#define MAG_HYL							0x13
#define MAG_HYH							0x14
#define MAG_HZL							0x15
#define MAG_HZH							0x16
#define MAG_ST2							0x18
#define MAG_CNTL2						0x31
#define MAG_CNTL3						0x32
#define MAG_TS1							0x33
#define MAG_TS2							0x34

/**************************************************************************//**
* @name ICM20948 register banks
* @{
******************************************************************************/

#define ICM20948_REG_INT_PIN_CFG            0x0F    /**< Interrupt Pin Configuration register                   */
#define ICM20948_BIT_INT_ACTL               0x80                        /**< Active low setting bit                                 */
#define ICM20948_BIT_INT_OPEN               0x40                        /**< Open collector configuration bit                       */
#define ICM20948_BIT_INT_LATCH_EN           0x20                        /**< Latch enable bit                                       */

#define ICM20948_REG_INT_ENABLE             0x10    /**< Interrupt Enable register                              */
#define ICM20948_BIT_WOM_INT_EN             0x08                        /**< Wake-up On Motion enable bit                           */

#define ICM20948_REG_INT_ENABLE_1           0x11    /**< Interrupt Enable 1 register                            */
#define ICM20948_BIT_RAW_DATA_0_RDY_EN      0x01                        /**< Raw data ready interrupt enable bit                    */

#define ICM20948_REG_INT_ENABLE_2           0x12    /**< Interrupt Enable 2 register                            */
#define ICM20948_BIT_FIFO_OVERFLOW_EN_0     0x01                        /**< FIFO overflow interrupt enable bit                     */

#endif	/* __ICM20948_H__ */
