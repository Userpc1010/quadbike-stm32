/*
* icm20948.c
*
*  Created on: Dec 26, 2020
*      Author: mokhwasomssi
*/

#include "icm20948.h"
#include "../../Core/Inc/tim.h"
#include "../ST7735/st7735.h"

static float gyro_scale_factor;
static float accel_scale_factor;

/* Static Functions */
static void     cs_high();
static void     cs_low();

static void     select_user_bank(userbank ub);

static uint8_t  read_single_icm20948_reg(userbank ub, uint8_t reg);
static void     write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val);
static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len);
static void     write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len);

static uint8_t  read_single_ak09916_reg(uint8_t reg);
static void     write_single_ak09916_reg(uint8_t reg, uint8_t val);
static void     read_multiple_ak09916_reg(uint8_t reg, uint8_t len, uint8_t * data);

void delay_us(uint16_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);

    while((__HAL_TIM_GET_COUNTER(&htim2) - start) < us);
}

/* Main Functions */
void icm20948_init()
{

	while(!icm20948_who_am_i());

	icm20948_device_reset();
	icm20948_wakeup();

	icm20948_clock_source(1);
	icm20948_odr_align_enable();

	icm20948_spi_slave_enable();

	icm20948_gyro_low_pass_filter(6);
	icm20948_accel_low_pass_filter(6);
	//icm20948_gyro_dlpf_bypass();

	icm20948_gyro_sample_rate_divider(0);
	icm20948_accel_sample_rate_divider(0);

	HAL_Delay(1000);

	icm20948_gyro_calibration();
	icm20948_accel_calibration();

	icm20948_gyro_full_scale_select(_500dps);
	icm20948_accel_full_scale_select(_4g);

	ICM20948_enable_irq(1, 0);
}

void ak09916_init()
{
	icm20948_i2c_master_reset();
	icm20948_i2c_master_enable();
	icm20948_i2c_master_clk_frq(15);

	while(!ak09916_who_am_i()) delay_us(10);

	ak09916_soft_reset();
	ak09916_operation_mode_setting(continuous_measurement_100hz);
}

void icm20948_gyro_read(axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_GYRO_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]);
}

void icm20948_accel_read(axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(ub_0, B0_ACCEL_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]) + accel_scale_factor;
	// Add scale factor because calibraiton function offset gravity acceleration.
}

uint8_t ak09916_mag_read(axises* data)
{
	uint8_t temp[6];
	uint8_t drdy, hofl;	// data ready, overflow

	drdy = read_single_ak09916_reg(MAG_ST1) & 0x01;

	if(drdy != 0x01) return 0;

	read_multiple_ak09916_reg(MAG_HXL, 6, temp);

	hofl = read_single_ak09916_reg(MAG_ST2) & 0x08;

	if(hofl) return 0;

	data->x = (int16_t)(temp[1] << 8 | temp[0]);
	data->y = (int16_t)(temp[3] << 8 | temp[2]);
	data->z = (int16_t)(temp[5] << 8 | temp[4]);

	return 1;
}

void icm20948_gyro_read_dps(axises* data)
{
	icm20948_gyro_read(data);

	data->x /= gyro_scale_factor;
	data->y /= gyro_scale_factor;
	data->z /= gyro_scale_factor;
}

void icm20948_accel_read_g(axises* data)
{
	icm20948_accel_read(data);

	data->x /= accel_scale_factor;
	data->y /= accel_scale_factor;
	data->z /= accel_scale_factor;
}

/**
 * @brief Синхронное пакетное чтение Status + Accel + Gyro (32 байта)
 * @return 1 если данные новые (DRDY), 0 если нет или ошибка
 */
uint8_t icm20948_read_imu_sync(axises* gyro, axises* accel)
{
    uint8_t tx_buf[32] = {0};
    uint8_t rx_buf[32] = {0};

    // 0. Подготовка адреса (0x1A - INT_STATUS_1)
    tx_buf[0] = B0_INT_STATUS_1 | READ;
    // Остальные байты tx_buf уже 0x00 благодаря инициализации

    // 1. Обнуляем остальные 31 байт (итого 32)
    for(uint8_t i = 1; i < 32; i++) tx_buf[i] = 0x00;

    // 2. Старт транзакции
    cs_low();

    // Передаем/принимаем 32 байта. Таймаут 1мс (на 7МГц это за глаза)
    if (HAL_SPI_TransmitReceive(ICM20948_SPI, tx_buf, rx_buf, 32, 1000) != HAL_OK) {
        cs_high();
        return 0;
    }

    cs_high();

    // 3. Проверка статуса (регистр 0x1A в rx_buf[1])
    uint8_t status = rx_buf[1];
    if (!(status & 0x01)) {
        return 0; // Новых данных еще нет
    }

    // 4. Сборка сырых данных (Индексы как в твоем parse_dma)
    // Смещение: 0x2D (Accel) - 0x1A (Status) = 19 байт. Индекс = 1 + 19 = 20.

    // Акселерометр (20-25)
    int16_t raw_ax = (int16_t)(rx_buf[20] << 8 | rx_buf[21]);
    int16_t raw_ay = (int16_t)(rx_buf[22] << 8 | rx_buf[23]);
    int16_t raw_az = (int16_t)(rx_buf[24] << 8 | rx_buf[25]) + (int16_t)accel_scale_factor;

    // Гироскоп (26-31)
    int16_t raw_gx = (int16_t)(rx_buf[26] << 8 | rx_buf[27]);
    int16_t raw_gy = (int16_t)(rx_buf[28] << 8 | rx_buf[29]);
    int16_t raw_gz = (int16_t)(rx_buf[30] << 8 | rx_buf[31]);

    // 5. Перевод в физические величины
    accel->x = (float)raw_ax / accel_scale_factor;
    accel->y = (float)raw_ay / accel_scale_factor;
    accel->z = (float)raw_az / accel_scale_factor;

    gyro->x = (float)raw_gx / gyro_scale_factor;
    gyro->y = (float)raw_gy / gyro_scale_factor;
    gyro->z = (float)raw_gz / gyro_scale_factor;

    return 1;
}


/**
 * @brief Запуск асинхронного чтения Статуса + Accel + Gyro через DMA
 * Читаем 32 байта: 1 адрес + 1 статус + 18 байт пропуска + 12 байт данных
 */
void icm20948_read_imu_dma_start(uint8_t* tx_buf, uint8_t* rx_buf)
{
    // 1. Устанавливаем стартовый адрес чтения 0x1A
    tx_buf[0] = B0_INT_STATUS_1 | READ;

    // 2. Обнуляем остальные 31 байт (итого 32)
    for(uint8_t i = 1; i < 32; i++) tx_buf[i] = 0x00;

    cs_low();

    // 3. Запускаем транзакцию ровно на 32 байта
    HAL_SPI_TransmitReceive_DMA(ICM20948_SPI, tx_buf, rx_buf, 32);
}


/**
 * @brief Обработка данных из длинного DMA буфера
 * @return 1 если данные были новыми (DRDY), 0 если ложное срабатывание
 */
uint8_t icm20948_parse_dma(uint8_t* dma_rx_buf, axises* gyro, axises* accel)
{
    cs_high();

    // dma_rx_buf[0] - мусор (время передачи адреса)
    // dma_rx_buf[1] - контент регистра 0x1A (INT_STATUS_1)
    uint8_t status = dma_rx_buf[1];

    // Проверяем бит RAW_DATA_0_RDY_INT (бит 0)
    if (!(status & 0x01)) {
        return 0; // Данные не готовы, выходим
    }

    // Смещения для данных (начинаются с регистра 0x2D):
    // 0x2D (Accel) - 0x1A (Status) = 19 байт разницы.
    // С учетом dma_rx_buf[0] (мусор), индекс Accel_X_H будет: 1 + 19 = 20

    // 1. Сборка сырых данных Акселерометра (Индексы: 20-25)
    int16_t raw_ax = (int16_t)(dma_rx_buf[20] << 8 | dma_rx_buf[21]);
    int16_t raw_ay = (int16_t)(dma_rx_buf[22] << 8 | dma_rx_buf[23]);
    int16_t raw_az = (int16_t)(dma_rx_buf[24] << 8 | dma_rx_buf[25]) + (int16_t)accel_scale_factor;

    // 2. Сборка сырых данных Гироскопа (Индексы: 26-31)
    int16_t raw_gx = (int16_t)(dma_rx_buf[26] << 8 | dma_rx_buf[27]);
    int16_t raw_gy = (int16_t)(dma_rx_buf[28] << 8 | dma_rx_buf[29]);
    int16_t raw_gz = (int16_t)(dma_rx_buf[30] << 8 | dma_rx_buf[31]);

    // 3. Перевод в физику
    accel->x = (float)raw_ax / accel_scale_factor;
    accel->y = (float)raw_ay / accel_scale_factor;
    accel->z = (float)raw_az / accel_scale_factor;

    gyro->x = (float)raw_gx / gyro_scale_factor;
    gyro->y = (float)raw_gy / gyro_scale_factor;
    gyro->z = (float)raw_gz / gyro_scale_factor;

    return 1; // Успешно обработано
}


uint8_t ak09916_mag_read_Gauss(axises* data)
{
	axises temp;

	if(ak09916_mag_read(&temp) == 0)
	return 0;

	data->x = temp.x * 0.0015f;
	data->y = temp.y * 0.0015f;
	data->z = temp.z * 0.0015f;

	return 1;
}


/* Sub Functions */
uint8_t icm20948_who_am_i()
{
	uint8_t icm20948_id = read_single_icm20948_reg(ub_0, B0_WHO_AM_I);

	if(icm20948_id == ICM20948_ID)
		return 1;
	else
		return 0;
}

uint8_t ak09916_who_am_i()
{
	uint8_t ak09916_id = read_single_ak09916_reg(MAG_WIA2);

	if(ak09916_id == AK09916_ID)
		return 1;
	else
		return 0;
}

void icm20948_device_reset()
{
	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
	HAL_Delay(100);
}

void ak09916_soft_reset()
{
	write_single_ak09916_reg(MAG_CNTL3, 0x01);
	HAL_Delay(100);
}

void icm20948_wakeup()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val &= 0xBF;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

void icm20948_sleep()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= 0x40;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
	HAL_Delay(100);
}

void icm20948_spi_slave_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x10;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_reset()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x02;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_enable()
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_USER_CTRL);
	new_val |= 0x20;

	write_single_icm20948_reg(ub_0, B0_USER_CTRL, new_val);
	HAL_Delay(100);
}

void icm20948_i2c_master_clk_frq(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL);
	new_val |= config;

	write_single_icm20948_reg(ub_3, B3_I2C_MST_CTRL, new_val);
}

void icm20948_clock_source(uint8_t source)
{
	uint8_t new_val = read_single_icm20948_reg(ub_0, B0_PWR_MGMT_1);
	new_val |= source;

	write_single_icm20948_reg(ub_0, B0_PWR_MGMT_1, new_val);
}

void icm20948_odr_align_enable()
{
	write_single_icm20948_reg(ub_2, B2_ODR_ALIGN_EN, 0x01);
}

void icm20948_gyro_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);
	new_val |= config << 3;

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_low_pass_filter(uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);
	new_val |= config << 3;

	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}

void icm20948_gyro_dlpf_bypass()
{
    // Читаем текущее значение регистра конфигурации (Bank 2, 0x01)
    uint8_t val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);

    // Сбрасываем бит 0 (GYRO_FCHOICE = 0)
    // Это отключает внутренний цифровой фильтр
    val &= ~(0x01);

    write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, val);
}


void icm20948_gyro_sample_rate_divider(uint8_t divider)
{
	write_single_icm20948_reg(ub_2, B2_GYRO_SMPLRT_DIV, divider);
}

void icm20948_accel_sample_rate_divider(uint16_t divider)
{
	uint8_t divider_1 = (uint8_t)(divider >> 8);
	uint8_t divider_2 = (uint8_t)(0x0F & divider);

	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
	write_single_icm20948_reg(ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);
}

void ak09916_operation_mode_setting(operation_mode mode)
{
	write_single_ak09916_reg(MAG_CNTL2, mode);
	HAL_Delay(100);
}

void icm20948_gyro_calibration()
{
	axises temp;
	int32_t gyro_bias[3] = {0};
	uint8_t gyro_offset[6] = {0};

	for(int i = 0; i < 10000; i++)
	{
		icm20948_gyro_read(&temp);
		gyro_bias[0] += temp.x;
		gyro_bias[1] += temp.y;
		gyro_bias[2] += temp.z;

		delay_us(889); //1125 Hz ~888,88888888888888888888888888889 mcs
		//delay_us(113); //8889 Hz ~112,4985937675779052761840476994 mcs
	}

	gyro_bias[0] /= 10000;
	gyro_bias[1] /= 10000;
	gyro_bias[2] /= 10000;

	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
	// Biases are additive, so change sign on calculated average gyro biases
	gyro_offset[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF;
	gyro_offset[1] = (-gyro_bias[0] / 4)       & 0xFF;
	gyro_offset[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	gyro_offset[3] = (-gyro_bias[1] / 4)       & 0xFF;
	gyro_offset[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	gyro_offset[5] = (-gyro_bias[2] / 4)       & 0xFF;

	write_multiple_icm20948_reg(ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
}


void icm20948_accel_calibration()
{
	axises temp;
	uint8_t* temp2;
	uint8_t* temp3;
	uint8_t* temp4;

	int32_t accel_bias[3] = {0};
	int32_t accel_bias_reg[3] = {0};
	uint8_t accel_offset[6] = {0};

	for(int i = 0; i < 5000; i++)
	{
		icm20948_accel_read(&temp);
		accel_bias[0] += temp.x;
		accel_bias[1] += temp.y;
		accel_bias[2] += temp.z;

		delay_us(889); //1125 Hz ~888,88888888888888888888888888889 mcs
		//delay_us(225); //4448 Hz ~224,82014388489208633093525179856 mcs
	}

	accel_bias[0] /= 5000;
	accel_bias[1] /= 5000;
	accel_bias[2] /= 5000;

	uint8_t mask_bit[3] = {0, 0, 0};

	temp2 = read_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, 2);
	accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
	mask_bit[0] = temp2[1] & 0x01;

	temp3 = read_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, 2);
	accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
	mask_bit[1] = temp3[1] & 0x01;

	temp4 = read_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, 2);
	accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
	mask_bit[2] = temp4[1] & 0x01;

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  	accel_offset[1] = (accel_bias_reg[0])      & 0xFE;
	accel_offset[1] = accel_offset[1] | mask_bit[0];

	accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  	accel_offset[3] = (accel_bias_reg[1])      & 0xFE;
	accel_offset[3] = accel_offset[3] | mask_bit[1];

	accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	accel_offset[5] = (accel_bias_reg[2])      & 0xFE;
	accel_offset[5] = accel_offset[5] | mask_bit[2];

	write_multiple_icm20948_reg(ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
	write_multiple_icm20948_reg(ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
	write_multiple_icm20948_reg(ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
}


// Function which accumulates magnetometer data after device initialization.
// It calculates the bias and scale in the x, y, and z axes.
void icm20948_mag_calibration(float * bias_dest, float * scale_dest)
{
  uint16_t ii = 0, sample_count = 5000; axises temp;

  int32_t mag_bias[3]  = {0, 0, 0}, mag_scale[3] = {0, 0, 0};

  int16_t mag_max[3]  = {0x8000, 0x8000, 0x8000}, mag_min[3]  = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

  ST7735_FillScreen(ST7735_BLACK);

  ST7735_WriteString(0, 0, "Mag Calibration: Wave device in a figure 8 until done!", Font_7x10, ST7735_GREEN, ST7735_BLACK);

  // shoot for ~fifteen seconds of mag data
  // at 100 Hz ODR, new mag data is available every 10 ms

  for (ii = 0; ii < sample_count; ii++)
  {
	if (ak09916_mag_read(&temp)){  // Read the mag data

	mag_temp[0] = temp.x;

	mag_temp[1] = temp.y;

	mag_temp[2] = temp.z;


    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj])
      {
        mag_max[jj] = mag_temp[jj];
      }
      if (mag_temp[jj] < mag_min[jj])
      {
        mag_min[jj] = mag_temp[jj];
      }
    }
	}

	HAL_Delay(10); // At 100 Hz ODR, new mag data is available every 10 ms
  }

  // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // Get hard iron correction
  // Get 'average' x mag bias in counts
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2;
  // Get 'average' y mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2;
  // Get 'average' z mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2;

  // Save mag biases in G for main program
  bias_dest[0] = (float)mag_bias[0] * 0.0015f;// * factoryMagCalibration[0];
  bias_dest[1] = (float)mag_bias[1] * 0.0015f;// * factoryMagCalibration[1];
  bias_dest[2] = (float)mag_bias[2] * 0.0015f;// * factoryMagCalibration[2];

  // Get soft iron correction estimate
  // Get average x axis max chord length in counts
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2;
  // Get average y axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2;
  // Get average z axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2;

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  scale_dest[0] = avg_rad / ((float)mag_scale[0]);
  scale_dest[1] = avg_rad / ((float)mag_scale[1]);
  scale_dest[2] = avg_rad / ((float)mag_scale[2]);

  if(scale_dest[0] <= 0.01f && scale_dest[0] >= -0.01f) scale_dest[0] = 1.0f;
  if(scale_dest[1] <= 0.01f && scale_dest[1] >= -0.01f) scale_dest[1] = 1.0f;
  if(scale_dest[2] <= 0.01f && scale_dest[2] >= -0.01f) scale_dest[2] = 1.0f;

  ST7735_WriteString(0, 50, "Mag Calibration done!", Font_7x10, ST7735_GREEN, ST7735_BLACK);
}


void icm20948_gyro_full_scale_select(gyro_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1);

	switch(full_scale)
	{
		case _250dps :
			new_val |= 0x00;
			gyro_scale_factor = 131.0;
			break;
		case _500dps :
			new_val |= 0x02;
			gyro_scale_factor = 65.5;
			break;
		case _1000dps :
			new_val |= 0x04;
			gyro_scale_factor = 32.8;
			break;
		case _2000dps :
			new_val |= 0x06;
			gyro_scale_factor = 16.4;
			break;
	}

	write_single_icm20948_reg(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_full_scale_select(accel_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG);

	switch(full_scale)
	{
		case _2g :
			new_val |= 0x00;
			accel_scale_factor = 16384;
			break;
		case _4g :
			new_val |= 0x02;
			accel_scale_factor = 8192;
			break;
		case _8g :
			new_val |= 0x04;
			accel_scale_factor = 4096;
			break;
		case _16g :
			new_val |= 0x06;
			accel_scale_factor = 2048;
			break;
	}

	write_single_icm20948_reg(ub_2, B2_ACCEL_CONFIG, new_val);
}


/* Static Functions */
static void cs_high()
{
	HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, SET);
}

static void cs_low()
{
	HAL_GPIO_WritePin(ICM20948_SPI_CS_PIN_PORT, ICM20948_SPI_CS_PIN_NUMBER, RESET);
}

static void select_user_bank(userbank ub)
{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | REG_BANK_SEL;
	write_reg[1] = ub;

	cs_low();
	HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 10);
	cs_high();
}

static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg)
{
	uint8_t read_reg = READ | reg;
	uint8_t reg_val;
	select_user_bank(ub);

	cs_low();
	HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
	HAL_SPI_Receive(ICM20948_SPI, &reg_val, 1, 1000);
	cs_high();

	return reg_val;
}

static void write_single_icm20948_reg(userbank ub, uint8_t reg, uint8_t val)
{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | reg;
	write_reg[1] = val;

	select_user_bank(ub);

	cs_low();
	HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 1000);
	cs_high();
}

static uint8_t* read_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t len)
{
	uint8_t read_reg = READ | reg;
	static uint8_t reg_val[6];
	select_user_bank(ub);

	cs_low();
	HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);
	HAL_SPI_Receive(ICM20948_SPI, reg_val, len, 1000);
	cs_high();

	return reg_val;
}

static void write_multiple_icm20948_reg(userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
{
	uint8_t write_reg = WRITE | reg;
	select_user_bank(ub);

	cs_low();
	HAL_SPI_Transmit(ICM20948_SPI, &write_reg, 1, 1000);
	HAL_SPI_Transmit(ICM20948_SPI, val, len, 1000);
	cs_high();
}

static uint8_t read_single_ak09916_reg(uint8_t reg)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);

	delay_us(50);

	return read_single_icm20948_reg(ub_0, B0_EXT_SLV_SENS_DATA_00);
}

static void write_single_ak09916_reg(uint8_t reg, uint8_t val)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, WRITE | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_DO, val);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x81);

}

static void read_multiple_ak09916_reg(uint8_t reg, uint8_t len, uint8_t * data)
{
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(ub_3, B3_I2C_SLV0_CTRL, 0x80 | len);

	uint8_t read_reg = READ | B0_EXT_SLV_SENS_DATA_00;
	select_user_bank(ub_0);

	delay_us(50);

	cs_low();
	HAL_SPI_Transmit(ICM20948_SPI, &read_reg, 1, 1000);

	for(int8_t i = 0; i < len; i++ ){

	HAL_SPI_Receive(ICM20948_SPI, &data[i], 1, 1000);

	delay_us(50);
	}
	cs_high();
}

uint8_t ICM20948_DRDY(){

  return read_single_icm20948_reg(ub_0, B0_INT_STATUS_1);
}

void ICM20948_enable_irq(uint8_t dataReadyEnable, uint8_t womEnable/* wakeup on motion */)
{
    /* Enable one or both of interrupt sources if required */
    if ( dataReadyEnable ) {
    	write_single_icm20948_reg(ub_0, ICM20948_REG_INT_ENABLE_1, ICM20948_BIT_RAW_DATA_0_RDY_EN);
    }
    else {
    	write_single_icm20948_reg(ub_0, ICM20948_REG_INT_ENABLE_1, 0);
    }

    if ( womEnable ) {
    	write_single_icm20948_reg(ub_0, ICM20948_REG_INT_ENABLE, ICM20948_BIT_WOM_INT_EN);
    }
    else {
    	write_single_icm20948_reg(ub_0, ICM20948_REG_INT_ENABLE, 0);
    }

    /* INT pin: active low, open drain, IT status read clears. It seems that latched mode does not work, the INT pin cannot be cleared if set */
    write_single_icm20948_reg(ub_0, ICM20948_REG_INT_PIN_CFG, ICM20948_BIT_INT_ACTL | ICM20948_BIT_INT_OPEN);

}
