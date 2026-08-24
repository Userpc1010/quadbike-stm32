/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "quadspi.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../MahonyAHRS/MahonyAHRS.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Режимы работы
typedef enum {
    CTRL_MODE_MANUAL = 0,      // ручной / Vector Pursuit (PID)
    CTRL_MODE_MPPI = 1         // прямое управление от MPPI
} ctrl_mode_t;

// Структура для PID-режима
struct PCA9685 { // Управление движением
    float w_cmd;
    float setpoint;
    uint8_t stop;
};

// Структура для MPPI-режима
struct MPPI_Ctrl {
    float steer_angle;   // рад
    float throttle;      // -1..1
    float brake;         // 0..1
};

struct Telem // Управление движением
{
 // Команды
 float w_yaw;
 float stering_angle;
 float velosity_1d_mps;
};

typedef struct
{
  int16_t velocity;
  uint32_t last_counter_value;
}encoder_instance;

typedef struct {
    float gx_bias;  // rad/s
    float gy_bias;
    float gz_bias;
} gyro_bias_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#define MAX_PRECISION	(10)

// Параметры AS5600 через аналоговый выход
#define AS5600_ADC_MAX    4095    // 3.3 В → 4095 ADC
#define STEER_ADC_MIN     1127U   // Значение АЦП при мин. угле 1067 (0.86В) мультиметр, смещено 0.908В Смещение составило +0.048В
#define STEER_ADC_MAX     3013U   // Значение АЦП при макс. угле 2953 (2.38В) мультиметр, смещено 2.427В Смещение составило +0.047В
#define STEER_RANGE_DEG   180.0f  // Укажите реальный ход сервы в градусах (например, 180) 1.667В Это то напряжение, которое контроллер теперь считает идеальным центром

// Коэффициент фильтрации (0.01 - очень плавно, 1.0 - без фильтра)
#define STEER_FILTER_K    0.15f

// Пороги безопасности
#define SAFE_SPEED_FORWARD_CHANGE  20.0f
#define SAFE_SPEED_BACKWARD_CHANGE -20.0f

// ================================ PID Коэффициенты Мотора ================================ //

#define kp_m 5.0f
#define kd_m 10.0f
#define ki_m 0.00001f    // Интегральный коэффициент
#define deriv_threshold_speed 0.25f

// Ограничения интеграла (защита от переполнения и накопления)
#define INTEGRAL_LIMIT_MOTOR   1000000000.0f    // Максимальное накопление интеграла (вперёд)
#define INTEGRAL_LIMIT_MOTOR_NEG -1000000000.0f // Минимальное накопление интеграла (назад)

#define INTEGRAL_LIMIT_BRAKE   1000000000.0f      // Максимальное накопление интеграла тормоза
#define INTEGRAL_LIMIT_BRAKE_NEG 0.0f             // Тормоз только вперёд (не накапливаем отрицательный)

#define kp_b 1.0f
#define kd_b 2.0f
#define ki_b 0.00000001f
#define deriv_threshold_brake 0.25f

// Пределы для двигателя (PCA9685_SetPin ожидает 0-4095)
#define MOTOR_PWM_MAX  4095
#define MOTOR_PWM_MIN  -4095

// Пределы для сервопривода (PCA9685_SetServoAngle ожидает 0-180 градусов)
#define SERVO_ANGLE_MAX  180
#define SERVO_ANGLE_MIN  0

// ================================ Feed Forward (упреждение) ================================ //

// Коэффициент feed forward (помогает быстрее реагировать на задание скорости)
#define FF_GAIN_MOTOR   0.5f    // 0.5 = 50% упреждения

// ============================================================
// ПАРАМЕТРЫ РЕГУЛЯТОРА УГЛОВОЙ СКОРОСТИ
// ============================================================

// Параметры PID
#define YAW_KP 0.1f                       // Пропорциональный коэффициент
#define YAW_KI 0.005f                    // Интегральный коэффициент
#define YAW_KD 0.05f                    // Дифференциальный коэффициент

// Геометрия
#define WHEELBASE 0.46f   // Колёсная база [м]

// Ограничения
#define MAX_STEER_ANGLE 0.523599f    // Макс. угол сервы [рад] (~30 градусов ранее был 20)
#define max_centrifugal 4.0f      // макс. допустимое боковое ускорение (м/с²) ≈0.4g
#define safe_max_coff   1.25f     // Буфер для ограничения угла поврота колеса по центробежной силе

// Константы
#define ENCODER_TICKS_PER_REV   4096        // 1024 PPR * 4
#define GEAR_RATIO              6.545454f   // 72/11
#define WHEEL_DIAMETER          0.945f      // м
#define WHEEL_CIRCUMFERENCE     (WHEEL_DIAMETER * M_PI)  // 2.969 м
#define Servo_to_wheel_ratio    0.9166f      // Передаточное число для сервы ((20 + 35) / 2) / 30 =.

// Коэффициент: тики → метры
#define TICKS_TO_METERS         (WHEEL_CIRCUMFERENCE / (ENCODER_TICKS_PER_REV * GEAR_RATIO))

// ============================================================
// ПАРАМЕТРЫ HEARTBEAT (контроль связи с Mini-PC)
// ============================================================

#define HEARTBEAT_TIMEOUT_MS     500     // Таймаут связи (мс)
#define HEARTBEAT_COUNTER_MAX     10     // Максимум пропущенных пакетов

#define YAW_RATE_CUTOFF_FREQ 2.0f

//#define GYRO_CUTOFF_FREQ 0.00692f
//#define ACCEL_CUTOFF_FREQ 0.00692f
// Раньше был 0.00692 для 9кГц, теперь для 1125Гц при той же инерции:
#define GYRO_CUTOFF_FREQ 0.0543f
#define ACCEL_CUTOFF_FREQ 0.0543f
#define YAW_RATE_ALPHA 0.234f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

  float vFilt;
  float vPrev;

  float velosity;
  float servo_angle_deg;
  float filtered_angle = 0.0f;

  // Heartbeat (контроль связи с Mini-PC)
  uint32_t last_rx_time;           // Время последнего полученного пакета
  uint8_t heartbeat_fail_counter;  // Счётчик пропущенных heartbeat
  uint8_t connection_lost;         // Флаг потери связи

  float integral_error;
  float prev_error;
  float prev_target_steer;

  int16_t AccData[3];
  int16_t GyroData[3];
  int16_t Mag[3];

  float roll, pitch, yaw;

  float prev_yaw_rad;

   axises gyro;
   axises accel;
   axises accel_Mahony;

   // Структуры фильтров для 3-х осей
   arm_biquad_casd_df1_inst_f32 S_gyroX, S_gyroY, S_gyroZ;

   // Буферы состояний (для каждого фильтра 2-го порядка нужно 4 float)
   float32_t gyroX_state[4];
   float32_t gyroY_state[4];
   float32_t gyroZ_state[4];

   // �?спользуем атрибут секции .dtcm_data (проверь свой linker script, обычно он так называется)
   __attribute__((section(".dtcm_data"))) float accX_buffer[5] = {0};
   __attribute__((section(".dtcm_data"))) float accY_buffer[5] = {0};
   __attribute__((section(".dtcm_data"))) float accZ_buffer[5] = {0};

   float gx_filt = 0, gy_filt = 0, gz_filt = 0;
   float ax_filt = 0, ay_filt = 0, az_filt = 0;
   float yaw_rate_filtered = 0.0f;
   uint8_t yaw_filter_initialized = 0;

   // LPF 10Hz при частоте дискретизации 9000Hz, Q=0.707
   // Формат CMSIS: {b0, b1, b2, a1, a2}
   // (a1 и a2 инвертированы)
   //float32_t gyro_biquad_coeffs[5] = {
   //    0.00001215f, 0.00002431f, 0.00001215f, // b0, b1, b2
   //    1.99011385f, -0.99016247f               // -a1, -a2
   //};

   // LPF 10Hz при частоте дискретизации 1125Hz, Q=0.707
   // Формат CMSIS: {b0, b1, b2, a1, a2}
   // (a1 и a2 инвертированы для arm_biquad_cascade_df1_f32)
   float32_t gyro_biquad_coeffs[5] = {
       0.000756f, 0.001512f, 0.000756f, // b0, b1, b2
       1.920810f, -0.923835f            // -a1, -a2
   };

  // Размер 32 идеально совпадает с размером кэш-линии H7
  uint8_t imu_dma_tx[32] __attribute__((section(".ARM.__at_0x24000000"))) __attribute__((aligned(32)));
  uint8_t imu_dma_rx[32] __attribute__((section(".ARM.__at_0x24000000"))) __attribute__((aligned(32)));

  float k;
  float filVal;

  float sum;
  uint16_t sumcount;

  float sum_M;
  uint16_t sumcount_M;

  uint32_t lastUpdate_check;
  uint32_t lastUpdate; // used to calculate integration interval
  uint32_t lastUpdate_encoder;
  uint32_t lastUpdate_icm20948;
  uint32_t Now;                         // used to calculate integration interval

  uint32_t deltaTime;

  float deltaT;

  float last_Integral_Motor;
  float last_Integral_Brake;

  float Last_Error_Motor;
  float Last_Error_Brake;

  int16_t Output_Motor;
  int16_t Output_Brake;
  int16_t Output_ServoBrake;

   uint8_t flag_pca9685;
   uint8_t flag_encoder;
   uint8_t flag_nrf24l01;

   // data array to be read
   uint8_t rx_data[NRF24L01P_PAYLOAD_LENGTH] = {0};

   uint8_t UART4_BUFFER [256] = {0};
   uint8_t UART5_BUFFER [256] = {0};

   volatile ctrl_mode_t current_mode = CTRL_MODE_MANUAL;
   struct PCA9685 pca9685;
   struct MPPI_Ctrl mppi_ctrl;

   struct Telem telem;

   encoder_instance encoder;

   //char buffer[10];

  #pragma pack(push, 1) // Гарантируем отсутствие "дырок" между float в структуре
  typedef struct {
    uint8_t header[4];      // "TELE"
    float w_yaw;            // 4 байта
    float stering_angle;    // 4 байта
    float velosity_1d_mps;  // 4 байта
  } TelePacket;
  #pragma pack(pop)

  TelePacket tele_out = {.header = {'T', 'E', 'L', 'E'}}; // Заголовок ставим один раз

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// for tx interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END PFP */

void set_servo_angle_tim5(float angle_deg);
void send_mode_ack(ctrl_mode_t mode);
void reset_controllers(void);
float getSafeMaxSteer(float v_fwd);
float compute_target_steer(float w_des, float v_current);
float yaw_rate_controller(float dt);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ============================================================
// ФУНКЦИИ
// ============================================================


static const double rounders[MAX_PRECISION + 1] =
{
	0.5,				// 0
	0.05,				// 1
	0.005,				// 2
	0.0005,				// 3
	0.00005,			// 4
	0.000005,			// 5
	0.0000005,			// 6
	0.00000005,			// 7
	0.000000005,		// 8
	0.0000000005,		// 9
	0.00000000005		// 10
};


char * ftoa(double f, char * buf, int precision)
{
	char * ptr = buf;
	char * p = ptr;
	char * p1;
	char c;
	long intPart;

	// check precision bounds
	if (precision > MAX_PRECISION)
		precision = MAX_PRECISION;

	// sign stuff
	if (f < 0)
	{
		f = -f;
		*ptr++ = '-';
	}

	if (precision < 0)  // negative precision == automatic precision guess
	{
		if (f < 1.0) precision = 6;
		else if (f < 10.0) precision = 5;
		else if (f < 100.0) precision = 4;
		else if (f < 1000.0) precision = 3;
		else if (f < 10000.0) precision = 2;
		else if (f < 100000.0) precision = 1;
		else precision = 0;
	}

	// round value according the precision
	if (precision)
		f += rounders[precision];

	// integer part...
	intPart = f;
	f -= intPart;

	if (!intPart)
		*ptr++ = '0';
	else
	{
		// save start pointer
		p = ptr;

		// convert (reverse order)
		while (intPart)
		{
			*p++ = '0' + intPart % 10;
			intPart /= 10;
		}

		// save end pos
		p1 = p;

		// reverse result
		while (p > ptr)
		{
			c = *--p;
			*p = *ptr;
			*ptr++ = c;
		}

		// restore end pos
		ptr = p1;
	}

	// decimal part
	if (precision)
	{
		// place decimal point
		*ptr++ = '.';

		// convert
		while (precision--)
		{
			f *= 10.0;
			c = f;
			*ptr++ = '0' + c;
			f -= c;
		}
	}

	// terminating zero
	*ptr = 0;

	return buf;
}

/**
 * @brief Сброс регулятора
 */
void reset_yaw_controller(void)
{
    integral_error = 0.0f;
    prev_error = 0.0f;
    prev_target_steer = 0.0f;
}

void reset_controllers(void) {
    integral_error = 0.0f;
    prev_error = 0.0f;
    last_Integral_Motor = 0.0f;
    last_Integral_Brake = 0.0f;
    Last_Error_Motor = 0.0f;
    Last_Error_Brake = 0.0f;

    // Возврат сервы в центр
    servo_angle_deg = 90.0f;
    set_servo_angle_tim5(servo_angle_deg);

    // Остановка мотора
    Output_Motor = 0;
    Output_Brake = 0;
}

// Подтверждение режима

void send_mode_ack(ctrl_mode_t mode) {
    uint8_t ack[5] = {'M','O','D','E', (uint8_t)mode};
    HAL_UART_Transmit_DMA(&huart4, ack, 5);
}

/**
 * @brief Проверка связи с Mini-PC
 * Вызывать в основном цикле (loop)
 */
void check_connection(void)
{
    uint32_t now = HAL_GetTick();

    // Если связь была потеряна, но команда уже сброшена — ничего не делаем
    if(connection_lost) return;

    // Проверяем, был ли пакет за последние HEARTBEAT_TIMEOUT_MS мс
    if(now - last_rx_time > HEARTBEAT_TIMEOUT_MS)
    {
        heartbeat_fail_counter++;

        if(heartbeat_fail_counter >= HEARTBEAT_COUNTER_MAX)
        {
            // Связь потеряна — аварийная остановка
            connection_lost = 1;

            current_mode = CTRL_MODE_MANUAL;

            pca9685.w_cmd = 0.0f;      // Обнуляем команду угловой скорости
            pca9685.setpoint = 0.0f;    // Обнуляем команду линейной скорости

            mppi_ctrl.steer_angle = 0.0f;
            mppi_ctrl.throttle = 0.0f;
            mppi_ctrl.brake = 1.0f;  // полный тормоз

            // Опционально: включить аварийную остановку
            // pca9685.stop = 2;        // если есть такая функция

            // Сброс PID регулятора (чтобы не было рывка при восстановлении)
            reset_controllers();
        }
    }
    else
    {
        // Связь есть — сбрасываем счётчик
        heartbeat_fail_counter = 0;
    }
}

void init_gyro_filters(void) {
    arm_biquad_cascade_df1_init_f32(&S_gyroX, 1, gyro_biquad_coeffs, gyroX_state);
    arm_biquad_cascade_df1_init_f32(&S_gyroY, 1, gyro_biquad_coeffs, gyroY_state);
    arm_biquad_cascade_df1_init_f32(&S_gyroZ, 1, gyro_biquad_coeffs, gyroZ_state);
}

/**
 * @brief Чтение угла с калибровкой и фильтрацией шумов
 * @return Угол в градусах (0 - STEER_RANGE_DEG)
 */
float read_steering_angle(void)
{
    uint16_t adc_value = 0;

    // 1. Чтение АЦП
    HAL_ADC_Start(&hadc3);
    if(HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
        adc_value = HAL_ADC_GetValue(&hadc3);
    }
    HAL_ADC_Stop(&hadc3);

    // 2. Масштабирование (Map) из сырых данных в градусы
    // Формула: (val - min) * range / (max - min)
    float raw_angle = (float)(adc_value - STEER_ADC_MIN) * STEER_RANGE_DEG / (float)(STEER_ADC_MAX - STEER_ADC_MIN);

    // 3. Ограничение (Clamp), чтобы не вылетало за границы при шумах
    if (raw_angle < 0.0f) raw_angle = 0.0f;
    if (raw_angle > STEER_RANGE_DEG) raw_angle = STEER_RANGE_DEG;

    // 4. Цифровая фильтрация (EMA фильтр) для стабильности показаний
    filtered_angle = (raw_angle * STEER_FILTER_K) + (filtered_angle * (1.0f - STEER_FILTER_K));

    return filtered_angle;
}

// Для удобства управления: -180°..0..+180°
float get_steering_angle_signed(void)
{
    float angle = read_steering_angle();

    // Преобразуем 0-360° в -180..+180°
    if(angle > 180.0f) angle = angle - 360.0f;
    if(angle < -180.0f) angle = angle + 360.0f;

    return angle;  // -180..+180
}

/**
 * @brief Ограничение угла поврота сервы в зависимости от центробежной скорости
 *
 */
float getSafeMaxSteer(float v_fwd) {
    if (fabs(v_fwd) < 0.5f) return MAX_STEER_ANGLE;
    float v2 = v_fwd * v_fwd;
    float safe_tan = (max_centrifugal * WHEELBASE) / v2;
    return min(MAX_STEER_ANGLE, atan(safe_tan));
}


/**
 * @brief Установка угла сервы через TIM5 (аппаратный Ш�?М)
 * @param angle_deg Угол в градусах (0-180)
 */
void set_servo_angle_tim5(float angle_deg)
{
    // Ограничение угла
    if(angle_deg > 180.0f) angle_deg = 180.0f;
    if(angle_deg < 0.0f) angle_deg = 0.0f;

    // Расчёт pulse в мкс: 0° = 500 мкс, 180° = 2500 мкс
    // pulse_us = 500 + (angle_deg / 180.0f) * 2000
    float pulse_us = 500.0f + (angle_deg / 180.0f) * 2000.0f;

    // Тактовая частота таймера = 1 МГц (1 тик = 1 мкс)
    // Значение CCR = pulse_us (так как 1 тик = 1 мкс)
    uint32_t ccr = (uint32_t)pulse_us;

    // Ограничение по ARR (ARR = 9999)
    if(ccr > 9999) ccr = 9999;

    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, ccr);
}

/**
 * @brief Ограничение значения
 */
static inline float clamp(float val, float min_val, float max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

/**
 * @brief Получить отфильтрованную угловую скорость рыскания из MahonyAHRS
 * Возвращает угловую скорость в стабилизированной системе координат (earth frame)
 */
float get_filtered_yaw_rate(void)
{

    // Получаем сырую угловую скорость
    float yaw_rate_raw = MahonyAHRSgetStabilizedYawRateRad( //Down
    		  gx_filt * DEGTORAD,
			  gy_filt * DEGTORAD,
			 -gz_filt * DEGTORAD
    );

    // �?нициализация при первом вызове
    if (!yaw_filter_initialized) {
        yaw_rate_filtered = yaw_rate_raw;
        yaw_filter_initialized = 1;
        return yaw_rate_raw;
    }

    // EMA фильтр: output = alpha * input + (1 - alpha) * previous_output
    yaw_rate_filtered = YAW_RATE_ALPHA * yaw_rate_raw + (1.0f - YAW_RATE_ALPHA) * yaw_rate_filtered;

    return yaw_rate_filtered;
}
/**
 * @brief Вычисление желаемого угла сервы из желаемой угловой скорости
 */
float compute_target_steer(float w_des, float v_current)
{
    float abs_v = fabsf(v_current);

    if (abs_v < 0.1f) {
        return 0.0f;  // На малой скорости не рулим
    }

    float v_for_kin = abs_v;
    float target = atan2f(w_des * WHEELBASE, v_for_kin);

    if (v_current < 0.0f) {
        target = -target;  // инвертируем при заднем ходе
    }

    return target;
}

/**
 * @brief Регулятор угловой скорости (вызывать с частотой 50-100 Гц)
 * @param dt Время с предыдущего вызова (сек)
 */
float yaw_rate_controller(float dt)
{
    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;

    // Динамическое ограничение
    float safe_max = getSafeMaxSteer(telem.velosity_1d_mps);
    float safe_max_angle = safe_max * safe_max_coff;

    // 1. Получить текущую угловую скорость (уже в earth frame)
    telem.w_yaw = get_filtered_yaw_rate();

    // 2. Ошибка (с учётом знака скорости)
    float error = pca9685.w_cmd - telem.w_yaw;
    if (telem.velosity_1d_mps < 0.0f) {
        error = -error;
    }

    // 3. Feed-forward из кинематики
    float ff = compute_target_steer(pca9685.w_cmd, telem.velosity_1d_mps);
    ff = clamp(ff, -safe_max_angle, safe_max_angle);

    // 4. Пропорциональная составляющая
    float P = YAW_KP * error;

    // 5. Интегральная составляющая с I-term relax
    integral_error += error * dt;

    // I-term relax: затухание при вращении
    if (fabsf(telem.w_yaw) > 1.0f) {
        integral_error *= 0.9f;
    }

    // Полный сброс при нулевой уставке и малой скорости
    if (fabsf(pca9685.w_cmd) < 0.001f && fabsf(telem.w_yaw) < 0.05f) {
        integral_error = 0.0f;
    }

    // Ограничение интеграла
    integral_error = clamp(integral_error, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
    float I = YAW_KI * integral_error;

    // 6. Дифференциальная составляющая
    float D = YAW_KD * (error - prev_error) / dt;
    prev_error = error;

    // 7. Суммарный целевой угол
    float target_steer = ff + (P + I + D);
    target_steer = clamp(target_steer, -safe_max_angle, safe_max_angle);

    // 8. Anti-windup: если выход насыщен и ошибка тянет дальше — не накапливать интеграл
    if ((target_steer >= safe_max && error > 0.0f) ||
        (target_steer <= -safe_max && error < 0.0f)) {
        integral_error = clamp(integral_error, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
        // Можно оставить интеграл как есть или слегка затушить
        integral_error *= 0.95f;
    }

    return target_steer;
}

float calculate (float A2, float B2, float C2) {
    uint8_t A2_gt_B2 = (A2 > B2) ? 1 : 0;
    uint8_t C2_gt_A2 = (C2 - A2 > 0) ? 1 : 0;
    uint8_t A2_lt_0 = (A2 < 0) ? 1 : 0;
    uint8_t B2_lt_0 = (B2 < 0) ? 1 : 0;
    uint8_t C2_lt_0 = (C2 < 0) ? 1 : 0;
    uint8_t B2_eq_A2 = (B2 == A2) ? 1 : 0;

    if (A2_gt_B2) {
        return (C2_gt_A2) ? ((A2_lt_0 && B2_lt_0) ? C2 : -B2) : 0;
    } else if (A2_lt_0) {
        return (B2_lt_0 && C2_lt_0) ? 0 : (B2_eq_A2 ? 0 : B2);
    } else {
        return -C2;
    }
}

float expRunningAverage(float newVal) {
  filVal += (newVal - filVal) * k;
  return filVal;
}

// Вторым аргументом передаем указатель на массив из 5 элементов (float buffer[5])
float process_median5(float newValue, float* buf) {
    // 1. Сдвигаем буфер (на H7 это мгновенно)
    buf[0] = buf[1];
    buf[1] = buf[2];
    buf[2] = buf[3];
    buf[3] = buf[4];
    buf[4] = newValue;

    // 2. Копируем во временные переменные, чтобы компилятор
    // положил их в быстрые регистры процессора
    float a = buf[0];
    float b = buf[1];
    float c = buf[2];
    float d = buf[3];
    float e = buf[4];
    float t;

    // 3. Та самая сеть сортировки (9 операций fminf/fmaxf)
    t = a; a = fminf(a, b); b = fmaxf(t, b);
    t = c; c = fminf(c, d); d = fmaxf(t, d);
    t = a; a = fminf(a, c); c = fmaxf(t, c);
    t = b; b = fminf(b, d); d = fmaxf(t, d);
    t = a; a = fminf(a, e); e = fmaxf(t, e);
    t = b; b = fminf(b, c); c = fmaxf(t, c);
    t = d; d = fminf(d, e); e = fmaxf(t, e);
    t = b; b = fminf(b, d); d = fmaxf(t, d);
    t = c; c = fminf(c, d); d = fmaxf(t, d);

    return c; // Медиана
}

// Loop 1 Hz
void update_lidar_gprmc(void)
{
    static uint32_t fake_time = 120000;
    static char packet[80];

    fake_time++;
    if (fake_time % 100 >= 60) fake_time += 40;
    if ((fake_time / 100) % 100 >= 60) fake_time += 4000;

    // Собираем строку с $, телом и местом под чек-сумму
    int len = sprintf(packet, "$GPRMC,%06lu.00,A,0000.0000,N,00000.0000,E,0.0,0.0,060826,,,A", fake_time);

    // Считаем XOR по содержимому после $
    uint8_t checksum = 0;
    for (int i = 1; i < len; i++) checksum ^= packet[i];

    // Дописываем чек-сумму и \r\n
    len += sprintf(packet + len, "*%02X\r\n", checksum);

    // Одним вызовом
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)packet, len);

}

//Loop 100Hz
void update_encoder(void)
{
    Now = htim2.Instance->CNT;
    deltaTime = Now - lastUpdate_encoder;
    lastUpdate_encoder = Now;

    if (deltaTime > 100000) deltaTime = 100000;
    if (deltaTime < 1000) deltaTime = 1000;

    uint32_t temp_counter = __HAL_TIM_GET_COUNTER(&htim3);

    // Расчёт скорости энкодера
    if (temp_counter == encoder.last_counter_value)
    {
        encoder.velocity = 0;
    }
    else if (temp_counter > encoder.last_counter_value)
    {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
            encoder.velocity = -encoder.last_counter_value - (__HAL_TIM_GET_AUTORELOAD(&htim3) - temp_counter);
        else
            encoder.velocity = temp_counter - encoder.last_counter_value;
    }
    else
    {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
            encoder.velocity = temp_counter - encoder.last_counter_value;
        else
            encoder.velocity = temp_counter + (__HAL_TIM_GET_AUTORELOAD(&htim3) - encoder.last_counter_value);
    }
    encoder.last_counter_value = temp_counter;
    velosity = (float)-encoder.velocity;

    // Low-pass filter (25 Hz cutoff)
    vFilt = 0.854f * vFilt + 0.0728f * velosity + 0.0728f * vPrev;
    vPrev = velosity;

    // ================================ Обновление скорости (м/с) ================================ //
    float deltaTime_sec = deltaTime / 1000000.0f;  // мкс → сек
    telem.velosity_1d_mps = vFilt * TICKS_TO_METERS / deltaTime_sec;

    // ================================ MAHONY =================================================== //

       // gx, gy, gz уже в rad/s, ax, ay, az уже в m/s²
       //MahonyAHRSupdateIMU(gx_filt * DEGTORAD, -gy_filt * DEGTORAD, -gz_filt * DEGTORAD, -accel_Mahony.x, accel_Mahony.y, accel_Mahony.z, deltaTM); //Up

       MahonyAHRSupdateIMU(-gx_filt * DEGTORAD, gy_filt * DEGTORAD, -gz_filt * DEGTORAD, -accel_Mahony.x, -accel_Mahony.y, accel_Mahony.z, deltaTime_sec);  //Down

       telem.w_yaw = get_filtered_yaw_rate();

       sum_M += deltaTime_sec;
       sumcount_M++;

       // ================================ РЕЖИМ MPPI ================================ //
        if (current_mode == CTRL_MODE_MPPI)
        {
               // 1. Серва рулевая
               float steer_deg = mppi_ctrl.steer_angle * RADTODEG;
               float servo_angle = 90.0f + steer_deg * (30.0f / (MAX_STEER_ANGLE * RADTODEG));
               if (servo_angle > 120.0f) servo_angle = 120.0f;
               if (servo_angle < 60.0f) servo_angle = 60.0f;
               set_servo_angle_tim5(servo_angle);

               // 2. Мотор
               float throttle = mppi_ctrl.throttle;
               uint16_t pwm_abs = (uint16_t)(fabsf(throttle) * 4095.0f);
               if (pwm_abs > 4095) pwm_abs = 4095;

               if (throttle > 0.01f) {
                   PCA9685_Forward();
                   PCA9685_SetPin(0, pwm_abs, 0);
               } else if (throttle < -0.01f) {
                   PCA9685_Back();
                   PCA9685_SetPin(0, pwm_abs, 0);
               } else {
                   PCA9685_SetPin(0, 0, 0);
               }

               // 3. Тормоз
               float brake_angle = mppi_ctrl.brake * 125.0f;
               if (brake_angle < 0.0f) brake_angle = 0.0f;
               if (brake_angle > 125.0f) brake_angle = 125.0f;
               PCA9685_SetServoAngle(1, brake_angle);

               flag_pca9685 = 0;
               return;  // ПИД-регуляторы не вызываем
    }

    // ================================ Ограничение скорости по кривизне ================================ //
    if (pca9685.setpoint >= 0.0f) {
        float curvature = fabsf(pca9685.w_cmd) / fmaxf(fabsf(telem.velosity_1d_mps), 0.1f);
        float safe_max = getSafeMaxSteer(telem.velosity_1d_mps);
        float min_radius = WHEELBASE / tanf(safe_max);
        float v_limited = pca9685.setpoint;

        if (curvature > 1e-6f) {
            float radius = 1.0f / curvature;
            if (radius < min_radius) {
                v_limited = pca9685.setpoint * (radius / min_radius);
            }
        }
        if (v_limited < pca9685.setpoint) {
            pca9685.setpoint = v_limited;
        }
    }

    // ================================ PID Control Motor ================================ //

    float feed_forward_motor = pca9685.setpoint * FF_GAIN_MOTOR;

    float Error_Motor = pca9685.setpoint - vFilt;
    float Error_Motor_ABS = fabsf(Error_Motor);

    float Delta_Motor = (Error_Motor - Last_Error_Motor) / deltaTime;
    Last_Error_Motor = Error_Motor;

    if (Delta_Motor > deriv_threshold_speed)  Delta_Motor -= deriv_threshold_speed;
    if (Delta_Motor < -deriv_threshold_speed) Delta_Motor += deriv_threshold_speed;

    float Integral_Motor = 0.0f;

    // I-Term Relax: накопление только при умеренной ошибке
    if (Error_Motor_ABS > 0.5f && Error_Motor_ABS < 3.0f && fabsf(pca9685.setpoint) > 0.5f) {
        Integral_Motor = last_Integral_Motor + Error_Motor * deltaTime;
    } else {
        last_Integral_Motor *= 0.9f;
    }

    // Сброс при смене знака уставки
    static float prev_setpoint = 0.0f;
    if ((pca9685.setpoint > 0.1f && prev_setpoint < -0.1f) ||
        (pca9685.setpoint < -0.1f && prev_setpoint > 0.1f)) {
        last_Integral_Motor = 0.0f;
    }
    prev_setpoint = pca9685.setpoint;

    if (Integral_Motor > INTEGRAL_LIMIT_MOTOR) Integral_Motor = INTEGRAL_LIMIT_MOTOR;
    if (Integral_Motor < INTEGRAL_LIMIT_MOTOR_NEG) Integral_Motor = INTEGRAL_LIMIT_MOTOR_NEG;

    last_Integral_Motor = Integral_Motor;

    float pid_output_motor = kp_m * Error_Motor + Delta_Motor * kd_m + Integral_Motor * ki_m;
    Output_Motor = (int16_t)(pid_output_motor + feed_forward_motor);

    if (Output_Motor > MOTOR_PWM_MAX) Output_Motor = MOTOR_PWM_MAX;
    if (Output_Motor < MOTOR_PWM_MIN) Output_Motor = MOTOR_PWM_MIN;

    if (pca9685.setpoint == 0.0f) {
        Output_Motor = 0;
        last_Integral_Motor = 0.0f;
        Integral_Motor = 0.0f;
    }

    if ((Output_Motor >= MOTOR_PWM_MAX && Error_Motor > 0) ||
        (Output_Motor <= MOTOR_PWM_MIN && Error_Motor < 0)) {
        last_Integral_Motor = Integral_Motor;
    }

    // ================================ PID Control Brake ================================ //

    float Error_Brake = expRunningAverage(calculate(pca9685.setpoint, vFilt, Error_Motor));

    float Delta_Brake = (Error_Brake - Last_Error_Brake) / deltaTime;
    Last_Error_Brake = Error_Brake;

    if (Delta_Brake > deriv_threshold_brake) Delta_Brake -= deriv_threshold_brake;
    if (Delta_Brake < -deriv_threshold_brake)  Delta_Brake += deriv_threshold_brake;

    float Integral_Brake = 0.0f;

    if (Error_Motor_ABS > 0.5f && Error_Motor_ABS < 3.0f) {
        Integral_Brake = last_Integral_Brake + Error_Brake * deltaTime;
    } else {
        last_Integral_Brake *= 0.9f;
    }

    if (Integral_Brake > INTEGRAL_LIMIT_BRAKE) Integral_Brake = INTEGRAL_LIMIT_BRAKE;
    if (Integral_Brake < INTEGRAL_LIMIT_BRAKE_NEG) Integral_Brake = INTEGRAL_LIMIT_BRAKE_NEG;

    last_Integral_Brake = Integral_Brake;

    Output_Brake = (int16_t)(kp_b * Error_Brake + Delta_Brake * kd_b + Integral_Brake * ki_b);

    if (Output_Brake > SERVO_ANGLE_MAX) Output_Brake = SERVO_ANGLE_MAX;
    if (Output_Brake < SERVO_ANGLE_MIN) Output_Brake = SERVO_ANGLE_MIN;

    if (Error_Brake < 1.0f) {
        Output_Brake = 0;
        last_Integral_Brake = 0.0f;
        Integral_Brake = 0.0f;
    }

    if (Output_Brake >= SERVO_ANGLE_MAX && Error_Brake > 0.0f) {
        last_Integral_Brake = Integral_Brake;
    }

    // ================================ угол сервы ================================ //

    servo_angle_deg = yaw_rate_controller(deltaTime_sec) * RADTODEG;  // ±30°
    servo_angle_deg = 90.0f + servo_angle_deg;  // 60-120°

    if (servo_angle_deg > 120.0f) servo_angle_deg = 120.0f;
    if (servo_angle_deg < 60.0f) servo_angle_deg = 60.0f;

    set_servo_angle_tim5(servo_angle_deg);

    flag_pca9685 = 1;
}

void loop (void) {

	Now = htim2.Instance->CNT;

	 if((Now - lastUpdate_check) > 10000) {  lastUpdate_check = Now; // каждые 10 мс (100 Гц)

	 check_connection();

	 telem.stering_angle = (get_steering_angle_signed() - 90.0f) * Servo_to_wheel_ratio;

	 // Просто копируем значения (H7 сделает это мгновенно)
	 tele_out.w_yaw = telem.w_yaw;
	 tele_out.stering_angle = telem.stering_angle;
	 tele_out.velosity_1d_mps = telem.velosity_1d_mps;

	 // Отправляем весь пакет целиком (16 байт)
	 // 4 байта заголовок + 12 байт данных = 16 (идеально для 32-битной шины)
	 HAL_UART_Transmit_IT(&huart4, (uint8_t*)&tele_out, sizeof(TelePacket));

	}

     if((Now - lastUpdate) > 100000) {
         lastUpdate = Now;
         MahonyAHRSgetEuler(&roll, &pitch, &yaw);

         char screen_lines[8][24];
         char f_buf[16];

         // 0. СКОРОСТЬ
         ftoa(telem.velosity_1d_mps, f_buf, 2);
         sprintf(screen_lines[0], "Speed: %s m/s", f_buf);

         // 1. STEER
         ftoa(telem.stering_angle, f_buf, 1);
         sprintf(screen_lines[1], "SteerA: %s deg", f_buf);

         // 2. SERVO & BRAKE (Объединили вывод управления)
         ftoa(servo_angle_deg, f_buf, 1);
         sprintf(screen_lines[2], "S:%s B:%d M:%s ", f_buf, Output_Brake, (current_mode == CTRL_MODE_MPPI) ? "1" : "2");

         // 3. ROLL
         sprintf(screen_lines[3], "Roll:  %.2f", roll);

         // 4. PITCH
         sprintf(screen_lines[4], "Pitch: %.2f", pitch);

         // 5. YAW
         sprintf(screen_lines[5], "Yaw:   %.2f", yaw);

         // 6. YAW RATE (Сделал 3 знака после запятой)
         ftoa(telem.w_yaw * RADTODEG, f_buf, 3);
         sprintf(screen_lines[6], "W-Yaw: %s d/s", f_buf);

         // 7. Считаем частоты
         float Hz = (float)sumcount / (sum > 0 ? sum : 1);
         sumcount = 0; sum = 0;
         float Hz_M = (float)sumcount_M / (sum_M > 0 ? sum_M : 1);
         sumcount_M = 0; sum_M = 0;
         sprintf(screen_lines[7], "S/M: %.1f / %.1f", Hz, Hz_M);

         // --- ОТПРАВКА НА ЭКРАН (Асинхронно) ---
         ST7735_PrintTelemetry_IT(screen_lines, Font_7x10, ST7735_GREEN, ST7735_BLACK);
     }


	if(flag_nrf24l01) { flag_nrf24l01 = 0;

	  nrf24l01p_rx_receive(rx_data); // read data when data ready flag is set
	}

//	if(flag_pca9685) {

//		  if(pca9685.stop > 1) Output_Motor = 0;
//		  if(Output_Motor > 0 && vFilt > -20.0f) {PCA9685_Forward(); PCA9685_SetPin(0, Output_Motor, 0);}
//		  if(Output_Motor == 0) { PCA9685_SetPin(0, 0, 0);}
//		  if(Output_Motor == 0 && fabsf(vFilt) < 0.01f) { PCA9685_SetPin(0, 0, 0); PCA9685_Forward();}
//		  if(Output_Motor < 0 && vFilt < 20.0f) {PCA9685_Back(); PCA9685_SetPin(0, -Output_Motor, 0); }

//		  if(pca9685.turn > 110.0f) pca9685.turn = 110.0f;
//		  if(pca9685.turn < 70.0f) pca9685.turn = 70.0f;
//		  PCA9685_SetServoAngle(1, pca9685.turn);
//		  Output_ServoBrake = pca9685.stop + Output_Brake;
//		  if(Output_ServoBrake > 120) Output_ServoBrake = 120;
//		  PCA9685_SetServoAngle(2, Output_ServoBrake);

//		  flag_pca9685 = 0;
//	}

	if(flag_pca9685)
	{
	    // Аварийная остановка
	    if(pca9685.stop > 1)
	    {
	        Output_Motor = 0;
	    }

	    // Получаем абсолютное значение для PWM
	    uint16_t pwm_abs = (Output_Motor > 0) ? Output_Motor : -Output_Motor;
	    if(pwm_abs > 4095) pwm_abs = 4095;

	    // Управление
	    if(Output_Motor > 0 && vFilt > SAFE_SPEED_BACKWARD_CHANGE)
	    {
	        PCA9685_Forward();
	        PCA9685_SetPin(0, pwm_abs, 0);
	    }
	    else if(Output_Motor < 0 && vFilt < SAFE_SPEED_FORWARD_CHANGE)
	    {
	        PCA9685_Back();
	        PCA9685_SetPin(0, pwm_abs, 0);
	    }
	    else if(Output_Motor == 0)
	    {
	        PCA9685_SetPin(0, 0, 0);
	        if(fabsf(vFilt) < 0.01f)
	        {
	            PCA9685_Forward();
	        }
	    }

	    // Тормоз
	    Output_ServoBrake = pca9685.stop + Output_Brake;
	    if(Output_ServoBrake > 125) Output_ServoBrake = 125;
	    if(Output_ServoBrake < 0) Output_ServoBrake = 0;
	    PCA9685_SetServoAngle(1, Output_ServoBrake);

	    flag_pca9685 = 0;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  connection_lost = 0;
  heartbeat_fail_counter = 0;
  last_rx_time = 0;

  integral_error = 0.0f;
  prev_error = 0.0f;
  prev_target_steer = 0.0f;

  Output_Motor = 0;
  Output_Brake = 0;
  Output_ServoBrake = 0;

  Last_Error_Motor = 0.0f;
  Last_Error_Brake = 0.0f;

  last_Integral_Motor = 0.0f;
  last_Integral_Brake = 0.0f;

  deltaTime = 0;
  lastUpdate_check = 0;

  vFilt = 0.0f;
  vPrev = 0.0f;

  velosity = 0.0f;
  prev_yaw_rad = 0.0f;

  flag_encoder = 0;

  current_mode = CTRL_MODE_MANUAL;
  mppi_ctrl.steer_angle = 0.0f;
  mppi_ctrl.throttle = 0.0f;
  mppi_ctrl.brake = 0.0f;

  pca9685.w_cmd = 0.0f;
  pca9685.stop = 0;
  pca9685.setpoint = 0.0f;

  servo_angle_deg = 90.0f;
  telem.velosity_1d_mps = 0.0f;
  telem.stering_angle = 0.0f;
  telem.w_yaw = 0.0f;

  encoder.velocity = 0;
  encoder.last_counter_value = 32767;

  flag_pca9685  = 0;
  flag_nrf24l01 = 0;

  k = 0.01f;
  filVal = 0.0f;

  gx_filt = 0.0f;
  gy_filt = 0.0f;
  gz_filt = 0.0f;

  ax_filt = 0.0f;
  ay_filt = 0.0f;
  az_filt = 0.0f;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_QUADSPI_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC3_Init();
  MX_TIM5_Init();
  MX_RTC_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE BEGIN 2 */

  // 1. Запускаем ведомый TIM8 на выдачу импульса 50 мкс на ножке PC9
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim1);  // Запускаем сам счетчик TIM1

   HAL_TIM_Base_Start(&htim2);

   __HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0

   HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

   __HAL_TIM_SET_COUNTER(&htim3, 32767);

   HAL_TIM_Base_Start_IT(&htim4);

   HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_BUFFER, 256);

  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_BUFFER, 256);

  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);

  init_gyro_filters();

  MahonyAHRSreset();

  reset_yaw_controller();

  nrf24l01p_rx_init(2500, _250kbps);

  ST7735_Init();
  // Очистка экрана
  ST7735_FillScreen(ST7735_BLACK);

  ST7735_WriteString(0, 0, "System Init...", Font_7x10, ST7735_WHITE, ST7735_BLACK);

  // Запуск инициализации PCA9685
  PCA9685_STATUS status = PCA9685_Init(&hi2c1);

  if (status == PCA9685_OK) {
      ST7735_WriteString(0, 15, "PCA9685: OK", Font_7x10, ST7735_GREEN, ST7735_BLACK);
  } else {
      char err_msg[20];
      sprintf(err_msg, "PCA9685: ERR %d", (int)status);
      ST7735_WriteString(0, 15, err_msg, Font_7x10, ST7735_RED, ST7735_BLACK);
  }

  PCA9685_SetPin(0, 1, 0);
  PCA9685_SetServoAngle(1, 0.0f);
  set_servo_angle_tim5(servo_angle_deg);

  icm20948_init();

  HAL_Delay(500);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      loop();
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 15;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 1. Прерывание от датчика (Data Ready)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == icm20948_INT_Pin) icm20948_read_imu_dma_start(imu_dma_tx, imu_dma_rx);

    if(GPIO_Pin == NRF24L01_INT_Pin) flag_nrf24l01 = 1;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    if (huart->Instance == UART4)
    {
        // Команда смены режима: 'M','O','D','E', mode_byte
        if (UART4_BUFFER[0] == 'M' && UART4_BUFFER[1] == 'O' && UART4_BUFFER[2] == 'D' && UART4_BUFFER[3] == 'E')
        {
            ctrl_mode_t requested_mode = (UART4_BUFFER[4] == 1) ? CTRL_MODE_MPPI : CTRL_MODE_MANUAL;

            if (requested_mode != current_mode)
            {
                current_mode = requested_mode;
                send_mode_ack(current_mode);
                reset_controllers();
            }
            else
            {
                // Режим не изменился, но подтверждаем, что живём в этом режиме
                send_mode_ack(current_mode);
            }
        }
        // Данные ручного режима: 'P','C','A' + struct PCA9685
        else if (UART4_BUFFER[0] == 'P' && UART4_BUFFER[1] == 'C' && UART4_BUFFER[2] == 'A')
        {
            if (current_mode == CTRL_MODE_MANUAL)
            {
                uint8_t ee = 3;
                uint8_t* p = (uint8_t*)(void*)&pca9685;
                for (int count = sizeof(pca9685); count; --count) *p++ = UART4_BUFFER[ee++];

                flag_pca9685 = 1;

                // Heartbeat
                last_rx_time = HAL_GetTick();
                heartbeat_fail_counter = 0;
                connection_lost = 0;
            }
            else
            {
                // Пришли данные не того режима — отправляем текущий режим
                send_mode_ack(current_mode);
            }
        }
        // Данные MPPI режима: 'M','P','I' + struct MPPI_Ctrl
        else if (UART4_BUFFER[0] == 'M' && UART4_BUFFER[1] == 'P' && UART4_BUFFER[2] == 'I')
        {
            if (current_mode == CTRL_MODE_MPPI)
            {
                uint8_t ee = 3;
                uint8_t* p = (uint8_t*)(void*)&mppi_ctrl;
                for (int count = sizeof(mppi_ctrl); count; --count) *p++ = UART4_BUFFER[ee++];

                flag_pca9685 = 1;

                // Heartbeat
                last_rx_time = HAL_GetTick();
                heartbeat_fail_counter = 0;
                connection_lost = 0;
            }
            else
            {
                // Пришли данные не того режима — отправляем текущий режим
                send_mode_ack(current_mode);
            }
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART4_BUFFER, 256);
        __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
    }

    if (huart->Instance == UART5)
    {
        HAL_UART_Transmit_IT(&huart4, UART5_BUFFER, size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_BUFFER, 256);
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3) {

    	if (icm20948_parse_dma(imu_dma_rx, &gyro, &accel)) {

        Now = htim2.Instance->CNT;
        deltaT = ((Now - lastUpdate_icm20948)/1000000.0f); // set integration time by time elapsed since last filter update
        lastUpdate_icm20948 = Now;

        sum += deltaT; // sum for averaging filter update rate
        sumcount++;

        float gx,gy,gz;

        arm_biquad_cascade_df1_f32(&S_gyroX, &gyro.x, &gx, 1);
        arm_biquad_cascade_df1_f32(&S_gyroY, &gyro.y, &gy, 1);
        arm_biquad_cascade_df1_f32(&S_gyroZ, &gyro.z, &gz, 1);

        gx_filt = GYRO_CUTOFF_FREQ * gx + (1.0f - GYRO_CUTOFF_FREQ) * gx_filt;
        gy_filt = GYRO_CUTOFF_FREQ * gy + (1.0f - GYRO_CUTOFF_FREQ) * gy_filt;
        gz_filt = GYRO_CUTOFF_FREQ * gz + (1.0f - GYRO_CUTOFF_FREQ) * gz_filt;

        // 3. Проверка вектора гравитации (по СЫРЫМ данным)
        float accel_magnitude = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
        bool use_accel = (accel_magnitude > 0.9f) && (accel_magnitude < 1.1f); // 0.9^2 и 1.1^2

        // 5. Подготовка данных для Mahony
        if (use_accel) {

        	float ax = process_median5(accel.x, accX_buffer);
        	float ay = process_median5(accel.y, accY_buffer);
        	float az = process_median5(accel.z, accZ_buffer);

        	ax_filt = ACCEL_CUTOFF_FREQ * ax + (1.0f - ACCEL_CUTOFF_FREQ) * ax_filt;
        	ay_filt = ACCEL_CUTOFF_FREQ * ay + (1.0f - ACCEL_CUTOFF_FREQ) * ay_filt;
        	az_filt = ACCEL_CUTOFF_FREQ * az + (1.0f - ACCEL_CUTOFF_FREQ) * az_filt;

        	accel_Mahony.x = ax_filt;
			accel_Mahony.y = ay_filt;
			accel_Mahony.z = az_filt;

        }
        else {

        	accel_Mahony.x = 0.0f;
        	accel_Mahony.y = 0.0f;
        	accel_Mahony.z = 0.0f;
        }
        }

    }

}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {

	ST7735_IT_Callback(hspi); // Наша старая добрая функция очистки флага

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// measure velocity, position
	if(htim == &htim4) update_encoder();
    if(htim == &htim1) update_lidar_gprmc();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
