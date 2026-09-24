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
#include "i2c.h"
#include "quadspi.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  int16_t velocity;
  uint32_t last_counter_value;
}encoder_instance;


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

#define KP_SPEED_MOTOR  5.0f
#define KD_SPEED_MOTOR  10.0f
#define KI_SPEED_MOTOR  0.00001f
#define DERIV_THRESH_MOTOR   0.25f

// Ограничения интеграла (защита от переполнения и накопления)
#define INTEGRAL_LIMIT_MOTOR       4.095e8f
#define INTEGRAL_LIMIT_MOTOR_NEG  -4.095e8f

#define INTEGRAL_LIMIT_BRAKE       4.095e8f
#define INTEGRAL_LIMIT_BRAKE_NEG   0.0f

#define KP_SPEED_BRAKE 1.0f
#define KD_SPEED_BRAKE 1.0f
#define KI_SPEED_BRAKE 0.00000001f
#define DERIV_THRESH_BRAKE 0.25f

#define MOTOR_PWM_MAX  4095

#define SERVO_ANGLE_MAX  125

// ============================================================
// ПАРАМЕТРЫ РЕГУЛЯТОРА УГЛОВОЙ СКОРОСТ�?
// ============================================================

// Геометрия
#define WHEELBASE 0.70f   // Колёсная база [м]

// Ограничения
#define MAX_STEER_ANGLE 0.471239    // Макс. угол сервы [рад] (~30 градусов ранее был 20)
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
#define TICKS_PER_METER         (ENCODER_TICKS_PER_REV * GEAR_RATIO / WHEEL_CIRCUMFERENCE)

// ============================================================
// ПАРАМЕТРЫ HEARTBEAT (контроль связи с Mini-PC)
// ============================================================

#define HEARTBEAT_TIMEOUT_MS     500     // Таймаут связи (мс)
#define HEARTBEAT_COUNTER_MAX     10     // Максимум пропущенных пакетов

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

  float vFilt;

  float vFilt_stage1 = 0.0f;

  float alpha = 0.118f;   // fc ≈ 2 Гц, fs = 100 Гц

  arm_biquad_cascade_df2T_instance_f32 S_velocity_lpf;
  float32_t velocity_lpf_state[2] = {0};   // 2 состояния на 1 секцию

  arm_biquad_cascade_df2T_instance_f32 S_steer_lpf;
  float32_t steer_lpf_state[2] = {0};

  const float32_t lpf_coeffs[5] = {
       0.0009448f,   // b0
       0.0018896f,   // b1
       0.0009448f,   // b2
       1.93404f,     // -a1
      -0.93784f      // -a2
  };

  const float32_t steer_lpf_coeffs[5] = {
       0.06745f,    // b0
       0.13490f,    // b1
       0.06745f,    // b2
       1.14290f,    // -a1
      -0.41279f     // -a2
  };

  float velosity;
  float servo_angle_deg;
  float filtered_angle = 0.0f;

  // Heartbeat (контроль связи с Mini-PC)
  uint32_t last_rx_time;           // Время последнего полученного пакета
  uint8_t heartbeat_fail_counter;  // Счётчик пропущенных heartbeat
  uint8_t connection_lost;         // Флаг потери связи

  float k;
  float filVal;

  uint32_t lastUpdate; // used to calculate integration interval
  uint32_t lastUpdate_check;
  uint32_t lastUpdate_encoder;
  uint32_t Now;                         // used to calculate integration interval

  uint32_t deltaTime;

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

   uint8_t UART4_BUFFER [PKT_MAX_PAYLOAD + 5] = {0};
   uint8_t UART5_BUFFER [PKT_MAX_PAYLOAD + 5] = {0};

   volatile ctrl_mode_t current_mode = CTRL_MODE_MANUAL;

   Pkt_PCA_t  pca9685;
   Pkt_MPI_t  mppi_ctrl;
   Pkt_Tele_t telem;

   encoder_instance encoder;

   static pkt_rx_t rx_pkt;

   /* ============ RX ============ */
   volatile uint8_t  uart4_rx_buf[2][UART4_RX_BUF_SIZE];
   volatile uint8_t  uart4_rx_active = 0;
   volatile uint16_t uart4_rx_idx = 0;

   volatile uint8_t  uart4_pkt_queue[UART4_PKT_QUEUE_SIZE][UART4_RX_BUF_SIZE];
   volatile uint16_t uart4_pkt_len_queue[UART4_PKT_QUEUE_SIZE];
   volatile uint8_t  uart4_pkt_head = 0;
   volatile uint8_t  uart4_pkt_tail = 0;
   volatile uint32_t uart4_pkt_dropped = 0;

   volatile uint32_t uart4_total_bytes = 0;
   volatile uint32_t uart4_total_packets = 0;

   /* ============ TX ============ */
   volatile uint8_t  uart4_tx_buf[UART4_TX_BUF_SIZE];
   volatile uint16_t uart4_tx_head = 0;
   volatile uint16_t uart4_tx_tail = 0;
   volatile uint8_t  uart4_tx_busy = 0;

   volatile uint32_t uart4_tx_total_bytes = 0;
   volatile uint32_t uart4_tx_overflow = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// for tx interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END PFP */

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

void reset_controllers(void) {
    last_Integral_Motor = 0.0f;
    last_Integral_Brake = 0.0f;
    Last_Error_Motor = 0.0f;
    Last_Error_Brake = 0.0f;

    // Возврат сервы в центр
    servo_angle_deg = 90.0f;

    // Остановка мотора
    Output_Motor = 0;
    Output_Brake = 0;
}

/**
 * @brief  Неблокирующая отправка через UART4.
 *         Кладёт байты в кольцевой буфер и запускает прерывание TXE.
 * @param  data  указатель на данные
 * @param  len   длина
 * @retval 1 — всё уложено в буфер, 0 — не хватило места
 */
uint8_t uart4_send(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uart4_tx_head + 1) % UART4_TX_BUF_SIZE;
        if (next == uart4_tx_tail) {
            // буфер полон — пакет не влез
            uart4_tx_overflow++;
            return 0;
        }
        uart4_tx_buf[uart4_tx_head] = data[i];
        uart4_tx_head = next;
    }

    // Если передача не идёт — запускаем её, включив прерывание TXE
    if (!uart4_tx_busy) {
        uart4_tx_busy = 1;
        __HAL_UART_ENABLE_IT(&huart4, UART_IT_TXE);
    }

    uart4_tx_total_bytes += len;
    return 1;
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

        connection_lost = 0;
    }
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

    // 2. Масштабирование в градусы
    float raw_angle = (float)(adc_value - STEER_ADC_MIN) * STEER_RANGE_DEG
                    / (float)(STEER_ADC_MAX - STEER_ADC_MIN);

    // 3. Clamp — защита от вылета за границы
    if (raw_angle < 0.0f) raw_angle = 0.0f;
    if (raw_angle > STEER_RANGE_DEG) raw_angle = STEER_RANGE_DEG;

    // 4. Биквад Баттерворта 2-го порядка, fc = 10 Гц
    float32_t angle_in = raw_angle;
    float32_t angle_out;
    arm_biquad_cascade_df2T_f32(&S_steer_lpf, &angle_in, &angle_out, 1);
    filtered_angle = angle_out;

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

void uart(void)
{
    while (uart4_pkt_tail != uart4_pkt_head) {
        uint16_t len = uart4_pkt_len_queue[uart4_pkt_tail];
        for (uint16_t i = 0; i < len; i++) {
            if (pkt_rx_feed(&rx_pkt, uart4_pkt_queue[uart4_pkt_tail][i])) {
                switch (rx_pkt.cmd) {
                    case PKT_CMD_PCA:
                        if (rx_pkt.len == sizeof(Pkt_PCA_t) &&
                            current_mode == CTRL_MODE_MANUAL) {
                            Pkt_PCA_t *p = (Pkt_PCA_t*)rx_pkt.payload;
                            pca9685.w_cmd    = p->w_cmd;
                            pca9685.setpoint = p->setpoint;
                            pca9685.stop     = p->stop;
                            flag_pca9685 = 1;
                            last_rx_time = HAL_GetTick();
                            heartbeat_fail_counter = 0;
                            connection_lost = 0;
                        }
                        break;

                    case PKT_CMD_MPI:
                        if (rx_pkt.len == sizeof(Pkt_MPI_t) &&
                            current_mode == CTRL_MODE_MPPI) {
                            Pkt_MPI_t *p = (Pkt_MPI_t*)rx_pkt.payload;
                            mppi_ctrl.steer_angle = p->steer_angle;
                            mppi_ctrl.throttle    = p->throttle;
                            mppi_ctrl.brake       = p->brake;
                            flag_pca9685 = 1;
                            last_rx_time = HAL_GetTick();
                            heartbeat_fail_counter = 0;
                            connection_lost = 0;
                        }
                        break;

                    case PKT_CMD_MODE:
                        if (rx_pkt.len >= 1) {
                            ctrl_mode_t req = (rx_pkt.payload[0] == 1)
                                            ? CTRL_MODE_MPPI : CTRL_MODE_MANUAL;
                            if (req != current_mode) {
                                current_mode = req;
                                reset_controllers();
                            }
                            uint8_t ack_buf[8];
                            uint8_t m = (uint8_t)current_mode;
                            uint16_t n = pkt_build(ack_buf, PKT_CMD_MODE_ACK, &m, 1);
                            if (!uart4_send(ack_buf, n)) {
                                uart4_tx_overflow++;
                            }
                        }
                        break;

                    case PKT_CMD_PING: {
                        uint8_t pong_buf[8];
                        uint16_t n = pkt_build(pong_buf, PKT_CMD_PONG, NULL, 0);
                        if (!uart4_send(pong_buf, n)) {
                            uart4_tx_overflow++;
                        }
                        break;
                    }

                    default:
                        break;
                }
            }
        }
        uart4_pkt_tail = (uart4_pkt_tail + 1) % UART4_PKT_QUEUE_SIZE;
    }
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

    float32_t v_in = velosity;
    float32_t v_out;
    arm_biquad_cascade_df2T_f32(&S_velocity_lpf, &v_in, &v_out, 1);
    vFilt = v_out;

    vFilt_stage1 = alpha * velosity + (1.0f - alpha) * vFilt_stage1;
    vFilt = alpha * vFilt_stage1 + (1.0f - alpha) * vFilt;

    // ================================ Обновление скорости (м/с) ================================ //
    float deltaTime_sec = deltaTime / 1000000.0f;  // мкс → сек
    float setpoint_ticks = pca9685.setpoint * TICKS_PER_METER * deltaTime_sec;
    telem.velosity_1d_mps = vFilt * TICKS_TO_METERS / deltaTime_sec;

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

        // ===== РЕГУЛЯТОР УГЛОВОЙ СКОРОСТИ =====

        float safe_max = getSafeMaxSteer(telem.velosity_1d_mps);
        float safe_max_angle = safe_max * safe_max_coff;

        float abs_v = fabsf(telem.velosity_1d_mps);

        float ads_s = fabsf(pca9685.setpoint);

        if (abs_v > 0.1f) {

           float blend = ads_s / 6.5f;

           // Взвешенный знаменатель
           float v_denom = (1.0f - blend) * ads_s + blend * abs_v;

           float steer_rad = atan2f(pca9685.w_cmd * WHEELBASE, v_denom);

           steer_rad = clamp(steer_rad, -safe_max_angle, safe_max_angle);

           if (steer_rad >  MAX_STEER_ANGLE) steer_rad =  MAX_STEER_ANGLE;
           if (steer_rad < -MAX_STEER_ANGLE) steer_rad = -MAX_STEER_ANGLE;

           servo_angle_deg = 90.0f - steer_rad * RADTODEG;

           if (servo_angle_deg > 120.0f) servo_angle_deg = 120.0f;
           if (servo_angle_deg < 60.0f)  servo_angle_deg = 60.0f;

        }

       set_servo_angle_tim5(servo_angle_deg);

    // ================================ Ограничение скорости по кривизне ================================ //
    if (pca9685.setpoint >= 0.0f) {
        float curvature = fabsf(pca9685.w_cmd) / fmaxf(abs_v, 0.1f);
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

    float Error_Motor = setpoint_ticks - vFilt;

    float Delta_Motor = (Error_Motor - Last_Error_Motor) / deltaTime; Last_Error_Motor = Error_Motor;

    if (Delta_Motor > DERIV_THRESH_MOTOR)  Delta_Motor -= DERIV_THRESH_MOTOR;
    if (Delta_Motor < -DERIV_THRESH_MOTOR) Delta_Motor += DERIV_THRESH_MOTOR;

    float Integral_Motor = last_Integral_Motor + Error_Motor * deltaTime; last_Integral_Motor = Integral_Motor;

    if (Integral_Motor > INTEGRAL_LIMIT_MOTOR)  Integral_Motor = INTEGRAL_LIMIT_MOTOR;
    if (Integral_Motor < INTEGRAL_LIMIT_MOTOR_NEG) Integral_Motor = INTEGRAL_LIMIT_MOTOR_NEG;

    Output_Motor = (int16_t)(KP_SPEED_MOTOR * Error_Motor + KD_SPEED_MOTOR * Delta_Motor + KI_SPEED_MOTOR * Integral_Motor);

    if (fabsf(pca9685.setpoint) < 1e-6f) { Output_Motor = 0; last_Integral_Motor = 0.0f; Integral_Motor = 0.0f; }

    // ================================ PID Control Brake ================================ //

    float Error_Brake = expRunningAverage(calculate(setpoint_ticks, vFilt, Error_Motor));

    float Delta_Brake = (Error_Brake - Last_Error_Brake) / deltaTime; Last_Error_Brake = Error_Brake;

    if (Delta_Brake > DERIV_THRESH_BRAKE) Delta_Brake -= DERIV_THRESH_BRAKE;
    if (Delta_Brake < -DERIV_THRESH_BRAKE)  Delta_Brake += DERIV_THRESH_BRAKE;

    float Integral_Brake = last_Integral_Brake + Error_Brake * deltaTime; last_Integral_Brake = Integral_Brake;

    if (Integral_Brake > INTEGRAL_LIMIT_BRAKE)  Integral_Brake = INTEGRAL_LIMIT_BRAKE;
    if (Integral_Brake < INTEGRAL_LIMIT_BRAKE_NEG) Integral_Brake = INTEGRAL_LIMIT_BRAKE_NEG;

    Output_Brake = (int16_t)(KP_SPEED_BRAKE * Error_Brake + Delta_Brake * KD_SPEED_BRAKE + Integral_Brake * KI_SPEED_BRAKE);

    if (Error_Brake < 1.0f) { Output_Brake = 0; last_Integral_Brake = 0.0f; Integral_Brake = 0.0f;}

    flag_pca9685 = 1;
}

void loop (void) {

	Now = htim2.Instance->CNT;

	uart();

	if ((Now - lastUpdate_check) > 10000) { // каждые 10 мс (100 Гц)

		lastUpdate_check = Now;

		check_connection();

	     telem.stering_angle = (get_steering_angle_signed() - 90.0f) * Servo_to_wheel_ratio;

	     Pkt_Tele_t tel;
	     tel.stering_angle = telem.stering_angle;
	     tel.velosity_1d_mps = telem.velosity_1d_mps;

	     uint8_t buf[32];
	     uint16_t n = pkt_build(buf, PKT_CMD_TELE, &tel, sizeof(tel));
	     if (!uart4_send(buf, n)) {
	         uart4_tx_overflow++;
	     }
	 }

	 if((Now - lastUpdate) > 100000) {
	     char screen_lines[8][24];
	     char f_buf[16];

	     // 0. СКОРОСТЬ
	     ftoa(telem.velosity_1d_mps, f_buf, 2);
	     sprintf(screen_lines[0], "Speed: %s m/s", f_buf);

	     // 1. УГОЛ РУЛЯ
	     ftoa(telem.stering_angle, f_buf, 1);
	     sprintf(screen_lines[1], "SteerA: %s deg", f_buf);

	     // 2. СЕРВО + ТОРМОЗ + РЕЖ�?М
	     ftoa(servo_angle_deg, f_buf, 1);
	     sprintf(screen_lines[2], "S:%s B:%d M:%s", f_buf,
	             Output_Brake, (current_mode == CTRL_MODE_MPPI) ? "1" : "2");

	     // 3. УСТАВК�? от miniPC (v и w)
	     sprintf(screen_lines[3], "sp:%.2f/%.2f", pca9685.setpoint, pca9685.w_cmd);

	     // 4. СВЯЗЬ (lost + heartbeat counter)
	     sprintf(screen_lines[4], "L:%d hb:%d rx:%lu",
	             connection_lost, heartbeat_fail_counter, last_rx_time);

	     // 5. PID МОТОР (ошибка и выход) + период
	     float sp_ticks_lcd = pca9685.setpoint * TICKS_PER_METER * (deltaTime / 1000000.0f);
	     float error_lcd = sp_ticks_lcd - vFilt;
	     ftoa(error_lcd, f_buf, 1);
	     sprintf(screen_lines[5], "E:%s OM:%d dt:%lu",
	             f_buf, Output_Motor, deltaTime);

	     // 6. ТОРМОЗ (угол) + скорость в тиках
	     ftoa(vFilt, f_buf, 1);
	     sprintf(screen_lines[6], "vF:%s Br:%d", f_buf, Output_Brake);

	     // 7. UART4 диагностика
	     sprintf(screen_lines[7], "P%lu D%lu T%lu",
	             uart4_total_packets, uart4_pkt_dropped, uart4_tx_total_bytes);

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
	    if(pwm_abs > MOTOR_PWM_MAX) pwm_abs = MOTOR_PWM_MAX;

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
	    if(Output_ServoBrake > SERVO_ANGLE_MAX) Output_ServoBrake = SERVO_ANGLE_MAX;
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

  arm_biquad_cascade_df2T_init_f32(

   &S_velocity_lpf,
   1,                          // 1 секция = 2-й порядок
   lpf_coeffs,
   velocity_lpf_state

  );

  arm_biquad_cascade_df2T_init_f32(
     &S_steer_lpf,
     1,
     steer_lpf_coeffs,
     steer_lpf_state
  );

  connection_lost = 0;
  heartbeat_fail_counter = 0;
  last_rx_time = 0;

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

  velosity = 0.0f;

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

  encoder.velocity = 0;
  encoder.last_counter_value = 32767;

  flag_pca9685  = 0;
  flag_nrf24l01 = 0;

  k = 0.01f;
  filVal = 0.0f;

  uart4_rx_idx = 0;

  uart4_tx_head = 0;
  uart4_tx_tail = 0;
  uart4_tx_busy = 0;
  uart4_tx_overflow = 0;
  uart4_tx_total_bytes = 0;

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
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC3_Init();
  MX_TIM5_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
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

   __HAL_UART_CLEAR_OREFLAG(&huart4);
   __HAL_UART_CLEAR_FEFLAG(&huart4);
   __HAL_UART_CLEAR_NEFLAG(&huart4);
   __HAL_UART_CLEAR_PEFLAG(&huart4);

   __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);   // прерывание на байт
   __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);   // прерывание на IDLE
   __HAL_UART_CLEAR_IDLEFLAG(&huart4);

  pkt_rx_init(&rx_pkt);

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

  HAL_Delay(50);


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
    if(GPIO_Pin == NRF24L01_INT_Pin) flag_nrf24l01 = 1;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {

	ST7735_IT_Callback(hspi);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// measure velocity, position
	if(htim == &htim4) update_encoder();
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
