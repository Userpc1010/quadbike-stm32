//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Modified: added dt parameter for variable sample rate
//
//=====================================================================================================

#include "MahonyAHRS.h"
#include <math.h>
#include "../../DSP/Include/arm_math.h"

//---------------------------------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------------------------------

#define twoKpDef 0.05f	// proportional gain
#define RAD_TO_DEG  57.295779513082320876f
#define Quat_HALFPI 1.5707963267948966192313216916398f
#define Quat_PI 3.1415926535897932384626433832795f
#define Quat_TWOPI 6.283185307179586476925286766559f
#define Quat_TODEG(x) ((x) * 57.2957796f)

//---------------------------------------------------------------------------------------------------
// Variable definitions
//---------------------------------------------------------------------------------------------------

volatile float twoKp = twoKpDef;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
//---------------------------------------------------------------------------------------------------
// Function declarations
//---------------------------------------------------------------------------------------------------

float invSqrt(float x);

//====================================================================================================
// Functions
//====================================================================================================

/**
 * @brief Update AHRS with accelerometer and gyroscope only (IMU mode)
 * @param gx, gy, gz Gyroscope readings in rad/s
 * @param ax, ay, az Accelerometer readings in m/s²
 * @param dt Time step in seconds
 */
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	if(dt <= 0.0f || dt > 0.1f) dt = 0.005f;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		gx = gx * (1.0f - twoKp) + halfex * twoKp;
		gy = gy * (1.0f - twoKp) + halfey * twoKp;
		gz = gz * (1.0f - twoKp) + halfez * twoKp;

	}

	// Интеграция гироскопа
	gx *= (0.5f * dt);
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Get Euler angles from quaternion
//---------------------------------------------------------------------------------------------------

void MahonyAHRSgetEuler(float *roll, float *pitch, float *yaw)
{
	float CBn[5];

	  float q0q0 = q0 * q0;

	  //x-y-z
	  CBn[0] = 2.0f * (q0q0 + q1 * q1) - 1.0f;
	  CBn[1] = 2.0f * (q1 * q2 + q0 * q3);
	  CBn[2] = 2.0f * (q1 * q3 - q0 * q2);
	  //CBn[3] = 2.0f * (q1 * q2 - q0 * q3);
	  //CBn[4] = 2.0f * (q0q0 + q2 * q2) - 1.0f;
	  CBn[3] = 2.0f * (q2 * q3 + q0 * q1);
	  //CBn[6] = 2.0f * (q1 * q3 + q0 * q2);
	  //CBn[7] = 2.0f * (q2 * q3 - q0 * q1);
	  CBn[4] = 2.0f * (q0q0 + q3 * q3) - 1.0f;

	  //roll
	  *roll = atan2f(-CBn[3], CBn[4]);

	  if (*roll > Quat_PI ) *roll -= Quat_TWOPI;
	  if (*roll < -Quat_PI) *roll += Quat_TWOPI;

	  //pitch
	  *pitch = atan2f(-CBn[2], CBn[4]);

	  if (*pitch >  Quat_PI) *pitch -= Quat_TWOPI;
	  if (*pitch < -Quat_PI) *pitch += Quat_TWOPI;

	  //yaw
	  *yaw = atan2f(-CBn[1], CBn[0]);

	  if (*yaw >  Quat_PI) *yaw -= Quat_TWOPI;
	  if (*yaw < -Quat_PI) *yaw += Quat_TWOPI;

	  *roll = Quat_TODEG(*roll);
	  *pitch = Quat_TODEG(*pitch);
	  *yaw = Quat_TODEG(*yaw);
}

/**
 * @brief Вычислить угловую скорость относительно мировой вертикали (Z-earth)
 * @param gx, gy, gz Сырые или отфильтрованные данные гироскопа (rad/s)
 * @return Стабилизированная угловая скорость рысканья (rad/s)
 */
float MahonyAHRSgetStabilizedYawRateRad(float gx, float gy, float gz)
{
    // Компоненты вектора вертикали Z в системе координат корпуса
    // (Третья строка матрицы вращения DCM)
    float r31 = 2.0f * (q1 * q3 - q0 * q2);
    float r32 = 2.0f * (q2 * q3 + q0 * q1);
    float r33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // Скалярное произведение вектора гироскопа на вектор вертикали
    // Дает проекцию вращения на ось, параллельную гравитации
    float yaw_rate_earth = (gx * r31) + (gy * r32) + (gz * r33);

    return yaw_rate_earth;
}

void MahonyAHRSreset(void)
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

void MahonyAHRSgetQuaternion(float *q0_out, float *q1_out, float *q2_out, float *q3_out)
{
    *q0_out = q0;
    *q1_out = q1;
    *q2_out = q2;
    *q3_out = q3;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
//---------------------------------------------------------------------------------------------------


float invSqrt(float x) {
    float out;
    arm_status status = arm_sqrt_f32(x, &out);
    return 1.0f / out;
}

