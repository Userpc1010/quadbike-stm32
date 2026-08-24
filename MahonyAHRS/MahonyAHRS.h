
//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Modified: added dt parameter for variable sample rate
//
//=====================================================================================================

#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------------------------------------
// Includes
//----------------------------------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

//----------------------------------------------------------------------------------------------------
// Variable declaration
//----------------------------------------------------------------------------------------------------

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations
//---------------------------------------------------------------------------------------------------

/**
 * @brief Update AHRS with accelerometer and gyroscope only (IMU mode)
 * @param gx, gy, gz Gyroscope readings in rad/s
 * @param ax, ay, az Accelerometer readings in m/s²
 * @param dt Time step in seconds
 * @param velocity Linear velocity in m/s (for centrifugal compensation)
 */
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
/**
 * @brief Get current Euler angles from quaternion
 * @param roll, pitch, yaw Output angles in degrees
 */
void MahonyAHRSgetEuler(float *roll, float *pitch, float *yaw);
/**
 * @brief Вычислить угловую скорость относительно мировой вертикали (Z-earth)
 * @param gx, gy, gz Сырые или отфильтрованные данные гироскопа (rad/s)
 * @return Стабилизированная угловая скорость рысканья (rad/s)
 */
float MahonyAHRSgetStabilizedYawRateRad(float gx, float gy, float gz);
/**
 * @brief Reset quaternion to identity
 */
void MahonyAHRSreset(void);

/**
 * @brief Get quaternion components (for debugging)
 */
void MahonyAHRSgetQuaternion(float *q0_out, float *q1_out, float *q2_out, float *q3_out);

#ifdef __cplusplus
}
#endif

#endif /* MahonyAHRS_h */
