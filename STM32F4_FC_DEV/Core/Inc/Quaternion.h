#ifndef _QUATERNION_H
#define _QUATERNION_H

#include "main.h"
#include <math.h>
#include "ICM20602.h"

#define PI 					3.1415926535897932f
#define RAD2DEG             57.295779515f
#define DEG2RAD             0.017453289f
#define GYRO_DEGFSEL3 ‭    	0.06103515625f                	// 0.06103515625‬  = (4000/65536)
#define ACC_DEGFSEL3 ‭     	0.00048828125f                	// 0.000488281  = (32/65536)

// MadgwickQuaternion
#define ALPHA               0.98f //0.996                     // complementary filter coefficient
#define BETA                0       // 0.9                       // complementary filter coefficient
#define CAL_COUNTER         1000

// MahonyAHRS
#define twoKpDef  			(2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  			(2.0f * 0.0f) // 2 * integral gain




void Quaternion_Update(float* q);
float invSqrt(float x);
void MahonyAHRSupdateIMU(float* gx, float* gy, float* gz, float* ax, float* ay, float* az, float sampleFreq);

#endif
