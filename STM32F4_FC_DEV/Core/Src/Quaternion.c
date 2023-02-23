#include "Quaternion.h"




float Roll;
float Pitch;
float Yaw;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            			// vector to hold quaternion

float GYROX_RATE=0.0f, GYROY_RATE=0.0f, GYROZ_RATE=0.0f, ACCX_RATE=0.0f, ACCY_RATE=0.0f, ACCZ_RATE=0.0f;
float gx_cal=0.0f, gy_cal=0.0f, gz_cal=0.0f,	ax_cal=0.0f, ay_cal=0.0f;


float gyroBias[3] 			= {0, 0, 0};
float accelBias[3] 			= {0, 0, 0}; // Bias corrections for gyro and accelerometer
float beta 					= 0.6f;
float zeta 					= 0.03f;

//MahonyAHRS
float twoKp = 1;                      // 2 * proportional gain (Kp)
float twoKi = 0.0f;                      // 2 * integral gain (Ki)
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki


void GetRPY(float* sampleFreq)
{


	  ICM20602_Get6AxisRawData(&ICM20602.acc_x_raw, &ICM20602.gyro_x_raw);	//	39.11us
	  GYROX_RATE = (ICM20602.gyro_x_raw - gx_cal) * 0.06103515625 * 0.017453289;
	  GYROY_RATE = (ICM20602.gyro_y_raw - gy_cal) * 0.06103515625 * 0.017453289;
	  GYROZ_RATE = (ICM20602.gyro_z_raw - gz_cal) * 0.06103515625 * 0.017453289;

	  ACCX_RATE = (ICM20602.acc_x_raw - ax_cal) * 0.00048828125;
	  ACCY_RATE = (ICM20602.acc_y_raw - ay_cal) * 0.00048828125;
	  ACCZ_RATE = (ICM20602.acc_z_raw) * 0.00048828125;

//		  MadgwickQuaternionUpdate(&ACCX_RATE,&ACCY_RATE,&ACCZ_RATE,&GYROX_RATE,&GYROY_RATE,&GYROZ_RATE);	//57us
	  MahonyAHRSupdateIMU(&GYROX_RATE,&GYROY_RATE,&GYROZ_RATE, &ACCX_RATE,&ACCY_RATE,&ACCZ_RATE, *sampleFreq);		//42us
	  Quaternion_Update(&q);	//10us

//		  printf("%.2f\n",(sampleFreq[0]));
//		  printf("%.d %.d %.d\n", ICM20602.gyro_x_raw, ICM20602.gyro_y_raw, ICM20602.gyro_z_raw);
//		  printf("%.1f %.1f %.1f\n", GYROX_RATE, GYROY_RATE, GYROZ_RATE);
//		  printf("%.1f %.1f %.1f\n", ACCX_RATE, ACCY_RATE, ACCZ_RATE);
}

void Quaternion_Update(float* q)
{
	float q1, q2, q3, q4;

	q1 = q[0]; //x
	q2 = q[1]; //y
	q3 = q[2]; //z
	q4 = q[3]; //w


	Yaw 	= -atan2f(2.0f * (q2*q3 + q1*q4), q1*q1 + q2*q2 - q3*q3 - q4*q4);
	Pitch  	= -asinf(2.0f * (q2*q4 - q1*q3));
	Roll   	= atan2f(2.0f * (q1*q2 + q3*q4), q1*q1 - q2*q2 - q3*q3 + q4*q4);

	Pitch *= RAD2DEG;
	Roll  *= RAD2DEG;
	Yaw   *= RAD2DEG;
	
	if(Yaw>=0)
		Yaw = 360.f - Yaw;
	else	
		Yaw = -Yaw;
	
//	 printf("%d %d %d\n", (int)(Roll), (int)(Pitch), (int)(Yaw));
//	 printf("%d %d %d\n", (int)(q[0]*100), (int)(q[1]*100),(int)(q[2]*100));


}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}





void MadgwickQuaternionUpdate(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float sampleFreq)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax[0] * ax[0] + ay[0] * ay[0] + az[0] * az[0]);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax[0] *= norm;
    ay[0] *= norm;
    az[0] *= norm;

    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax[0];
    f2 = _2q1 * q2 + _2q3 * q4 - ay[0];
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az[0];
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gy[0]roscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    // Compute and remove gyroscope biases
    gbiasx += gerrx / sampleFreq * zeta;
    gbiasy += gerry / sampleFreq * zeta;
    gbiasz += gerrz / sampleFreq * zeta;
    gx[0] -= gbiasx;
    gy[0] -= gbiasy;
    gz[0] -= gbiasz;

    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx[0] - _halfq3 * gy[0] - _halfq4 * gz[0];
    qDot2 =  _halfq1 * gx[0] + _halfq3 * gz[0] - _halfq4 * gy[0];
    qDot3 =  _halfq1 * gy[0] - _halfq2 * gz[0] + _halfq4 * gx[0];
    qDot4 =  _halfq1 * gz[0] + _halfq2 * gy[0] - _halfq3 * gx[0];

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) / sampleFreq;
    q2 += (qDot2 -(beta * hatDot2)) / sampleFreq;
    q3 += (qDot3 -(beta * hatDot3)) / sampleFreq;
    q4 += (qDot4 -(beta * hatDot4)) / sampleFreq;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}


void MahonyAHRSupdateIMU(float* gx, float* gy, float* gz, float* ax, float* ay, float* az, float sampleFreq) {

  float norm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax[0] == 0.0f) && (ay[0] == 0.0f) && (az[0] == 0.0f))) {

    // Normalise accelerometer measurement
    norm = sqrt(ax[0] * ax[0] + ay[0] * ay[0] + az[0] * az[0]);
    ax[0] /= norm;
    ay[0] /= norm;
    az[0] /= norm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q[1] * q[3] - q[0] * q[2];
    halfvy = q[0] * q[1] + q[2] * q[3];
    halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay[0] * halfvz - az[0] * halfvy);
    halfey = (az[0] * halfvx - ax[0] * halfvz);
    halfez = (ax[0] * halfvy - ay[0] * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx[0] += integralFBx;  // apply integral feedback
      gy[0] += integralFBy;
      gz[0] += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx[0] += twoKp * halfex;
    gy[0] += twoKp * halfey;
    gz[0] += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx[0] *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy[0] *= (0.5f * (1.0f / sampleFreq));
  gz[0] *= (0.5f * (1.0f / sampleFreq));
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx[0] - qc * gy[0] - q[3] * gz[0]);
  q[1] += (qa * gx[0] + qc * gz[0] - q[3] * gy[0]);
  q[2] += (qa * gy[0] - qb * gz[0] + q[3] * gx[0]);
  q[3] += (qa * gz[0] + qb * gy[0] - qc * gx[0]);

  // Normalise quaternion
  norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] /= norm;
  q[1] /= norm;
  q[2] /= norm;
  q[3] /= norm;
}

