#ifndef __MAHONYAHRS_H__
#define __MAHONYAHRS_H__

#include "stdint.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "AHRS.h"

#define TWO_KP_DEF	(2.0f * 0.5f)	// 2 * proportional gain
#define TWO_KI_DEF	(2.0f * 0.0f)	// 2 * integral gain
#define SAMPLE_FREQ_DEF 800.0f 		// sample frequency in Hz

class MahonyAHRS : public AHRS
{
public:
  MahonyAHRS(float two_kp0 = TWO_KP_DEF, float two_ki0 = TWO_KI_DEF);
  virtual void init(void);
  virtual void update(Vector3f accel, Vector3f gyro);
  const float sample_freq;
  
  
protected:
  volatile float two_kp;
  volatile float two_ki;
  Vector3f integral_fb; // integral error terms scaled by Ki
};

#endif
