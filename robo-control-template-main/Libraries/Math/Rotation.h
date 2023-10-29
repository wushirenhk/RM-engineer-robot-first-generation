#ifndef __ROTATION_H__
#define __ROTATION_H__

#include "stdint.h"
#include "Vector3.h"
#include <math.h>
#include "arm_math.h"

template <typename T>
class Rotation
{
public:
  T pitch, roll, yaw;

  // trivial ctor
  Rotation<T>()
    : pitch(0)
    , roll(0)
    , yaw(0) {}

  // setting ctor
  Rotation<T>(const T pitch0, const T roll0, const T yaw0)
    : pitch(pitch0)
    , roll(roll0)
    , yaw(yaw0) {}

  Rotation<T>(const Vector3<T> vec)
  {
    yaw = atan2(vec.y, vec.x) / PI * 180.0;
    pitch = -atan2(vec.z, sqrtf(vec.x * vec.x + vec.y * vec.y)) / PI * 180.0;
  }
};

typedef Rotation<int16_t>                Rotation3i;
typedef Rotation<uint16_t>               Rotation3ui;
typedef Rotation<int32_t>                Rotation3l;
typedef Rotation<uint32_t>               Rotation3ul;
typedef Rotation<float>                  Rotation3f;
typedef Rotation<double>                 Rotation3d;

enum RotationPreset
{
  ROTATION_NONE                = 0,
  ROTATION_YAW_45              = 1,
  ROTATION_YAW_90              = 2,
  ROTATION_YAW_135             = 3,
  ROTATION_YAW_180             = 4,
  ROTATION_YAW_225             = 5,
  ROTATION_YAW_270             = 6,
  ROTATION_YAW_315             = 7,
  ROTATION_ROLL_180            = 8,
  ROTATION_ROLL_180_YAW_45     = 9,
  ROTATION_ROLL_180_YAW_90     = 10,
  ROTATION_ROLL_180_YAW_135    = 11,
  ROTATION_PITCH_180           = 12,
  ROTATION_ROLL_180_YAW_225    = 13,
  ROTATION_ROLL_180_YAW_270    = 14,
  ROTATION_ROLL_180_YAW_315    = 15,
  ROTATION_ROLL_90             = 16,
  ROTATION_ROLL_90_YAW_45      = 17,
  ROTATION_ROLL_90_YAW_90      = 18,
  ROTATION_ROLL_90_YAW_135     = 19,
  ROTATION_ROLL_270            = 20,
  ROTATION_ROLL_270_YAW_45     = 21,
  ROTATION_ROLL_270_YAW_90     = 22,
  ROTATION_ROLL_270_YAW_135    = 23,
  ROTATION_PITCH_90            = 24,
  ROTATION_PITCH_270           = 25,
  ROTATION_PITCH_180_YAW_90    = 26, // same as ROTATION_ROLL_180_YAW_270
  ROTATION_PITCH_180_YAW_270   = 27, // same as ROTATION_ROLL_180_YAW_90
  ROTATION_ROLL_90_PITCH_90    = 28,
  ROTATION_ROLL_180_PITCH_90   = 29,
  ROTATION_ROLL_270_PITCH_90   = 30,
  ROTATION_ROLL_90_PITCH_180   = 31,
  ROTATION_ROLL_270_PITCH_180  = 32,
  ROTATION_ROLL_90_PITCH_270   = 33,
  ROTATION_ROLL_180_PITCH_270  = 34,
  ROTATION_ROLL_270_PITCH_270  = 35,
  ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
  ROTATION_ROLL_90_YAW_270     = 37,
  ROTATION_ROLL_90_PITCH_68_YAW_293 = 38, // this is actually, roll 90, pitch 68.8, yaw 293.3
  ROTATION_PITCH_315           = 39,
  ROTATION_ROLL_90_PITCH_315   = 40,
  ROTATION_PITCH_7             = 41,
  ROTATION_ROLL_45             = 42,
  ROTATION_ROLL_315            = 43,
  ///////////////////////////////////////////////////////////////////////
  // Do not add more rotations without checking that there is not a conflict
  // with the MAVLink spec. MAV_SENSOR_ORIENTATION is expected to match our
  // list of rotations here. If a new rotation is added it needs to be added
  // to the MAVLink messages as well.
  ///////////////////////////////////////////////////////////////////////
  ROTATION_MAX,
  ROTATION_CUSTOM_OLD          = 100,
  ROTATION_CUSTOM_1            = 101,
  ROTATION_CUSTOM_2            = 102,
  ROTATION_CUSTOM_END,
};

#endif
