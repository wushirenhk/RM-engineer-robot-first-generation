#ifndef __MOTOR_DATA_H__
#define __MOTOR_DATA_H__

#include "Task.h"

#define RoboMaster_GM6020     0
#define RoboMaster_3508       1
#define RoboMaster_2006       2
#define RMD_X8                3
#define DM_J4310              4
#define BM_M15                5
#define MG8016                6

#define MOTOR_CW              1    // 电流正负与转矩正负（右手定则判定）相同
#define MOTOR_CCW            -1    // 电流正负与转矩正负（右手定则判定）相反

typedef struct _Motor_Linear_Movement_t
{
  float offset; // 线基准（m）
  float displacement; // 线位移（m）
  float relative_displacement; // 相对线基准的线位移（m）
  float velocity; // 线速度（m/s）
  // float acceleration; // 线性加速度

} Motor_Linear_Movement_t;

typedef struct _Motor_Angular_Movement_t
{
  float angle; // 角位移（rad）
  float polar_angle; // 极角(-PI~PI)
  float angular_velocity; // 角速度（rad/s，可能精度不够）
  float angular_velocity_imu; // 由IMU计算出来的角速度（rad/s）
  // float acceleration; // 角加速度（rad/s^2）

	
	
} Motor_Angular_Movement_t;

typedef struct _Motor_Common_Measurement_t
{
  timeus_t timestamp; // 本次时间戳（uint32_t）

  Motor_Angular_Movement_t rotator, output; // 转子，输出轴
  Motor_Linear_Movement_t linear; // 线性

} Motor_Common_Measurement_t;

#endif
