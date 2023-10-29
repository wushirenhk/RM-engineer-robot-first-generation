#ifdef ENGINEER

#ifndef GIMBAL_CONTROL_TASK_H
#define GIMBAL_CONTROL_TASK_H

#include "Motor.h"
#include "Motor_RM_Tasks.h"


typedef enum
{
  GIMBAL_FREE = 0, // 自由控制模式
  GIMBAL_LOCK = 1, // 取金矿模式
 
} Gimbal_Control_Mode;

typedef enum
{
  Overturn_OFF = 0, // 下电模式
  Overturn_CW  = 1, // 顺时针
  Overturn_CCW  = 2, // 顺时针  
  Overturn_IN =  3, // 收起矿石
  Overturn_OUT = 4  // 拿出矿石
} Overturn_Control_Mode;


class GimbalControlTask : public Task_Base
{
public:
  friend class Robot;
  friend class RemoteControlTask;
  friend class MainControlTask;
  friend class RefereeSystemTask;
  friend class Arm_ControlTask;
  friend class ChassisControlTask;

  GimbalControlTask(
    Robot &robot0,
    Motor_RM_Params_t *Overturn1, PID_Params_t *Overturn1_ang, PID_Params_t *Overturn1_ang_vel,
    Motor_RM_Params_t *Overturn2, PID_Params_t *Overturn2_ang, PID_Params_t *Overturn2_ang_vel,
    Motor_RM_Params_t *gimbal_6020,PID_Params_t *gimbal_6020_ang, PID_Params_t *gimbal_6020_ang_vel,

    timeus_t interval_tick_us0 = 0
  );

  virtual ~GimbalControlTask(void) {}

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);
	  
  Motor_RM_PIDControlTask* getOverturn1PIDControlTaskPointer(void)
  {
    return motor_Overturn1_pid_task;
  }
  Motor_RM_PIDControlTask* getOverturn2PIDControlTaskPointer(void)
  {
    return motor_Overturn2_pid_task;
  }
  
protected:
	
  Motor_RM_PIDControlTask *motor_Overturn1_pid_task, *motor_Overturn2_pid_task, *motor_gimbal_6020_pid_task;
  Gimbal_Control_Mode gimbal_mode;
  Overturn_Control_Mode overturn_mode;

  float exp_angular_velocity_CW;
  float exp_angular_velocity_IN;
  float overturn1_angle;
  float overturn2_angle;

  // 云台（控制角度）
  float gimbal_yaw_angle = -2.64f;
  float gimbal_pitch_angle = 115;
  float angle_glod_yaw = -2.64f;
  float angle_glod_pitch = 105;  
  float angle_yaw_Orebin = 0.5f;
  float angle_pitch_Orebin = 115;
  float angle_free_foreward = -1.07;

   //角速度
  float gimbal_yaw_vel;
  float gimbal_pitch_vel;
  //限幅
  uint8_t gimbal_max_yaw_vel = 7;

 //翻矿电机限幅
  float overturn1_angular_velocity_max = 5.0f;
  float overturn1_angular_velocity_min = -5.0f;
  float overturn2_angular_velocity_max = 5.0f;
  float overturn2_angular_velocity_min = -5.0f;

};


#endif
#endif
