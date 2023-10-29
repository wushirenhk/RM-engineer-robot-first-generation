#ifdef ENGINEER
#ifndef PARAMS_H
#define PARAMS_H

#include "Motor_RM.h"
#include "Motor_MC.h"
#include "Motor_DM.h"
#include "Motor_M15.h"
#include "PID_Controller.h"
#include "Motor_MG.h"

//任务周期
typedef struct _Control_Tasks_Interval_t
{
  float ammo_task_interval;
  float chassis_task_interval;
  float led_task_interval;
  float referee_system_task_interval;
  float main_control_task_interval;
  float can1_send_0x1ff_task_interval;
  float can2_send_0x1ff_task_interval;
  float can2_send_0x200_task_interval;
  float can1_send_0x200_task_interval;
  float can1_send_0x141_task_interval;
  float can1_send_0x142_task_interval;
  float can1_send_0x143_task_interval;
  float can1_send_0x144_task_interval;
  float can1_send_0x101_task_interval;
  float can1_send_0x102_task_interval;
  float can1_send_0x32_task_interval;
  float can1_send_0x145_task_interval;
  float can1_send_0x146_task_interval;
  float arm_task_interval;
  float gimbal_task_interval;
} Control_Tasks_Interval_t;

//初始化参数结构体，包含了电机参数，PID参数和任务时间周期参数
typedef struct _Init_Params_t
{
  Motor_RM_Params_t chassis_motor_1,
                    chassis_motor_2,
                    chassis_motor_3,
                    chassis_motor_4,
	                 gimbal_6020,
	                 arm_3508,
	                 overturn1_3508,
	                 overturn2_3508;

  PID_Params_t chassis_motor_1_ang_vel,
               chassis_motor_2_ang_vel,
               chassis_motor_3_ang_vel,
               chassis_motor_4_ang_vel,
		         arm_3508_ang,
					arm_3508_ang_vel,
	            gimbal_6020_ang,
					gimbal_6020_ang_vel,
	            overturn1_3508_ang,
	            overturn1_3508_ang_vel,
	            overturn2_3508_ang,              
               overturn2_3508_ang_vel;
					
					
  Motor_MC_Params_t arm_motor_bottom,
	                 arm_motor_middle,
	                 arm_motor_top_roll,
	                 arm_motor_top_pitch;
	
  Motor_DM_Params_t arm_dm_1,
	                 arm_dm_2;
	
  Motor_M15_Params_t	arm_M15_1;
	
  Motor_MG_Params_t arm_mg_1,
                    arm_mg_2;
	
	
  Control_Tasks_Interval_t control_tasks_interval;

} Init_Params_t;

class Params
{
	
public:
  Params() {}
  void initMotorsParams();
  Init_Params_t motor_params;

};




#endif
#endif