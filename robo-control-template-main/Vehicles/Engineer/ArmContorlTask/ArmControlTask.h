#ifdef ENGINEER
#ifndef ARM_CONTROL_TASK_H
#define ARM_CONTROL_TASK_H

#include "Motor.h"
#include "Motor_MC_Tasks.h"
#include "Motor_DM_Task.h"
#include "Motor_M15_Task.h"
#include "Motor_MG_Tasks.h"
#include "Motor_RM_Tasks.h"
#include "../Robot/Params.h"

typedef enum
{
  Arm_OFF = 0 , 
  Arm_normal = 1,       // 机械臂正常运行
  Arm_AirConnection = 2,//机械臂空接
  Arm_SilveryOre = 3,   // 机械臂取银矿石
  Arm_GroundOre = 4,    // 机械臂取地面矿石
  Arm_OrePlacement = 5, //机械臂放置矿石
  Arm_HoldOre = 6       //机械臂抓住矿石前进
} Arm_Control_Mode;

typedef enum
{
  state0 = 0,
  state1 = 1,
  state2 = 2, 
  state3 = 3,
  state4 = 4
} Arm_Mode_Check;


class Arm_ControlTask : public Task_Base  
{
   friend class Robot;
	friend class RemoteControlTask;
	friend class RefereeSystemTask;
	friend class Motor_MC_ControlTask;
   friend class Motor_MG_ControlTask;
   friend class Motor_MG_ControlTask;	
public:		
	Arm_ControlTask(
    Robot &robot0,
    Motor_RM_Params_t *arm_3508,PID_Params_t *arm_3508_ang, PID_Params_t *arm_3508_ang_vel,

    Motor_DM_Params_t *arm_dm_1,
    Motor_DM_Params_t *arm_dm_2,
	 Motor_MG_Params_t *arm_mg_1,
	 timeus_t interval_tick_us0 = 0
   );
	
  virtual ~Arm_ControlTask(void) {}
  //任务基类要求必须的函数
  virtual void init(void);

  virtual void update(timeus_t dT_us);
  
  virtual void uninit(void);	
	  
  uint8_t airpump_flag[3];
	
  void fivetimes_plan();//五次三项式
	  
  void arm_mode_switch();	  
	  
  int arm_time_delay(uint16_t time);
	  
protected:
	Motor_RM_PIDControlTask *motor_3508_pid_task;
   Motor_DM_PSControlTask *motor_dm1_task,*motor_dm2_task;
	Motor_MG_ControlTask *motor_mg1_task;
   
   uint8_t switch_lock;
   uint8_t airpump_lock[2];
   Arm_Control_Mode arm_mode;
   Arm_Control_Mode last_arm_mode;

   Arm_Mode_Check normal;
   Arm_Mode_Check AirConnection;
   Arm_Mode_Check SilveryOre;
   Arm_Mode_Check GroundOre;
   Arm_Mode_Check OrePlacement;
   Arm_Mode_Check HoldOre;
   uint16_t timecnt;
   
  //机械臂四个电机绝对位置控制（增量）

  
  float exp_angle;
  float arm_3508_angle;

  float arm3508_angular_velocity_max;
  float arm3508_angular_velocity_min;

  // 云台（遥控器控制转速）
  float gimbal_yaw_vel;
  float gimbal_pitch_vel;
  

 //不同状态下各个电机的角度控制（包括保持状态与过渡状态）
 //状态1：正常行驶状态
 int32_t mg1_state_normal_angle;uint16_t mg1_state_normal_speed;
 float dm1_state_normal_angle;float dm1_state_normal_speed;
 float dm2_state_normal_angle;float dm2_state_normal_speed;
 float arm3508_state_normal_angle;

 
 //状态2：空接(金矿)状态
 int32_t mg1_state_AirConnection_angle;uint16_t mg1_state_AirConnection_speed;
 float dm1_state_AirConnection_angle;float dm1_state_AirConnection_speed;
 float dm2_state_AirConnection_angle;float dm2_state_AirConnection_speed;
 float arm3508_state_AirConnection_angle;
 
 //状态3：取银矿石
 int32_t mg1_state_SilveryOre_angle;uint16_t mg1_state_SilveryOre_speed;
 float dm1_state_SilveryOre_angle;float dm1_state_SilveryOre_speed;
 float dm2_state_SilveryOre_angle;float dm2_state_SilveryOre_speed;
 float arm3508_state_SilveryOre_angle;
 
 //状态4：取地面矿石
 int32_t mg1_state_GroundOre_angle;uint16_t mg1_state_GroundOre_speed;
 float dm1_state_GroundOre_angle;float dm1_state_GroundOre_speed;
 float dm2_state_GroundOre_angle;float dm2_state_GroundOre_speed;
 float arm3508_state_GroundOre_angle;
 
 //状态5：放置矿石于翻矿机
 int32_t mg1_state_OrePlacement_angle;int16_t mg1_state_OrePlacement_speed;
 float dm1_state_OrePlacement_angle;float dm1_state_OrePlacement_speed;
 float dm2_state_OrePlacement_angle;float dm2_state_OrePlacement_speed;
 float arm3508_state_OrePlacement_angle;

 //状态6：抓住矿石前进
 int32_t mg1_state_HoldOre_angle;int16_t mg1_state_HoldOre_speed;
 float dm1_state_HoldOre_angle;float dm1_state_HoldOre_speed;
 float dm2_state_HoldOre_angle;float dm2_state_HoldOre_speed;
 float arm3508_state_HoldOre_angle;
 
 
 
};


#endif

#endif
