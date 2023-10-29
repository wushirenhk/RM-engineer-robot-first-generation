#ifdef ENGINEER
#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "AttitudeSolutionTask.h"
#include "Scheduler.h"
#include "InertialSensor.h"
#include "InertialSensor_BMI088.h"
#include "RCProtocol.h"
#include "Error.h"
#include "LEDControlTask.h"
#include "stdint.h"
#include "MahonyAHRS.h"
#include "Helper.h"
#include "SongPlayerTask.h"
#include "CanDevice.h"
#include "Motor.h"

#include "/Referee/RefereeSystemTask.h"
#include "/Chassis/ChassisControlTask.h"
//#include "/Control/MainControlTask.h"
#include "/Remote/RemoteControlTask.h"
//#include "/Control/ComputerVisionControlTask.h"
#include "/Robot/Params.h"
#include "ArmContorlTask/ArmControlTask.h"
#include "Gimbal/GimbalControlTask.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;



class Robot
{
public:
	friend class RefereeSystem;
  friend class AttitudeSolutionTask;
  friend class LEDControlTask;
  friend class RefereeSystemTask;
  friend class SongPlayerTask;
  friend class Motor_RM_PIDControlTask;
  friend class CANSendTask;
  friend class PID_GyrotempTask;
  friend class ChassisControlTask;
  friend class GimbalControlTask;
//  friend class MainControlTask;
  friend class RemoteControlTask;
//  friend class ComputerVisionControlTask;
  friend class IMUDataSyncTask;
  friend class Arm_ControlTask;
  friend class Motor_MC_ControlTask;
  friend class Motor_DM_PSControlTask;
  friend class Motor_M15_PoControlTask;
  friend class Motor_MG_ControlTask;

  Robot()
  {
 
  };

  void init(void);
  void setParams(void);
  void run(void);

  /* 调出保护域的任务类指针 */
  Scheduler getScheduler(void)
  {						
    return scheduler;
  }
  AttitudeSolutionTask* getAttitudeSolutionTaskPointer(void)
  {
    return attitude_solution_task_p;
  }
  LEDControlTask* getLEDControlTaskPointer(void)
  {
    return led_control_task_p;
  }
  RCProtocol* getRCProtocolPointer(void)
  {
    return &rc_protocol;
  }
  InertialSensor* getInertialSensorPointer(void)
  {
    return &inertial_sensors;
  }
  RefereeSystemTask *getRefereeSystemTaskPointer(void)
  {
    return referee_system_task_p;
  }
  RemoteControlTask *getRemoteControlTask(void)
  {
    return remote_control_task_p;
  }
//  Helper *getHelperPointer(void)
//  {
//    return &helper;
//  }
  CanDevice *getCAN1DevicePointer(void)
  {
    return &can1_device;
  }
  CanDevice *getCAN2DevicePointer(void)
  {
    return &can2_device;
  }
  uint8_t getBMI088BackendId()
  {
    return bmi088_id;
  }
	ChassisControlTask *getChassisControlTaskPointer(void)
	{
		return chassis_task_p;
	}
	
	Arm_ControlTask *getArmControlTaskPointer(void)
	{
		return arm_task_p;
	}
		GimbalControlTask *getGimbalControlTaskPointer()
	{
		return gimbal_task_p;
	}
	

	/* 调出保护域的任务类指针 */
	
protected:
	/* 定义Library库中类的对象 */
  InertialSensor inertial_sensors; // imu对象前台
  uint8_t bmi088_id;

  RCProtocol rc_protocol;//RC前台
  Helper helper;
  Scheduler scheduler;
  CanDevice can1_device, can2_device;
  Motor motors;//Motor前台
  Params params;
  
  /* 定义Library库中类的对象 */

  /*任务类指针*/ 
  AttitudeSolutionTask *attitude_solution_task_p;
  LEDControlTask * led_control_task_p;
  RefereeSystemTask *referee_system_task_p;

  CANSendTask   *can2_send_task_0x200_p,
                *can1_send_task_0x200_p,
                *can2_send_task_0x1ff_p,
                *can1_send_task_0x1ff_p,
					 *can1_send_task_0x101_p,
					 *can1_send_task_0x102_p,					 
					 *can1_send_task_0x32_p,
					 *can1_send_task_0x145_p,
					 *can1_send_task_0x146_p;					 
					 
					 /*BM临时设置使用*/
					 //*can1_send_task_0x105_p; 
//  MainControlTask *main_control_task_p;
  RemoteControlTask *remote_control_task_p;
//  ComputerVisionControlTask *cv_control_task_p;
  ChassisControlTask *chassis_task_p;
  Arm_ControlTask *arm_task_p;//机械臂任务指针
  GimbalControlTask *gimbal_task_p;
  
	/*任务类指针*/ 
};



#endif

#endif
