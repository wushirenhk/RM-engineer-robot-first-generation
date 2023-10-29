#ifdef ENGINEER

#ifndef CHASSIS_CONTROL_TASK_H
#define CHASSIS_CONTROL_TASK_H

#include "Motor.h"
#include "Motor_RM_Tasks.h"
#include "Filters.h"
#include "/Robot/Params.h"

//#define CAN_CHASSIS_MOTOR_SUPERCAP_CTRL_ID      0x300
//#define CAN_CHASSIS_CTRL_ID                     0x200
//#define CAN_3508_MOTOR1_ID                      0x201
//#define CAN_3508_MOTOR2_ID                      0x202
//#define CAN_3508_MOTOR3_ID                      0x203
//#define CAN_3508_MOTOR4_ID                      0x204

#define INITIAL_HALF_WHEEL_BASE .42f// 半轴距[m]
#define INITIAL_HALF_TREAD .42// 半轮距[m]
#define INITIAL_ROTATE_RATIO_X .0f// 旋转中心在x方向上的归一化坐标
#define INITIAL_ROTATE_RATIO_Y .0f// 旋转中心在y方向上的归一化坐标
#define INITIAL_WHEEL_VEL_RATIO 1.0f// 轮速缩放
#define INITIAL_XY_Chassis_Slow 1.0f// //平移轮速缩放

struct suercap_RX_data_pack
{
	float supercap_voltage;
	float supercap_energy_percent;
};

typedef enum
{
  Chassis_OFF = 0, // 底盘失能
  Chassis_FAST = 1, // 快速行驶（赶路用）
  Chassis_SLOW = 2, // 慢速行驶（微调用）
} Chassis_Control_Mode;

typedef enum
{
  Chassis_NORMAL = 0, // 正常
  Chassis_ORE = 1 // 看矿石

} Chassis_direction_Mode;

class ChassisControlTask : public Task_Base
{
public:
  friend class Robot;
  friend class MainControlTask;
  friend class RemoteControlTask;
  friend class RefereeSystemTask;

  ChassisControlTask(
    Robot &robot0,
    Motor_RM_Params_t *m1, PID_Params_t *m1_ang, PID_Params_t *m1_ang_vel,
    Motor_RM_Params_t *m2, PID_Params_t *m2_ang, PID_Params_t *m2_ang_vel,
    Motor_RM_Params_t *m3, PID_Params_t *m3_ang, PID_Params_t *m3_ang_vel,
    Motor_RM_Params_t *m4, PID_Params_t *m4_ang, PID_Params_t *m4_ang_vel,
    timeus_t interval_tick_us0 = 0
  ) ;

  virtual ~ChassisControlTask(void) {};

  virtual void init(void);

  virtual void update(timeus_t dT_us);

  virtual void uninit(void);

	void setControlMode(Chassis_Control_Mode mode)
  {
    chassis_mode = mode;
  }
  
	Chassis_Control_Mode getControlMode()
  {
    return chassis_mode;
  }
	
	float getexp_vel(void)
	{
		return exp_vel[2];
	}
	
	Motor_RM_PIDControlTask *getMotor_RM_PIDControlTaskPointer(void)
	{
		return motor_3_pid_task;
	}	
	
	uint8_t *get_can_tx_data()
	{
		return can_tx_data;
	}
	
	LowPassFilter<float> get_lowpass_filter()
	{
		return lowpass_filter;
	}
//	PIDControlTask getChassis_follow_PIDTask()
//	{
//		return chassis_follow_pid_task;
//	}	
	
	Chassis_direction_Mode chassis_direction_mode;
	
protected:
  // uint8_t can_tx_data[8];
  Motor_RM_PIDControlTask *motor_1_pid_task, *motor_2_pid_task, *motor_3_pid_task, *motor_4_pid_task;
	Chassis_Control_Mode chassis_mode;

	LowPassFilter<float> lowpass_filter;
	
	uint8_t can_tx_data[8];
  float vx; // 底盘x方向速度[m/s]
  float vy; // y方向速度[m/s]
  float vw; // 绕z轴角速度[rad/s]
  float vx_filter;//滤波后的底盘x方向速度[m/s]
  float vy_filter;//滤波后的底盘x方向速度[m/s]
	
  float half_wheel_base; // 半轴距[m]
  float half_tread; // 半轮距[m]

  float rotate_ratio_x; // 旋转中心在x方向上的归一化坐标[-1.0, 1.0]，轮轴上为1.0或-1.0
  float rotate_ratio_y; // 旋转中心在y方向上的归一化坐标[-1.0, 1.0]，轮轴上为1.0或-1.0

  float wheel_vel_ratio; // 旋转轮速缩放
	float xy_chassis_slow;//平移轮速缩放
  float exp_vel[4] = {0}; // 四个电机期望线速度[m/s]
	float rotate_ratio[4];
  int16_t motor_send[4] = {0}; // 最终发给电机的电流值
  /*     forward
            X              1.0
           /|\
       M2   |   M1
    Y<------|------        0.0
       M3   |   M4
            |
            |             -1.0
  1.0      0.0       -1.0
  */
  
	float max_vx = 4.8f;
	float min_vx = -4.8f;
	float max_vy = 4.8f;
	float min_vy = -4.8f;
	float max_vw = 4.5f;
   float min_vw = -4.5f;
  
  	float max_vx_slow = 0.3f;
	float min_vx_slow = -0.3f;
	float max_vy_slow = 0.3f;
	float min_vy_slow = -0.3f;
	float max_vw_slow = 0.3f;
   float min_vw_slow = -0.3f;
	
};
#endif

#endif