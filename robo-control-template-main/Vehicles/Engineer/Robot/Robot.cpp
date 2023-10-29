/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   Robot.cpp
** 文件说明：   机器人基类
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     季献淏     	   2022-07-10
***************************************************************************/
#include "Robot.h"
#include "arm_math.h"
#include "Motor_RM_Tasks.h"
#include "Params.h"
#include "tim.h"
#include "UARTDriver.h"
/***********************************************************************
** 函 数 名： Robot::init()
** 函数说明： 机器人系统初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Robot::init()
{

  // 初始化各项参数
  params.initMotorsParams();
  //创建对象
  attitude_solution_task_p = new AttitudeSolutionTask(*this); // 姿态解算, 在外部中断中进行
  led_control_task_p = new LEDControlTask(*this); // LED控制
  referee_system_task_p = new RefereeSystemTask(*this); // 裁判系统
//  main_control_task_p = new MainControlTask(*this); // 主控制
  remote_control_task_p = new RemoteControlTask(*this);//遥控
	
  bmi088_id = inertial_sensors.addBackend(new InertialSensor_BMI088(inertial_sensors, ROTATION_YAW_180)); // BMI088 绕本体Z轴旋转180度

  rc_protocol.init();
  scheduler.init();
  can1_device.init(&hcan1);
  can2_device.init(&hcan2);

	//arm_control任务new
	 arm_task_p = new Arm_ControlTask(
		*this,
		&params.motor_params.arm_3508,
      &params.motor_params.arm_3508_ang,
      &params.motor_params.arm_3508_ang_vel,
		
		&params.motor_params.arm_dm_1,
		
		&params.motor_params.arm_dm_2,
		
		&params.motor_params.arm_mg_1
	
	);
	//chassis 任务new
  chassis_task_p = new ChassisControlTask(
    *this,

    &params.motor_params.chassis_motor_1,
    NULL,
    &params.motor_params.chassis_motor_1_ang_vel,

    &params.motor_params.chassis_motor_2,
    NULL,
    &params.motor_params.chassis_motor_2_ang_vel,

    &params.motor_params.chassis_motor_3,
    NULL,
    &params.motor_params.chassis_motor_3_ang_vel,

    &params.motor_params.chassis_motor_4,
    NULL,
    &params.motor_params.chassis_motor_4_ang_vel
  );
	 
		//gimbal_control任务new
	 gimbal_task_p = new GimbalControlTask(
		*this,
    &params.motor_params.overturn1_3508,
    &params.motor_params.overturn1_3508_ang,
    &params.motor_params.overturn1_3508_ang_vel,

    &params.motor_params.overturn2_3508,
    &params.motor_params.overturn2_3508_ang,
    &params.motor_params.overturn2_3508_ang_vel,
		
	 &params.motor_params.gimbal_6020,
	 &params.motor_params.gimbal_6020_ang,		
	 &params.motor_params.gimbal_6020_ang_vel
		
	);
	 
	 
	 can1_send_task_0x101_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x101,
    params.motor_params.control_tasks_interval.can1_send_0x101_task_interval
  );
	 
	 can1_send_task_0x102_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x102,
    params.motor_params.control_tasks_interval.can1_send_0x102_task_interval
  );
	 
	 can1_send_task_0x145_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x145,
    params.motor_params.control_tasks_interval.can1_send_0x145_task_interval
  );
	 can1_send_task_0x146_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x146,
    params.motor_params.control_tasks_interval.can1_send_0x146_task_interval
  );	 
	 
	  //can2_0x200发送初始化	 
	 can2_send_task_0x200_p = new CANSendTask(
    *this,
    &this->can2_device,
    0x200,
    params.motor_params.control_tasks_interval.can2_send_0x200_task_interval
  );
	 
	 //can1_0x200发送初始化	 
	 can1_send_task_0x200_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x200,
    params.motor_params.control_tasks_interval.can1_send_0x200_task_interval
  );
	 
	 
	  //can2 0x1FF发送初始化	 
	 can2_send_task_0x1ff_p = new CANSendTask(
    *this,
    &this->can2_device,
    0x1ff,
    params.motor_params.control_tasks_interval.can2_send_0x1ff_task_interval
  );
	 
	 //can1 0x1FF发送初始化	 
	 can1_send_task_0x1ff_p = new CANSendTask(
    *this,
    &this->can1_device,
    0x1ff,
    params.motor_params.control_tasks_interval.can1_send_0x1ff_task_interval
  );
	 
	 
	 /* 任务初始化 */
	 	/* 任务周期初始化 */
    arm_task_p->setInterval(
    params.motor_params.control_tasks_interval.arm_task_interval
  );//机械臂
	 
	  chassis_task_p->setInterval(
    params.motor_params.control_tasks_interval.chassis_task_interval
  );//底盘
	 
	  gimbal_task_p->setInterval(
    params.motor_params.control_tasks_interval.gimbal_task_interval
  );//翻矿,云台
	 
	  led_control_task_p->setInterval(
    params.motor_params.control_tasks_interval.led_task_interval
  );
	
	  referee_system_task_p->setInterval(
    params.motor_params.control_tasks_interval.referee_system_task_interval
  );
	
   	//注册任务
     arm_task_p->init();
	  chassis_task_p->init();
	  gimbal_task_p->init();
	  attitude_solution_task_p->inited = true;
     attitude_solution_task_p->init();	
     led_control_task_p->init();	 
	 
	 	   // 定时器外设初始化
	   HAL_TIM_Base_Start_IT(&htim2);
      HAL_TIM_Base_Start_IT(&htim5);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//舵机1
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//舵机2
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);//舵机3
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);//蜂鸣器
		HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);//IMU加热电阻
	   
	 
		/* 注册任务 */ 
		scheduler.registerTask(referee_system_task_p);
	 scheduler.registerTask(arm_task_p);
	 scheduler.registerTask(chassis_task_p);
	 scheduler.registerTask(gimbal_task_p);
//	 scheduler.registerTask(can1_send_task_0x32_p);
	 scheduler.registerTask(can1_send_task_0x101_p);
	 scheduler.registerTask(can1_send_task_0x102_p);
	 scheduler.registerTask(can1_send_task_0x145_p);
	 scheduler.registerTask(can1_send_task_0x146_p);
	 scheduler.registerTask(can2_send_task_0x200_p);
	 scheduler.registerTask(can1_send_task_0x200_p);
	 scheduler.registerTask(can2_send_task_0x1ff_p);
	 scheduler.registerTask(can1_send_task_0x1ff_p);
	 	/* 注册任务 */
}

/***********************************************************************
** 函 数 名： Robot::run()
** 函数说明： 机器人系统运行函数，此函数运行在主循环中
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Robot::run()
{
  scheduler.run();
}

void Robot::setParams()
{

}
