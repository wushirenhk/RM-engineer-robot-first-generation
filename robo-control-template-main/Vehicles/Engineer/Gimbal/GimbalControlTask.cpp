/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   GimbalControlTask.cpp
** 文件说明：   云台和翻矿控制任务
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     林子涵     	     2022-07-26
**  1.1                  增加了具体控制内容                赵钟源            2023-02-12
**  1.2                  工程车的具体使用                孔明骏            2023-03-28
***************************************************************************/
#include "/Gimbal/GimbalControlTask.h"
#include "/Robot/Robot.h"

/***********************************************************************
** 函 数 名： GimbalControlTask构造函数
** 函数说明： 指定robot引用，CAN设备引用，指定yaw和pitch轴电机CAN总线id
**            和CAN总线数据包发送起始下标，指定任务更新时间间隔，同时
**            给yaw和pitch轴电机注册Motor_RM_PIDControlTask任务
**---------------------------------------------------------------------
** 输入参数： robot引用、CAN设备引用、yaw和pitch轴电机CAN总线id、
**            CAN总线数据包发送起始下标、任务更新时间间隔
** 返回参数： 无
***********************************************************************/
GimbalControlTask::GimbalControlTask(
  Robot &robot0,
  Motor_RM_Params_t *Overturn1, PID_Params_t *Overturn1_ang, PID_Params_t *Overturn1_ang_vel,
  Motor_RM_Params_t *Overturn2, PID_Params_t *Overturn2_ang, PID_Params_t *Overturn2_ang_vel,
  Motor_RM_Params_t *gimbal_6020,PID_Params_t *gimbal_6020_ang, PID_Params_t *gimbal_6020_ang_vel,
  timeus_t interval_tick_us0
) : Task_Base(robot0)
{
  this->interval_tick_us = interval_tick_us0;		
	
	  if(Overturn1 != NULL)
  {
    if(Overturn1->canx == 1)
    {
      this->motor_Overturn1_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          Overturn1->can_rx_id,
          Overturn1->can_tx_id,
          Overturn1->can_tx_data_start_pos,
          RoboMaster_3508,
          Overturn1->interval);
    }
    else if(Overturn1->canx == 2)
    {
      this->motor_Overturn1_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          Overturn1->can_rx_id,
          Overturn1->can_tx_id,
          Overturn1->can_tx_data_start_pos,
          RoboMaster_3508,
          Overturn1->interval);
    }
    this->motor_Overturn1_pid_task->motor_backend_p->setParams(*Overturn1);
  }

  if(Overturn2 != NULL)
  {
    if(Overturn2->canx == 1)
    {
      this->motor_Overturn2_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          Overturn2->can_rx_id,
          Overturn2->can_tx_id,
          Overturn2->can_tx_data_start_pos,
          RoboMaster_3508,
          Overturn2->interval);
    }
    else if(Overturn2->canx == 2)
    {
      this->motor_Overturn2_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          Overturn2->can_rx_id,
          Overturn2->can_tx_id,
          Overturn2->can_tx_data_start_pos,
          RoboMaster_3508,
          Overturn2->interval);
    }
    this->motor_Overturn2_pid_task->motor_backend_p->setParams(*Overturn2);
  }
  
    if(gimbal_6020 != NULL)
  {
    if(gimbal_6020->canx == 1)
    {
      this->motor_gimbal_6020_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          gimbal_6020->can_rx_id,
          gimbal_6020->can_tx_id,
          gimbal_6020->can_tx_data_start_pos,
          RoboMaster_GM6020,
          gimbal_6020->interval);
    }
    else if(gimbal_6020->canx == 2)
    {
      this->motor_gimbal_6020_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          gimbal_6020->can_rx_id,
          gimbal_6020->can_tx_id,
          gimbal_6020->can_tx_data_start_pos,
          RoboMaster_GM6020,
          gimbal_6020->interval);
    }
    this->motor_gimbal_6020_pid_task->motor_backend_p->setParams(*gimbal_6020);
  }

  if(Overturn1_ang_vel != NULL)
  {
    this->motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*Overturn1_ang_vel);
    this->motor_Overturn1_pid_task->angular_velocity_control_task_p->setInterval((*Overturn1_ang_vel).interval);
  }

  if(Overturn1_ang != NULL)
  {
    this->motor_Overturn1_pid_task->angle_control_task_p->setPIDControllerParams(*Overturn1_ang);
    this->motor_Overturn1_pid_task->angle_control_task_p->setInterval((*Overturn1_ang).interval);
  }


  if(Overturn2_ang_vel != NULL)
  {
    this->motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*Overturn2_ang_vel);
    this->motor_Overturn2_pid_task->angular_velocity_control_task_p->setInterval((*Overturn2_ang_vel).interval);
  }

  if(Overturn2_ang != NULL)
  {
    this->motor_Overturn2_pid_task->angle_control_task_p->setPIDControllerParams(*Overturn2_ang);
    this->motor_Overturn2_pid_task->angle_control_task_p->setInterval((*Overturn2_ang).interval);
  }

  if(gimbal_6020_ang_vel != NULL)
  {
    this->motor_gimbal_6020_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*gimbal_6020_ang_vel);
    this->motor_gimbal_6020_pid_task->angular_velocity_control_task_p->setInterval((*gimbal_6020_ang_vel).interval);
  }

  if(gimbal_6020_ang != NULL)
  {
    this->motor_gimbal_6020_pid_task->angle_control_task_p->setPIDControllerParams(*gimbal_6020_ang);
    this->motor_gimbal_6020_pid_task->angle_control_task_p->setInterval((*gimbal_6020_ang).interval);
  }	
	
	
	
}




/***********************************************************************
** 函 数 名： GimbalControlTask::init()
** 函数说明： 注册云台各个电机Motor_RM_PIDControlTask任务
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void GimbalControlTask::init(void)
{
  inited = true;
  if(motor_Overturn1_pid_task != NULL)
    robot.scheduler.registerTask(motor_Overturn1_pid_task);
  if(motor_Overturn2_pid_task != NULL)
    robot.scheduler.registerTask(motor_Overturn2_pid_task);
  if(motor_gimbal_6020_pid_task != NULL)
    robot.scheduler.registerTask(motor_gimbal_6020_pid_task);

  if(motor_Overturn1_pid_task->angle_control_task_p != NULL)
    robot.scheduler.registerTask(motor_Overturn1_pid_task->angle_control_task_p );
  if(motor_Overturn1_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_Overturn1_pid_task->angular_velocity_control_task_p );

  if(motor_Overturn2_pid_task->angle_control_task_p != NULL)
    robot.scheduler.registerTask(motor_Overturn2_pid_task->angle_control_task_p );
  if(motor_Overturn2_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_Overturn2_pid_task->angular_velocity_control_task_p );

  if(motor_gimbal_6020_pid_task->angle_control_task_p != NULL)
    robot.scheduler.registerTask(motor_gimbal_6020_pid_task->angle_control_task_p );
  if(motor_gimbal_6020_pid_task->angular_velocity_control_task_p != NULL)
    robot.scheduler.registerTask(motor_gimbal_6020_pid_task->angular_velocity_control_task_p );
  
  overturn_mode = Overturn_OFF;
  gimbal_mode = GIMBAL_FREE;
}

/***********************************************************************
** 函 数 名： GimbalControlTask::uninit()
** 函数说明： 任务反初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void GimbalControlTask::uninit(void) {}
	
/***********************************************************************
** 函 数 名： GimbalControlTask::update(timeus_t dT_us)
** 函数说明： 通过云台电机编码器和IMU计算云台Yaw轴机构、车底盘
**            等部分的空间姿态，更新云台各个电机控制任务，并发送
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void GimbalControlTask::update(timeus_t dT_us)
{
	  float dT_s = dT_us / 1e6f;
   //   USART1_DMA_Debug_Printf("1=%-16f2=%-16f\r\n",motor_Overturn2_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity , motor_Overturn2_pid_task->angular_velocity_control_task_p->getOutput());
	/**************************翻矿控制************************************************/
   /*********************************************************************************/	
   if(overturn_mode == Overturn_CW)
	{

	 	 // 设置1电机角速度期望值（内环）
      motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn1_angular_velocity_max
	    );

		
	 	 // 设置2电机角速度期望值（内环）
      motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn2_angular_velocity_min
	    );

	}
	   if(overturn_mode == Overturn_CCW)
	{

	 	 // 设置1电机角速度期望值（内环）
      motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn1_angular_velocity_min
	    );

		
	 	 // 设置2电机角速度期望值（内环）
      motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn2_angular_velocity_max
	    );

	}
	
	   if(overturn_mode == Overturn_OUT)
	{

	 	 // 设置1电机角速度期望值（内环）
      motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn1_angular_velocity_min
	    );
	
	 	 // 设置2电机角速度期望值（内环）
      motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn2_angular_velocity_min
	    );
		
	}
		   
	   if(overturn_mode == Overturn_IN)
	{

	 	 // 设置1电机角速度期望值（内环）
      motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn1_angular_velocity_max
	    );
	
	 	 // 设置2电机角速度期望值（内环）
      motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   overturn2_angular_velocity_max
	    );
		
	}	

	if(overturn_mode == Overturn_OFF)
	{
		 // 1电机输出值
      motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	     0
	    );
	
	    // 2电机输出值
	   motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
        0
	    );
	}
	
	 // 获取电机角速度反馈值
    motor_Overturn1_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_Overturn1_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
	 motor_Overturn2_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_Overturn2_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
			
	    // 1电机输出值
	    motor_Overturn1_pid_task->motor_backend_p->setMotorInput(
       motor_Overturn1_pid_task->angular_velocity_control_task_p->getOutput()
	    );  
	
	    // 2电机输出值
	    motor_Overturn2_pid_task->motor_backend_p->setMotorInput(
       motor_Overturn2_pid_task->angular_velocity_control_task_p->getOutput()
	    ); 

 
	

	
	
	/********************************6020控制******************************************/
   /*********************************************************************************/	
	
	/* gimbal_6020电机控制 */
         /*↓*/	
switch (gimbal_mode)
{	
	case(0):
    gimbal_yaw_angle += gimbal_yaw_vel * dT_s;
	break;
	
	case(1):
	 gimbal_yaw_angle = motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + angle_glod_yaw;
	 gimbal_pitch_angle = angle_glod_pitch;
	
}
    // 获取电机角度返回值
    motor_gimbal_6020_pid_task->angle_control_task_p->setPIDControllerFeedback(motor_gimbal_6020_pid_task->motor_backend_p->getCommonMeasurement().output.angle);	
    
	 // 获取电机角速度反馈值
    motor_gimbal_6020_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_gimbal_6020_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);	
	
	
		// 设置电机角度期望值(外环)
		motor_gimbal_6020_pid_task->angle_control_task_p->setPIDControllerExpect(
			gimbal_yaw_angle
			);
			
	   motor_gimbal_6020_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   motor_gimbal_6020_pid_task->angle_control_task_p->getOutput()
		);
		
		// gimbal电机输出值
	   motor_gimbal_6020_pid_task->motor_backend_p->setMotorInput(
      motor_gimbal_6020_pid_task->angular_velocity_control_task_p->getOutput()
	    ); 
	
			/* 舵机控制 */
	
	  robot.helper.setServo(3, gimbal_pitch_angle);
	
}







