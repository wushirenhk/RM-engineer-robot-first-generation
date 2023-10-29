/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   Engineer
** 文 件 名：   ArmControlTask.cpp
** 文件说明：   脉塔电机控制任务
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						       孔明骏     	        2023-02-11

***************************************************************************/
#include "ArmControlTask.h"
#include "../Robot/Robot.h"
#include "Helper.h"
#include "../Robot/Params.h"
#include "/Gimbal/GimbalControlTask.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;


int k=75;//mg电机整体角度调节
double s2=0;//dm2电机整体角度调节

/***********************************************************************
** 函 数 名： Arm_ControlTask构造函数
** 函数说明： 指定robot引用，CAN设备引用，指定yaw和pitch轴电机CAN总线id
**            和CAN总线数据包发送起始下标，指定任务更新时间间隔，
**---------------------------------------------------------------------
** 输入参数： robot引用、CAN设备引用、射击摩擦轮电机CAN总线id、
**            CAN总线数据包发送起始下标、任务更新时间间隔
** 返回参数： 无
***********************************************************************/

Arm_ControlTask::Arm_ControlTask(
    Robot &robot0,
    Motor_RM_Params_t *arm_3508,PID_Params_t *arm_3508_ang, PID_Params_t *arm_3508_ang_vel,
    Motor_DM_Params_t *arm_dm_1,
    Motor_DM_Params_t *arm_dm_2,
	 Motor_MG_Params_t *arm_mg_1,
	 timeus_t interval_tick_us0
	 ):Task_Base(robot0)
{
		this->interval_tick_us = interval_tick_us0;
	
	/********************new控制任务********************/
	/*****************************↓***************************/
  //RM电机//
	  if(arm_3508 != NULL)
	  {
    if(arm_3508->canx == 1)
    {
          this->motor_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can1_device,
          arm_3508->can_rx_id,
          arm_3508->can_tx_id,
          arm_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          arm_3508->interval);
    }
	 else  if(arm_3508->canx == 2)
    {
          this->motor_3508_pid_task = new Motor_RM_PIDControlTask(robot0,
          robot0.can2_device,
          arm_3508->can_rx_id,
          arm_3508->can_tx_id,
          arm_3508->can_tx_data_start_pos,
          RoboMaster_3508,
          arm_3508->interval);
    }
      this->motor_3508_pid_task->motor_backend_p->setParams(*arm_3508);
  }

	
	//DM电机//
  //一号电机
	    if(arm_dm_1 != NULL)
  {
    if(arm_dm_1->canx == 1)
    {
          this->motor_dm1_task = new Motor_DM_PSControlTask(robot0,
          robot0.can1_device,
          arm_dm_1->can_rx_id,
          arm_dm_1->can_tx_id,
          arm_dm_1->can_tx_data_start_pos,
          DM_J4310,
          arm_dm_1->interval);
    }
	 else  if(arm_dm_1->canx == 2)
    {
          this->motor_dm1_task = new Motor_DM_PSControlTask(robot0,
          robot0.can2_device,
          arm_dm_1->can_rx_id,
          arm_dm_1->can_tx_id,
          arm_dm_1->can_tx_data_start_pos,
          DM_J4310,
          arm_dm_1->interval);
    }
	  this->motor_dm1_task->motor_backend_p->setParams(*arm_dm_1);
  }
  //二号电机
  	    if(arm_dm_2 != NULL)
  {
    if(arm_dm_2->canx == 1)
    {
          this->motor_dm2_task = new Motor_DM_PSControlTask(robot0,
          robot0.can1_device,
          arm_dm_2->can_rx_id,
          arm_dm_2->can_tx_id,
          arm_dm_2->can_tx_data_start_pos,
          DM_J4310,
          arm_dm_2->interval);
    }
	 else  if(arm_dm_2->canx == 2)
    {
          this->motor_dm2_task = new Motor_DM_PSControlTask(robot0,
          robot0.can2_device,
          arm_dm_2->can_rx_id,
          arm_dm_2->can_tx_id,
          arm_dm_2->can_tx_data_start_pos,
          DM_J4310,
          arm_dm_2->interval);
    }
	  this->motor_dm2_task->motor_backend_p->setParams(*arm_dm_2);
  }
  
		
  //MG电机
  //一号电机
   	if(arm_mg_1 != NULL)
  {
    if(arm_mg_1->canx == 1)
    {
          this->motor_mg1_task = new Motor_MG_ControlTask(robot0,
          robot0.can1_device,
          arm_mg_1->can_rx_id,
          arm_mg_1->can_tx_id,
          arm_mg_1->can_tx_data_start_pos,
          MG8016,
          arm_mg_1->interval);
    }
	 else  if(arm_mg_1->canx == 2)
    {
          this->motor_mg1_task = new Motor_MG_ControlTask(robot0,
          robot0.can2_device,
          arm_mg_1->can_rx_id,
          arm_mg_1->can_tx_id,
          arm_mg_1->can_tx_data_start_pos,
          MG8016,
          arm_mg_1->interval);
    }
	  this->motor_mg1_task->motor_backend_p->setParams(*arm_mg_1);
  }
  
  /*****************************↑*************************/
  	/********************new控制任务********************/
	
    if(arm_3508_ang_vel != NULL)
  {
    this->motor_3508_pid_task->angular_velocity_control_task_p->setPIDControllerParams(*arm_3508_ang_vel);
    this->motor_3508_pid_task->angular_velocity_control_task_p->setInterval((*arm_3508_ang_vel).interval);
  }

  if(arm_3508_ang != NULL)
  {
    this->motor_3508_pid_task->angle_control_task_p->setPIDControllerParams(*arm_3508_ang);
    this->motor_3508_pid_task->angle_control_task_p->setInterval((*arm_3508_ang).interval);
  }  
  
  
  /*****************************↑*************************/
  	/********************new控制任务********************/
    	//middle电机的角速度和角加速度值，以及运行周期

}



/***********************************************************************
** 函 数 名： Arm_ControlTask::init()
** 函数说明： 注册各个摩擦轮发射电机Motor_RM_PIDControlTask任务
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Arm_ControlTask::init(void)
{
	   inited = true;
		
	
	  //电机的数据获取任务注册
		if(motor_3508_pid_task != NULL)
     robot.scheduler.registerTask(motor_3508_pid_task);
	   if(motor_dm1_task != NULL)
     robot.scheduler.registerTask(motor_dm1_task);
	  	if(motor_dm2_task != NULL)
     robot.scheduler.registerTask(motor_dm2_task);
	  	if(motor_mg1_task != NULL)
     robot.scheduler.registerTask(motor_mg1_task);			
		
	   /* PID任务注册 */
         /*↓*/

      if(motor_3508_pid_task->getAngleTaskPointer() != NULL)
     robot.scheduler.registerTask(motor_3508_pid_task->getAngleTaskPointer());
      if(motor_3508_pid_task->getAngularVelocityTaskPointer() != NULL)
     robot.scheduler.registerTask(motor_3508_pid_task->getAngularVelocityTaskPointer());
		
	       /*↑*/
 	   /* PID任务注册 */		
		  
		  
		arm_mode = Arm_OFF;
		arm_3508_angle = -0.1;  //弧度		
		
		motor_mg1_task->motor_backend_p->params.max_speed = 1;
			
		motor_dm1_task->motor_backend_p->params.position_rad = 2.06;      //弧度
		motor_dm2_task->motor_backend_p->params.position_rad =s2-0.36; 
	   motor_dm1_task->motor_backend_p->params.speed_rad = 0.5;      //弧度/sz
		motor_dm2_task->motor_backend_p->params.speed_rad = 0.5;	

      exp_angle = 0;
      arm3508_angular_velocity_max = 5;
      arm3508_angular_velocity_min = -5;		
}

/***********************************************************************
** 函 数 名： ArmRM_ControlTask::uninit()
** 函数说明： 任务反初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Arm_ControlTask::uninit(void) 
{

}

/***********************************************************************
** 函 数 名： Arm_ControlTask::arm_time_delay(uint16_t time)
** 函数说明： arm任务延时（无中断式），单位ms
**---------------------------------------------------------------------
** 输入参数： time，延时时间，单位ms
** 返回参数： 无
***********************************************************************/
int Arm_ControlTask::arm_time_delay(uint16_t time) 
{
	timecnt++;
	
	if(timecnt == time * 4 / 10)
	{
		timecnt = 0;
	   return 1;
	}
	else{return 0;}
}

/***********************************************************************
** 函 数 名： Arm_ControlTask::arm_mode_switch()
** 函数说明： 机械臂模式切换
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Arm_ControlTask::arm_mode_switch()
{
  /*判断此时状态以使用不同控制方式*/
           /*↓*/
		 switch (arm_mode){
		 case Arm_OFF:	 
			 
		 motor_mg1_task->motor_backend_p->params.angle_control =  motor_mg1_task->motor_backend_p->params.multiturn_angle;
//		 motor_mg1_task->motor_backend_p->params.angle_control = -86*600;
		 motor_mg1_task->motor_backend_p->params.max_speed = 1;		 
		 
		 motor_dm1_task->motor_backend_p->params.speed_rad = 0;
		 motor_dm2_task->motor_backend_p->params.speed_rad = 0; 
		 
		 
		 
		 break;
		 /****************************normal状态开始转换************************************/
		 case Arm_normal:	

		    switch (last_arm_mode)
			 {
				 case Arm_OFF:
					 if(normal == state0)
					 {	 
					 arm3508_state_normal_angle = -0.1;  //弧度
					 mg1_state_normal_angle = 0+k;      //度（精度可调）
				    mg1_state_normal_speed = 200;   //度/s,要除以6
				    dm1_state_normal_angle = 2.06;      //弧度
				    dm2_state_normal_angle = 1.9+s2; 
				    dm1_state_normal_speed = 2;      //弧度/sz
				    dm2_state_normal_speed = 2;
				    robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;
                robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(500) == 1)
					 {
					 normal = state1;
					 switch_lock = 0;
					 }
					 }
				    
				 break;
					 
				 case Arm_normal:
					 
				 break;
				 
				 case Arm_AirConnection:
					 if(normal == state0)
					 {	 
					 arm3508_state_normal_angle = -8.5;  
					 mg1_state_normal_angle = 120+k;      
				    mg1_state_normal_speed = 400;  
				    dm1_state_normal_angle = 2.06;      
				    dm2_state_normal_angle = 2.213+s2; 
				    dm1_state_normal_speed = 2;      
				    dm2_state_normal_speed = 1;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
                robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_glod_yaw;
					 if(arm_time_delay(1000) == 1)
					 {
					 normal = state1;
					 }
					 }
					 if(normal == state1)
					 {	 
					 arm3508_state_normal_angle = -0.1;  
					 mg1_state_normal_angle = k-12;     
				    mg1_state_normal_speed = 400;   
				    dm1_state_normal_angle = 2.06;      
				    dm2_state_normal_angle = 2.145+s2; 
				    dm1_state_normal_speed = 2;      
				    dm2_state_normal_speed = 1;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;						 
                robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(500) == 1)
					 {
					  normal = state2;
					  switch_lock = 0;
					 }
					 }	
				 break;
				 
				 case Arm_SilveryOre:
					 if(normal == state0)
					 {	 
					 arm3508_state_normal_angle = -8.5;  
					 mg1_state_normal_angle = k+98;
				    mg1_state_normal_speed = 400;
				    dm1_state_normal_angle = 2.05;
    			    dm2_state_normal_angle = motor_dm2_task->motor_backend_p->params.position_rad;
				    dm1_state_normal_speed = 2;
				    dm2_state_normal_speed = 2;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
                robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_glod_yaw;
					 if(arm_time_delay(1000) == 1)
					 {
					  normal = state1;
					 }
					 }	
					 
					 if(normal == state1)
					 {	 
					 arm3508_state_normal_angle = -0.1;  
					 mg1_state_normal_angle = k-12;     
				    mg1_state_normal_speed = 400;   
				    dm1_state_normal_angle = 2.06;      
				    dm2_state_normal_angle = 2.25+s2; 
				    dm1_state_normal_speed = 2;      
				    dm2_state_normal_speed = 2;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;
                robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					  normal = state2;
					  switch_lock = 0;
					 }
					 }					 
				 break;
				 
				 case Arm_GroundOre:
					 if(normal == state0)
					 {	 
					 arm3508_state_normal_angle = -7;  
					 mg1_state_normal_angle = k+139;     
				    mg1_state_normal_speed = 600;   
				    dm1_state_normal_angle = 2.06;      
				    dm2_state_normal_angle = 2.713+s2; 
				    dm1_state_normal_speed = 2;      
				    dm2_state_normal_speed = 2;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
                robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_glod_yaw;
					 if(arm_time_delay(500) == 1)
					 {
					  normal = state1;
					 } 
					 }	
					 if(normal == state1)
					 {	 
					 arm3508_state_normal_angle = -0.1;  
					 mg1_state_normal_angle = k-12;     
				    mg1_state_normal_speed = 600;   
				    dm1_state_normal_angle = 2.06;      
				    dm2_state_normal_angle = 2.145+s2; 
				    dm1_state_normal_speed = 2;      
				    dm2_state_normal_speed = 2;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;
                robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 robot.gimbal_task_p->gimbal_pitch_angle = robot.gimbal_task_p->gimbal_pitch_angle;						 
					 if(arm_time_delay(1000) == 1)
					 {
					  normal = state2;
					  switch_lock = 0;
					 }
					 }						 
				 break;
				 
				 case Arm_OrePlacement:
					 if(normal == state0)
					 {	 
					 arm3508_state_normal_angle = 0.1;  //弧度
					 mg1_state_normal_angle = k+31;      //度（精度可调）
				    mg1_state_normal_speed = 400;   //度/s,要除以6
				    dm1_state_normal_angle = 2.06;      //弧度
				    dm2_state_normal_angle = 2.263+s2; 
				    dm1_state_normal_speed = 2;      //弧度/sz
				    dm2_state_normal_speed = 2;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
                robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(500) == 1)
					 {
					  normal = state1;
					 }
					 }			
					 if(normal == state1)
					 {	 
					 arm3508_state_normal_angle = -0.1;  
					 mg1_state_normal_angle = k-7;      
				    mg1_state_normal_speed = 300;   
				    dm1_state_normal_angle = 2.02;     
				    dm2_state_normal_angle = 2.263+s2; 
				    dm1_state_normal_speed = 2;     
				    dm2_state_normal_speed = 2;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;	
                robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					  normal = state2;
					  switch_lock = 0;
					 }
					 }						 
				 break;
				 
				 case Arm_HoldOre:
					 if(normal == state0)
					 {	 
					 arm3508_state_normal_angle = -0.1;  
					 mg1_state_normal_angle =k+18;      
				    mg1_state_normal_speed = 300;  
				    dm1_state_normal_angle = 2.02;      
				    dm2_state_normal_angle = 2.263+s2; 
				    dm1_state_normal_speed = 2;      
				    dm2_state_normal_speed = 2;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;
					 robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_yaw_Orebin;
					 if(arm_time_delay(1000) == 1)
					 {
					 normal = state1;
					  switch_lock = 0;
					 }
					 }					 
			 }
		 if(switch_lock != 0)
		 {
		 robot.gimbal_task_p->gimbal_mode = GIMBAL_FREE;
	    arm_3508_angle = arm3508_state_normal_angle;
		 motor_dm1_task->motor_backend_p->params.position_rad = dm1_state_normal_angle;
		 motor_dm2_task->motor_backend_p->params.position_rad = dm2_state_normal_angle;
		 motor_dm1_task->motor_backend_p->params.speed_rad = dm1_state_normal_speed;
		 motor_dm2_task->motor_backend_p->params.speed_rad = dm2_state_normal_speed;
		 	 
		 motor_mg1_task->motor_backend_p->params.angle_control = mg1_state_normal_angle * 600;			 
		 motor_mg1_task->motor_backend_p->params.max_speed = mg1_state_normal_speed;		 
		 }
		 
		 break;
			 /****************************AirConnection状态开始转换************************************/
	 	 case Arm_AirConnection:	
		    switch (last_arm_mode)
			 {
				 
				 case Arm_OFF:
				    
				 break;
					 
				 case Arm_normal:
					 
				 if(AirConnection == state0)
					 {	 
					 arm3508_state_AirConnection_angle = -8.8;  
					 mg1_state_AirConnection_angle = k+95;      
				    mg1_state_AirConnection_speed = 600;  
				    dm1_state_AirConnection_angle = 2.06;      
				    dm2_state_AirConnection_angle = 0.843+s2; 
				    dm1_state_AirConnection_speed = 2;      
				    dm2_state_AirConnection_speed = 1;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
					 if(arm_time_delay(400) == 1)
					 {
					  AirConnection = state1;
					 }
					 }
				 if(AirConnection == state1)
					 {	 
					 arm3508_state_AirConnection_angle = -8.8;  
					 mg1_state_AirConnection_angle = k+95;      
				    mg1_state_AirConnection_speed = 400;  
				    dm1_state_AirConnection_angle = 2.06;      
				    dm2_state_AirConnection_angle = 0.843+s2; 
				    dm1_state_AirConnection_speed = 2;      
				    dm2_state_AirConnection_speed = 1;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
					 if(arm_time_delay(1000) == 1)
					 {
					  AirConnection = state2;
					  switch_lock = 0;
					 }
					 }						 
				 break;
				 
				 case Arm_AirConnection:

				 break;
				 
				 case Arm_SilveryOre:
		 
				 break;
				 
				 case Arm_GroundOre:
	 
				 break;
				 
				 case Arm_OrePlacement:
		 
				 break;
				 
				 case Arm_HoldOre:
				 break;
				 		 
			 }
		 if(switch_lock != 0)
		 {			 
	    arm_3508_angle = arm3508_state_AirConnection_angle;
		 motor_dm1_task->motor_backend_p->params.position_rad = dm1_state_AirConnection_angle;
		 motor_dm2_task->motor_backend_p->params.position_rad = dm2_state_AirConnection_angle;
		 motor_dm1_task->motor_backend_p->params.speed_rad = dm1_state_AirConnection_speed;
		 motor_dm2_task->motor_backend_p->params.speed_rad = dm2_state_AirConnection_speed;
		 	 
		 motor_mg1_task->motor_backend_p->params.angle_control = mg1_state_AirConnection_angle * 600;		 
		 motor_mg1_task->motor_backend_p->params.max_speed = mg1_state_AirConnection_speed;
			 
		 robot.gimbal_task_p->gimbal_mode = GIMBAL_LOCK; 
		 }
		 
		 break;
			 /****************************Arm_SilveryOre状态开始转换************************************/
		 case Arm_SilveryOre:	
		    switch (last_arm_mode)
			 {

				 case Arm_OFF:

				 break;
					 
				 case Arm_normal:
					 
					 if(SilveryOre == state0)
					 {	 
					 arm3508_state_SilveryOre_angle = -1;
					 mg1_state_SilveryOre_angle = k+73;
				    mg1_state_SilveryOre_speed = 500;
				    dm1_state_SilveryOre_angle = 2.05;
				    dm2_state_SilveryOre_angle = 0.483+s2;
				    dm1_state_SilveryOre_speed = 2;
				    dm2_state_SilveryOre_speed = 2;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
					 if(arm_time_delay(1000) == 1)
					 {
					 SilveryOre = state1;
					 switch_lock = 0;
					 }
					 }		 
				 break;
				 
				 case Arm_AirConnection:

				 break;
				 
				 case Arm_SilveryOre:		 
				 break;
				 
				 case Arm_GroundOre:

				 break;
				 
				 case Arm_OrePlacement:
		 
				 break;
				 
				 case Arm_HoldOre:

             break;				 
			 }		
		 if(switch_lock != 0)
		 {		 
	    arm_3508_angle = arm3508_state_SilveryOre_angle;
		 motor_dm1_task->motor_backend_p->params.position_rad = dm1_state_SilveryOre_angle;
		 motor_dm2_task->motor_backend_p->params.position_rad = dm2_state_SilveryOre_angle;
		 motor_dm1_task->motor_backend_p->params.speed_rad = dm1_state_SilveryOre_speed;
		 motor_dm2_task->motor_backend_p->params.speed_rad = dm2_state_SilveryOre_speed;
		 	 
		 motor_mg1_task->motor_backend_p->params.angle_control = mg1_state_SilveryOre_angle * 600;		 
		 motor_mg1_task->motor_backend_p->params.max_speed = mg1_state_SilveryOre_speed;
		 robot.gimbal_task_p->gimbal_mode = GIMBAL_LOCK;
		 }
		 break;
			 
			/****************************Arm_GroundOre状态开始转换************************************/
		 case Arm_GroundOre:	 
		    switch (last_arm_mode)
			 {

				 case Arm_OFF:
				    
				 break;
					 
				 case Arm_normal:
					 if(GroundOre == state0)
					 {	 
					 arm3508_state_GroundOre_angle = -4;
					 mg1_state_GroundOre_angle = k+104;
				    mg1_state_GroundOre_speed = 400;
				    dm1_state_GroundOre_angle = 2.05;
				    dm2_state_GroundOre_angle = -0.377+s2;
				    dm1_state_GroundOre_speed = 2;
				    dm2_state_GroundOre_speed = 2;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
                robot.gimbal_task_p->gimbal_yaw_angle = robot.gimbal_task_p->motor_gimbal_6020_pid_task->motor_backend_p->msr.round_cnt * 6.28f + robot.gimbal_task_p->angle_glod_yaw;
					 if(arm_time_delay(1000) == 1)
					 {
					 GroundOre = state1;
					 switch_lock = 0;
					 }
					 }					 
				 break;
				 
				 case Arm_AirConnection:

				 break;
				 
				 case Arm_SilveryOre:
			 
				 break;
				 
				 case Arm_GroundOre:
	 
				 break;
				 
				 case Arm_OrePlacement:
	 
				 break;
				 
				 case Arm_HoldOre:

             break;				 
			 }
		 if(switch_lock != 0)
		 {			 
	    arm_3508_angle = arm3508_state_GroundOre_angle;
		 motor_dm1_task->motor_backend_p->params.position_rad = dm1_state_GroundOre_angle;
		 motor_dm2_task->motor_backend_p->params.position_rad = dm2_state_GroundOre_angle;
		 motor_dm1_task->motor_backend_p->params.speed_rad = dm1_state_GroundOre_speed;
		 motor_dm2_task->motor_backend_p->params.speed_rad = dm2_state_GroundOre_speed;
		 	 
		 motor_mg1_task->motor_backend_p->params.angle_control = mg1_state_GroundOre_angle * 600;	 
		 motor_mg1_task->motor_backend_p->params.max_speed = mg1_state_GroundOre_speed;
		 robot.gimbal_task_p->gimbal_mode = GIMBAL_FREE;
		 }
		 break;
			 
		/****************************Arm_OrePlacement状态开始转换************************************/
		 case Arm_OrePlacement:	 
			 switch (last_arm_mode)
			 {

				 case Arm_OFF:

				    
				 break;
					 
				 case Arm_normal:
					 if(OrePlacement == state0)
					 {	 
					 arm3508_state_OrePlacement_angle = -5;  //弧度
					 mg1_state_OrePlacement_angle = k-14;      //度（精度可调）
				    mg1_state_OrePlacement_speed = 400;   //度/s,要除以6
				    dm1_state_OrePlacement_angle = 2.06;      //弧度
				    dm2_state_OrePlacement_angle = 0.988+s2; 
				    dm1_state_OrePlacement_speed = 2;      //弧度/sz
				    dm2_state_OrePlacement_speed = 1;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
					 if(arm_time_delay(500) == 1)
					 {
					 OrePlacement = state1;}
					 }
					 if(OrePlacement == state1)
					 {
					 arm3508_state_OrePlacement_angle = -5;  //弧度
					 mg1_state_OrePlacement_angle = k+16;      //度（精度可调）
				    mg1_state_OrePlacement_speed = 300;   //度/s,要除以6
				    dm1_state_OrePlacement_angle = 2.06;      //弧度
				    dm2_state_OrePlacement_angle = s2-0.307; 
				    dm1_state_OrePlacement_speed = 2;      //弧度/sz
				    dm2_state_OrePlacement_speed = 0.5;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
				    if(arm_time_delay(1000) == 1)
					 {
					  OrePlacement = state2;
					  switch_lock = 0;
					 }
					 }			 
				 break;
				 
				 case Arm_AirConnection:

				 break;
				 
				 case Arm_SilveryOre:
		 
				 break;
				 
				 case Arm_GroundOre:

				 break;
				 
				 case Arm_OrePlacement:
	 
				 break;
				 
				 case Arm_HoldOre:
					 if(OrePlacement == state0)
					 {	 
					 arm3508_state_OrePlacement_angle = -8.5;  //弧度
					 mg1_state_OrePlacement_angle = k+21;      //度（精度可调）
				    mg1_state_OrePlacement_speed = 500;   //度/s,要除以6
				    dm1_state_OrePlacement_angle = 2.06;      //弧度
				    dm2_state_OrePlacement_angle = s2-0.307; 
				    dm1_state_OrePlacement_speed = 2;      //弧度/sz
				    dm2_state_OrePlacement_speed = 1;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
					 if(arm_time_delay(500) == 1)
					 {
					 OrePlacement = state1;
					 switch_lock = 0;
					 }
					 }		
            break;					 
			 }	
		 if(switch_lock != 0)
		 {			 
	    arm_3508_angle = arm3508_state_OrePlacement_angle;
		 motor_dm1_task->motor_backend_p->params.position_rad = dm1_state_OrePlacement_angle;
		 motor_dm2_task->motor_backend_p->params.position_rad = dm2_state_OrePlacement_angle;
		 motor_dm1_task->motor_backend_p->params.speed_rad = dm1_state_OrePlacement_speed;
		 motor_dm2_task->motor_backend_p->params.speed_rad = dm2_state_OrePlacement_speed;
		 	 
		 motor_mg1_task->motor_backend_p->params.angle_control = mg1_state_OrePlacement_angle * 600;		 
		 motor_mg1_task->motor_backend_p->params.max_speed = mg1_state_OrePlacement_speed;
		 robot.gimbal_task_p->gimbal_mode = GIMBAL_LOCK;
		 }
		 
		 break;
			 			 
		/****************************Arm_HoldOre状态开始转换************************************/
		 case Arm_HoldOre:	 
			 switch (last_arm_mode)
			 {

				 case Arm_OFF:
			    
				 break;
					 
				 case Arm_normal:
					 if(HoldOre == state0)
					 {	 
					 arm3508_state_HoldOre_angle = -0.5;  //弧度
					 mg1_state_HoldOre_angle = k+24;      //度（精度可调）
				    mg1_state_HoldOre_speed = 400;   //度/s,要除以6
				    dm1_state_HoldOre_angle = 2.06;      //弧度
				    dm2_state_HoldOre_angle = 1.713+s2; 
				    dm1_state_HoldOre_speed = 3;      //弧度/sz
				    dm2_state_HoldOre_speed = 1.5;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
					 if(arm_time_delay(500) == 1)
					 {
					 HoldOre = state1;
					 }
					 }
					 if(HoldOre == state1)
					 {	 
					 arm3508_state_HoldOre_angle = -0.5;  //弧度
					 mg1_state_HoldOre_angle = k+24;      //度（精度可调）
				    mg1_state_HoldOre_speed = 400;   //度/s,要除以6
				    dm1_state_HoldOre_angle = 2.06;      //弧度
				    dm2_state_HoldOre_angle = 2.399+s2; 
				    dm1_state_HoldOre_speed = 3;      //弧度/sz
				    dm2_state_HoldOre_speed = 1.5;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;
					 if(arm_time_delay(500) == 1)
					 {
					 HoldOre = state2;
					 switch_lock = 0;
					 }
					 } 
					 
				 break;
				 
				 case Arm_AirConnection:
					 if(HoldOre == state0)
					 {	 
					 arm3508_state_HoldOre_angle = -0.5;  //弧度
					 mg1_state_HoldOre_angle = k+24;      //度（精度可调）
				    mg1_state_HoldOre_speed = 300;   //度/s,要除以6
				    dm1_state_HoldOre_angle = 2.06;      //弧度
				    dm2_state_HoldOre_angle = 2.399+s2; 
				    dm1_state_HoldOre_speed = 3;      //弧度/sz
				    dm2_state_HoldOre_speed = 1.25;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
					 if(arm_time_delay(1000) == 1)
					 {
					 HoldOre = state1;
					 switch_lock = 0;
					 }
					 } 
				 break;
				 
				 case Arm_SilveryOre:
					 if(HoldOre == state0)
					 {	 					 
					 arm3508_state_HoldOre_angle = -8;  
					 mg1_state_HoldOre_angle = k+61;
				    mg1_state_HoldOre_speed = 400;
				    dm1_state_HoldOre_angle = 2.05;
				    dm2_state_HoldOre_angle = motor_dm2_task->motor_backend_p->params.position_rad;
				    dm1_state_HoldOre_speed = 2;
				    dm2_state_HoldOre_speed = 1;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;
					 if(arm_time_delay(1000) == 1)
					 {
					 HoldOre = state1;
					 }
					 }								 
					 if(HoldOre == state1)
					 {	 
					 arm3508_state_HoldOre_angle = -0.5;  //弧度
					 mg1_state_HoldOre_angle = k+24;      //度（精度可调）
				    mg1_state_HoldOre_speed = 300;   //度/s,要除以6
				    dm1_state_HoldOre_angle = 2.06;      //弧度
				    dm2_state_HoldOre_angle = motor_dm2_task->motor_backend_p->params.position_rad; 
				    dm1_state_HoldOre_speed = 2;      //弧度/sz
				    dm2_state_HoldOre_speed = 1;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;
					 if(arm_time_delay(1000) == 1)
					 {
					 HoldOre = state2;
					 switch_lock = 0;
					 }
					 }
					 
				 break;
				 
				 case Arm_GroundOre:
					 if(HoldOre == state0)
					 {	 
					 arm3508_state_HoldOre_angle = -0.5;  //弧度
					 mg1_state_HoldOre_angle = k+24;      //度（精度可调）
				    mg1_state_HoldOre_speed = 300;   //度/s,要除以6
				    dm1_state_HoldOre_angle = 2.06;      //弧度
				    dm2_state_HoldOre_angle = 2.399+s2; 
				    dm1_state_HoldOre_speed = 2;      //弧度/sz
				    dm2_state_HoldOre_speed = 1.25;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;
					 if(arm_time_delay(1000) == 1)
					 {
					 HoldOre = state1;
					 switch_lock = 0;
					 }
					 } 
				 break;
				 
				 case Arm_OrePlacement:
					 if(HoldOre == state0)
					 {	 
					 arm3508_state_HoldOre_angle = -0.5;  //弧度
					 mg1_state_HoldOre_angle = k+24;      //度（精度可调）
				    mg1_state_HoldOre_speed = 300;   //度/s,要除以6
				    dm1_state_HoldOre_angle = 2.06;      //弧度
				    dm2_state_HoldOre_angle = 2.399+s2; 
				    dm1_state_HoldOre_speed = 2;      //弧度/sz
				    dm2_state_HoldOre_speed = 1.5;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_NORMAL;
					 if(arm_time_delay(1000) == 1)
					 {
					 HoldOre = state1;
					 switch_lock = 0;
					 }
					 } 	 
				 break;
				 
				 case Arm_HoldOre:

             break;					 
			 }		
		 if(switch_lock != 0)
		 {			 
	    arm_3508_angle = arm3508_state_HoldOre_angle;
		 motor_dm1_task->motor_backend_p->params.position_rad = dm1_state_HoldOre_angle;
		 motor_dm2_task->motor_backend_p->params.position_rad = dm2_state_HoldOre_angle;
		 motor_dm1_task->motor_backend_p->params.speed_rad = dm1_state_HoldOre_speed;
		 motor_dm2_task->motor_backend_p->params.speed_rad = dm2_state_HoldOre_speed;
		 	 
		 motor_mg1_task->motor_backend_p->params.angle_control = mg1_state_HoldOre_angle * 600;	 
		 motor_mg1_task->motor_backend_p->params.max_speed = mg1_state_HoldOre_speed;
		 robot.gimbal_task_p->gimbal_mode = GIMBAL_FREE;
		 }
		 
		 break;		 
}
	//输出角度与速度限幅

    if(arm_3508_angle >= -0.5) {arm_3508_angle = -0.5;}
    if(arm_3508_angle <= -8.95) {arm_3508_angle = -8.95;}	 
	 
    if(motor_dm1_task->motor_backend_p->params.position_rad >= 3.1f) { motor_dm1_task->motor_backend_p->params.position_rad = 3.1f;}
    if(motor_dm1_task->motor_backend_p->params.position_rad <= 0.5f) { motor_dm1_task->motor_backend_p->params.position_rad = 0.5f;}
    if(motor_dm1_task->motor_backend_p->params.speed_rad >= 5.0f)     { motor_dm1_task->motor_backend_p->params.speed_rad = 5.0f;}
    if(motor_dm1_task->motor_backend_p->params.speed_rad <= 0)    { motor_dm1_task->motor_backend_p->params.speed_rad = 0;}

	 if(motor_dm2_task->motor_backend_p->params.position_rad <= s2-0.75f) { motor_dm2_task->motor_backend_p->params.position_rad = s2-0.75f;}
    if(motor_dm2_task->motor_backend_p->params.position_rad >= s2+2.4f) { motor_dm2_task->motor_backend_p->params.position_rad = s2+2.4f;}
    if(motor_dm2_task->motor_backend_p->params.speed_rad >= 5.0f)     { motor_dm2_task->motor_backend_p->params.speed_rad = 5.0f;}
    if(motor_dm2_task->motor_backend_p->params.speed_rad <= 0)     { motor_dm2_task->motor_backend_p->params.speed_rad = 0;}
	 
	 if(motor_mg1_task->motor_backend_p->params.angle_control >= 66600+600*k) { motor_mg1_task->motor_backend_p->params.angle_control = 66600+600*k;}
    if(motor_mg1_task->motor_backend_p->params.angle_control <= 600*k-17840){ motor_mg1_task->motor_backend_p->params.angle_control = 600*k-17640;}
    if(motor_mg1_task->motor_backend_p->params.max_speed >= 700)    { motor_mg1_task->motor_backend_p->params.max_speed = 700;}
    if(motor_mg1_task->motor_backend_p->params.max_speed <= 0 )      { motor_mg1_task->motor_backend_p->params.max_speed = 1;}
	 
//	 if(motor_mg1_task->motor_backend_p->crossline_flag ==1)
//	 {
//		motor_mg1_task->motor_backend_p->params.angle_control = motor_mg1_task->motor_backend_p->params.angle_control - 216000;  
//	 }
	 
}


/***********************************************************************
** 函 数 名： Arm_ControlTask::update(timeus_t dT_us)
** 函数说明：
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Arm_ControlTask::update(timeus_t dT_us)
{
   float dT_s = dT_us / 1e6f;
	  //机械臂微调
	  motor_dm1_task->motor_backend_p->params.position_rad += robot.remote_control_task_p->dm1_delta;
	  motor_dm2_task->motor_backend_p->params.position_rad += robot.remote_control_task_p->dm2_delta;	
	  arm_3508_angle += robot.remote_control_task_p->up_3508_delta;
	  motor_mg1_task->motor_backend_p->params.angle_control += robot.remote_control_task_p->mg1_delta;
	
    //机械臂模式切换
     arm_mode_switch();
	
	/* 机械臂RM电机控制 */
         /*↓*/	
    // 获取电机角度返回值
    motor_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(motor_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);	

	 // 获取电机角速度反馈值
    motor_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
	
	// 设置电机角度期望值(外环)
		motor_3508_pid_task->angle_control_task_p->setPIDControllerExpect(
			arm_3508_angle
			);

//限制3058内环角速度输出值//
/////////////////////////////////////////////////////////

		if(motor_3508_pid_task->angle_control_task_p->getOutput() <= arm3508_angular_velocity_max && motor_3508_pid_task->angle_control_task_p->getOutput() >= arm3508_angular_velocity_min)
		{
	 	 // 设置电机角速度期望值（内环）
      motor_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   motor_3508_pid_task->angle_control_task_p->getOutput()
//	  exp_angular_velocity
	    );
		}
		
		if(motor_3508_pid_task->angle_control_task_p->getOutput() > arm3508_angular_velocity_max) 
		{
	 	 // 设置电机角速度期望值（内环）
      motor_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   arm3508_angular_velocity_max
	    );
		}	
		
		if(motor_3508_pid_task->angle_control_task_p->getOutput() < arm3508_angular_velocity_min)
		{
		// 设置电机角速度期望值（内环）
      motor_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   arm3508_angular_velocity_min
	    );
		}
		
			
/////////////////////////////////////////////////////////			
//限制3058与6020内环角速度输出值//

    // 电机正常输出
	    motor_3508_pid_task->motor_backend_p->setMotorInput(
       motor_3508_pid_task->angular_velocity_control_task_p->getOutput()
	    );
	 
	       /*↑*/		 
	/* 机械臂RM电机控制 */
	
	/* 机械臂DM电机控制 */
         /*↓*/

	    motor_dm1_task->motor_backend_p->setMotorPositon(motor_dm1_task->motor_backend_p->params.position_rad,motor_dm1_task->motor_backend_p->params.speed_rad);	

	    motor_dm2_task->motor_backend_p->setMotorPositon(motor_dm2_task->motor_backend_p->params.position_rad,motor_dm2_task->motor_backend_p->params.speed_rad);	
		
		
			/*↑*/		 
	/* 机械臂DM电机控制 */
		
		
		
	/* 机械臂MG电机控制 */
         /*↓*/
		//直接改了输出值
		
		   /*↑*/		 
	/* 机械臂MG电机控制 */
			
	



	
	
	
}

