/***************************************************************************
**   					             ��������ѧ ��BUGս��
**   					               �������ƣ����Լ���!
**-------------------------------------------------------------------------
** ��    Ŀ��   Engineer
** �� �� ����   ArmControlTask.cpp
** �ļ�˵����   ���������������
**-------------------------------------------------------------------------
**						*�޶�*
**	*�汾*							*�޸�����*							*�޸���*      			 *����*
**	 1.0							   ��ʼ�汾						       ������     	        2023-02-11

***************************************************************************/
#include "ArmControlTask.h"
#include "../Robot/Robot.h"
#include "Helper.h"
#include "../Robot/Params.h"
#include "/Gimbal/GimbalControlTask.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;


int k=75;//mg�������Ƕȵ���
double s2=0;//dm2�������Ƕȵ���

/***********************************************************************
** �� �� ���� Arm_ControlTask���캯��
** ����˵���� ָ��robot���ã�CAN�豸���ã�ָ��yaw��pitch����CAN����id
**            ��CAN�������ݰ�������ʼ�±ָ꣬���������ʱ������
**---------------------------------------------------------------------
** ��������� robot���á�CAN�豸���á����Ħ���ֵ��CAN����id��
**            CAN�������ݰ�������ʼ�±ꡢ�������ʱ����
** ���ز����� ��
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
	
	/********************new��������********************/
	/*****************************��***************************/
  //RM���//
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

	
	//DM���//
  //һ�ŵ��
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
  //���ŵ��
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
  
		
  //MG���
  //һ�ŵ��
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
  
  /*****************************��*************************/
  	/********************new��������********************/
	
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
  
  
  /*****************************��*************************/
  	/********************new��������********************/
    	//middle����Ľ��ٶȺͽǼ��ٶ�ֵ���Լ���������

}



/***********************************************************************
** �� �� ���� Arm_ControlTask::init()
** ����˵���� ע�����Ħ���ַ�����Motor_RM_PIDControlTask����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Arm_ControlTask::init(void)
{
	   inited = true;
		
	
	  //��������ݻ�ȡ����ע��
		if(motor_3508_pid_task != NULL)
     robot.scheduler.registerTask(motor_3508_pid_task);
	   if(motor_dm1_task != NULL)
     robot.scheduler.registerTask(motor_dm1_task);
	  	if(motor_dm2_task != NULL)
     robot.scheduler.registerTask(motor_dm2_task);
	  	if(motor_mg1_task != NULL)
     robot.scheduler.registerTask(motor_mg1_task);			
		
	   /* PID����ע�� */
         /*��*/

      if(motor_3508_pid_task->getAngleTaskPointer() != NULL)
     robot.scheduler.registerTask(motor_3508_pid_task->getAngleTaskPointer());
      if(motor_3508_pid_task->getAngularVelocityTaskPointer() != NULL)
     robot.scheduler.registerTask(motor_3508_pid_task->getAngularVelocityTaskPointer());
		
	       /*��*/
 	   /* PID����ע�� */		
		  
		  
		arm_mode = Arm_OFF;
		arm_3508_angle = -0.1;  //����		
		
		motor_mg1_task->motor_backend_p->params.max_speed = 1;
			
		motor_dm1_task->motor_backend_p->params.position_rad = 2.06;      //����
		motor_dm2_task->motor_backend_p->params.position_rad =s2-0.36; 
	   motor_dm1_task->motor_backend_p->params.speed_rad = 0.5;      //����/sz
		motor_dm2_task->motor_backend_p->params.speed_rad = 0.5;	

      exp_angle = 0;
      arm3508_angular_velocity_max = 5;
      arm3508_angular_velocity_min = -5;		
}

/***********************************************************************
** �� �� ���� ArmRM_ControlTask::uninit()
** ����˵���� ���񷴳�ʼ��
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Arm_ControlTask::uninit(void) 
{

}

/***********************************************************************
** �� �� ���� Arm_ControlTask::arm_time_delay(uint16_t time)
** ����˵���� arm������ʱ�����ж�ʽ������λms
**---------------------------------------------------------------------
** ��������� time����ʱʱ�䣬��λms
** ���ز����� ��
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
** �� �� ���� Arm_ControlTask::arm_mode_switch()
** ����˵���� ��е��ģʽ�л�
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Arm_ControlTask::arm_mode_switch()
{
  /*�жϴ�ʱ״̬��ʹ�ò�ͬ���Ʒ�ʽ*/
           /*��*/
		 switch (arm_mode){
		 case Arm_OFF:	 
			 
		 motor_mg1_task->motor_backend_p->params.angle_control =  motor_mg1_task->motor_backend_p->params.multiturn_angle;
//		 motor_mg1_task->motor_backend_p->params.angle_control = -86*600;
		 motor_mg1_task->motor_backend_p->params.max_speed = 1;		 
		 
		 motor_dm1_task->motor_backend_p->params.speed_rad = 0;
		 motor_dm2_task->motor_backend_p->params.speed_rad = 0; 
		 
		 
		 
		 break;
		 /****************************normal״̬��ʼת��************************************/
		 case Arm_normal:	

		    switch (last_arm_mode)
			 {
				 case Arm_OFF:
					 if(normal == state0)
					 {	 
					 arm3508_state_normal_angle = -0.1;  //����
					 mg1_state_normal_angle = 0+k;      //�ȣ����ȿɵ���
				    mg1_state_normal_speed = 200;   //��/s,Ҫ����6
				    dm1_state_normal_angle = 2.06;      //����
				    dm2_state_normal_angle = 1.9+s2; 
				    dm1_state_normal_speed = 2;      //����/sz
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
					 arm3508_state_normal_angle = 0.1;  //����
					 mg1_state_normal_angle = k+31;      //�ȣ����ȿɵ���
				    mg1_state_normal_speed = 400;   //��/s,Ҫ����6
				    dm1_state_normal_angle = 2.06;      //����
				    dm2_state_normal_angle = 2.263+s2; 
				    dm1_state_normal_speed = 2;      //����/sz
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
			 /****************************AirConnection״̬��ʼת��************************************/
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
			 /****************************Arm_SilveryOre״̬��ʼת��************************************/
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
			 
			/****************************Arm_GroundOre״̬��ʼת��************************************/
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
			 
		/****************************Arm_OrePlacement״̬��ʼת��************************************/
		 case Arm_OrePlacement:	 
			 switch (last_arm_mode)
			 {

				 case Arm_OFF:

				    
				 break;
					 
				 case Arm_normal:
					 if(OrePlacement == state0)
					 {	 
					 arm3508_state_OrePlacement_angle = -5;  //����
					 mg1_state_OrePlacement_angle = k-14;      //�ȣ����ȿɵ���
				    mg1_state_OrePlacement_speed = 400;   //��/s,Ҫ����6
				    dm1_state_OrePlacement_angle = 2.06;      //����
				    dm2_state_OrePlacement_angle = 0.988+s2; 
				    dm1_state_OrePlacement_speed = 2;      //����/sz
				    dm2_state_OrePlacement_speed = 1;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
					 if(arm_time_delay(500) == 1)
					 {
					 OrePlacement = state1;}
					 }
					 if(OrePlacement == state1)
					 {
					 arm3508_state_OrePlacement_angle = -5;  //����
					 mg1_state_OrePlacement_angle = k+16;      //�ȣ����ȿɵ���
				    mg1_state_OrePlacement_speed = 300;   //��/s,Ҫ����6
				    dm1_state_OrePlacement_angle = 2.06;      //����
				    dm2_state_OrePlacement_angle = s2-0.307; 
				    dm1_state_OrePlacement_speed = 2;      //����/sz
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
					 arm3508_state_OrePlacement_angle = -8.5;  //����
					 mg1_state_OrePlacement_angle = k+21;      //�ȣ����ȿɵ���
				    mg1_state_OrePlacement_speed = 500;   //��/s,Ҫ����6
				    dm1_state_OrePlacement_angle = 2.06;      //����
				    dm2_state_OrePlacement_angle = s2-0.307; 
				    dm1_state_OrePlacement_speed = 2;      //����/sz
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
			 			 
		/****************************Arm_HoldOre״̬��ʼת��************************************/
		 case Arm_HoldOre:	 
			 switch (last_arm_mode)
			 {

				 case Arm_OFF:
			    
				 break;
					 
				 case Arm_normal:
					 if(HoldOre == state0)
					 {	 
					 arm3508_state_HoldOre_angle = -0.5;  //����
					 mg1_state_HoldOre_angle = k+24;      //�ȣ����ȿɵ���
				    mg1_state_HoldOre_speed = 400;   //��/s,Ҫ����6
				    dm1_state_HoldOre_angle = 2.06;      //����
				    dm2_state_HoldOre_angle = 1.713+s2; 
				    dm1_state_HoldOre_speed = 3;      //����/sz
				    dm2_state_HoldOre_speed = 1.5;
					 robot.chassis_task_p->chassis_direction_mode = Chassis_ORE;
					 if(arm_time_delay(500) == 1)
					 {
					 HoldOre = state1;
					 }
					 }
					 if(HoldOre == state1)
					 {	 
					 arm3508_state_HoldOre_angle = -0.5;  //����
					 mg1_state_HoldOre_angle = k+24;      //�ȣ����ȿɵ���
				    mg1_state_HoldOre_speed = 400;   //��/s,Ҫ����6
				    dm1_state_HoldOre_angle = 2.06;      //����
				    dm2_state_HoldOre_angle = 2.399+s2; 
				    dm1_state_HoldOre_speed = 3;      //����/sz
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
					 arm3508_state_HoldOre_angle = -0.5;  //����
					 mg1_state_HoldOre_angle = k+24;      //�ȣ����ȿɵ���
				    mg1_state_HoldOre_speed = 300;   //��/s,Ҫ����6
				    dm1_state_HoldOre_angle = 2.06;      //����
				    dm2_state_HoldOre_angle = 2.399+s2; 
				    dm1_state_HoldOre_speed = 3;      //����/sz
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
					 arm3508_state_HoldOre_angle = -0.5;  //����
					 mg1_state_HoldOre_angle = k+24;      //�ȣ����ȿɵ���
				    mg1_state_HoldOre_speed = 300;   //��/s,Ҫ����6
				    dm1_state_HoldOre_angle = 2.06;      //����
				    dm2_state_HoldOre_angle = motor_dm2_task->motor_backend_p->params.position_rad; 
				    dm1_state_HoldOre_speed = 2;      //����/sz
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
					 arm3508_state_HoldOre_angle = -0.5;  //����
					 mg1_state_HoldOre_angle = k+24;      //�ȣ����ȿɵ���
				    mg1_state_HoldOre_speed = 300;   //��/s,Ҫ����6
				    dm1_state_HoldOre_angle = 2.06;      //����
				    dm2_state_HoldOre_angle = 2.399+s2; 
				    dm1_state_HoldOre_speed = 2;      //����/sz
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
					 arm3508_state_HoldOre_angle = -0.5;  //����
					 mg1_state_HoldOre_angle = k+24;      //�ȣ����ȿɵ���
				    mg1_state_HoldOre_speed = 300;   //��/s,Ҫ����6
				    dm1_state_HoldOre_angle = 2.06;      //����
				    dm2_state_HoldOre_angle = 2.399+s2; 
				    dm1_state_HoldOre_speed = 2;      //����/sz
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
	//����Ƕ����ٶ��޷�

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
** �� �� ���� Arm_ControlTask::update(timeus_t dT_us)
** ����˵����
**---------------------------------------------------------------------
** ��������� ��
** ���ز����� ��
***********************************************************************/
void Arm_ControlTask::update(timeus_t dT_us)
{
   float dT_s = dT_us / 1e6f;
	  //��е��΢��
	  motor_dm1_task->motor_backend_p->params.position_rad += robot.remote_control_task_p->dm1_delta;
	  motor_dm2_task->motor_backend_p->params.position_rad += robot.remote_control_task_p->dm2_delta;	
	  arm_3508_angle += robot.remote_control_task_p->up_3508_delta;
	  motor_mg1_task->motor_backend_p->params.angle_control += robot.remote_control_task_p->mg1_delta;
	
    //��е��ģʽ�л�
     arm_mode_switch();
	
	/* ��е��RM������� */
         /*��*/	
    // ��ȡ����Ƕȷ���ֵ
    motor_3508_pid_task->angle_control_task_p->setPIDControllerFeedback(motor_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angle);	

	 // ��ȡ������ٶȷ���ֵ
    motor_3508_pid_task->angular_velocity_control_task_p->setPIDControllerFeedback(motor_3508_pid_task->motor_backend_p->getCommonMeasurement().output.angular_velocity);
	
	// ���õ���Ƕ�����ֵ(�⻷)
		motor_3508_pid_task->angle_control_task_p->setPIDControllerExpect(
			arm_3508_angle
			);

//����3058�ڻ����ٶ����ֵ//
/////////////////////////////////////////////////////////

		if(motor_3508_pid_task->angle_control_task_p->getOutput() <= arm3508_angular_velocity_max && motor_3508_pid_task->angle_control_task_p->getOutput() >= arm3508_angular_velocity_min)
		{
	 	 // ���õ�����ٶ�����ֵ���ڻ���
      motor_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   motor_3508_pid_task->angle_control_task_p->getOutput()
//	  exp_angular_velocity
	    );
		}
		
		if(motor_3508_pid_task->angle_control_task_p->getOutput() > arm3508_angular_velocity_max) 
		{
	 	 // ���õ�����ٶ�����ֵ���ڻ���
      motor_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   arm3508_angular_velocity_max
	    );
		}	
		
		if(motor_3508_pid_task->angle_control_task_p->getOutput() < arm3508_angular_velocity_min)
		{
		// ���õ�����ٶ�����ֵ���ڻ���
      motor_3508_pid_task->angular_velocity_control_task_p->setPIDControllerExpect(
	   arm3508_angular_velocity_min
	    );
		}
		
			
/////////////////////////////////////////////////////////			
//����3058��6020�ڻ����ٶ����ֵ//

    // ����������
	    motor_3508_pid_task->motor_backend_p->setMotorInput(
       motor_3508_pid_task->angular_velocity_control_task_p->getOutput()
	    );
	 
	       /*��*/		 
	/* ��е��RM������� */
	
	/* ��е��DM������� */
         /*��*/

	    motor_dm1_task->motor_backend_p->setMotorPositon(motor_dm1_task->motor_backend_p->params.position_rad,motor_dm1_task->motor_backend_p->params.speed_rad);	

	    motor_dm2_task->motor_backend_p->setMotorPositon(motor_dm2_task->motor_backend_p->params.position_rad,motor_dm2_task->motor_backend_p->params.speed_rad);	
		
		
			/*��*/		 
	/* ��е��DM������� */
		
		
		
	/* ��е��MG������� */
         /*��*/
		//ֱ�Ӹ������ֵ
		
		   /*��*/		 
	/* ��е��MG������� */
			
	



	
	
	
}

