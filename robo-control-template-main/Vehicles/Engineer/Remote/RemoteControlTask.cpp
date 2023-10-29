#include "/Remote/RemoteControlTask.h"
#include "/Robot/Robot.h"

#define JOYSTICK_DEAD_ZONE 5
#define JOYSTICK_MAX 700

/* ----------------------- RC Switch Definition----------------------------- */
#define SWITCH_UP                ((uint16_t)1)
#define SWITCH_MID               ((uint16_t)3)
#define SWITCH_DWN               ((uint16_t)2)

/* ----------------------- ROBOT Mode Definition---------------------------- */
#define MOT	OR_DISABLE										((uint16_t)1)
#define CHASSIS_FOLLOW_GIMBAL						((uint16_t)2)
#define CHASSIS_GYRO_ROTATION						((uint16_t)3)
#define CHASSIS_FREE										((uint16_t)4)

/***********************************************************************
** 函 数 名： isBeyondDeadZone()
** 函数说明： 判断是否越过死区
**---------------------------------------------------------------------
** 输入参数： rc_data
** 返回参数： 无
***********************************************************************/
bool isBeyondDeadZone(int16_t rc_data)
{
  if(rc_data >= JOYSTICK_DEAD_ZONE || rc_data <= -JOYSTICK_DEAD_ZONE)
    return true;
  else
    return false;
}

/***********************************************************************
** 函 数 名： RemoteControlTask::init()
** 函数说明： 判断是否完成初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void RemoteControlTask::init(void)
{
  inited = true;
}

/***********************************************************************
** 函 数 名： RemoteControlTask::update()
** 函数说明： 处理接收到的rc_data[]中的数据，完成遥控器控制
**---------------------------------------------------------------------
** 输入参数： 更新周期
** 返回参数： 无
***********************************************************************/
void RemoteControlTask::update(timeus_t dT_us)
{	
	// 比赛中重新上电后将dm电机使能
//   if(robot.referee_system_task_p->robot_referee_status.game_robot_status.remain_HP != 0 && robot.arm_task_p->motor_dm1_task->motor_backend_p->start_flag <100)
//   {
//		 robot.arm_task_p->motor_dm1_task->motor_backend_p->start_motor();
//		 robot.arm_task_p->motor_dm1_task->motor_backend_p->start_flag++;
//   }
//	 if(robot.referee_system_task_p->robot_referee_status.game_robot_status.remain_HP != 0 && robot.arm_task_p->motor_dm2_task->motor_backend_p->start_flag <100)
//   {
//		 robot.arm_task_p->motor_dm2_task->motor_backend_p->start_motor();
//		 robot.arm_task_p->motor_dm2_task->motor_backend_p->start_flag++;
//   }

	//刷新UI  左键刷新UI
		if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.rc_protocol.getRCData().mouse.press_l)
		{
			robot.referee_system_task_p->set_num_0();
			robot.referee_system_task_p->set_t_0();
		}	
		
		/* 键盘鼠标控制 遥控器微调 Begin */
	         /* ↓ */
	//比赛时候，都拨到下面
	if(robot.rc_protocol.getRCData().sw_right == SWITCH_DWN && robot.rc_protocol.getRCData().sw_left == SWITCH_DWN )
{	

		if(robot.rc_protocol.getRCData().keyboard.key_bit.Z && robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.arm_task_p->arm_mode != Arm_normal && robot.arm_task_p->switch_lock == 0)
		{
			robot.arm_task_p->normal = state0;
			robot.arm_task_p->last_arm_mode = robot.arm_task_p->arm_mode;
			robot.arm_task_p->arm_mode = Arm_normal;
			robot.arm_task_p->switch_lock = 1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.V && robot.arm_task_p->arm_mode == Arm_normal && robot.arm_task_p->switch_lock == 0)
		{
			robot.arm_task_p->AirConnection = state0;
			robot.arm_task_p->last_arm_mode = robot.arm_task_p->arm_mode;
			robot.arm_task_p->arm_mode = Arm_AirConnection;
			robot.arm_task_p->switch_lock = 1;
		}
		
		if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.rc_protocol.getRCData().keyboard.key_bit.X && robot.arm_task_p->arm_mode == Arm_normal && robot.arm_task_p->switch_lock == 0)
		{
			robot.arm_task_p->SilveryOre = state0;
			robot.arm_task_p->last_arm_mode = robot.arm_task_p->arm_mode;
			robot.arm_task_p->arm_mode = Arm_SilveryOre;
			robot.arm_task_p->switch_lock = 1;
		}
				
		if(robot.rc_protocol.getRCData().keyboard.key_bit.C && robot.arm_task_p->arm_mode == Arm_normal && robot.arm_task_p->switch_lock == 0)
		{
			robot.arm_task_p->GroundOre = state0;
			robot.arm_task_p->last_arm_mode = robot.arm_task_p->arm_mode;
			robot.arm_task_p->arm_mode = Arm_GroundOre;
			robot.arm_task_p->switch_lock = 1;
		}
		
		
		if(robot.rc_protocol.getRCData().keyboard.key_bit.B && robot.arm_task_p->arm_mode != Arm_OFF && robot.arm_task_p->arm_mode != Arm_HoldOre  && robot.arm_task_p->switch_lock == 0)
		{
			robot.arm_task_p->HoldOre = state0;
			robot.arm_task_p->last_arm_mode = robot.arm_task_p->arm_mode;
			robot.arm_task_p->arm_mode = Arm_HoldOre;
			robot.arm_task_p->switch_lock = 1;
		}
		
		if(robot.rc_protocol.getRCData().keyboard.key_bit.Z && robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 1 && (robot.arm_task_p->arm_mode == Arm_normal || robot.arm_task_p->arm_mode == Arm_HoldOre) && robot.arm_task_p->switch_lock == 0)
		{
			robot.arm_task_p->OrePlacement = state0;
			robot.arm_task_p->last_arm_mode = robot.arm_task_p->arm_mode;
			robot.arm_task_p->arm_mode = Arm_OrePlacement;
			robot.arm_task_p->switch_lock = 1;
		}
		
		//ctrl控制翻矿正反方向
		if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 && robot.rc_protocol.getRCData().keyboard.key_bit.F)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_CW;
		}
		else if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL && robot.rc_protocol.getRCData().keyboard.key_bit.F)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_CCW;			
		}
		else if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL && robot.rc_protocol.getRCData().mouse.press_l)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_OUT;			
		}
		else if(robot.rc_protocol.getRCData().keyboard.key_bit.CTRL && robot.rc_protocol.getRCData().keyboard.key_bit.X)
		{
			robot.gimbal_task_p->overturn_mode = Overturn_IN;			
		}
		else
		{
			robot.gimbal_task_p->overturn_mode = Overturn_OFF;
		}
		
      //鼠标左键控制翻矿收入与取出


		
		/*控制六个气泵*/
		if(robot.rc_protocol.getRCData().keyboard.key_bit.R && robot.arm_task_p->airpump_lock[0] == 0 &&robot.arm_task_p->airpump_flag[0] == 0)//R启动气泵
		{
          robot.arm_task_p->airpump_flag[0] = 1;
			 robot.arm_task_p->airpump_lock[0] = 1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.R && robot.arm_task_p->airpump_lock[0] == 0 && robot.arm_task_p->airpump_flag[0] == 1)//再按R关闭气泵
		{
          robot.arm_task_p->airpump_flag[0] = 0;
			 robot.arm_task_p->airpump_lock[0]=  1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.R == 0 && time_cnt[0] < 70 && robot.arm_task_p->airpump_lock[0] ==  1) //1秒最多手动切换一次
		{
			time_cnt[0] ++;
		}
		if(time_cnt[0] ==50)
		{
			time_cnt[0] = 0;
			robot.arm_task_p->airpump_lock[0] = 0;
		}
		
		/*控制抬升气泵*/
		if(robot.rc_protocol.getRCData().keyboard.key_bit.G && robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 0 &&  robot.arm_task_p->airpump_lock[1] == 0)//G抬升机械臂
		{
          robot.arm_task_p->airpump_flag[1] = 1;
			 robot.arm_task_p->airpump_flag[2] = 0;
			
			 robot.arm_task_p->airpump_lock[1] = 1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.G && robot.rc_protocol.getRCData().keyboard.key_bit.CTRL == 1 && robot.arm_task_p->airpump_lock[1] == 0)//再按G放下机械臂
		{
          robot.arm_task_p->airpump_flag[1] = 0;
			 robot.arm_task_p->airpump_flag[2] = 1; 
			
			 robot.arm_task_p->airpump_lock[1]=  1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.G == 0 && time_cnt[1] < 210 && robot.arm_task_p->airpump_lock[1] ==  1) //2秒最多手动切换一次
		{
			time_cnt[1] ++;
		}
		if(time_cnt[1] == 60)
		{
			time_cnt[1] = 0;
			robot.arm_task_p->airpump_lock[1] = 0;
         if(robot.arm_task_p->airpump_flag[1] == 0 && robot.arm_task_p->airpump_flag[2] == 1)		
			{				
         robot.arm_task_p->airpump_flag[1] = 0;
			robot.arm_task_p->airpump_flag[2] = 0; 	
			}				
		}
		
		
		//底盘速度模式切换
		if(robot.rc_protocol.getRCData().keyboard.key_bit.SHIFT && robot.chassis_task_p->chassis_mode == Chassis_SLOW && shift_lock == 0)
		{
			robot.chassis_task_p->chassis_mode = Chassis_FAST;
			shift_lock = 1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.SHIFT && robot.chassis_task_p->chassis_mode == Chassis_FAST && shift_lock == 0)
		{
			robot.chassis_task_p->chassis_mode = Chassis_SLOW;
			shift_lock = 1;
		}
		if(robot.rc_protocol.getRCData().keyboard.key_bit.SHIFT == 0 && shift_tim < 70 && shift_lock == 1) //2秒最多手动切换一次
		{
			shift_tim ++;
		}
		if(shift_tim == 30)
		{
			shift_tim = 0;
			shift_lock = 0;
		}

//云台控制模式0：自由移动
if(robot.gimbal_task_p->gimbal_mode == GIMBAL_FREE)		
{
	 //鼠标X轴控制图传的yaw轴6020电机
	 if(robot.rc_protocol.getRCData().mouse.vx != 0)
    {
      robot.gimbal_task_p->gimbal_yaw_vel = -robot.rc_protocol.getRCData().mouse.vx / 15.0f;
      if(robot.gimbal_task_p->gimbal_yaw_vel > robot.gimbal_task_p->gimbal_max_yaw_vel)
      {
        robot.gimbal_task_p->gimbal_yaw_vel = robot.gimbal_task_p->gimbal_max_yaw_vel;
      }
      else if(robot.gimbal_task_p->gimbal_yaw_vel < -robot.gimbal_task_p->gimbal_max_yaw_vel)
      {
        robot.gimbal_task_p->gimbal_yaw_vel = -robot.gimbal_task_p->gimbal_max_yaw_vel;
      }
    }
    else // 无输入置零
    {
      robot.gimbal_task_p->gimbal_yaw_vel = 0;
    }
	 
	//鼠标Y轴控制图传的pitcn轴舵机		
	robot.gimbal_task_p->gimbal_pitch_angle += robot.rc_protocol.getRCData().mouse.vy/30.0f;
	if(robot.gimbal_task_p->gimbal_pitch_angle >= 160)
   {
		robot.gimbal_task_p->gimbal_pitch_angle = 160;
	}
	if(robot.gimbal_task_p->gimbal_pitch_angle <= 70)
	{
		robot.gimbal_task_p->gimbal_pitch_angle = 70;
	}	
}

//鼠标右键控制云台模式切换
	if(robot.rc_protocol.getRCData().mouse.press_r && press_r_lock == 0)
	{
	  if(robot.gimbal_task_p->gimbal_mode == GIMBAL_LOCK){robot.gimbal_task_p->gimbal_mode = GIMBAL_FREE;press_r_lock = 1;}
	  else {robot.gimbal_task_p->gimbal_mode = GIMBAL_LOCK;press_r_lock = 1;}	  
	}
	if(robot.rc_protocol.getRCData().mouse.press_r == 0 && press_r_tim < 30 && press_r_lock == 1) //1秒最多手动切换一次
	{
	    press_r_tim ++;
		 if(press_r_tim == 30)
		{
			press_r_tim = 0;
			press_r_lock = 0;
		}
	}




  //遥控器微调角度
	//左前后，控制dm2
	if(robot.arm_task_p->switch_lock == 0)
	{
		if(robot.rc_protocol.getRCData().ch3 >=300)         {dm2_delta =  0.003;}
		else if (robot.rc_protocol.getRCData().ch3 <=- 300) {dm2_delta = -0.003;}
		else{dm2_delta = 0;}
	//左左右，控制dm1
		if(robot.rc_protocol.getRCData().ch2 >=500)         {dm1_delta =  0.002;}
		else if (robot.rc_protocol.getRCData().ch2 <=- 500) {dm1_delta = -0.002;}
		else{dm1_delta = 0;}	
	//右前后，控制上下3508
		if(robot.rc_protocol.getRCData().ch1 >=300)         {up_3508_delta =  -0.03;}
		else if (robot.rc_protocol.getRCData().ch1 <=- 300) {up_3508_delta =  0.03;}
		else{up_3508_delta = 0;}
   //右左右，控制mg1
		if(robot.rc_protocol.getRCData().ch0 >= 300)         {mg1_delta =  50;}
		else if (robot.rc_protocol.getRCData().ch0 <=- 300) {mg1_delta =  -50;}
		else{mg1_delta = 0;}
	}
	
}

	         /* ↑ */	
	  /* 键盘鼠标控制 遥控器微调 End */


else{	
	/* 遥控器控制 Begin
	遥控器左上,分别底盘，机械臂，右开关负责控制每个模块的具体模式 */
	
	/*遥控器说明
	左上 + arm_off + 右上 == 底盘上下电
	左上 + 右中 == 开关气泵
	左中 + chassis_off + 右上 == 机械臂切换模式
	左中 + 右中 == 开关气瓶 
	*/
	
 if(robot.rc_protocol.getRCData().sw_left != SWITCH_UP || robot.rc_protocol.getRCData().sw_right != SWITCH_DWN)
 {		
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP && robot.arm_task_p->arm_mode == Arm_OFF)  //左上右上	
	{
		  if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.rc_protocol.getRCData().wheel<-500)
		{
			robot.chassis_task_p->chassis_mode = Chassis_FAST;//左上右上+滑轮=底盘上电
		}
		  if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.rc_protocol.getRCData().wheel> 500)
		{
			robot.chassis_task_p->chassis_mode = Chassis_OFF;//左上右中+滑轮=底盘下电
		}
	}
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP)  //左上右上	
	{
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel <-500)
		{
			robot.arm_task_p->airpump_flag[0] = 1;//左上右中+滑轮=启动气泵
		}
		if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID && robot.rc_protocol.getRCData().wheel > 500)
		{
			robot.arm_task_p->airpump_flag[0] = 0;//左上右中+滑轮=关闭气泵
		}

	}		
	
   if(robot.rc_protocol.getRCData().sw_left == SWITCH_MID )//左中右
	{	
			if(robot.rc_protocol.getRCData().sw_right == SWITCH_MID)//左中右中	
			{
				if(robot.rc_protocol.getRCData().wheel < -500)
				{
			      robot.arm_task_p->airpump_flag[1] = 1;//左中右上+滑轮=启动气瓶
					robot.arm_task_p->airpump_flag[2] = 0;//左中右上+滑轮=启动气瓶
				}
				else if(robot.rc_protocol.getRCData().wheel > 500)
				{
			      robot.arm_task_p->airpump_flag[1] = 0;//左中右上+滑轮=关闭气瓶
					robot.arm_task_p->airpump_flag[2] = 1;//左中右上+滑轮=关闭气瓶
				}
				else
				{
			      robot.arm_task_p->airpump_flag[1] = 0;//左中右上+滑轮=关闭气瓶
					robot.arm_task_p->airpump_flag[2] = 0;//左中右上+滑轮=关闭气瓶					
				}
			}
		
			if(robot.rc_protocol.getRCData().sw_right == SWITCH_UP && robot.chassis_task_p->chassis_mode == Chassis_OFF)
			{
				if(robot.rc_protocol.getRCData().wheel < -500)
				{
			   robot.arm_task_p->normal = state0;
			   robot.arm_task_p->last_arm_mode = Arm_OFF;
				robot.arm_task_p->arm_mode = Arm_normal;
				robot.arm_task_p->switch_lock = 1;
				}
				if(robot.rc_protocol.getRCData().wheel > 500)
				{
			   robot.arm_task_p->last_arm_mode = Arm_normal;
				robot.arm_task_p->arm_mode = Arm_OFF;				
				}
			}
	}
		
		
	   
			//底盘速度
     	if(robot.chassis_task_p->chassis_mode == Chassis_FAST)	
		{
		  robot.chassis_task_p->vx = robot.rc_protocol.getRCData().ch3 / 600.0f;

        robot.chassis_task_p->vy = -robot.rc_protocol.getRCData().ch2 / 600.0f;

	     robot.chassis_task_p->vw = -robot.rc_protocol.getRCData().ch0 / 600.0f;
		}
}	
	
	if(robot.arm_task_p->switch_lock == 0)
	{
		if(robot.rc_protocol.getRCData().ch3 >=300)         {dm2_delta =  0.002;}
		else if (robot.rc_protocol.getRCData().ch3 <=- 300) {dm2_delta = -0.002;}
		else{dm2_delta = 0;}
	//左左右，控制dm1
		if(robot.rc_protocol.getRCData().ch2 >=300)         {dm1_delta =  0.002;}
		else if (robot.rc_protocol.getRCData().ch2 <=- 300) {dm1_delta = -0.002;}
		else{dm1_delta = 0;}	
	//右前后，控制上下3508
		if(robot.rc_protocol.getRCData().ch1 >=300)         {up_3508_delta =  -0.01;}
		else if (robot.rc_protocol.getRCData().ch1 <=- 300) {up_3508_delta =  0.01;}
		else{up_3508_delta = 0;}
   //右左右，控制mg1
		if(robot.rc_protocol.getRCData().ch0 >=300)         {mg1_delta =  20;}
		else if (robot.rc_protocol.getRCData().ch0 <=- 300) {mg1_delta = -20;}
		else{mg1_delta = 0;}
	}

 if(robot.rc_protocol.getRCData().sw_left == SWITCH_UP && robot.rc_protocol.getRCData().sw_right == SWITCH_DWN)
{		
			   robot.arm_task_p->last_arm_mode = Arm_normal;
	
				robot.arm_task_p->arm_mode = Arm_OFF;	
	
            robot.chassis_task_p->chassis_mode = Chassis_OFF;
}
}

	/* 遥控器控制 End*/
	
 

}

void RemoteControlTask::uninit(void)
{

}
