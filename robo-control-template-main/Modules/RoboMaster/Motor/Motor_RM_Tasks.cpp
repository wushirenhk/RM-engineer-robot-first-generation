/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   MotorRM_ControlTask.cpp
** 文件说明：   电机控制任务
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     林子涵     	     2022-07-18
**	 1.1							   补充注释						     赵钟源     	     2022-12-10
***************************************************************************/

#include "Motor_RM_Tasks.h"
#include "/Robot/Robot.h"


/***********************************************************************
** 函 数 名： Motor_RM_PIDControlTask构造函数
** 函数说明： 指定robot引用，CAN设备引用，指定电机CAN总线id
**            和CAN总线数据包发送起始下标，指定电机类型，
**            实例化并注册电机角度环和速度环PID_MotorRM_ControlTask任务，
**            实例化Motor_RM基类，并在robot0引用中添加Backend
**---------------------------------------------------------------------
** 输入参数： robot引用、CAN设备引用、电机CAN总线id、
**            CAN总线数据包发送起始下标、电机类型
** 返回参数： 无
***********************************************************************/
Motor_RM_PIDControlTask::Motor_RM_PIDControlTask(
  Robot &robot0,
  CanDevice &_can_device,
  uint16_t _can_rx_id, uint16_t _can_tx_id,
  uint8_t _can_tx_data_start_pos,
  int8_t _motor_type,
  timeus_t interval_tick_us0
)
  : Task_Base(robot0), can_device(_can_device)
{

  this->interval_tick_us = interval_tick_us0; // 电机编码器解算频率

  angle_control_task_p =
    new PIDControlTask(robot, 0);
  angular_velocity_control_task_p =
    new PIDControlTask(robot, 0);

//  robot.scheduler.registerTask(angle_control_task_p);
//  robot.scheduler.registerTask(angular_velocity_control_task_p);

  can_tx_data_start_pos = _can_tx_data_start_pos;

  motor_backend_p = new Motor_RM(robot0.motors, _can_device, _can_tx_id, _can_rx_id, _can_tx_data_start_pos, _motor_type);
  robot0.motors.addBackend(motor_backend_p);

}

/***********************************************************************
** 函 数 名： Motor_RM_PIDControlTask::init()
** 函数说明： 任务初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Motor_RM_PIDControlTask::init(void)
{

}

/***********************************************************************
** 函 数 名： Motor_RM_PIDControlTask::uninit()
** 函数说明： 任务反初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void Motor_RM_PIDControlTask::uninit(void)
{

}




/***********************************************************************
** 函 数 名： PIDControlTask::update(timeus_t dT_us)
** 函数说明： 任务更新，运行motor_backend_p->update()
**---------------------------------------------------------------------
** 输入参数： 更新时间间隔（us）
** 返回参数： 无
***********************************************************************/
void Motor_RM_PIDControlTask::update(timeus_t dT_us)
{
  if(motor_backend_p != NULL)
  {
    motor_backend_p->update(dT_us);
  }

//  pid_motor_angular_velocity_control_task_p->calculatePID(dT_us, )
//  send();
}
