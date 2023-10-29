/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   CANSendTask.cpp
** 文件说明：   CAN发送任务，一个CANSendTask需要指定Robot基类、CAN发送设备、
                CAN发送数据链接和发送周期。
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     林子涵     	   2022-07-18
**	 1.1							  加入TxLink					     林子涵     	   2022-07-26
**	 1.2							   补充注释						     赵钟源     	   2022-12-10
***************************************************************************/

#include "CANSendTask.h"

/***********************************************************************
** 函 数 名： CANSendTask构造函数
** 函数说明： 指定CAN设备、CAN发送链接（CAN_Tx_Data_Link_t）、
              发送时间间隔
**---------------------------------------------------------------------
** 输入参数： CAN设备、CAN发送链接（CAN_Tx_Data_Link_t）、
              发送时间间隔
** 返回参数： 无
***********************************************************************/
CANSendTask::CANSendTask(Robot &robot0, CanDevice *can_device0, CAN_Tx_Data_Link_t *tx_link0, timeus_t interval_tick_us0) : Task_Base(robot0)
{
  can_tx_link = tx_link0;
  can_device = can_device0;
  this->interval_tick_us = interval_tick_us0;
}

/***********************************************************************
** 函 数 名： CANSendTask构造函数
** 函数说明： 指定CAN设备、CAN发送链接（CAN_Tx_Data_Link_t）、
              发送时间间隔
**---------------------------------------------------------------------
** 输入参数： CAN设备、CAN发送链接（CAN_Tx_Data_Link_t）、
              发送时间间隔
** 返回参数： 无
***********************************************************************/
CANSendTask::CANSendTask(Robot &robot0, CanDevice *can_device0, uint16_t tx_id, timeus_t interval_tick_us0) : Task_Base(robot0)
{
  for(int i = 0; i < can_device0->can_tx_link_count; i++){
    if(can_device0->can_tx_data[i].std_id == tx_id)
      can_tx_link = &can_device0->can_tx_data[i];
  }
  can_device = can_device0;
  this->interval_tick_us = interval_tick_us0;
}

/***********************************************************************
** 函 数 名： CANSendTask::init()
** 函数说明： 初始化CAN发送任务
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void CANSendTask::init(void)
{

}

/***********************************************************************
** 函 数 名： CANSendTask::update(timeus_t dT_us)
** 函数说明： 更新CAN发送任务
**---------------------------------------------------------------------
** 输入参数： 更新时间间隔（us）
** 返回参数： 无
***********************************************************************/
void CANSendTask::update(timeus_t dT_us)
{
  can_header.StdId = can_tx_link->std_id;
  can_header.IDE = CAN_ID_STD;
  can_header.RTR = CAN_RTR_DATA;
  can_header.DLC = can_tx_link->dlc;
  for(int i = 0; i < can_tx_link->dlc; i++)
    if(can_tx_link->data[i] != NULL)
      can_send_data[i] = *can_tx_link->data[i];
    else
      can_send_data[i] = 0;
  can_device->send(&can_header, can_send_data);
}

/***********************************************************************
** 函 数 名： CANSendTask::uninit()
** 函数说明： 任务反初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void CANSendTask::uninit(void)
{

}