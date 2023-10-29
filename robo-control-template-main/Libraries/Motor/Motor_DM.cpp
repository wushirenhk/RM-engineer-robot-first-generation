/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   Engineer
** 文 件 名：   Motor_DM.cpp
** 文件说明：   DM电机数据读取处理
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						       孔明骏     	       2022-03-11
***************************************************************************/

#include "Motor_DM.h"
#include "Motor.h"
#include "Motor_Backend.h"
#include "CanDevice.h"
#include "arm_math.h"

/***********************************************************************
** 函 数 名： Motor_DM::Motor_DM()
** 函数说明： 将定义的电机对象的CAN的接收与发送数据地址添加到can_device中
**---------------------------------------------------------------------
** 输入参数： 电机测量数据指针、电机参数指针、CAN数据包指针、耗时（us）
** 返回参数： 无
***********************************************************************/
Motor_DM::Motor_DM(Motor &motor0,
                   CanDevice &_can_device,
                   uint16_t _can_tx_id,
                   uint16_t _can_rx_id,
                   uint8_t _can_tx_data_start_pos,
                   int8_t _motor_type) :
  Motor_Backend(motor0),
  can_device(_can_device)
{
  can_tx_data_start_pos = _can_tx_data_start_pos;
  can_rx_data.std_id = _can_rx_id;
  can_tx_id = _can_tx_id;
  can_device.addRxLink(&can_rx_data);
  can_device.addTxLink(_can_tx_id, 8, _can_tx_data_start_pos, can_tx_data, 8);
  motor_type = _motor_type;
}

/***********************************************************************
** 函 数 名： updateMotorMeasurement()0
** 函数说明： 从CAN数据更新电机测量信息
**---------------------------------------------------------------------
** 输入参数： 电机测量数据指针、电机参数指针、CAN数据包指针、耗时（us）
** 返回参数： 无
***********************************************************************/
void Motor_DM::updateMotorMeasurement()
{
  static bool first_update = 1;
  // 时间戳
  com_msr.timestamp = msr.timestamp = micros();
 // CAN数据转化为电机测量数据
  msr.CAN_ID = (uint8_t)((can_rx_data.data)[0] & 0x0F);
  msr.ERR = (uint8_t)(can_rx_data.data)[0] >> 4;
  msr.ecd_int = (uint16_t)((can_rx_data.data)[1] << 8 | (can_rx_data.data)[2]);
  msr.speed_rad_int = (uint16_t)((can_rx_data.data)[3] << 4 | (can_rx_data.data)[4] >> 4);
  msr.torque_int = (uint16_t)((can_rx_data.data)[4] << 8|(can_rx_data.data)[5])& 0x0FFF;
  msr.ecd = uint_to_float(msr.ecd_int, P_MIN, P_MAX, 16);
  msr.speed_rad = uint_to_float(msr.speed_rad_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
  msr.torque = uint_to_float(msr.torque_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
  msr.t_mos = (float)((can_rx_data.data)[6]);
  msr.t_rotor = (float)((can_rx_data.data)[7]);

  	
}

void Motor_DM::updateMeasurement()
{
  updateMotorMeasurement();
}

void Motor_DM::init(void)
{
  params.half_max_value_ecd = params.max_value_ecd / 2;
}

bool Motor_DM::update(timeus_t dT_us)
{
  updateMotorMeasurement();
  publishMeasurement();
  return true;
}

void Motor_DM::uninit(void) {}
	