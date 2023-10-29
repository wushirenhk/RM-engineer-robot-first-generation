/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   Motor_RM.cpp
** 文件说明：   RoboMaster电机数据读取处理
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     林子涵     	     2022-07-18
**	 1.1							   补充注释						     赵钟源     	     2022-12-10
***************************************************************************/

#include "Motor_RM.h"
#include "Motor.h"
#include "Motor_Backend.h"
#include "CanDevice.h"
#include "arm_math.h"

/***********************************************************************
** 函 数 名： Motor_RM::Motor_RM()
** 函数说明： 将定义的电机对象的CAN的接收与发送数据地址添加到can_device中
**---------------------------------------------------------------------
** 输入参数： 电机测量数据指针、电机参数指针、CAN数据包指针、耗时（us）
** 返回参数： 无
***********************************************************************/
Motor_RM::Motor_RM(Motor &motor0,
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
  can_device.addTxLink(_can_tx_id, 8, _can_tx_data_start_pos, can_tx_data, 2);
  motor_type = _motor_type;
}

/***********************************************************************
** 函 数 名： updateMotorMeasurement()0
** 函数说明： 从CAN数据更新电机测量信息
**---------------------------------------------------------------------
** 输入参数： 电机测量数据指针、电机参数指针、CAN数据包指针、耗时（us）
** 返回参数： 无
***********************************************************************/
void Motor_RM::updateMotorMeasurement()
{
  static bool first_update = 1;


  // 时间戳
  com_msr.timestamp = msr.timestamp = micros();

  // CAN数据转化为电机测量数据
  msr.ecd = (uint16_t)((can_rx_data.data)[0] << 8 | (can_rx_data.data)[1]);
  msr.speed_rpm = (uint16_t)((can_rx_data.data)[2] << 8 | (can_rx_data.data)[3]);
  msr.given_current = (uint16_t)((can_rx_data.data)[4] << 8 | (can_rx_data.data)[5]);
  msr.temperate = (can_rx_data.data)[6];

  // 重映射编码器值
  if(msr.ecd >= 0 && msr.ecd < params.offset_ecd + params.half_max_value_ecd)
    msr.ecd = msr.ecd - params.offset_ecd;
  else if(msr.ecd >= params.offset_ecd + params.half_max_value_ecd && msr.ecd < params.max_value_ecd)
    msr.ecd = msr.ecd - params.max_value_ecd - params.offset_ecd;

  // 计算编码器增量值
  msr.delta_ecd = msr.ecd - msr.last_ecd;
  
  // 编码器数值跳变处理
  if (msr.delta_ecd < -params.half_max_value_ecd)
  {
    msr.round_cnt++;
    msr.delta_ecd += params.max_value_ecd;
	 msr.total_ecd = msr.total_ecd + 8192;
  }
  else if (msr.delta_ecd > params.half_max_value_ecd)
  {
    msr.round_cnt--;
    msr.delta_ecd -= params.max_value_ecd;
	 msr.total_ecd = msr.total_ecd - 8192;
  }
  
  // 计算圈数增量值
  msr.delta_round_cnt = msr.round_cnt - msr.last_round_cnt;
  
  // 转子角位移
  com_msr.rotator.angle = 2 * params.direction * PI *
                          (msr.round_cnt +
                           1.0f * msr.ecd / params.max_value_ecd);



  

//  /*  TODO: 转子转速（不能直接用speed_rpm计算，精度太差）
  com_msr.rotator.angular_velocity = msr.speed_rpm / 60.0f * 2 * PI * params.direction;
//  static float delta_T_us;
//  static float last_T_us;
//  static float last_displacement;

//  if((msr.round_cnt != msr.last_round_cnt) || (msr.ecd != msr.last_ecd))
//  {
//    delta_T_us = com_msr.timestamp - last_T_us;
//    com_msr.rotator.angular_velocity = (com_msr.rotator.angle - last_displacement)*1e6f/delta_T_us;

//    last_displacement = com_msr.rotator.angle;
//    last_T_us = com_msr.timestamp;
//  }
//  */

  // 编码器原始数据转为电机极角数据
  com_msr.rotator.polar_angle = msr.ecd * PI * params.direction / params.half_max_value_ecd;

  /*  TODO: 计算输出轴的极角
  */
  com_msr.output.angle = com_msr.rotator.angle / params.reduction_ratio;
  com_msr.output.angular_velocity = com_msr.rotator.angular_velocity / params.reduction_ratio;
  com_msr.linear.velocity = com_msr.output.angular_velocity * params.output_radius;
  com_msr.linear.displacement = com_msr.output.angle * params.output_radius;
  com_msr.linear.relative_displacement = com_msr.linear.displacement - com_msr.linear.offset;
  
  // 保存上次的值
  msr.last_ecd = msr.ecd;
  msr.last_round_cnt = msr.round_cnt;
}

//void Motor_RM::updateMotorAngle(Motor_RM_Measurement_t *msr_p,
//                                Motor_RM_Params_t *params,
//                                Motor_Common_Measurement_t *com_p)
//{


////  int16_t tmp = msr.ecd - params.offset_ecd;
////  if(tmp >= - params.offset_ecd && tmp < params.half_max_value_ecd)
////    com_msr.rotator.polar_angle = tmp * PI / params.half_max_value_ecd;
////  else if(tmp >= params.half_max_value_ecd && tmp < - params.offset_ecd)
////    com_msr.rotator.polar_angle = (params.max_value_ecd - tmp) * PI / params.half_max_value_ecd;

//  // 编码器原始数据转为电机角位移数据
////  tmp += params.max_value_ecd;
////  com_msr.rotator.polar_angle = ((tmp >= params.max_value_ecd/2) ? (tmp - params.max_value_ecd) : tmp) / (params.max_value_ecd*1.0f) * 2 * PI * params.direction;

//  //ptr->spd = ptr_ex->status->speed_rpm * ptr->dir / 60.0f * 2 * PI;
//  //ptr->spd = UpdateGimbalAng_Freq * ptr_ex->delt_angle / 8192.0f * 2 * PI * ptr->dir;
//}

void Motor_RM::updateMeasurement()
{
  updateMotorMeasurement();
}

void Motor_RM::init(void)
{
  params.half_max_value_ecd = params.max_value_ecd / 2;
}

bool Motor_RM::update(timeus_t dT_us)
{
  updateMotorMeasurement();
  publishMeasurement();
  return true;
}

void Motor_RM::uninit(void) {}
