/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   Motor_MG.cpp
** 文件说明：   瓴控科技MG电机数据读取处理
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     沈申浩     	     2023-12-22
***************************************************************************/

#include "Motor_MG.h"
#include "Motor.h"
#include "Motor_Backend.h"
#include "CanDevice.h"
#include "arm_math.h"


Motor_MG::Motor_MG(Motor &motor0,
                   CanDevice &_can_device,
                   uint16_t _can_tx_id,
                   uint16_t _can_rx_id,
                   uint8_t _can_tx_data_start_pos,
                   int8_t _motor_type) :
  Motor_Backend(motor0),
  can_device(_can_device)
{
  can_tx_data_start_pos = _can_tx_data_start_pos;//MG电机发送所有data，默认为0
  can_rx_data.std_id = _can_rx_id;
  can_tx_id = _can_tx_id;
  can_device.addRxLink(&can_rx_data);
  can_device.addTxLink(_can_tx_id, 8, _can_tx_data_start_pos, can_tx_data, 8);
  motor_type = _motor_type;
}

/********************************************************************************/
/**********************************Get读取命令***********************************/
/********************************************************************************/
//读取PID值（0x30）
///*驱动回复
//DATA[2] 位置环P参数     DATA[2]= anglePidKp
//DATA[3] 位置环I参数     DATA[3]= anglePidKi
//DATA[4] 速度环P参数     DATA[4]= speedPidKp
//DATA[5] 速度环I参数     DATA[5]= speedPidKi
//DATA[6] 转矩环P参数     DATA[6]= iqPidKp
//DATA[7] 转矩环I参数     DATA[7]= iqPidKi*/
void Motor_MG::getPIDParams()
{
		if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x30;
	can_tx_data[1] = 0;
	can_tx_data[2] = 0;
	can_tx_data[3] = 0;
	can_tx_data[4] = 0; 
	can_tx_data[5] = 0;
	can_tx_data[6] = 0;
	can_tx_data[7] = 0;
	  can_update_flag = can_tx_data[0];
	}
}
//读取加速度（0x33）
///*驱动回复
//DATA[4] 加速度低字节 1   DATA[4]= (uint8_t)Accel
//DATA[5] 加速度字节 2     DATA[5]= (uint8_t)Accel+1
//DATA[6] 加速度字节 3     DATA[6]= (uint8_t)Accel+2
//DATA[7] 加速度字节 4     DATA[7]= (uint8_t)Accel+3*/
void Motor_MG::getAccel()
{
		if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//单发模式
  can_tx_data[0] = 0x33;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
	  can_update_flag = can_tx_data[0];
	}
}

//读取编码器数据命令（0x90）
///*驱动回复
//DATA[2] 编码器位置低字节     DATA[2]= *(uint8_t *) (&encoder)
//DATA[3] 编码器位置高字节     DATA[3]= *((uint8_t *) (&encoder)+1)
//DATA[4] 编码器原始位置低字节 DATA[4]= *(uint8_t *) (&encoderRaw)
//DATA[5] 编码器原始位置高字节 DATA[5]= *((uint8_t *) (&encoderRaw)+1)
//DATA[6] 编码器零偏低字节     DATA[6]= *(uint8_t *)(&encoderOffset)
//DATA[7] 编码器零偏低字节     DATA[7]= *((uint8_t *)(&encoderOffset)+1)*/
void Motor_MG::getMultiLoopEcd()
{
		if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//单发模式
  can_tx_data[0] = 0x90;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
	  can_update_flag = can_tx_data[0];
	}
}

//读取多圈角度命令（0x92）
///*驱动回复
//DATA[1] 角度低字节 1  DATA[1]= *(uint_8_t *) (&motorAngle)
//DATA[2] 角度字节 2    DATA[2]= *((uint8_t *) (&motorAngle)+1)
//DATA[3] 角度字节 3    DATA[3]= *((uint8_t *) (&motorAngle)+2)
//DATA[4] 角度字节 4    DATA[4]= *((uint8_t *) (&motorAngle)+3)
//DATA[5] 角度字节 5    DATA[5]= *((uint8_t *) (&motorAngle)+4)
//DATA[6] 角度字节 6    DATA[6]= *((uint8_t *) (&motorAngle)+5)
//DATA[7] 角度字节 7    DATA[7]= *((uint8_t *) (&motorAngle)+6)*/
void Motor_MG::getMultiLoopAngle()
{
		if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//单发模式
  can_tx_data[0] = 0x92;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0;
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
  can_update_flag = can_tx_data[0];
	}
}

//读取单圈角度命令（0x94）
///*驱动回复
//DATA[6] 角度低字节 1  DATA[6]= *(uint_8_t *) (&circleAngle)
//DATA[7] 角度高字节 7  DATA[7]= *((uint8_t *) (&circleAngle)+1)*/
void Motor_MG::getSingleLoopAngle()
{
		if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//单发模式
	can_tx_data[0] = 0x94;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0;
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
	  can_update_flag = can_tx_data[0];
	}
}

//读取电机的状态1（温度、电压）和错误状态标志(0x9A)
///*回复数据域
//DATA[1] 电机温度     DATA[1] = *(uint8_t *)(temperature)
//DATA[2] NULL 0x00
//DATA[3] 电压低字节 	 DATA[3] = *(uint8_t *)(&voltage)
//DATA[4] 电压高字节   DATA[4] = *((uint8_t *)(&voltage)+1)
//DATA[5] NULL 0x00
//DATA[6] NULL 0x00
//DATA[7] 错误状态字节 DATA[7] = *(uint8_t *)(errorState)*/
void Motor_MG::getState1andErrorState()
{
	if(can_update_flag == 0)
	{

//	can_device.can_tx_data->flag = 0;//单发模式
	can_tx_data[0] = 0x9A;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}

//读取电机状态2（温度、电压、转速、编码器位置）（0x9C）
///*回复数据域
//DATA[1] 电机温度 DATA[1] = (uint8_t)(temperature)
//DATA[2] 转矩电流低字节 DATA[2] = (uint8_t)(iq)
//DATA[3] 转矩电流高字节 DATA[3] = ((uint8_t)(iq)+1
//DATA[4] 电机速度低字节 DATA[4] = (uint8_t)(speed)
//DATA[5] 电机速度高字节 DATA[5] = (uint8_t)(speed>>8)
//DATA[6] 编码器位置低字节 DATA[6] = (uint8_t)(encoder)
//DATA[7] 编码器位置高字节 DATA[7] = (uint8_t)(encoder)+1*/
void Motor_MG::getState2()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x9C;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
  can_update_flag = can_tx_data[0];
	}
}

//读取电机状态3（温度、相电流）（0x9D）
///*回复数据域
//DATA[1] 电机温度       DATA[1] = (uint8_t)(temperature)
//DATA[2] A 相电流低字节 DATA[2] = (uint8_t)(iA)
//DATA[3] A 相电流高字节 DATA[3] = (uint8_t)(iA)+1
//DATA[4] B 相电流低字节 DATA[4] = (uint8_t)(iB)
//DATA[5] B 相电流高字节 DATA[5] = (uint8_t)(iB)+1
//DATA[6] C 相电流低字节 DATA[6] = (uint8_t)(iC)
//DATA[7] C 相电流高字节 DATA[7] = (uint8_t)(iC)+1*/
void Motor_MG::getState3()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x9D;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}

/********************************************************************************/
/**********************************Set写入命令***********************************/
/********************************************************************************/
//写入PID参数到RAM（0x31）
void Motor_MG::setPIDParams_RAM(Motor_MG_PID_Params_t _PIDParams)
{
	if(can_update_flag == 0)
	{
	pid_params = _PIDParams;
	can_tx_data[0] = 0x31;
  can_tx_data[1] = 0;
  can_tx_data[2] = (uint8_t)(pid_params.PosKp_change);
  can_tx_data[3] = (uint8_t)(pid_params.PosKi_change);
  can_tx_data[4] = (uint8_t)(pid_params.SpdKp_change); 
  can_tx_data[5] = (uint8_t)(pid_params.SpdKi_change);
  can_tx_data[6] = (uint8_t)(pid_params.TqKp_change);
  can_tx_data[7] = (uint8_t)(pid_params.TqKi_change);
		can_update_flag = can_tx_data[0];
	}
}
//写入PID参数到ROM（0x32）
void Motor_MG::setPIDParams_ROM(Motor_MG_PID_Params_t _PIDParams)
{
	if(can_update_flag == 0)
	{
	pid_params = _PIDParams;
	can_tx_data[0] = 0x32;
  can_tx_data[1] = 0;
  can_tx_data[2] = (uint8_t)(pid_params.PosKp_change);
  can_tx_data[3] = (uint8_t)(pid_params.PosKi_change);
  can_tx_data[4] = (uint8_t)(pid_params.SpdKp_change); 
  can_tx_data[5] = (uint8_t)(pid_params.SpdKi_change);
  can_tx_data[6] = (uint8_t)(pid_params.TqKp_change);
  can_tx_data[7] = (uint8_t)(pid_params.TqKi_change);
   can_update_flag = can_tx_data[0];
	}
}
//写入加速度到RAM(0x34)

//写入编码器多圈值到 ROM 作为电机零点命令（0x91）
void Motor_MG::setStartPosition_ROM(int32_t _ecdOffset)
{
	if(can_update_flag == 0)
	{
	params.ecd_offset = _ecdOffset;
	can_tx_data[0] = 0x91;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0;
  can_tx_data[5] = 0;
  can_tx_data[6] = (uint8_t)(params.ecd_offset);
  can_tx_data[7] = (uint8_t)(params.ecd_offset>>8);
		can_update_flag = can_tx_data[0];
	}
}
//写入编码器当前多圈位置到 ROM 作为电机零点命令（0x19）需掉电(频繁写入影响寿命)
void Motor_MG::setCurruntEcdasStartPosition()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x19;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}

/*******************************************************/
/********************Command控制命令********************/
/*******************************************************/
//关闭电机输出，同时清除电机运行状态和之前接收的控制指令(0x80)
void Motor_MG::cmdMotorShutOff()
{
		if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x80;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}
//停止电机命令，不清除电机运行状态和之前接收的控制指令(0x81)
void Motor_MG::cmdMotorStop()
{
	if(can_update_flag == 0)
	{
  can_tx_data[0] = 0x81;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
  can_update_flag = can_tx_data[0];
	}
}
//电机运行命令，恢复电机停止命令前的控制方式(0x88)
void Motor_MG::cmdMotorRun()
{
  if(can_update_flag == 0)
	{
    can_tx_data[0] = 0x88;
    can_tx_data[1] = 0;
    can_tx_data[2] = 0;
    can_tx_data[3] = 0;
    can_tx_data[4] = 0; 
    can_tx_data[5] = 0;
    can_tx_data[6] = 0;
    can_tx_data[7] = 0;
			  can_update_flag = can_tx_data[0];
	}
}
//转矩闭环控制命令（0xA1）/单位（0.01A/LSB）
void Motor_MG::cmdTqControl(int16_t _tqControl)
{
	if(can_update_flag == 0)
	{
	params.tq_control = _tqControl;
	can_tx_data[0] = 0xA1;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = (uint8_t)(params.tq_control); 		//转矩电流控制值低字节
  can_tx_data[5] = (uint8_t)(params.tq_control>>8);	//转矩电流控制值高字节
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}
//速度闭环控制（0xA2）/单位（0.01dps/LSB）
void Motor_MG::cmdSpeedControl(int32_t _speedControl)
{
	if(can_update_flag == 0)
	{
	params.speed_control = _speedControl;
	can_tx_data[0] = 0xA2;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = * (uint8_t *)(&params.speed_control); //速度控制低字节
  can_tx_data[5] = *((uint8_t *)(&params.speed_control)+1);
  can_tx_data[6] = *((uint8_t *)(&params.speed_control)+2);
  can_tx_data[7] = *((uint8_t *)(&params.speed_control)+3);//速度控制高字节
		can_update_flag = can_tx_data[0];
	}
}
//绝对位置闭环控制（0xA4）/单位（1dps/LSB,0.01degree/LSB）
void Motor_MG::cmdAngleControl(int16_t _maxSpeed,int32_t _angleControl)
{
//	can_device.can_tx_data->flag = 0;//单发模式
	  if(can_update_flag == 0)
	{
	params.max_speed = _maxSpeed;
	params.angle_control = _angleControl;
	can_tx_data[0] = 0xA4;
  can_tx_data[1] = 0x00;
  can_tx_data[2] = (uint8_t)(params.max_speed);		//速度限制低字节
  can_tx_data[3] = (uint8_t)(params.max_speed>>8);//速度限制高字节
  can_tx_data[4] = (uint8_t)(params.angle_control);		//位置控制低字节
  can_tx_data[5] = (uint8_t)(params.angle_control>>8);	//位置控制
  can_tx_data[6] = (uint8_t)(params.angle_control>>16);//位置控制
  can_tx_data[7] = (uint8_t)(params.angle_control>>24);//位置控制高字节
  can_update_flag = can_tx_data[0];
	}
}
//位置闭环控制命令 6 增量式（0xA8）/单位（1dps/LSB,0.01degree/LSB）
void Motor_MG::cmdDeltaAngleControl(int16_t _maxSpeed,int32_t _delta_angleControl)
{
	if(can_update_flag == 0)
	{
	params.max_speed = _maxSpeed;
	params.delta_angle_control = _delta_angleControl;
	can_tx_data[0] = 0xA8;
  can_tx_data[1] = 0x00;
  can_tx_data[2] = (uint8_t)(params.max_speed);		//速度限制低字节
  can_tx_data[3] = (uint8_t)(params.max_speed>>8);//速度限制高字节
  can_tx_data[4] = (uint8_t)(params.delta_angle_control);		//位置控制低字节
  can_tx_data[5] = (uint8_t)(params.delta_angle_control>>8);	//位置控制
  can_tx_data[6] = (uint8_t)(params.delta_angle_control>>16);//位置控制
  can_tx_data[7] = (uint8_t)(params.delta_angle_control>>24);//位置控制高字节
		can_update_flag = can_tx_data[0];
	}
}

void  Motor_MG::MG_motor_reply()
{
	if(can_rx_data.data[0] == can_update_flag)
 {
	 can_update_flag = 0;
	 
    switch(can_rx_data.data[0]){
	  case 0x30://读取PID命令
	  pid_params.PosKi_now = can_rx_data.data[2];//*1/256
	  pid_params.PosKi_now = can_rx_data.data[3];//*0.5/256
	  pid_params.SpdKp_now = can_rx_data.data[4];//*0.05/256
	  pid_params.SpdKi_now = can_rx_data.data[5];//*0.005/256
	  pid_params.TqKi_now =  can_rx_data.data[6];//*0.5/256
	  pid_params.TqKi_now =  can_rx_data.data[7];//*0.0005/256
	  
	  break;
 
	  case 0x90://读取多圈编码器位置，为编码器原始位置减去编码器多圈零偏（初始位置）后的值
	  params.multiturn_position = ((uint16_t)(can_rx_data.data[2] | can_rx_data.data[3] << 8));	  
	  //读取多圈编码器位置，不减零偏
	  params.multiturn_position_raw = ((uint16_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8));
	  //读取多圈编码器零偏位置
	  params.ecd_offset = ((uint16_t)(can_rx_data.data[6] | can_rx_data.data[7] << 8));
	  break;	  

	  case 0x92://读取多圈角度命令
	  params.multiturn_angle = ((int64_t)(can_rx_data.data[1] | can_rx_data.data[2] << 8 
	                                     | can_rx_data.data[3] << 16 | can_rx_data.data[4] << 24 
		                                  |  (int64_t)can_rx_data.data[5] << 32 | (int64_t)can_rx_data.data[6] << 40 
		                                  |  (int64_t)can_rx_data.data[7] << 48 ));
//		 *(uint8_t*)((&params.multiturn_angle)) = can_rx_data.data[1];
//	    *(uint8_t*)((&params.multiturn_angle)+1) = can_rx_data.data[2];
//		 *(uint8_t*)((&params.multiturn_angle)+2) = can_rx_data.data[3];
//		 *(uint8_t*)((&params.multiturn_angle)+3) = can_rx_data.data[4];
//		 *(uint8_t*)((&params.multiturn_angle)+4) = can_rx_data.data[5];
//		 *(uint8_t*)((&params.multiturn_angle)+5) = can_rx_data.data[6];
//		 *(uint8_t*)((&params.multiturn_angle)+6) = can_rx_data.data[7];
//	  if(params.multiturn_angle < 0)
//	  {
//		  crossline_flag = 1;	  
//	  }
	  break;
	  
		case 0x94:
			params.singleturn_angle=((uint16_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8));
		
		break;
	  
		case 0x9C://读取电机的状态2
	  params.temperate = can_rx_data.data[1];
//		params.errorState = can_rx_data.data[7];
//DATA[3] 电压低字节 	 DATA[3] = *(uint8_t *)(&voltage)
//DATA[4] 电压高字节   DATA[4] = *((uint8_t *)(&voltage)+1)
		break;
		  
     }
	}
}

/***********************************************************************
** 函 数 名： updateMotorMeasurement()
** 函数说明： 从CAN数据更新电机测量信息
**---------------------------------------------------------------------
** 输入参数： 电机测量数据指针、电机参数指针、CAN数据包指针、耗时（us）
** 返回参数： 无
***********************************************************************/
void Motor_MG::updateMotorMeasurement()
{
  // 时间戳
	static bool first_update = 1;
  // 时间戳
  com_msr.timestamp = msr.timestamp = micros();
}

void Motor_MG::updateMeasurement()
{
  updateMotorMeasurement();
}

void Motor_MG::init(void)
{
//  params.half_max_value_ecd = params.max_value_ecd / 2;
}

bool Motor_MG::update(timeus_t dT_us)
{
  updateMotorMeasurement();
  publishMeasurement();
  return true;
}

void Motor_MG::uninit(void) {}





















