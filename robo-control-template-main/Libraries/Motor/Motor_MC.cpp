/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   truck-arm
** 文 件 名：   Motor_MC.cpp
** 文件说明：   脉塔智能电机数据读取处理
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     沈申浩     	     2022-12-22
***************************************************************************/

#include "Motor_MC.h"
#include "Motor.h"
#include "Motor_Backend.h"
#include "CanDevice.h"
#include "arm_math.h"


Motor_MC::Motor_MC(Motor &motor0,
                   CanDevice &_can_device,
                   uint16_t _can_tx_id,
                   uint16_t _can_rx_id,
                   uint8_t _can_tx_data_start_pos,
                   int8_t _motor_type) :
  Motor_Backend(motor0),
  can_device(_can_device)
{
  can_tx_data_start_pos = _can_tx_data_start_pos;//脉塔电机发送所有data，默认为0
  can_rx_data.std_id = _can_rx_id;
  can_tx_id = _can_tx_id;
  can_device.addRxLink(&can_rx_data);
  can_device.addTxLink(_can_tx_id, 8, _can_tx_data_start_pos, can_tx_data, 8);
  motor_type = _motor_type;
}
/********************************************************************************/
/**********************************Get读取命令***********************************/
/********************************************************************************/
////读取多圈编码器位置指令（0x60）
///*回复数据域
//DATA[4] 编码器位置低字节 1 DATA[4] = (uint8_t)(encoder)
//DATA[5] 编码器位置字节 2 DATA[5] = (uint8_t)(encoder>>8)
//DATA[6] 编码器位置字节 3 DATA[6] = (uint8_t)(encoder>>16)
//DATA[7] 编码器位置字节 4 DATA[7] = (uint8_t)(encoder>>24)*/
void Motor_MC::getMultiLoopEcdPosition()
{
		if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//单发模式
  can_tx_data[0] = 0x60;
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
////读取多圈编码器原始位置指令（0x61）
///*回复数据域
//DATA[4] 编码器原始位置字节 1 DATA[4] = (uint8_t)(encoderRaw)
//DATA[5] 编码器原始位置字节 2 DATA[5] = (uint8_t)(encoderRaw>>8)
//DATA[6] 编码器原始位置字节 3 DATA[6] = (uint8_t)(encoderRaw>>16)
//DATA[7] 编码器原始位置字节 4 DATA[7] = (uint8_t)(encoderRaw>>24)*/
void Motor_MC::getMultiLoopEcdRawPosition()
{
	if(can_update_flag == 0)
	{
//	can_device.can_tx_data->flag = 0;//单发模式
  can_tx_data[0] = 0x61;
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
////读取多圈编码器零偏数据命令（0x62）
///*回复数据域
//DATA[4] 编码器零偏字节 1 DATA[4] = (uint8_t)(encoderOffset)
//DATA[5] 编码器零偏字节 2 DATA[5] = (uint8_t)(encoderOffset>>8)
//DATA[6] 编码器零偏字节 3 DATA[6] = (uint8_t)(encoderOffset>>16)
//DATA[7] 编码器零偏字节 4 DATA[7] = (uint8_t)(encoderOffset>>24)*/
void Motor_MC::getMultiLoopEcdOffset()
{
//	can_device.can_tx_data->flag = 0;//单发模式
	if(can_update_flag == 0)
	{
  can_tx_data[0] = 0x62;
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
////读取多圈角度命令（0x92）
///*回复数据域
//DATA[4] 角度低字节 1 DATA[4] = (uint8_t)(motorAngle)
//DATA[5] 角度字节 2 DATA[5] = (uint8_t)(motorAngle>>8)
//DATA[6] 角度字节 3 DATA[6] = (uint8_t)(motorAngle>>16)
//DATA[7] 角度字节 4 DATA[7] = (uint8_t)(motorAngle>>24)
void Motor_MC::getMultiLoopAngle()
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
//读取电机的状态1（温度、电压）和错误状态标志(0x9A)
///*回复数据域
//DATA[0] 命令字节 0x9A
//DATA[1] 电机温度 DATA[1] = (uint8_t)(temperature)
//DATA[2] NULL 0x00
//DATA[3] 抱闸释放指令 DATA[3] = (uint8_t)(RlyCtrlRslt)
//DATA[4] 电压低字节 DATA[4] = (uint8_t)(voltage)
//DATA[5] 电压高字节 DATA[5] = (uint8_t)(voltage>>8)
//DATA[6] 错误状态低字节 1 DATA[6] = (uint8_t)(errorState)
//DATA[7] 错误状态字节 2 DATA[7] = (uint8_t)(errorState>>8)*/
void Motor_MC::getState1andErrorState()
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
//读取电机状态2（温度、转速、编码器位置）（0x9C）
///*回复数据域
//DATA[0] 命令字节 0x9C
//DATA[1] 电机温度 DATA[1] = (uint8_t)(temperature)
//DATA[2] 转矩电流低字节 DATA[2] = (uint8_t)(iq)
//DATA[3] 转矩电流高字节 DATA[3] = (uint8_t)(iq>>8)
//DATA[4] 电机速度低字节 DATA[4] = (uint8_t)(speed)
//DATA[5] 电机速度高字节 DATA[5] = (uint8_t)(speed>>8)
//DATA[6] 电机角度低字节 DATA[6] = (uint8_t)(degree)
//DATA[7] 电机角度高字节 DATA[7] = (uint8_t)(degree>>8)*/
void Motor_MC::getState2()
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
//DATA[0] 命令字节 0x9D
//DATA[1] 电机温度 DATA[1] = (uint8_t)(temperature)
//DATA[2] A 相电流低字节 DATA[2] = (uint8_t)(iA)
//DATA[3] A 相电流高字节 DATA[3] = (uint8_t)(iA>>8)
//DATA[4] B 相电流低字节 DATA[4] = (uint8_t)(iB)
//DATA[5] B 相电流高字节 DATA[5] = (uint8_t)(iB>>8)
//DATA[6] C 相电流低字节 DATA[6] = (uint8_t)(iC)
//DATA[7] C 相电流高字节 DATA[7] = (uint8_t)(iC>>8)*/
void Motor_MC::getState3()
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
//读取当前电机运行模式(0x70)
///*回复数据域
// 电流环模式(0x01)
// 速度环模式(0x02)
// 位置环模式(0x03)
//DATA[0] 命令字节 0x70
//DATA[7] 电机运行模式 DATA[7] = (uint8_t)(runmode)*/
void Motor_MC::getRunMode()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x70;
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
//读取当前电机功率（0x71）/单位（0.1w/LSB）
///*回复数据域 
//DATA[6] 电机运行功率低字节 DATA[6] = (uint8_t)(motorpower)
//DATA[7] 电机运行功率高字节 DATA[7] = (uint8_t)(motorpower>>8))*/
void Motor_MC::getMotorPower()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x71;
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
//获取系统运行时间(0xB1)单位(ms)
///*回复数据域 单位（0.1w/LSB）
//DATA[6] 电机运行功率低字节 DATA[6] = (uint8_t)(motorpower)
//DATA[7] 电机运行功率高字节 DATA[7] = (uint8_t)(motorpower>>8))*/
void Motor_MC::getMotorTimestamp()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0xB1;
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
//通讯ID 0x300多电机指令（0x79），1读CAN ID（0x240 + ID）
void Motor_MC::getMotorCANID()
{
	if(can_update_flag == 0)
	{
	bool wReadWriteFlag = 1;
	can_tx_data[0] = 0x79;
  can_tx_data[1] = 0;
  can_tx_data[2] = wReadWriteFlag;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;
		can_update_flag = can_tx_data[0];
	}
}

//读取PID值（0x30）
void Motor_MC::getPIDParams()
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


/*Set写入命令*/
//写入PID参数到RAM（0x31）
void Motor_MC::setPIDParams_RAM(Motor_MC_PID_Params_t _PIDParams)
{
	if(can_update_flag == 0)
	{
	pid_params = _PIDParams;
	can_tx_data[0] = 0x31;
  can_tx_data[1] = 0;
  can_tx_data[2] = (uint8_t)(pid_params.CurKp_change);
  can_tx_data[3] = (uint8_t)(pid_params.CurKi_change);
  can_tx_data[4] = (uint8_t)(pid_params.SpdKp_change); 
  can_tx_data[5] = (uint8_t)(pid_params.SpdKp_change);
  can_tx_data[6] = (uint8_t)(pid_params.PosKp_change);
  can_tx_data[7] = (uint8_t)(pid_params.PosKi_change);
		can_update_flag = can_tx_data[0];
	}
}
//写入PID参数到ROM（0x32）
void Motor_MC::setPIDParams_ROM(Motor_MC_PID_Params_t _PIDParams)
{
	if(can_update_flag == 0)
	{
	pid_params = _PIDParams;
	can_tx_data[0] = 0x32;
  can_tx_data[1] = 0;
  can_tx_data[2] = (uint8_t)(pid_params.CurKp_change);
  can_tx_data[3] = (uint8_t)(pid_params.CurKi_change);
  can_tx_data[4] = (uint8_t)(pid_params.SpdKp_change); 
  can_tx_data[5] = (uint8_t)(pid_params.SpdKi_change);
  can_tx_data[6] = (uint8_t)(pid_params.PosKp_change);
  can_tx_data[7] = (uint8_t)(pid_params.PosKi_change);
   can_update_flag = can_tx_data[0];
	}
}
//写入编码器多圈值到 ROM 作为电机零点命令（0x63）需掉电
void Motor_MC::setStartPosition_ROM(int32_t _ecdOffset)
{
	if(can_update_flag == 0)
	{
	params.ecd_offset = _ecdOffset;
	can_tx_data[0] = 0x63;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = (uint8_t)(params.ecd_offset); 
  can_tx_data[5] = (uint8_t)(params.ecd_offset>>8);
  can_tx_data[6] = (uint8_t)(params.ecd_offset>>16);
  can_tx_data[7] = (uint8_t)(params.ecd_offset>>24);
		can_update_flag = can_tx_data[0];
	}
}
//写入编码器当前多圈位置到 ROM 作为电机零点命令（0x64）需掉电
void Motor_MC::setCurruntEcdasStartPosition()
{
	if(can_update_flag == 0)
	{
	can_tx_data[0] = 0x64;
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
//通讯ID 0x300多电机指令（0x79），0写CAN ID（1~32）
void Motor_MC::setMotorsCANID(uint16_t _CANID)
{
	if(can_update_flag == 0)
	{
	params.can_tx_id = _CANID;
	bool wReadWriteFlag = 0;
	can_tx_data[0] = 0x79;
  can_tx_data[1] = 0;
  can_tx_data[2] = wReadWriteFlag;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = (uint8_t)(_CANID);
		can_update_flag = can_tx_data[0];
	}
}
/*******************************************************/
/********************Command控制命令*********************/
/*******************************************************/

//关闭电机输出，同时清除电机运行状态，不在任何闭环模式下(0x80)
void Motor_MC::cmdMotorShutOff()
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
//停止电机，将电机速度停下来，并使电机保持不动(0x81)
void Motor_MC::cmdMotorStop()
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
//转矩闭环控制命令（0xA1）/单位（0.01A/LSB）
void Motor_MC::cmdTqControl(int16_t _tqControl)
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
void Motor_MC::cmdSpeedControl(int32_t _speedControl)
{
	if(can_update_flag == 0)
	{
	params.speed_control = _speedControl;
	can_tx_data[0] = 0xA2;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = * (uint8_t *)(&params.speed_control); //位置控制低字节
  can_tx_data[5] = *((uint8_t *)(&params.speed_control)+1);
  can_tx_data[6] = *((uint8_t *)(&params.speed_control)+2);
  can_tx_data[7] = *((uint8_t *)(&params.speed_control)+3);//位置控制高字节
		can_update_flag = can_tx_data[0];
	}
}
//绝对位置闭环控制（0xA4）/单位（1dps/LSB,0.01degree/LSB）
void Motor_MC::cmdAngleControl(int16_t _maxSpeed,int32_t _angleControl)
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
//增量位置闭环控制（0xA8）/单位（1dps/LSB,0.01degree/LSB）
void Motor_MC::cmdDeltaAngleControl(int16_t _maxSpeed,int32_t _delta_angleControl)
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
//复位系统程序(0x76)
void Motor_MC::cmdSystemReset()
{

	can_tx_data[0] = 0x76;
  can_tx_data[1] = 0;
  can_tx_data[2] = 0;
  can_tx_data[3] = 0;
  can_tx_data[4] = 0; 
  can_tx_data[5] = 0;
  can_tx_data[6] = 0;
  can_tx_data[7] = 0;

}

void  Motor_MC::MC_motor_reply()
{
	if(can_rx_data.data[0] == can_update_flag)
 {
	 can_update_flag = 0;
	 
    switch(can_rx_data.data[0]){
	  case 0x30://读取PID命令
	  pid_params.CurKp_now = ((int32_t)(can_rx_data.data[2]));//*1/256
	  pid_params.CurKi_now = ((int32_t)(can_rx_data.data[3]));//*0.5/256
	  pid_params.SpdKp_now = ((int32_t)(can_rx_data.data[4]));//*0.05/256
	  pid_params.SpdKi_now = ((int32_t)(can_rx_data.data[5]));//*0.005/256
	  pid_params.PosKp_now = ((int32_t)(can_rx_data.data[6]));//*0.5/256
	  pid_params.PosKi_now = ((int32_t)(can_rx_data.data[7]));//*0.0005/256
	  
	  break;
 
	  case 0x60://读取多圈编码器位置，为编码器原始位置减去编码器多圈零偏（初始位置）后的值
	  params.multiturn_position = ((int32_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8 
	                           | can_rx_data.data[6] << 16 | can_rx_data.data[7] << 24));
	  break;
	  
	  case 0x61://读取多圈编码器位置，不减零偏
	  params.multiturn_position_raw = ((int32_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8 
	                           | can_rx_data.data[6] << 16 | can_rx_data.data[7] << 24));
	  break;
	  
	  case 0x62://读取多圈编码器零偏位置
	  params.ecd_offset = ((int32_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8 
	                           | can_rx_data.data[6] << 16 | can_rx_data.data[7] << 24));
	  break;	  

	  case 0x92://读取多圈角度命令
	  params.multiturn_angle = ((int32_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8 
	                           | can_rx_data.data[6] << 16 | can_rx_data.data[7] << 24))/100;
	  break;
	  
	  case 0x9C://读取电机的状态1（温度、电压）和错误状态标志
	  params.temperate  = ((uint8_t)(can_rx_data.data[1]));
	  params.tq_control =((int16_t)(can_rx_data.data[2] | can_rx_data.data[3] << 8 ));
	  params.speed_control = ((int16_t)(can_rx_data.data[4] | can_rx_data.data[5] << 8 )); 
	  
	  break;
		  
     }
	}
}


/***********************************************************************
** 函 数 名： updateMotorMeasurement()0
** 函数说明： 从CAN数据更新电机测量信息
**---------------------------------------------------------------------
** 输入参数： 电机测量数据指针、电机参数指针、CAN数据包指针、耗时（us）
** 返回参数： 无
***********************************************************************/
void Motor_MC::updateMotorMeasurement()
{
  // 时间戳
	  static bool first_update = 1;
  // 时间戳
  com_msr.timestamp = msr.timestamp = micros();

}



void Motor_MC::updateMeasurement()
{
  updateMotorMeasurement();
}

void Motor_MC::init(void)
{
//  params.half_max_value_ecd = params.max_value_ecd / 2;
}

bool Motor_MC::update(timeus_t dT_us)
{
  updateMotorMeasurement();
  publishMeasurement();
  return true;
}

void Motor_MC::uninit(void) {}
