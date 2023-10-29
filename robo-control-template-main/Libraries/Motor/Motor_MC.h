#ifndef __MOTOR_MC_H__
#define __MOTOR_MC_H__

#include "stdint.h"
#include "Motor_Backend.h"
#include "Vector3.h"
#include "main.h"
#include "CanDevice.h"
//#include "Task.h"
#include "PID_Controller.h"
//#include "Scheduler.h"


class Robot;

//typedef struct _CAN_Data_MC_Motor_t
//{
//  CAN_Rx_Data_Pack_t data;
//  uint8_t bytes_group; //0 = [0:1], 1 = [2:3], 2 = [4:5], 3 = [6:7]
//} CAN_Data_MC_Motor_t;

enum MCState
{
  MCSTATE_INITING = 0,
  MCSTATE_NORMAL =  1,
  MCSTATE_LOCK =    2,
  MCSTATE_DISABLE = 3
};

typedef struct _Motor_MC_PID_Params_t
{
	uint8_t CurKp_change;
	uint8_t CurKi_change;
	uint8_t SpdKp_change;
	uint8_t SpdKi_change;
	uint8_t PosKp_change;
	uint8_t PosKi_change;
   uint8_t CurKp_now;
	uint8_t CurKi_now;
	uint8_t SpdKp_now;
	uint8_t SpdKi_now;
	uint8_t PosKp_now;
	uint8_t PosKi_now;
}Motor_MC_PID_Params_t;


typedef struct _Motor_MC_Params_t
{
	/***MC电机参数***/
	/***↓***/
	int32_t ecd_offset;		//电机编码器多圈值偏移值，用于确定零点
	int16_t tq_control;		//转矩电流输出 单位（0.01A/LSB）
	int32_t speed_control;	//速度闭环  电机输出轴速度 单位（0.01dps/LSB）
	int32_t angle_control;  //绝对位置闭环控制 单位（0.01degree/LSB）
	int32_t delta_angle_control;//增量位置闭环增量角度 单位（0.01degree/LSB）
	int16_t max_speed;      //速度限制
	
	int32_t multiturn_position;//多圈编码器位置，减去零偏
	int32_t multiturn_position_raw;//多圈编码器位置，不减零偏
	int32_t multiturn_angle;//多圈角度，不减零偏
	
	 uint8_t temperate; // 温度
	 int32_t last_angle;
	/***↑***/
	/***MC电机参数***/
	
	
  float reduction_ratio; // 减速比 = 转子转速/输出轴转速
  float output_radius; // 电机输出轴所连接同步轮半径，用于计算线速度
  int8_t direction; // 方向，与实际坐标轴同向为1，否则为-1
  int16_t max_value_ecd; // 电机编码器最大的数值
  int16_t half_max_value_ecd; // 电机编码器最大的数值
  int16_t offset_ecd; // 电机编码器偏移值（原始数值）
  uint16_t can_tx_id; // CAN总线发送给电机用的ID
  uint16_t can_rx_id; // CAN总线电机发送给开发板用的ID
  uint8_t can_tx_data_start_pos; // CAN总线发送数据起始下标
  uint8_t canx; // CAN总线标号
  float interval; // 编码器数据解算频率
} Motor_MC_Params_t;

typedef struct _Motor_MC_Measurement_t
{
  // 时间戳
  timeus_t timestamp; // 本次时间戳

  // 原始电机数据
  int16_t ecd; // 当前编码器数值
  int16_t speed_rpm; // 当前转速（rpm）
  int16_t given_current; // 电流
  uint8_t temperate; // 温度
  int16_t last_ecd; // 上次编码器数值

  // 初步计算的电机数据
  int16_t delta_ecd; // 编码器增量
  int32_t round_cnt; // 圈数
  int32_t last_round_cnt; // 上次圈数
  int32_t delta_round_cnt; // 圈数增量
  // int32_t total_ecd; // 总计的编码器数值（可能会溢出）

} Motor_MC_Measurement_t;


// BMI088 backend类
class Motor_MC : public Motor_Backend
{
public:
   friend class Motor_Backend;
	friend class Motor_MC_ControlTask;
   friend class Arm_ControlTask;

  Motor_MC(Motor &motor0,
           CanDevice &_can_device,
           uint16_t _can_tx_id,
           uint16_t _can_rx_id,
           uint8_t _can_tx_data_start_pos,
           int8_t _motor_type);
	
/********************脉塔智能伺服电机控制****************/
/***********************CAN发送指令**********************/
	/*Get读取命令*/
	void getPIDParams();	//读取 PID 参数命令（0x30)
	void getAccel();			//读取加速度
	void getMultiLoopEcdPosition();			//读取多圈编码器位置指令（0x60）
	void getMultiLoopEcdRawPosition();	//读取多圈编码器原始位置指令（0x61）
	void getMultiLoopEcdOffset();	//读取多圈编码器零偏数据命令（0x62）
	void getMultiLoopAngle();			//读取当前电机的多圈绝对角度值(0x92)
	void getState1andErrorState();//读取电机的状态1（温度、电压）和错误状态标志(0x9A)
	void getState2();							//读取电机状态2（温度、转速、编码器位置）（0x9C）
	void getState3();							//读取电机状态3（温度、相电流）（0x9D）
	void getRunMode();//读取当前电机运行模式(0x70)1.电流环2.速度环3.位置环
	void getMotorPower();//读取当前电机功率（0x71）/单位（0.1w/LSB）
	void getMotorTimestamp();//获取系统运行时间(0xB1)单位(ms)
	void getMotorCANID();//通讯ID 0x300多电机指令（0x79），1读CAN ID返回值（0x240 + ID）
	
	/*Set写入命令*/
	void setPIDParams_RAM(Motor_MC_PID_Params_t _PIDParams);		//写入PID参数到RAM（0x31）
	void setPIDParams_ROM(Motor_MC_PID_Params_t _PIDParams);		//写入PID参数到ROM(0x32)
	void setStartPosition_ROM(int32_t _ecdOffset);//写入编码器多圈值到 ROM 作为电机零点命令（0x63）
	void setCurruntEcdasStartPosition();//写入编码器当前多圈位置到 ROM 作为电机零点命令（0x64）
	void setMotorsCANID(uint16_t _CANID);//通讯ID 0x300多电机指令（0x79），0写CAN ID（1~32）
	/*Command控制命令*/
	
	void cmdMotorShutOff();	//关闭电机输出，同时清除电机运行状态，不在任何闭环模式下(0x80)
	void cmdMotorStop();		//停止电机，将电机速度停下来，并使电机保持不动(0x81)
	void cmdTqControl(int16_t _tqControl);//转矩闭环控制命令（0xA1）/单位（0.01A/LSB）
	void cmdSpeedControl(int32_t _speedControl);//速度闭环控制（0xA2）/单位（0.01dps/LSB）
	void cmdAngleControl(int16_t _maxSpeed,int32_t _angleControl);//绝对位置闭环控制（0xA4）/单位（1dps/LSB,0.01degree/LSB）
	void cmdDeltaAngleControl(int16_t _maxSpeed,int32_t _angleControl);//增量位置闭环控制（0xA8）/单位（1dps/LSB,0.01degree/LSB）
 	void cmdSystemReset();//复位系统程序(0x76)
	
/***********************电机回复解包**********************/
  void  MC_motor_reply();//回复函数
  uint8_t can_update_flag = 0;//CAN更新标识
  uint8_t angle_update_flag = 1;//底部电机角度更新标识
  uint8_t reset_flag = 0;
  uint8_t PIDchange_flag = 0;  

  void updateMotorMeasurement();

  virtual bool update(timeus_t dT_us); // 更新电机输出
  virtual void init(void); // 初始化变量
  virtual void uninit(void); // 反初始化

  virtual void updateMeasurement(void);

  void setParams(Motor_MC_Params_t _params)
  {
    params = _params;
    params.half_max_value_ecd = params.max_value_ecd / 2;
  }
  Motor_MC_Measurement_t getMeasurement()
  {
    return msr;
  }
	
protected:
    
  void syncCANData()
  {
    can_tx_data[0] = motor_input >> 8;
    can_tx_data[1] = motor_input;
  }
  Motor_MC_PID_Params_t pid_params;
  Motor_MC_Measurement_t msr;
  Motor_MC_Params_t params;
  uint8_t can_tx_data[8];
  int16_t motor_input;
  uint8_t can_tx_data_start_pos;
	uint8_t reset_switch;//复位开关
	bool isHomed;
	bool first_update = 1;
  CAN_Rx_Data_Pack_t can_rx_data;
  uint16_t can_tx_id;
  CanDevice &can_device;
};

#endif
