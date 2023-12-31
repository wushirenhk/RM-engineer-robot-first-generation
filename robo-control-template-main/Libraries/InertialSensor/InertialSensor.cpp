/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   InertialSensor.cpp
** 文件说明：   IMU数据处理
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     季献淏     	   2022-07-09
**	 1.1							   补充注释						     赵钟源     	   2022-12-10
***************************************************************************/

#include "InertialSensor.h"
#include "InertialSensor_BMI088.h"
#include "InertialSensor_Backend.h"
#include "Error.h"

InertialSensor::InertialSensor()
{
  backend_count = 0;
}

/***********************************************************************
** 函 数 名： InertialSensor::addBackend()
** 函数说明： 添加惯性传感器backend对象
**---------------------------------------------------------------------
** 输入参数： 惯性传感器backend对象
** 返回参数： 是否添加成功，成功则返回backend下标，否则返回-1
***********************************************************************/
uint8_t InertialSensor::addBackend(InertialSensor_Backend *backend)
{
  if(!backend || backend_count >= MAX_INERTIALSENSOR_BACKEND_NUM)
  {
#ifdef DEBUG_MODE
    error.error_code.inertial_sensor_backend_id_out_of_range = 1;
    error.Error_Handler();
#endif
  }
  backend->id = backend_count;
  backends[backend_count++] = backend;
  backend->init();
  return backend_count - 1;
}

/***********************************************************************
** 函 数 名： InertialSensor::detectBackend()
** 函数说明： 手动或自动检测惯性传感器并添加backend
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void InertialSensor::detectBackend(void)
{
//  addBackend(new InertialSensor_BMI088(*this, ROTATION_YAW_270)); // BMI088 无旋转方向
}

void InertialSensor::init()
{
  detectBackend();
}

/***********************************************************************
** 函 数 名： InertialSensor::update()
** 函数说明： 轮询每个惯性传感器的更新函数，并判断数据是否有效
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void InertialSensor::update()
{
  for(uint16_t i = 0; i < backend_count; i ++)
  {
    valid[i] = backends[i]->update();
  }
}

/***********************************************************************
** 函 数 名： InertialSensor::getTemperature
** 函数说明： 查询对应ID的惯性传感器的温度
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
float InertialSensor::getTemperature(uint16_t id)
{
  return backends[id]->getTemperature();
}

/***********************************************************************
** 函 数 名： InertialSensor::calibGyro
** 函数说明： 计算对应ID的惯性传感器累加cnt次后获得的零漂值
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void InertialSensor::calibGyro(uint16_t id, uint32_t cnt)//默认ID为0，cnt为10000
{
  backends[id]->calibGyro(cnt);
}
