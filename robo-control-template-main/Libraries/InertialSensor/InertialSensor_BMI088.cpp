/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   InertialSensor_BMI088.cpp
** 文件说明：   BMI088数据读取
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     季献淏     	   2022-07-09
**	 1.1							   补充注释						     赵钟源     	   2022-12-10
***************************************************************************/

#include "InertialSensor_BMI088.h"
#include "Flash.h"

#define GYRO_FLASH_ADDRESS  ADDR_FLASH_SECTOR_11

extern SPI_HandleTypeDef hspi1;

#define BMI088_ACCEL_LOW HAL_GPIO_WritePin(BMI088_ACCEL_CS1_GPIO_Port, BMI088_ACCEL_CS1_Pin, GPIO_PIN_RESET) // 有效
#define BMI088_ACCEL_HIGH HAL_GPIO_WritePin(BMI088_ACCEL_CS1_GPIO_Port, BMI088_ACCEL_CS1_Pin, GPIO_PIN_SET)
#define BMI088_GYRO_LOW HAL_GPIO_WritePin(BMI088_GYRO_CS1_GPIO_Port, BMI088_GYRO_CS1_Pin, GPIO_PIN_RESET) // 有效
#define BMI088_GYRO_HIGH HAL_GPIO_WritePin(BMI088_GYRO_CS1_GPIO_Port, BMI088_GYRO_CS1_Pin, GPIO_PIN_SET)


InertialSensor_BMI088::InertialSensor_BMI088(InertialSensor &imu0, RotationPreset rotation_preset0) :
  InertialSensor_Backend(imu0),
  rotation_preset(rotation_preset0)
{

}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::init(void)
** 函数说明： 初始化BMI088
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 初始化成功返回ture，否则返回false
***********************************************************************/
bool InertialSensor_BMI088::init(void)
{
  uint8_t res = 0;
  imu.accel_orientation[id] = rotation_preset;
  imu.gyro_orientation[id] = rotation_preset;
  
  accel_lpf = LowPassFilter<Vector3f>(1.0f/400.0f, 40);
  
  flash_readAddress(GYRO_FLASH_ADDRESS+0, (uint32_t *)(&this->gyro_offset.x), 1);
  flash_readAddress(GYRO_FLASH_ADDRESS+4, (uint32_t *)(&this->gyro_offset.y), 1);
  flash_readAddress(GYRO_FLASH_ADDRESS+8, (uint32_t *)(&this->gyro_offset.z), 1);
  
  writeSingleRegAccel(0x7E, 0xB6); // soft_rest
  HAL_Delay(100);
  res = readSingleRegAccel(0x00);
  HAL_Delay(150);
  res = readSingleRegAccel(0x00);
  HAL_Delay(150);
  if(res != 0x1E) return false; // chip id 0x1E

  // ACC_CONF 800Hz, 145Hz
  writeSingleRegAccel(0x40, 0xAB);
  HAL_Delay(10);

  // ACC_RANGE 00-3g;01-6g;02-12g;03-24g
  writeSingleRegAccel(0x41, 0x03);
  HAL_Delay(10);

  // INT1_IO_CONF 0000 1000 INT1 as output pp acitiv_low
  writeSingleRegAccel(0x53, 0x08);
  HAL_Delay(10);

  //INT_MAP_DATA 0000 0100 INT1 drdy interrupt
  writeSingleRegAccel(0x58, 0x04);
  HAL_Delay(10);

  // power config-acitve mode
  writeSingleRegAccel(0x7C, 0x00);
  HAL_Delay(10);

  // power control-acc enable
  writeSingleRegAccel(0x7D, 0x04);
  HAL_Delay(10);

  writeSingleRegGyro(0x14, 0xB6); // soft_rest
  HAL_Delay(100);
  res = readSingleRegGyro(0x00);
  HAL_Delay(150);
  if(res != 0x0F) return false; // chip id 0x0F

  // GYRO_RANGE 00-2000°/s;01-1000°/s;02-500°/s;03-250°/s;04-125°/s
  writeSingleRegGyro(0x0F, 0x00);
  HAL_Delay(10);

  // GYRO_ODR,BANDWIDTH 00-2000Hz,532Hz;01-2000Hz,230Hz;02-1000Hz,116Hz;03-400Hz,47Hz;04-200Hz,23Hz;05-100Hz,12Hz;06-200Hz,64Hz;07-100Hz,32Hz
  writeSingleRegGyro(0x10, 0x02);
  HAL_Delay(10);

  // GYRO_LPM1 normal mode
  writeSingleRegGyro(0x11, 0x00);
  HAL_Delay(10);

  // GYRO_INT_CTRL 1000 0000 enable drdy
  writeSingleRegGyro(0x15, 0x80);
  HAL_Delay(10);

  // INT_IO_CONF 0000 0000 pp active_low
  writeSingleRegGyro(0x16, 0x00);
  HAL_Delay(10);

  // INT_IO_MAP 0000 0001 drdy to INT3
  writeSingleRegGyro(0x18, 0x01);
  HAL_Delay(10);

  imu.inited[id] = true;

  return true;
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::update(void)
** 函数说明： BMI088 updata函数
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 完成更新返回true
***********************************************************************/
bool InertialSensor_BMI088::update(void)
{
  updateAccel();//更新加速度
  updateGyro();//更新角速度

  correctAccel();//修正加速度零漂
  correctGyro();//修正零漂

	rotateAccel();//旋转加速度
	rotateGyro();//旋转角速度

  publishAccel();//上传加速度
  publishGyro();//上传角速度
  return true;
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::readWriteByte()
** 函数说明： SPI发送一个字节并返回一个字节
**---------------------------------------------------------------------
** 输入参数： 寄存器地址,写入数据
** 返回参数： 无
***********************************************************************/
uint8_t InertialSensor_BMI088::readWriteByte(uint8_t tx_data)
{
  uint8_t rx_data;
  HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, 1000);
  return rx_data;
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::writeSingleRegAccel()
** 函数说明： 写入BMI088加速度一个寄存器
**---------------------------------------------------------------------
** 输入参数： 寄存器地址,写入数据
** 返回参数： 无
***********************************************************************/
void InertialSensor_BMI088::writeSingleRegAccel(uint8_t reg, uint8_t data)
{
  BMI088_ACCEL_LOW;
  readWriteByte(reg);
  readWriteByte(data);
  BMI088_ACCEL_HIGH;
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::readSingleRegAccel()
** 函数说明： 读取BMI088加速度一个寄存器
**---------------------------------------------------------------------
** 输入参数： 寄存器地址
** 返回参数： 读取数据
***********************************************************************/
uint8_t InertialSensor_BMI088::readSingleRegAccel(uint8_t reg)
{
  uint8_t temp;
  BMI088_ACCEL_LOW;
  readWriteByte(reg | 0x80);
  readWriteByte(0x55);
  temp = readWriteByte(0x55);
  BMI088_ACCEL_HIGH;
  return temp;
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::readMultiRegAccel()
** 函数说明： 读取BMI088加速度多个寄存器
**---------------------------------------------------------------------
** 输入参数： 起始寄存器地址,读取数据缓冲区,读取个数
** 返回参数： 无
***********************************************************************/
void InertialSensor_BMI088::readMultiRegAccel(uint8_t reg, uint8_t* buf, uint8_t num)
{
  BMI088_ACCEL_LOW;
  readWriteByte(reg | 0x80);
  readWriteByte(0x55);
  while(num != 0)
  {
    *buf = readWriteByte(0x55);
    buf ++;
    num --;
  }
  BMI088_ACCEL_HIGH;
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::writeSingleRegGyro()
** 函数说明： 写入BMI088陀螺仪一个寄存器
**---------------------------------------------------------------------
** 输入参数： 寄存器地址,写入数据
** 返回参数： 无
***********************************************************************/
void InertialSensor_BMI088::writeSingleRegGyro(uint8_t reg, uint8_t data)
{
  BMI088_GYRO_LOW;
  readWriteByte(reg);
  readWriteByte(data);
  BMI088_GYRO_HIGH;
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::readSingleRegGyro()
** 函数说明： 读取BMI088陀螺仪一个寄存器
**---------------------------------------------------------------------
** 输入参数： 寄存器地址
** 返回参数： 读取数据
***********************************************************************/
uint8_t InertialSensor_BMI088::readSingleRegGyro(uint8_t reg)
{
  uint8_t temp;
  BMI088_GYRO_LOW;
  readWriteByte(reg | 0x80);
  //readWriteByte(0x55);
  temp = readWriteByte(0x55);
  BMI088_GYRO_HIGH;
  return temp;
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::readMultiRegGyro()
** 函数说明： 读取BMI088加速度多个寄存器
**---------------------------------------------------------------------
** 输入参数： 起始寄存器地址,读取数据缓冲区,读取个数
** 返回参数： 无
***********************************************************************/
void InertialSensor_BMI088::readMultiRegGyro(uint8_t reg, uint8_t* buf, uint8_t num)
{
  BMI088_GYRO_LOW;
  readWriteByte(reg | 0x80);
  //readWriteByte(0x55);
  while(num != 0)
  {
    *buf = readWriteByte(0x55);
    buf ++;
    num --;
  }
  BMI088_GYRO_HIGH;
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::updateAccel()
** 函数说明： 更新加速度
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void InertialSensor_BMI088::updateAccel(void)
{
  uint8_t buf[BMI088_BUF_LEN];

  readMultiRegAccel(0x12, buf, BMI088_BUF_LEN);

  accel_raw[0] = (int16_t)((buf[1] << 8) | buf[0]);
  accel.x = accel_raw[0] * BMI088_ACCEL_SCALE;
  accel_raw[1] = (int16_t)((buf[3] << 8) | buf[2]);
  accel.y = accel_raw[1] * BMI088_ACCEL_SCALE;
  accel_raw[2] = (int16_t)((buf[5] << 8) | buf[4]);
  accel.z = accel_raw[2] * BMI088_ACCEL_SCALE;

  // 低通滤波
  accel = accel_lpf.calc(1e6f/400.0f, accel);

}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::updateGyro()
** 函数说明： 更新陀螺仪
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void InertialSensor_BMI088::updateGyro(void)
{
  uint8_t buf[BMI088_BUF_LEN];

  readMultiRegGyro(0x02, buf, BMI088_BUF_LEN);

  gyro_raw[0] = (int16_t)((buf[1] << 8) | buf[0]);
  gyro.x = gyro_raw[0] * BMI088_GYRO_SCALE;
  gyro_raw[1] = (int16_t)((buf[3] << 8) | buf[2]);
  gyro.y = gyro_raw[1] * BMI088_GYRO_SCALE;
  gyro_raw[2] = (int16_t)((buf[5] << 8) | buf[4]);
  gyro.z = gyro_raw[2] * BMI088_GYRO_SCALE;


}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::rotateAccel(void)
** 函数说明： 旋转Accel
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void InertialSensor_BMI088::rotateAccel(void)
{
  Vector3f temp_vec = accel;
  if(imu.accel_orientation[id] == ROTATION_NONE)
  {
    accel.x = temp_vec.x;
    accel.y = temp_vec.y;
    accel.z = temp_vec.z;
  }
  else if (imu.accel_orientation[id] == ROTATION_YAW_270)
  {
    accel.x = - temp_vec.y;
    accel.y =   temp_vec.x;
    accel.z =   temp_vec.z;
  }
	else if (imu.accel_orientation[id] == ROTATION_YAW_180)
  {
    accel.x = - temp_vec.x;
    accel.y = - temp_vec.y;
    accel.z = temp_vec.z;
  }
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::rotateGyro(void)
** 函数说明： 旋转Gyro
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void InertialSensor_BMI088::rotateGyro(void)
{
  Vector3f temp_vec = gyro_calib;
  if(imu.gyro_orientation[id] == ROTATION_NONE)
  {
    gyro_calib.x = temp_vec.x;
    gyro_calib.y = temp_vec.y;
    gyro_calib.z = temp_vec.z;
  }
  else if (imu.accel_orientation[id] == ROTATION_YAW_270)
  {
    gyro_calib.x = - temp_vec.y;
    gyro_calib.y =   temp_vec.x;
    gyro_calib.z =   temp_vec.z;
  }
	else if (imu.accel_orientation[id] == ROTATION_YAW_180)
  {
    gyro_calib.x = - temp_vec.x;
    gyro_calib.y = - temp_vec.y;
    gyro_calib.z = temp_vec.z;
  }
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::getTemperature(void)
** 函数说明： 获得BMI088温度
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
float InertialSensor_BMI088::getTemperature(void)
{
  uint8_t buf[2];
  int16_t temp;

  readMultiRegAccel(0x22, buf, 2);
  temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
  if(temp > 1023)
  {
    temp -= 2048;
  }

  return temp * 0.125f + 23.0f;
}

/***********************************************************************
** 函 数 名： InertialSensor_BMI088::calibGyro(uint32_t cnt)
** 函数说明： 计算零漂值
**---------------------------------------------------------------------
** 输入参数： uint32_t cnt
** 返回参数： 无
***********************************************************************/
void InertialSensor_BMI088::calibGyro(uint32_t cnt)
{
  uint32_t n;
  Vector3f sum;
  sum.zero();

  for(n = 0; n < cnt; n++)
  {
		sum += gyro;

    HAL_Delay(3); // For 400Hz IMU
    //if(n%10 == 9)printf("%d%%\r\n", n/10+1);
  }
  //
  gyro_offset = sum / cnt;

  //write flash
  flash_eraseAddress(GYRO_FLASH_ADDRESS, 1);
  flash_eraseAddress(GYRO_FLASH_ADDRESS, 1);
  flash_eraseAddress(GYRO_FLASH_ADDRESS, 1);
  flash_writeAddress(GYRO_FLASH_ADDRESS + 0, (uint32_t *)(&gyro_offset.x), 1);
  flash_writeAddress(GYRO_FLASH_ADDRESS + 4, (uint32_t *)(&gyro_offset.y), 1);
  flash_writeAddress(GYRO_FLASH_ADDRESS + 8, (uint32_t *)(&gyro_offset.z), 1);
}
