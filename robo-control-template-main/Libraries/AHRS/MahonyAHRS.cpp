/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   MahonyAHRS.cpp
** 文件说明：   姿态解算
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     季献淏     	   2022-07-18
**   1.1              修改算法下饭操作           林子涵          2022-08-01
**	 1.2							   补充注释						     赵钟源     	   2022-12-10
***************************************************************************/

#include "MahonyAHRS.h"
#include "Quaternion.h"
#include "UARTDriver.h"
#include "arm_math.h"
#include "CmsisCompatibility.h"

/***********************************************************************
** 函 数 名： MahonyAHRS::MahonyAHRS()
** 函数说明： 构造函数
**---------------------------------------------------------------------
** 输入参数： 系数two_kp0，two_ki0
** 返回参数： 无
***********************************************************************/
MahonyAHRS::MahonyAHRS(float two_kp0, float two_ki0):sample_freq(SAMPLE_FREQ_DEF)
{
  two_ki = two_ki0;
  two_kp = two_kp0;

  integral_fb.zero();

  imu_quat.w = 1.0f;
  imu_quat.x = .0f;
  imu_quat.y = .0f;
  imu_quat.z = .0f;
  
}

void MahonyAHRS::init(void)
{

}
/***********************************************************************
** 函 数 名： MahonyAHRS::update()
** 函数说明： Mahony算法计算出四元数
**---------------------------------------------------------------------
** 输入参数： 三轴加速度，三轴角速度
** 返回参数： 无
***********************************************************************/
void MahonyAHRS::update(Vector3f accel, Vector3f gyro)
{
  float recipNorm;
  Vector3f halfv, halfe, q_temp;
	// accel 为非零向量时方可进入
  if(!accel.isZero())
  {
    accel.normalize(); // 标准化
		
    // 估计重力方向和垂直于磁通量的矢量
    halfv.x = imu_quat.x * imu_quat.z - imu_quat.w * imu_quat.y;
    halfv.y = imu_quat.w * imu_quat.x + imu_quat.y * imu_quat.z;
    halfv.z = imu_quat.w * imu_quat.w - 0.5f + imu_quat.z * imu_quat.z;

		// 误差是估计的重力方向与测量的重力方向的向量积之和
    halfe.x = accel.y * halfv.z - accel.z * halfv.y;
    halfe.y = accel.z * halfv.x - accel.x * halfv.z;
    halfe.z = accel.x * halfv.y - accel.y * halfv.x;

    if(two_ki > 0.0f)
    {
      integral_fb.x += two_ki * halfe.x * (1.0f / sample_freq);
      integral_fb.y += two_ki * halfe.y * (1.0f / sample_freq);
      integral_fb.z += two_ki * halfe.z * (1.0f / sample_freq);
      gyro.x += integral_fb.x;
      gyro.y += integral_fb.y;
      gyro.z += integral_fb.z;
    }
    else
    {
      integral_fb.zero();
    }
    gyro.x += two_kp * halfe.x;
    gyro.y += two_kp * halfe.y;
    gyro.z += two_kp * halfe.z;
  }

  gyro.x *= (0.5f * (1.0f / sample_freq));
  gyro.y *= (0.5f * (1.0f / sample_freq));
  gyro.z *= (0.5f * (1.0f / sample_freq));

  q_temp.x = imu_quat.w;
  q_temp.y = imu_quat.x;
  q_temp.z = imu_quat.y;

  imu_quat.w += (-q_temp.y * gyro.x - q_temp.z * gyro.y - imu_quat.z * gyro.z);
  imu_quat.x += ( q_temp.x * gyro.x + q_temp.z * gyro.z - imu_quat.z * gyro.y);
  imu_quat.y += ( q_temp.x * gyro.y - q_temp.y * gyro.z + imu_quat.z * gyro.x);
  imu_quat.z += ( q_temp.x * gyro.z + q_temp.y * gyro.y - q_temp.z * gyro.x);

  imu_quat.normalize();
  
  // pitch
  imu_euler.x = asinf(-2.0f * (imu_quat.x * imu_quat.z - imu_quat.w * imu_quat.y));
  
  // roll
  imu_euler.y = atan2f(2.0f * (imu_quat.w * imu_quat.x + imu_quat.y * imu_quat.z), 2.0f * (imu_quat.w * imu_quat.w + imu_quat.z * imu_quat.z) - 1.0f); 
  
  // yaw
  imu_euler.z = atan2f(2.0f * (imu_quat.w * imu_quat.z + imu_quat.x * imu_quat.y), 2.0f * (imu_quat.w * imu_quat.w + imu_quat.x * imu_quat.x) - 1.0f); 

  imu_coordinate_system = imu_quat.toRotationMatrix();

}
