/***************************************************************************
**   					             大连理工大学 凌BUG战队
**   					               凌以青云，翥以极心!
**-------------------------------------------------------------------------
** 项    目：   robo_template
** 文 件 名：   SongPlayerTask.cpp
** 文件说明：   蜂鸣器任务
**-------------------------------------------------------------------------
**						*修订*
**	*版本*							*修改内容*							*修改人*      			 *日期*
**	 1.0							   初始版本						     季献淏     	     2022-07-18
**	 1.1							   补充注释						     赵钟源     	     2022-12-10
***************************************************************************/

#include "SongPlayerTask.h"

#include "/Robot/Robot.h"

/***********************************************************************
** 函 数 名： SongPlayerTask::init
** 函数说明： 初始化
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void SongPlayerTask::init(void)
{
  bzply_n = 0;
  bzply_count = 1;
  song_end = false;
  buzzer_state = PLAYING_STOP;

  robot.helper.setBuzzerOff();
}
/***********************************************************************
** 函 数 名： SongPlayerTask::update
** 函数说明： 更新任务后注销任务
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 无
***********************************************************************/
void SongPlayerTask::update(timeus_t dT_us)
{
  song_end = playSong();
  if(song_end)
  {
    selfKill();
  }
}
/***********************************************************************
** 函 数 名： SongPlayerTask::playSong
** 函数说明： 播放歌曲
**---------------------------------------------------------------------
** 输入参数： 无
** 返回参数： 播放成功返回true，否则false
***********************************************************************/
bool SongPlayerTask::playSong(void)
{
  uint8_t off_delay, on_delay, level, step;

  off_delay = ((song[bzply_n] & 0xF000) >> 12);
  on_delay = ((song[bzply_n] & 0x0F00) >> 8) * 6;
  level = ((song[bzply_n] & 0x00F0) >> 4);
  step = (song[bzply_n] & 0x000F) - 1;

  if(bzply_count < on_delay)
  {
    bzply_count++;
    robot.helper.setBuzzerFrequence(music_steps[level][step]);//蜂鸣器播放音乐
  }
  else if(bzply_count < (on_delay + off_delay))
  {
    bzply_count++;
    robot.helper.setBuzzerOff();
  }
  else
  {
    bzply_count = 1;
    bzply_n++;
    if(bzply_n >= song_length)
    {
      return true;
    }
  }
  return false;
}

void SongPlayerTask::uninit(void)
{

}
