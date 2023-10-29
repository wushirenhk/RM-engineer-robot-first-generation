#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "main.h"
#include "arm_math.h"
#include "Task.h"
#include "CommonMath.h"

typedef enum
{
  PID_DELTA = 0,//增量式
  PID_ABSOLUTE = 1//位置式（绝对式）
} PID_Type;

//PID运算时的系数
typedef struct _PID_Params_t
{

  PID_Type type_selection;            // 类别设置

  float ki_able_error_range;          // ki作用域，设为0关闭此功能
  float kd_able_error_range;          // kd作用域，设为0关闭此功能

  float kp;                           // 比例系数
  float ki;                           // 积分系数
  float kd_fb;                        // 微分先行系数
  float kd_ex;                        // 微分系数
  float k_ff;                         // 前馈系数

  float max_out_value;                // 输出限幅
  float min_out_value;                // 输出限幅
  bool limit_output;                  // 开启输出限幅

  float max_integral;                 // 积分限幅
  float min_integral;                 // 积分限幅
  bool limit_integral;                // 开启积分限幅

  cmt::AdvancedAdd_Base<float> *add;       // PID中的加减法计算方式，比如用于应对极坐标在±PI位置的跳变
  
  float interval;

} PID_Params_t;

typedef struct _PID_Data_t
{

  float	expect;                       // 期望
  float d_expect;                     // 期望增量
  float dd_expect;                    // 期望增量的增量
  float last_expect;                  // 上次期望
  float last_d_expect;                // 上次（期望增量）

  float feedback;                     // 反馈
  float d_feedback;                   // 反馈增量
  float dd_feedback;                  // 反馈增量的增量
  float last_feedback;                // 上次反馈
  float last_d_feedback;              // 上次（反馈增量）

  float feedforward;                  // 前馈

  float error;                        // 误差
  float last_error;                   // 上次误差

  float last_dT_s;                    // 上次时间增量
  float last_freq;                    // 上次频率

  float integral;                     // 误差积分

  float out;                          // 输出值

} PID_Data_t;


class PID_Controller//类中主要是结构体PID_Data_t和PID_Params_t
{
public:
  PID_Controller();//构造函数
  PID_Controller(PID_Type _type_selection,//有参构造函数
                 cmt::AdvancedAdd_Base<float> *adder,
                 float _kp, float _ki,
                 float _kd_fb, float _kd_ex, float _k_ff,
                 float _max_out_value, float _min_out_value, bool _limit_output,
                 float _max_integral, float _min_integral, bool _limit_integral,
                 float _interval,
                 float _ki_able_error_range = 0,
                 float _kd_able_error_range = 0
                );
								 
								 
  PID_Controller(PID_Data_t &_pid_data, PID_Params_t &_pid_params);
	//拷贝构造函数
  PID_Controller(const PID_Controller & _pid_controller);

	//调出保护属性pid_data
  PID_Data_t getPidData()
  {
    return pid_data;
  }
	//调出保护属性pid_params
  PID_Params_t getPidParams()
  {
    return pid_params;
  }

  void setParams(PID_Type _type_selection,
                 cmt::AdvancedAdd_Base<float> *adder,
                 float _kp, float _ki,
                 float _kd_fb, float _kd_ex, float _k_ff,
                 float _max_out_value, float _min_out_value, bool _limit_output,
                 float _max_integral, float _min_integral, bool _limit_integral,
                 float _interval,
                 float _ki_able_error_range = 0,
                 float _kd_able_error_range = 0
                );
  void setParams(const PID_Params_t &_pid_params);

  float calculateWithCPU(
    timeus_t dT_us,      //周期（us）
    float _expect,				//期望值
    float _feedback,     //反馈值
    float _feedforward = 0.0f	  //前馈值
  );
		
	//运算PID
  float calculateWithCPU(timeus_t dT_us)      //周期（us）
  {
    return calculateWithCPU(dT_us,
                            pid_data.expect,
                            pid_data.feedback,
                            pid_data.feedforward
                           );
  }
	//设置积分限幅值
  void setIntegralLimit(float min, float max)
  {
    pid_params.min_integral = min;
    pid_params.max_integral = max;
  }
	//设置输出限幅值
  void setOutputLimit(float min, float max)
  {
    pid_params.max_out_value = max;
    pid_params.min_out_value = min;
  }
	//设置期望
  void setExpect(float expect)
  {
    pid_data.expect = expect;
  }
	//设置反馈值
  void setFeedback(float feedback)
  {
    pid_data.feedback = feedback;
  }
	//设置前馈值
  void setFeedforward(float feedforward)
  {
    pid_data.feedforward = feedforward;
  }
	//设置输入（期望+反馈+前馈）
  void setInput(float expect, float feedback, float feedforward)
  {
    pid_data.expect = expect;
    pid_data.feedback = feedback;
    pid_data.feedforward = feedforward;
  }
	//反馈PID输出值
  float getOutput()
  {
    return pid_data.out;
  }
	//pid_data结构体的成员清零
  void clear();
	
	//返回输出限幅后的结果
  float getLimitedOutput()
  {
    return limit(pid_data.out, pid_params.min_out_value, pid_params.max_out_value);
  }
	//更新输出限幅后的结果
  void limitOutput(void)
  {
    pid_data.out = limit(pid_data.out, pid_params.min_out_value, pid_params.max_out_value);
  }

	//返回pid_data的地址
  PID_Data_t *getPIDDataPointer()
  {
    return &pid_data;
  }

	//返回pid_params的地址
  PID_Params_t *getPIDParamsPointer()
  {
    return &pid_params;
  }

protected:
  PID_Data_t pid_data;
  PID_Params_t pid_params;

};

#endif
