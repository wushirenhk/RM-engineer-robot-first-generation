//#include "PID_Robot.h"

//PID_Robot::PID_Robot()
//{
//  pid_tree.pid_gyrotemp.pid_data = {.type_selection = PID_ABSOLUTE,
//                                    .expect = 4500,
//                                    .kp = 1, .ki = 0,
//                                    .kd_fb = 0,.kd_ex = 0, .k_ff = 0,
//                                    .max_out_value = 700, .min_out_value = 100,
//                                    .max_integral = 0, .min_integral = 0
//                                   };
//}

// PID_Robot a;

// PID_Robot a;

//bool PID_Robot::initPID(void)//初始化PID,填充结构体参数
//{

//  //陀螺仪加热
//  Pid_gyrotemp.type_selection = 0;
//  Pid_gyrotemp.Ref = 4500.0f;
//  Pid_gyrotemp.feedback = 0.0f;
//  Pid_gyrotemp.Error = 0.0f ;
//  Pid_gyrotemp.DError = 0.0f ;
//  Pid_gyrotemp.DDError = 0.0f ;
//  Pid_gyrotemp.PreError = 0.0f ;
//  Pid_gyrotemp.PreDError = 0.0f ;
//  Pid_gyrotemp.kp = 360.0f;
//  Pid_gyrotemp.ki = 140.0f;
//  Pid_gyrotemp.kd = 0.0f;
//  Pid_gyrotemp.ki_able_error_range = 0.0f;//100.0f;
//  Pid_gyrotemp.kd_able_error_range = 0.0f;//50.0f;
//  Pid_gyrotemp.max_out_value = 700.0f;
//  Pid_gyrotemp.min_out_value = 100.0f;
//  Pid_gyrotemp.out = 0.0f;

//  //云台内圈位置式 Z Y
//  PID.gimbal.dwn.z.spd.type_selection = 1;
//  PID.gimbal.dwn.z.spd.Ref = 0.0f;
//  PID.gimbal.dwn.z.spd.feedback = 0.0f;
//  PID.gimbal.dwn.z.spd.Error = 0.0f ;
//  PID.gimbal.dwn.z.spd.DError = 0.0f ;
//  PID.gimbal.dwn.z.spd.DDError = 0.0f ;
//  PID.gimbal.dwn.z.spd.PreError = 0.0f ;
//  PID.gimbal.dwn.z.spd.PreDError = 0.0f ;
//  PID.gimbal.dwn.z.spd.kp = 1000000.0f;//1750.0f;
//  PID.gimbal.dwn.z.spd.ki = 800000.0f;//475.0f;
//  PID.gimbal.dwn.z.spd.kd = 0.0f;//20.0f;
//  PID.gimbal.dwn.z.spd.ki_able_error_range = 0.0f;
//  PID.gimbal.dwn.z.spd.kd_able_error_range = 0.0f;
//  PID.gimbal.dwn.z.spd.max_out_value = 30000.0f;
//  PID.gimbal.dwn.z.spd.min_out_value = -30000.0f;
//  PID.gimbal.dwn.z.spd.out = 0.0f;

//  PID.gimbal.dwn.y.spd.type_selection = 1;
//  PID.gimbal.dwn.y.spd.Ref = 0.0f;
//  PID.gimbal.dwn.y.spd.feedback = 0.0f;
//  PID.gimbal.dwn.y.spd.Error = 0.0f ;
//  PID.gimbal.dwn.y.spd.DError = 0.0f ;
//  PID.gimbal.dwn.y.spd.DDError = 0.0f ;
//  PID.gimbal.dwn.y.spd.PreError = 0.0f ;
//  PID.gimbal.dwn.y.spd.PreDError = 0.0f ;
//  PID.gimbal.dwn.y.spd.kp = 1000000.0f;//1750.0f;//7000
//  PID.gimbal.dwn.y.spd.ki = 800000.0f;//465.0f;
//  PID.gimbal.dwn.y.spd.kd = 0.0f;//18.0f;
//  PID.gimbal.dwn.y.spd.ki_able_error_range = 0;
//  PID.gimbal.dwn.y.spd.kd_able_error_range = 0;
//  PID.gimbal.dwn.y.spd.max_out_value = 30000.0f;
//  PID.gimbal.dwn.y.spd.min_out_value = -30000.0f;
//  PID.gimbal.dwn.y.spd.out = 0.0f;

//  //云台外圈位置式 Z Y
//  PID.gimbal.dwn.z.ang.type_selection = 1;
//  PID.gimbal.dwn.z.ang.Ref = 0.0f;
//  PID.gimbal.dwn.z.ang.feedback = 0.0f;
//  PID.gimbal.dwn.z.ang.Error = 0.0f;
//  PID.gimbal.dwn.z.ang.DError = 0.0f;
//  PID.gimbal.dwn.z.ang.integral = 0.0f;
//  PID.gimbal.dwn.z.ang.kp = 1500.0f;//0.2f;
//  PID.gimbal.dwn.z.ang.ki = 10.0f;//0.2f;
//  PID.gimbal.dwn.z.ang.kd = 2.0f;//12.5f;
//  PID.gimbal.dwn.z.ang.ki_able_error_range = 0;
//  PID.gimbal.dwn.z.ang.kd_able_error_range = 0;
//  PID.gimbal.dwn.z.ang.max_out_value =  100.0f;
//  PID.gimbal.dwn.z.ang.min_out_value = -100.0f;
//  PID.gimbal.dwn.z.ang.max_integral = 10.0f;
//  PID.gimbal.dwn.z.ang.min_integral = -10.0f;
//  PID.gimbal.dwn.z.ang.out = 0.0f;

//  PID.gimbal.dwn.y.ang.type_selection = 1;
//  PID.gimbal.dwn.y.ang.Ref = 0.0f;
//  PID.gimbal.dwn.y.ang.feedback = 0.0f;
//  PID.gimbal.dwn.y.ang.Error = 0.0f;
//  PID.gimbal.dwn.y.ang.DError = 0.0f;
//  PID.gimbal.dwn.y.ang.integral = 0.0f;
//  PID.gimbal.dwn.y.ang.kp = 1500.0f;//0.00f;//
//  PID.gimbal.dwn.y.ang.ki = 10.0f;//0.1f;//
//  PID.gimbal.dwn.y.ang.kd = 0.0f;//2.05f;//
//  PID.gimbal.dwn.y.ang.ki_able_error_range = 0;
//  PID.gimbal.dwn.y.ang.kd_able_error_range = 0;
//  PID.gimbal.dwn.y.ang.max_out_value =  100.0f;
//  PID.gimbal.dwn.y.ang.min_out_value = -100.0f;
//  PID.gimbal.dwn.y.ang.max_integral = 10.0f;
//  PID.gimbal.dwn.y.ang.min_integral = -10.0f;
//  PID.gimbal.dwn.y.ang.out = 0.0f;

//  PID.booster.dwn.m0.spd.type_selection = 0;
//  PID.booster.dwn.m0.spd.Ref = 0.0f ;
//  PID.booster.dwn.m0.spd.feedback = 0.0f ;
//  PID.booster.dwn.m0.spd.Error = 0.0f ;
//  PID.booster.dwn.m0.spd.DError = 0.0f ;
//  PID.booster.dwn.m0.spd.DDError = 0.0f ;
//  PID.booster.dwn.m0.spd.PreError = 0.0f ;
//  PID.booster.dwn.m0.spd.PreDError = 0.0f ;
//  PID.booster.dwn.m0.spd.kp = 1000.0f;//4700.0f ;
//  PID.booster.dwn.m0.spd.ki = 1000.0f;//700.0f ;
//  PID.booster.dwn.m0.spd.kd = 0.0f;//10.0f ;
//  PID.booster.dwn.m0.spd.ki_able_error_range = 0;
//  PID.booster.dwn.m0.spd.kd_able_error_range = 0;
//  PID.booster.dwn.m0.spd.max_out_value = 16384 ;  //16384
//  PID.booster.dwn.m0.spd.min_out_value = -16384 ;
//  PID.booster.dwn.m0.spd.out = 0;

//  PID.booster.dwn.m1.spd.type_selection = 0;
//  PID.booster.dwn.m1.spd.Ref = 0.0f ;
//  PID.booster.dwn.m1.spd.feedback = 0.0f ;
//  PID.booster.dwn.m1.spd.Error = 0.0f ;
//  PID.booster.dwn.m1.spd.DError = 0.0f ;
//  PID.booster.dwn.m1.spd.DDError = 0.0f ;
//  PID.booster.dwn.m1.spd.PreError = 0.0f ;
//  PID.booster.dwn.m1.spd.PreDError = 0.0f ;
//  PID.booster.dwn.m1.spd.kp = 1000.0f;//4700.0f ;
//  PID.booster.dwn.m1.spd.ki = 1000.0f;//700.0f ;
//  PID.booster.dwn.m1.spd.kd = 0.0f;//10.0f ;
//  PID.booster.dwn.m1.spd.ki_able_error_range = 0;
//  PID.booster.dwn.m1.spd.kd_able_error_range = 0;
//  PID.booster.dwn.m1.spd.max_out_value = 16384 ;  //16384
//  PID.booster.dwn.m1.spd.min_out_value = -16384 ;
//  PID.booster.dwn.m1.spd.out = 0;

//  PID.booster.up.m0.spd.type_selection = 0;
//  PID.booster.up.m0.spd.Ref = 0.0f ;
//  PID.booster.up.m0.spd.feedback = 0.0f ;
//  PID.booster.up.m0.spd.Error = 0.0f ;
//  PID.booster.up.m0.spd.DError = 0.0f ;
//  PID.booster.up.m0.spd.DDError = 0.0f ;
//  PID.booster.up.m0.spd.PreError = 0.0f ;
//  PID.booster.up.m0.spd.PreDError = 0.0f ;
//  PID.booster.up.m0.spd.kp = 0.0f;//4700.0f ;
//  PID.booster.up.m0.spd.ki = 0.0f;//700.0f ;
//  PID.booster.up.m0.spd.kd = 0.0f;//10.0f ;
//  PID.booster.up.m0.spd.ki_able_error_range = 0;
//  PID.booster.up.m0.spd.kd_able_error_range = 0;
//  PID.booster.up.m0.spd.max_out_value = 16384 ;  //16384
//  PID.booster.up.m0.spd.min_out_value = -16384 ;
//  PID.booster.up.m0.spd.out = 0;

//  PID.booster.up.m1.spd.type_selection = 1;
//  PID.booster.up.m1.spd.Ref = 0.0f ;
//  PID.booster.up.m1.spd.feedback = 0.0f ;
//  PID.booster.up.m1.spd.Error = 0.0f ;
//  PID.booster.up.m1.spd.DError = 0.0f ;
//  PID.booster.up.m1.spd.DDError = 0.0f ;
//  PID.booster.up.m1.spd.PreError = 0.0f ;
//  PID.booster.up.m1.spd.PreDError = 0.0f ;
//  PID.booster.up.m1.spd.kp = 0.0f;//4700.0f ;
//  PID.booster.up.m1.spd.ki = 0.0f;//700.0f ;
//  PID.booster.up.m1.spd.kd = 0.0f;//10.0f ;
//  PID.booster.up.m1.spd.ki_able_error_range = 0;
//  PID.booster.up.m1.spd.kd_able_error_range = 0;
//  PID.booster.up.m1.spd.max_out_value = 16384 ;  //16384
//  PID.booster.up.m1.spd.min_out_value = -16384 ;
//  PID.booster.up.m1.spd.out = 0;

//  PID.booster.dwn.trigger.ang.type_selection = 1;
//  PID.booster.dwn.trigger.ang.Ref = 0.0f;
//  PID.booster.dwn.trigger.ang.feedback = 0.0f;
//  PID.booster.dwn.trigger.ang.Error = 0.0f ;
//  PID.booster.dwn.trigger.ang.DError = 0.0f ;
//  PID.booster.dwn.trigger.ang.DDError = 0.0f ;
//  PID.booster.dwn.trigger.ang.PreError = 0.0f ;
//  PID.booster.dwn.trigger.ang.PreDError = 0.0f ;
//  PID.booster.dwn.trigger.ang.kp = 30.0f;//1200.0f;
//  PID.booster.dwn.trigger.ang.ki = 10.0f;//600.0f;
//  PID.booster.dwn.trigger.ang.kd = 0.1f;//10.0f;
//  PID.booster.dwn.trigger.ang.ki_able_error_range = 0;
//  PID.booster.dwn.trigger.ang.kd_able_error_range = 0;
//  PID.booster.dwn.trigger.ang.max_out_value = 6000.0f;
//  PID.booster.dwn.trigger.ang.min_out_value = -6000.0f;
//  PID.booster.dwn.trigger.ang.max_integral = 0.5f;
//  PID.booster.dwn.trigger.ang.min_integral = -0.5f;
//  PID.booster.dwn.trigger.ang.out = 0.0f;


//  PID.booster.dwn.trigger.spd.type_selection = 0;
//  PID.booster.dwn.trigger.spd.Ref = 0.0f;
//  PID.booster.dwn.trigger.spd.feedback = 0.0f;
//  PID.booster.dwn.trigger.spd.Error = 0.0f ;
//  PID.booster.dwn.trigger.spd.DError = 0.0f ;
//  PID.booster.dwn.trigger.spd.DDError = 0.0f ;
//  PID.booster.dwn.trigger.spd.PreError = 0.0f ;
//  PID.booster.dwn.trigger.spd.PreDError = 0.0f ;
//  PID.booster.dwn.trigger.spd.kp = 1000.0f;//1200.0f;
//  PID.booster.dwn.trigger.spd.ki = 600.0f;//600.0f;
//  PID.booster.dwn.trigger.spd.kd = 0.0f;//10.0f;
//  PID.booster.dwn.trigger.spd.ki_able_error_range = 0;
//  PID.booster.dwn.trigger.spd.kd_able_error_range = 0;
//  PID.booster.dwn.trigger.spd.max_out_value = 15000.0f;
//  PID.booster.dwn.trigger.spd.min_out_value = -15000.0f;
//  PID.booster.dwn.trigger.spd.out = 0.0f;


//  PID.booster.up.trigger.ang.type_selection = 1;
//  PID.booster.up.trigger.ang.Ref = 0.0f;
//  PID.booster.up.trigger.ang.feedback = 0.0f;
//  PID.booster.up.trigger.ang.Error = 0.0f ;
//  PID.booster.up.trigger.ang.DError = 0.0f ;
//  PID.booster.up.trigger.ang.DDError = 0.0f ;
//  PID.booster.up.trigger.ang.PreError = 0.0f ;
//  PID.booster.up.trigger.ang.PreDError = 0.0f ;
//  PID.booster.up.trigger.ang.kp = 0.0f;//1200.0f;
//  PID.booster.up.trigger.ang.ki = 0.0f;//600.0f;
//  PID.booster.up.trigger.ang.kd = 0.0f;//10.0f;
//  PID.booster.up.trigger.ang.ki_able_error_range = 0;
//  PID.booster.up.trigger.ang.kd_able_error_range = 0;
//  PID.booster.up.trigger.ang.max_out_value = 6000.0f;
//  PID.booster.up.trigger.ang.min_out_value = -6000.0f;
//  PID.booster.up.trigger.ang.max_integral = 0.5f;
//  PID.booster.up.trigger.ang.min_integral = -0.5f;
//  PID.booster.up.trigger.ang.out = 0.0f;

//  PID.booster.up.trigger.spd.type_selection = 0;
//  PID.booster.up.trigger.spd.Ref = 0.0f;
//  PID.booster.up.trigger.spd.feedback = 0.0f;
//  PID.booster.up.trigger.spd.Error = 0.0f ;
//  PID.booster.up.trigger.spd.DError = 0.0f ;
//  PID.booster.up.trigger.spd.DDError = 0.0f ;
//  PID.booster.up.trigger.spd.PreError = 0.0f ;
//  PID.booster.up.trigger.spd.PreDError = 0.0f ;
//  PID.booster.up.trigger.spd.kp = 0.0f;//1200.0f;
//  PID.booster.up.trigger.spd.ki = 0.0f;//600.0f;
//  PID.booster.up.trigger.spd.kd = 0.0f;//10.0f;
//  PID.booster.up.trigger.spd.ki_able_error_range = 0;
//  PID.booster.up.trigger.spd.kd_able_error_range = 0;
//  PID.booster.up.trigger.spd.max_out_value = 8000.0f;
//  PID.booster.up.trigger.spd.min_out_value = -8000.0f;
//  PID.booster.up.trigger.spd.out = 0.0f;

//  PID.chassis.main0.spd.type_selection = 1;
//  PID.chassis.main0.spd.Ref = 0.0f;
//  PID.chassis.main0.spd.feedback = 0.0f;
//  PID.chassis.main0.spd.Error = 0.0f ;
//  PID.chassis.main0.spd.DError = 0.0f ;
//  PID.chassis.main0.spd.DDError = 0.0f ;
//  PID.chassis.main0.spd.PreError = 0.0f ;
//  PID.chassis.main0.spd.PreDError = 0.0f ;
//  PID.chassis.main0.spd.kp = 800.0f;//20.0f;
//  PID.chassis.main0.spd.ki = 150.0f;//5.0f;
//  PID.chassis.main0.spd.kd = 0.0f;//0.0f;
//  PID.chassis.main0.spd.ki_able_error_range = 0;
//  PID.chassis.main0.spd.kd_able_error_range = 0;
//  PID.chassis.main0.spd.max_integral = 1000.0f;
//  PID.chassis.main0.spd.min_integral = -1000.0f;
//  PID.chassis.main0.spd.max_out_value = 7000.0f;
//  PID.chassis.main0.spd.min_out_value = -7000.0f;

//  PID.chassis.main0.ang.type_selection = 1;
//  PID.chassis.main0.ang.Ref = 0.0f;
//  PID.chassis.main0.ang.feedback = 0.0f;
//  PID.chassis.main0.ang.Error = 0.0f ;
//  PID.chassis.main0.ang.DError = 0.0f ;
//  PID.chassis.main0.ang.DDError = 0.0f ;
//  PID.chassis.main0.ang.PreError = 0.0f ;
//  PID.chassis.main0.ang.PreDError = 0.0f ;
//  PID.chassis.main0.ang.kp = 8.0f;//20.0f;
//  PID.chassis.main0.ang.ki = 0.0f;//5.0f;
//  PID.chassis.main0.ang.kd = 0.0f;//0.0f;
//  PID.chassis.main0.ang.ki_able_error_range = 0;
//  PID.chassis.main0.ang.kd_able_error_range = 0;
//  PID.chassis.main0.ang.max_integral = 10.0f;
//  PID.chassis.main0.ang.min_integral = -10.0f;
//  PID.chassis.main0.ang.max_out_value = 8000.0f;
//  PID.chassis.main0.ang.min_out_value = -8000.0f;

/*
//Yaw Pitch
Pid_auto_aiming[0].type_selection = 1;
Pid_auto_aiming[0].Ref = 0.0f;
Pid_auto_aiming[0].feedback = 0.0f;
Pid_auto_aiming[0].Error = 0.0f ;
Pid_auto_aiming[0].DError = 0.0f ;
Pid_auto_aiming[0].DDError = 0.0f ;
Pid_auto_aiming[0].PreError = 0.0f ;
Pid_auto_aiming[0].PreDError = 0.0f ;
Pid_auto_aiming[0].kp = 5.0f;//20.0f;
Pid_auto_aiming[0].ki = 0.0f;//5.0f;
Pid_auto_aiming[0].kd = 0.1f;//0.0f;
Pid_auto_aiming[0].ki_able_error_range = 0;
Pid_auto_aiming[0].kd_able_error_range = 0;
Pid_auto_aiming[0].max_integral = 10.0f;
Pid_auto_aiming[0].min_integral = -10.0f;
Pid_auto_aiming[0].max_out_value = 8000.0f;
Pid_auto_aiming[0].min_out_value = -8000.0f;

Pid_auto_aiming[1].type_selection = 1;
Pid_auto_aiming[1].Ref = 0.0f;
Pid_auto_aiming[1].feedback = 0.0f;
Pid_auto_aiming[1].Error = 0.0f ;
Pid_auto_aiming[1].DError = 0.0f ;
Pid_auto_aiming[1].DDError = 0.0f ;
Pid_auto_aiming[1].PreError = 0.0f ;
Pid_auto_aiming[1].PreDError = 0.0f ;
Pid_auto_aiming[1].kp = 5.0f;//20.0f;
Pid_auto_aiming[1].ki = 0.0f;//5.0f;
Pid_auto_aiming[1].kd = 0.1f;//0.0f;
Pid_auto_aiming[1].ki_able_error_range = 0;
Pid_auto_aiming[1].kd_able_error_range = 0;
Pid_auto_aiming[1].max_integral = 10.0f;
Pid_auto_aiming[1].min_integral = -10.0f;
Pid_auto_aiming[1].max_out_value = 8000.0f;
Pid_auto_aiming[1].min_out_value = -8000.0f;
*/
// }
