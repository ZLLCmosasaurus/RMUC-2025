#include "stm32f3xx_hal.h"
#include "PID.h"
pid_type_def control_struct;

float LimitMin(float value, float min) {
  return value < min ? min : value;
}

float LimitMax(float value, float max) {
  return value > max ? max : value;
}
/*
void PID_init(pid_type_def *pid,  float p,  float i,  float d , float max_out, float max_iout,float min_out, float min_iout) 
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
	  pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->min_out = min_out;
    pid->min_iout = min_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

float PID_calc(pid_type_def *pid, float ref, float set) 
{ 
pid->error[2] = pid->error[1];
pid->error[1] = pid->error[0];
pid->set = set;
pid->fdb = ref;
pid->error[0] = set - ref;
pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
pid->Iout = pid->Ki * pid->error[0];
pid->Dbuf[2] = pid->Dbuf[1];
pid->Dbuf[1] = pid->Dbuf[0];
pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
pid->Dout = pid->Kd * pid->Dbuf[0];
pid->Iout=LimitMin(pid->Iout, pid->min_iout);
pid->Iout=LimitMax(pid->Iout, pid->max_iout);
pid->out += pid->Pout + pid->Iout + pid->Dout;
pid->out = LimitMin(pid->out, pid->min_out);
pid->out = LimitMax(pid->out, pid->max_out);

return pid->out;
}


void PID_clear(pid_type_def *pid) 
{
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
*/
void PID_init(pid_type_def *pid, float p, float i, float d, float kf, float max_out, float max_iout, float min_out, float min_iout) 
{
  pid->Kp = p;
  pid->Ki = i;
  pid->Kd = d;
  pid->Kf = kf;
  pid->max_out = max_out;
  pid->max_iout = max_iout;
  pid->min_out = min_out;
  pid->min_iout = min_iout;

  PID_clear(pid);  // 清零所有状态
  pid->is_first_run = 1;  // 标记为第一次运行
}
float PID_calc(pid_type_def *pid, float ref, float set) 
{
  // 更新误差和设定值
  pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->set = set;
  pid->fdb = ref;
  pid->error[0] = set - ref;

  // 计算比例项
  pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);

  // 计算积分项
  pid->Iout = pid->Ki * pid->error[0];
  pid->Iout = LimitMin(pid->Iout, pid->min_iout);
  pid->Iout = LimitMax(pid->Iout, pid->max_iout);

  // 计算微分项
  pid->Dbuf[2] = pid->Dbuf[1];
  pid->Dbuf[1] = pid->Dbuf[0];
  pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
  pid->Dout = pid->Kd * pid->Dbuf[0];

  // 计算前馈项（仅第一次运行时加入）
  if (pid->is_first_run) {
      pid->Ff_out = pid->Kf * (set - ref);  // 前馈输出
      pid->is_first_run = 0;  // 标记为非第一次运行
  } else {
      pid->Ff_out = 0.0f;  // 后续运行时前馈输出为0
      pid->Kf = 0.0f;     // 后续运行时前馈输出为0
  }

  // 计算最终输出
  pid->out += pid->Pout + pid->Iout + pid->Dout + pid->Ff_out;
  pid->out = LimitMin(pid->out, pid->min_out);
  pid->out = LimitMax(pid->out, pid->max_out);

  return pid->out;
}
void PID_clear(pid_type_def *pid) 
{
    // 清零误差缓冲区
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;

    // 清零微分缓冲区
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;

    // 清零PID输出和各分量
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;

    // 清零前馈输出
    pid->Ff_out = 0.0f;  // 确保前馈输出也被清零

    // 清零反馈值和设定值
    pid->fdb = pid->set = 0.0f;
    pid->is_first_run = 1;  // 标记为第一次运行
}