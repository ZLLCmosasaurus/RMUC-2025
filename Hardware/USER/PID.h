#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C"
{
#endif
typedef struct {
    float Kp, Ki, Kd, Kf;  // PID参数和前馈增益
    float max_out, max_iout, min_out, min_iout;
    float Dbuf[3];
    float error[3];
    float Pout, Iout, Dout, Ff_out;  // PID各分量和前馈输出
    float out;
    float set, fdb;
    int is_first_run;  // 添加标志变量，标记是否是第一次运行
} pid_type_def;
/*
typedef struct
{
    unsigned char mode;
    float Kp;
    float Ki;
    float Kd;
    float T;

    float max_out;
    float min_out;
    float max_iout;
    float min_iout;
    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];
    float error[3];

} pid_type_def;

float PID_calc(pid_type_def *pid, float ref, float set);
void PID_init(pid_type_def *pid,  float p,  float i,  float d , float max_out, float max_iout,float min_out, float min_iout) ;
void PID_clear(pid_type_def *pid) ;
 */
void PID_init(pid_type_def *pid, float p, float i, float d, float kf, float max_out, float max_iout, float min_out, float min_iout);
float PID_calc(pid_type_def *pid, float ref, float set);
void PID_clear(pid_type_def *pid);  
float LimitMax(float input,float max) ;

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

#ifdef __cplusplus
}
#endif

#endif
