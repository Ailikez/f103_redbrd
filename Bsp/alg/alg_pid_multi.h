#ifndef __ALG_PID_MULTI_H__
    #define __ALG_PID_MULTI_H__
    #define PARAM_ALL_POINTER
    typedef struct _pid{
        float SetValue;                    //设定值
        float RealValue;				   //实际值
        float err;						   //偏差值
        float err_last;                    //上一次偏差值
        float err_next;                    //下一次偏差值
        float Kp, Ki, Kd;				   //设定值
        float pid_result;                  //实际转换值
        float integral;					   //积分累积
        float umax;						   //偏差上限值
        float umin;						   //偏差下限值
    }PID_t;
    extern void PID_Init(PID_t *pid);
    #ifdef PARAM_ALL_POINTER
        extern float PID_Standard(PID_t *pid,
                        const float *set_value,
                        const float *current_value);
        extern float PID_Incr(PID_t *pid,
                        const float *set_value,
                        const float *current_value);
        extern float PID_Integral_1(PID_t *pid,
                        const float *set_value,
                        const float *current_value);
        extern float PID_Integral_2(PID_t *pid,
                        const float *set_value,
                        const float *current_value);
        extern float PID_Integral_3(PID_t *pid,
                        const float *set_value,
                        const float *current_value);
        extern float PID_Integral_4(PID_t *pid,
                        const float *set_value,
                        const float *current_value);
    #else
        extern float PID_Standard(PID_t *pid,
                        const float set_value,
                        const float current_value);
        extern float PID_Incr(PID_t *pid,
                        const float set_value,
                        const float current_value);
        extern float PID_Integral_1(PID_t *pid,
                        const float set_value,
                        const float current_value);
        extern float PID_Integral_2(PID_t *pid,
                        const float set_value,
                        const float current_value);
        extern float PID_Integral_3(PID_t *pid,
                        const float set_value,
                        const float current_value);
        extern float PID_Integral_4(PID_t *pid,
                        const float set_value,
                        const float current_value);
    #endif
#endif
