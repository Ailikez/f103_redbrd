#include "alg_pid_multi.h"

#define ITERATION_NUM           (500)
#define SET_VALUE               (300.0)
#define PID_CONV_REAL           (1.0)
#define PID_ERR_MAX             (200)
#define ITERATION_3_ERR_MAX     (200)
#define ITERATION_3_ERR_MIN     (180)
#define ITERATION_3_ERR_DIFF    (ITERATION_3_ERR_MAX - ITERATION_3_ERR_MIN)

#define _abs(num) ((num > 0)?(num):(-num))

void PID_Init(PID_t *pid)
{
	pid->SetValue = 0.0;
	pid->RealValue = 0.0;
	pid->err = 0.0;
	pid->err_last = 0.0;
	pid->err_next = 0.0;
	pid->pid_result = 0.0;
	pid->integral = 0.0;
	pid->Kp = 0.2;
	//pid->Ki = 0.015;
	//pid->Ki = 0.04;
	//pid->Ki = 0.1;
	pid->Ki = 0.2;
	pid->Kd = 0.2;
	pid->umax = 400;
	pid->umin = -200;
}
#ifdef PARAM_ALL_POINTER
    //标准PID公式
    float PID_Standard(PID_t *pid,
                       const float *set_value,
                       const float *current_value)
    {
        pid->SetValue = *set_value;
        pid->RealValue = *current_value;
        pid->err = pid->SetValue - pid->RealValue;
        pid->integral += pid->err;
        pid->pid_result = pid->Kp*pid->err 
                        + pid->Ki*pid->integral 
                        + pid->Kd*(pid->err - pid->err_last);
        pid->err_last = pid->err;
        return pid->pid_result;
    }
    //利用PID增量的PID公式
    float PID_Incr(PID_t *pid,
                   const float *set_value,
                   const float *current_value)
    {
        pid->SetValue = *set_value;
        pid->RealValue = *current_value;    
        pid->err = pid->SetValue - pid->RealValue;
        pid->pid_result = pid->Kp*(pid->err - pid->err_next)
                        + pid->Ki*pid->err
                        + pid->Kd*(pid->err - 2*pid->err_next + pid->err_last);
        pid->err_last = pid->err_next;
        pid->err_next = pid->err;
        return pid->pid_result;
    }

    float PID_Integral_1(PID_t *pid,
                        const float *set_value,
                        const float *current_value)
    {
        float index;
        pid->SetValue = *set_value;
        pid->RealValue = *current_value;    
        pid->err = pid->SetValue - pid->RealValue;
        if (_abs(pid->err) > PID_ERR_MAX)
        {
            index = 0.0;
        }
        else
        {
            index = 1.0;
            pid->integral += pid->err;
        }
        pid->pid_result = pid->Kp*pid->err
                        + index * pid->Ki*pid->integral
                        + pid->Kd*(pid->err - pid->err_last);
        pid->err_last = pid->err;
        return pid->pid_result;
    }
    //积分分离
    float PID_Integral_2(PID_t *pid,
                        const float *set_value,
                        const float *current_value)
    {
        float index;
        pid->SetValue = *set_value;
        pid->RealValue = *current_value; 
        pid->err = pid->SetValue - pid->RealValue;
        if (pid->RealValue > pid->umax)
        {
            if (_abs(pid->err) > PID_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                index = 1.0;
                if (pid->err < 0)
                    pid->integral += pid->err;
            }
        }
        else if (pid->RealValue < pid->umin)
        {
            if (_abs(pid->err) > PID_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                index = 1.0;
                if (pid->err > 0)
                    pid->integral += pid->err;
            }
        }
        else
        {
            if (_abs(pid->err) > PID_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                index = 1.0;
                pid->integral += pid->err;
            }
        }

        pid->pid_result = pid->Kp*pid->err
                        + index * pid->Ki*pid->integral
                        + pid->Kd*(pid->err - pid->err_last);
        pid->err_last = pid->err;
        return pid->pid_result;
    }
    //变积分PID公式 ： Ki*index
    float PID_Integral_3(PID_t *pid,
                        const float *set_value,
                        const float *current_value)
    {
        float index;
        pid->SetValue = *set_value;
        pid->RealValue = *current_value; 
        pid->err = pid->SetValue - pid->RealValue;
            if (_abs(pid->err) > ITERATION_3_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                if (_abs(pid->err) < ITERATION_3_ERR_MIN)
                    index = 1.0;
                else
                    index = (ITERATION_3_ERR_MAX - _abs(pid->err)) / ITERATION_3_ERR_DIFF;
                pid->integral += pid->err;
            }
        pid->pid_result = pid->Kp*pid->err
                        + index * pid->Ki*pid->integral
                        + pid->Kd*(pid->err - pid->err_last);
        pid->err_last = pid->err;
        return pid->pid_result;
    }
    //抗积分饱和（设置上下限），积分分离，变积分Ki*index
    float PID_Integral_4(PID_t *pid,
                        const float *set_value,
                        const float *current_value)
    {
        float index;
        pid->SetValue = *set_value;
        pid->RealValue = *current_value; 
        pid->err = pid->SetValue - pid->RealValue;
        if (pid->RealValue > pid->umax)
        {
            if (_abs(pid->err) > ITERATION_3_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                if (_abs(pid->err) < ITERATION_3_ERR_MIN)
                    index = 1.0;
                else
                    index = (ITERATION_3_ERR_MAX - _abs(pid->err)) / ITERATION_3_ERR_DIFF;
                if (pid->err < 0)
                    pid->integral += pid->err;
            }
        }
        else if (pid->RealValue < pid->umin)
        {
            if (_abs(pid->err) > ITERATION_3_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                if (_abs(pid->err) < ITERATION_3_ERR_MIN)
                    index = 1.0;
                else
                    index = (ITERATION_3_ERR_MAX - _abs(pid->err)) / ITERATION_3_ERR_DIFF;
                if (pid->err > 0)
                    pid->integral += pid->err;
            }
        }
        else
        {
            if (_abs(pid->err) > ITERATION_3_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                if (_abs(pid->err) < ITERATION_3_ERR_MIN)
                    index = 1.0;
                else
                    index = (ITERATION_3_ERR_MAX - _abs(pid->err)) / ITERATION_3_ERR_DIFF;
                pid->integral += pid->err;
            }
        }
        pid->pid_result = pid->Kp*pid->err
                        + index * pid->Ki*pid->integral
                        + pid->Kd*(pid->err - pid->err_last);
        pid->err_last = pid->err;
        return pid->pid_result;
    }
#else 
    //标准PID公式
    float PID_Standard(PID_t *pid,
                       const float set_value,
                       const float current_value)
    {
        pid->SetValue = set_value;
        pid->RealValue = current_value;
        pid->err = pid->SetValue - pid->RealValue;
        pid->integral += pid->err;
        pid->pid_result = pid->Kp*pid->err 
                        + pid->Ki*pid->integral 
                        + pid->Kd*(pid->err - pid->err_last);
        pid->err_last = pid->err;
        return pid->pid_result;
    }
    //利用PID增量的PID公式
    float PID_Incr(PID_t *pid,
                   const float set_value,
                   const float current_value)
    {
        pid->SetValue = set_value;
        pid->RealValue = current_value;    
        pid->err = pid->SetValue - pid->RealValue;
        pid->pid_result = pid->Kp*(pid->err - pid->err_next)
                        + pid->Ki*pid->err
                        + pid->Kd*(pid->err - 2*pid->err_next + pid->err_last);
        pid->err_last = pid->err_next;
        pid->err_next = pid->err;
        return pid->pid_result;
    }

    float PID_Integral_1(PID_t *pid,
                        const float set_value,
                        const float current_value)
    {
        float index;
        pid->SetValue = set_value;
        pid->RealValue = current_value;    
        pid->err = pid->SetValue - pid->RealValue;
        if (_abs(pid->err) > PID_ERR_MAX)
        {
            index = 0.0;
        }
        else
        {
            index = 1.0;
            pid->integral += pid->err;
        }
        pid->pid_result = pid->Kp*pid->err
                        + index * pid->Ki*pid->integral
                        + pid->Kd*(pid->err - pid->err_last);
        pid->err_last = pid->err;
        return pid->pid_result;
    }
    //积分分离
    float PID_Integral_2(PID_t *pid,
                        const float set_value,
                        const float current_value)
    {
        float index;
        pid->SetValue = set_value;
        pid->RealValue = current_value; 
        pid->err = pid->SetValue - pid->RealValue;
        if (pid->RealValue > pid->umax)
        {
            if (_abs(pid->err) > PID_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                index = 1.0;
                if (pid->err < 0)
                    pid->integral += pid->err;
            }
        }
        else if (pid->RealValue < pid->umin)
        {
            if (_abs(pid->err) > PID_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                index = 1.0;
                if (pid->err > 0)
                    pid->integral += pid->err;
            }
        }
        else
        {
            if (_abs(pid->err) > PID_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                index = 1.0;
                pid->integral += pid->err;
            }
        }

        pid->pid_result = pid->Kp*pid->err
                        + index * pid->Ki*pid->integral
                        + pid->Kd*(pid->err - pid->err_last);
        pid->err_last = pid->err;
        return pid->pid_result;
    }
    //变积分PID公式 ： Ki*index
    float PID_Integral_3(PID_t *pid,
                        const float set_value,
                        const float current_value)
    {
        float index;
        pid->SetValue = set_value;
        pid->RealValue = current_value; 
        pid->err = pid->SetValue - pid->RealValue;
            if (_abs(pid->err) > ITERATION_3_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                if (_abs(pid->err) < ITERATION_3_ERR_MIN)
                    index = 1.0;
                else
                    index = (ITERATION_3_ERR_MAX - _abs(pid->err)) / ITERATION_3_ERR_DIFF;
                pid->integral += pid->err;
            }
        pid->pid_result = pid->Kp*pid->err
                        + index * pid->Ki*pid->integral
                        + pid->Kd*(pid->err - pid->err_last);
        pid->err_last = pid->err;
        return pid->pid_result;
    }
    //抗积分饱和（设置上下限），积分分离，变积分Ki*index
    float PID_Integral_4(PID_t *pid,
                        const float set_value,
                        const float current_value)
    {
        float index;
        pid->SetValue = set_value;
        pid->RealValue = current_value; 
        pid->err = pid->SetValue - pid->RealValue;
        if (pid->RealValue > pid->umax)
        {
            if (_abs(pid->err) > ITERATION_3_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                if (_abs(pid->err) < ITERATION_3_ERR_MIN)
                    index = 1.0;
                else
                    index = (ITERATION_3_ERR_MAX - _abs(pid->err)) / ITERATION_3_ERR_DIFF;
                if (pid->err < 0)
                    pid->integral += pid->err;
            }
        }
        else if (pid->RealValue < pid->umin)
        {
            if (_abs(pid->err) > ITERATION_3_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                if (_abs(pid->err) < ITERATION_3_ERR_MIN)
                    index = 1.0;
                else
                    index = (ITERATION_3_ERR_MAX - _abs(pid->err)) / ITERATION_3_ERR_DIFF;
                if (pid->err > 0)
                    pid->integral += pid->err;
            }
        }
        else
        {
            if (_abs(pid->err) > ITERATION_3_ERR_MAX)
            {
                index = 0;
            }
            else
            {
                if (_abs(pid->err) < ITERATION_3_ERR_MIN)
                    index = 1.0;
                else
                    index = (ITERATION_3_ERR_MAX - _abs(pid->err)) / ITERATION_3_ERR_DIFF;
                pid->integral += pid->err;
            }
        }
        pid->pid_result = pid->Kp*pid->err
                        + index * pid->Ki*pid->integral
                        + pid->Kd*(pid->err - pid->err_last);
        pid->err_last = pid->err;
        return pid->pid_result;
    }
#endif