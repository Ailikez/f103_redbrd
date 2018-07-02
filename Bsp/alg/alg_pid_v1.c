/****************************所谓PID即为P（比例）+I（积分）+D（微分**************************/
**(1)P控制（比例控制）：    e(t) = SP - y(t);        SP--设定值   e(t)--误差值（设定的值和当前的误差） y(t)--反馈值（当前值）
**适用于滞后性不大的系统   u(t) = e(t) * P;        u(t)--输出值（需要输出的） P--比例系数
**
**(2)PI控制（比例+积分）：  u(t) = Kp*e(t) + Ki∑e(t) + u0
**可使系统进入稳态无稳态误差u(t)--输出 Kp--比例放大系数 ki--积分放大系数 e(t)--误差 u0--控制量基准值（基础偏差）
**
**(3)PD控制(微分控制)：     
**微分项解决响应速度问题   积分项解决稳定性
**(4)PID控制（比例积分微分控制）：
**微分项解决响应速度问题   u(t) = Kp*e(t) + Ki∑e(t)+ Kd[e(t) – e(t-1)] + u0            
**采样周期（即反馈值的采样周期）
**控制周期（就是每隔多长时间进行一次PID运算，并将结果输出）
****************************小结：P（比例）--即时性 I(积分)--响应速度 D（微分）--稳定性************************************
*/
#include <stdio.h>
#include <string.h>
#include "alg_pid_v1.h"
//#include<stdib.h>

/*
**PID计算部分
*/
double PIDCalc(PID *pp, double NextPoint)   
{
    double dError,                              //当前微分
           Error;                               //偏差
    Error = pp->SetPoint - NextPoint;           //偏差值=设定值-输入值（当前值）
    pp->SumError += Error;                      //积分=积分+偏差  --偏差的累加
    dError = pp->LastError - pp->PrevError;     //当前微分 = 最后误差 - 之前误差
    pp->PrevError = pp->LastError;              //更新“之前误差”
    pp->LastError = Error;                      //更新“最后误差”
    return (pp->Kp * Error                      //比例项 = 比例常数 * 偏差
        +   pp->Ki *  pp->SumError              //积分项 = 积分常数 * 误差积分
        +   pp->Kd * dError                     //微分项 = 微分常数 * 当前微分
        );
}
//为PID变量申请内存，范围指向pp的指针
void PIDInit (PID *pp)   
{
                                //memset是计算机中C / C++语言函数。将s所指向的某一块内存中的前n个
    memset(pp, 0, sizeof(PID)); //字节的内容全部设置为ch指定的ASCII值，块的大小由第三个参数指定，
}                               //这个函数通常为新申请的内存做初始化工作， 其返回值为指向s的指针。
