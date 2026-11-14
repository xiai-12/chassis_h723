//
// Created by zhaol on 2026/9/19.
//
#include "motion_plane.h"

#include "serial_cmd.h"

/*
 * 轨迹规划函数
 *  根据目标位置与实际位置的差值 err计算刹车速度v_brake = sqrt(2*a_cc*err)
 *  未达到v_max，继续按 acc线性加速
 *  目标值、实际值 单位是 m/s
 */

static float DIS_BRK = 0.015f;// 减速距离
static float DIS_TOL = 0.005f;// 冗余距离
static float MIN_SPD = 0.02f; // 最小速度，克服静摩擦
void Trace_Update(Trace* t, float dt)
{
    float err     = t->pos_target - t->pos_act;
    float err_abs = fabsf(err);
    float dir = (err >= 0.0f)?1.0f:-1.0f;

    float v_des = sqrtf(2.0f * t->acc * err_abs);
    if (v_des > t->vmax) v_des = t->vmax;

    float dv = v_des - t->vref, dv_max = t->acc * dt;
    if (dv >  dv_max) dv =  dv_max;
    if (dv < -dv_max) dv = -dv_max;
    t->vref += dv;

    if ((err < DIS_BRK && err > DIS_TOL) && t->vref < MIN_SPD) t->vref = MIN_SPD;
    if (err < DIS_TOL) t->vref = 0.00f;
}

// void Trace_Update(Trace* t,float dt)
// {
//     float err = t->pos_target - t->pos_act;  // err 单位为 m
//     float err_abs = fabsf(err);
//     float dir = (err >= 0.0f)?1.0f:-1.0f; // 判断电机运动方向
//     float v_brake = sqrtf(2.0f * t->acc * err_abs);// v_brake 刹车速度 单位 rpm
//     // 到目标位置附近，直接归零，防止在目标点附近反复振荡
//     if (err_abs < 0.01f)
//     {
//         t->vref = 0.0f;
//         return;
//     }
//     // 当前速度方向和目标方向相反
//     if (t->vref * dir < 0.0f)
//     {
//         t->vref += dir * t->acc * dt; // 超出一个周期，返回一次
//         if (t->vref * dir < 0.0f) t->vref = 0.0f;// 超出多个周期，直接清零
//         return;
//     }
//     /*
//      *  加速阶段：fabsf(t->vref) < v_brake 通过if判断将最大速度限制在 v_brake (v_brake < v_max,说明规划距离短； v_brake > v_max,说明规划距离长，且v_brake会逐渐减小到v_max)
//      *  减速阶段： 将速度钳位在 v_brake ,减速阶段通过 v_brake的减小
//      */
//     if (fabsf(t->vref) < v_brake)
//     {
//         // 加速阶段
//         t->vref += dir * t->acc * dt;
//         if (fabsf(t->vref) > t->vmax) t->vref = dir * t->vmax;
//         if (fabsf(t->vref) > v_brake)  t->vref = dir * v_brake;
//     }
//     else
//     {
//         t->vref = dir * v_brake;
//     }
// }


