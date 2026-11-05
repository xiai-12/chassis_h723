//
// Created by zhaol on 2026/9/19.
//
#include "motion_plane.h"

/*
 * 轨迹规划函数
 *  根据目标位置与实际位置的差值 err计算刹车速度v_brake = sqrt(2*a_cc*err)
 *  未达到v_max，继续按 acc线性加速
 *  目标值、实际值 单位是 m/s
 */
static float DIS_DEADZONE = 0.006f;    // 位置容差 (m)

void Trace_Update(Trace* t, float dt)
{
    float err     = t->pos_target - t->pos_act;
    float err_abs = fabsf(err);
    float dir     = (err >= 0.0f) ? 1.0f : -1.0f;

    float v_brake = sqrtf(2.0f * t->acc* err_abs);
    float v_des = fminf(t->vmax,v_brake);

    if (err_abs < DIS_DEADZONE) { v_des = 0.0f;}

    float dv = dir * v_des - t->vref;
    float dv_max = t->acc * dt;
    if (dv >  dv_max) dv =  dv_max;
    if (dv < -dv_max) dv = -dv_max;

    t->vref += dv;

    if (fabsf(t->vref) < 1e-4f) t->vref = 0.0f;
}


