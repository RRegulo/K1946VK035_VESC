/******************************************************************************
 * @file        FOC_Math.c
 * @brief       Порт из демо НПФ Вектор: clarke.c, park.c, ipark.c, filter.c,
 *              V_pid_reg3.c, V_rmp_ctrl.c — в float. Углы Park/IPark в
 *              радианах, sin/cos — быстрый VESC-аппроксиматор из VESC_Math.
 ******************************************************************************/

#include "FOC_Math.h"
#include "VESC_Math.h"

#include <math.h>

/* --- clarke.c --- */
void clarke_calc(TClarke *v)
{
    v->ds = v->as;
    v->qs = (v->as + 2.0f * v->bs) * 0.57735026918963f;
}

/* --- park.c, угол в радианах --- */
void park_calc(TPark *v)
{
    float sin_ang, cos_ang;

    utils_fast_sincos_better(v->ang, &sin_ang, &cos_ang);

    v->de = v->ds * cos_ang + v->qs * sin_ang;
    v->qe = v->qs * cos_ang - v->ds * sin_ang;
}

/* --- ipark.c, угол в радианах --- */
void ipark_calc(TIPark *v)
{
    float sin_ang, cos_ang;

    utils_fast_sincos_better(v->ang, &sin_ang, &cos_ang);

    v->ds = v->de * cos_ang - v->qe * sin_ang;
    v->qs = v->qe * cos_ang + v->de * sin_ang;
}

/* --- filter.c --- */
void TFilter_Calc(TFilter *p)
{
    p->output = p->T * (p->input - p->output) + p->output;
}

/* --- V_pid_reg3.c --- */
void pid_reg3_calc(TPidReg3 *v)
{
    v->e_reg3 = v->pid_ref_reg3 - v->pid_fdb_reg3;
    v->e_reg3Dz = v->e_reg3;

    if(v->DeadZone != 0) {
        if(v->e_reg3Dz > 0) {
            v->e_reg3Dz = v->e_reg3Dz - v->DeadZone;
            if(v->e_reg3Dz < 0)
                v->e_reg3Dz = 0;
        }
        if(v->e_reg3Dz < 0) {
            v->e_reg3Dz = v->e_reg3Dz + v->DeadZone;
            if(v->e_reg3Dz > 0)
                v->e_reg3Dz = 0;
        }
    }

    v->up_reg3 = v->Kp_reg3 * v->e_reg3Dz;

    v->uprsat_reg3 = v->up_reg3 + v->ui_reg3 + v->ud_reg3;

    if(v->uprsat_reg3 > v->pid_out_max)
        v->pid_out_reg3 = v->pid_out_max;
    else if(v->uprsat_reg3 < v->pid_out_min)
        v->pid_out_reg3 = v->pid_out_min;
    else
        v->pid_out_reg3 = v->uprsat_reg3;

    v->saterr_reg3 = v->pid_out_reg3 - v->uprsat_reg3 + v->saterr_reg3Add;

    if(v->Kf_d == 0) {
        v->e_reg3_filterOut = v->e_reg3;
    } else {
        v->e_reg3_filterOut = v->e_reg3_filterOut + v->Kf_d * (v->e_reg3 - v->e_reg3_filterOut);
    }

    if((v->DiffCounter++ + 1) >= v->DiffDelim) {
        if(v->KdFilterInitFlag == 1) {
            v->e_reg3_filterOut = v->e_reg3;
            v->up1_reg3 = v->e_reg3_filterOut;
            v->KdFilterInitFlag = 0;
        }
        v->ud_reg3 = v->Kd_reg3 * (v->e_reg3_filterOut - v->up1_reg3);
        v->up1_reg3 = v->e_reg3_filterOut;
        v->DiffCounter = 0;
    }

    if(v->Ki_reg3 != 0)
        v->ui_reg3 = v->ui_reg3 + v->Ki_reg3 * v->up_reg3 + v->Kc_reg3 * v->saterr_reg3;
    else
        v->ui_reg3 = 0;

    if(v->Kc_reg3 == 0) {
        if(v->ui_reg3 > v->pid_out_max)
            v->ui_reg3 = v->pid_out_max;
        else if(v->ui_reg3 < v->pid_out_min)
            v->ui_reg3 = v->pid_out_min;
    }
}

void pid_reg3_reset(TPidReg3 *v)
{
    v->pid_fdb_reg3 = 0;
    v->pid_ref_reg3 = 0;
    v->ui_reg3 = 0;
    v->KdFilterInitFlag = 1;
    v->pid_out_reg3 = 0;
}

/* --- V_rmp_ctrl.c (демо НПФ Вектор, Apache 2.0) --- */
void rmp_ctrl_calc(TRMPCtrl *p)
{
    if(p->output < p->input)
        p->output += p->step;
    if(p->output > p->input)
        p->output -= p->step;

    if((fabsf(p->output - p->input) <= p->step) || (p->T == 0.0f)) {
        p->output = p->input;
    }
}

void rmp_ctrl_slow_calc(TRMPCtrl *p)
{
    p->step = p->Ts / p->T;
}
