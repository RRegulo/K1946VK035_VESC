/******************************************************************************
 * @file        FOC_Math.h
 * @brief       FOC-математика: clarke/park/ipark, фильтр, pid_reg3, ramp.
 *              Порт из демо НПФ Вектор (clarke, park, ipark, filter,
 *              V_pid_reg3, V_rmp_ctrl) в float. Углы Park/IPark — радианы.
 *              Без HW-зависимостей: собирается и для цели (arm-none-eabi),
 *              и для хост-теста (gcc).
 ******************************************************************************/

#ifndef INCLUDE_FOC_MATH_H_
#define INCLUDE_FOC_MATH_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define _PI 3.1415926535f

/* --- Clarke (clarke.h демо) --- */
struct SClarke {
    float as;
    float bs;
    float ds;
    float qs;
    void (*calc)(struct SClarke *);
};
typedef struct SClarke TClarke;
#define CLARKE_DEFAULTS {0.0f, 0.0f, 0.0f, 0.0f, clarke_calc}
void clarke_calc(TClarke *v);

/* --- Park (park.h демо), угол в радианах --- */
struct SPark {
    float ds;
    float qs;
    float ang;
    float de;
    float qe;
    void (*calc)(struct SPark *);
};
typedef struct SPark TPark;
#define PARK_DEFAULTS {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, park_calc}
void park_calc(TPark *v);

/* --- IPark (ipark.h демо), угол в радианах --- */
struct SIPark {
    float ds;
    float qs;
    float ang;
    float de;
    float qe;
    void (*calc)(struct SIPark *);
};
typedef struct SIPark TIPark;
#define IPARK_DEFAULTS {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, ipark_calc}
void ipark_calc(TIPark *v);

/* --- Фильтр (filter.h демо) --- */
struct SFilter {
    float input;
    float output;
    float T;
    void (*calc)(struct SFilter *);
};
typedef struct SFilter TFilter;
#define FILTER_DEFAULTS {0.0f, 0.0f, 0.01f, TFilter_Calc}
void TFilter_Calc(TFilter *p);

/* --- PID reg3 (V_pid_reg3.h демо) --- */
typedef struct SPidReg3 {
    float pid_ref_reg3;
    float pid_fdb_reg3;
    float e_reg3;
    float e_reg3Dz;
    float Kp_reg3;
    float up_reg3;
    float ui_reg3;
    float ud_reg3;
    float uprsat_reg3;
    float saterr_reg3Add;
    float pid_out_max;
    float pid_out_min;
    float pid_out_reg3;
    float saterr_reg3;
    float Ki_reg3;
    float Kc_reg3;
    float Kd_reg3;
    float up1_reg3;
    float pid_out_test;
    float DeadZone;
    float Kf_d;
    float e_reg3_filterOut;
    int32_t DiffDelim;
    int32_t DiffCounter;
    int KdFilterInitFlag;
    void (*calc)(struct SPidReg3 *);
    void (*reset)(struct SPidReg3 *);
} TPidReg3;

void pid_reg3_calc(TPidReg3 *v);
void pid_reg3_reset(TPidReg3 *v);

/* --- Ramp (V_rmp_ctrl.h демо) --- */
struct SRMPCtrl {
    float input;
    float output;
    float Ts;
    float T;
    float step;
    void (*calc)(struct SRMPCtrl *);
    void (*slow_calc)(struct SRMPCtrl *);
};
typedef struct SRMPCtrl TRMPCtrl;
#define RMP_CTRL_DEFAULTS                                                                          \
    {0.0f, 0.0f, (1.0f / 6250.0f), 2.0f, 0.0f, rmp_ctrl_calc, rmp_ctrl_slow_calc}
void rmp_ctrl_calc(TRMPCtrl *p);
void rmp_ctrl_slow_calc(TRMPCtrl *p);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_FOC_MATH_H_ */
