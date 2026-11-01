/*
        Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se

        This file is part of the VESC firmware.

        The VESC firmware is free software: you can redistribute it and/or
   modify it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

        Ported from VESC (https://github.com/vedderb/bldc) - motor/foc_math.c
   and motor/foc_math.h. Only the sensorless flux observer and the PLL tracker
   are kept. The VESC motor_all_state_t/mc_configuration types are replaced by
   the minimal TFLuxObserverMotor/TFocObserverCfg below that carry only the
   fields the algorithm reads; the algorithm itself is unchanged.
 */

#ifndef FOC_OBSERVER_H_
#define FOC_OBSERVER_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FOC_OBSERVER_ORTEGA_ORIGINAL = 0,
    FOC_OBSERVER_MXLEMMING,
    FOC_OBSERVER_ORTEGA_LAMBDA_COMP,
    FOC_OBSERVER_MXLEMMING_LAMBDA_COMP,
    FOC_OBSERVER_MXV,
    FOC_OBSERVER_MXV_LAMBDA_COMP,
    FOC_OBSERVER_MXV_LAMBDA_COMP_LIN
} foc_observer_type;

typedef enum {
    SAT_COMP_DISABLED = 0,
    SAT_COMP_FACTOR,
    SAT_COMP_LAMBDA,
    SAT_COMP_LAMBDA_AND_FACTOR
} SAT_COMP_MODE;

typedef struct {
    float x1;
    float x2;
    float theta_pre; /* телеметрия: atan2(x2,x1) до клампа x (не из VESC) */
    float lambda_est;
    float i_alpha_last;
    float i_beta_last;
} observer_state;

typedef struct {
    float i_abs_filter;
    float id;
    float iq;
} TFocMotorState;

typedef struct {
    float foc_motor_r;
    float foc_motor_l;
    float foc_motor_flux_linkage;
    float foc_motor_ld_lq_diff;
    float foc_sat_comp;
    SAT_COMP_MODE foc_sat_comp_mode;
    float l_current_max;
    bool foc_temp_comp;
    foc_observer_type foc_observer_type;
    float foc_pll_kp;
    float foc_pll_ki;
} TFocObserverCfg;

typedef struct {
    TFocObserverCfg *m_conf;
    TFocMotorState m_motor_state;
    float m_gamma_now;
    float m_res_temp_comp;
} TFLuxObserverMotor;

void foc_observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta, float dt,
                         observer_state *state, float *phase, TFLuxObserverMotor *motor);

void foc_pll_run(float phase, float dt, float *phase_var, float *speed_var, TFocObserverCfg *conf);

#endif /* FOC_OBSERVER_H_ */
