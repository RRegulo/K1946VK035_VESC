/* FOC_Control.c — порт VESC mcpwm_foc.c::control_current + update_valpha_vbeta
 * под K1946VK035/K1921VK035 (Cortex-M4F, hard-float).
 *
 * Copyright Benjamin Vedder 2016-2024 (VESC firmware, GPLv3).
 * This file is a derivative of ref/bldc/motor/mcpwm_foc.c.
 */
#include "FOC_Control.h"
#include "FOC_Svm.h"
#include "VESC_Math.h"
#include <math.h>

void vesc_config_default(TVescConfig *cfg)
{
    cfg->foc_motor_r = 0.19f;
    cfg->foc_motor_l = 300.0e-6f;
    cfg->foc_motor_flux_linkage = 3500.0e-6f;
    cfg->foc_motor_ld_lq_diff = 0.0f;

    cfg->foc_observer_type = FOC_OBSERVER_MXLEMMING_LAMBDA_COMP;
    cfg->foc_observer_gain = 9.0e5f;
    cfg->foc_observer_gain_slow = 0.05f;
    cfg->foc_pll_kp = 2000.0f;
    cfg->foc_pll_ki = 30000.0f;

    cfg->foc_current_kp = 0.03f;
    cfg->foc_current_ki = 50.0f;
    cfg->foc_current_filter_const = 0.1f;

    /* Наши реальные dead-time: DBRED+DBFED=40+40 тиков @96МГц = 0.833µs на
   * период. VESC использует foc_dt_us*f_zv*1e-6. */
    cfg->foc_dt_us = 0.833f;
    cfg->foc_f_zv = 25000.0f;

    cfg->l_max_duty = 0.95f;
    cfg->foc_overmod_factor = 1.0f;
    cfg->foc_mag_vd_max = 0.98f;
    cfg->foc_cc_decoupling = FOC_CC_DECOUPLING_DISABLED;

    cfg->foc_sat_comp_mode = SAT_COMP_DISABLED;
    cfg->foc_sat_comp = 0.0f;
    cfg->foc_temp_comp = false;
    cfg->l_current_max = 8.0f;
}

void vesc_update_valpha_vbeta(TVescMotorState *state, const TVescConfig *cfg, float mod_alpha,
                              float mod_beta, float voltage_normalize)
{
    (void)voltage_normalize;

    float s = state->phase_sin;
    float c = state->phase_cos;

    /* Токи в фазах из фильтрованных id/iq (VESC mcpwm_foc.c:5101-5105). */
    const float i_alpha_filter = c * state->id_filter - s * state->iq_filter;
    const float i_beta_filter = c * state->iq_filter + s * state->id_filter;
    const float ia_filter = i_alpha_filter;
    const float ib_filter = -0.5f * i_alpha_filter + SQRT3_BY_2 * i_beta_filter;
    const float ic_filter = -0.5f * i_alpha_filter - SQRT3_BY_2 * i_beta_filter;

    const float mod_alpha_filter_sgn =
        (1.0f / 3.0f) * (2.0f * SIGN(ia_filter) - SIGN(ib_filter) - SIGN(ic_filter));
    const float mod_beta_filter_sgn = ONE_BY_SQRT3 * (SIGN(ib_filter) - SIGN(ic_filter));

    const float mod_comp_fact = cfg->foc_dt_us * 1e-6f * cfg->foc_f_zv;
    mod_alpha -= mod_alpha_filter_sgn * mod_comp_fact;
    mod_beta -= mod_beta_filter_sgn * mod_comp_fact;

    state->mod_alpha_measured = mod_alpha;
    state->mod_beta_measured = mod_beta;

    float abs_rpm = fabsf(RADPS2RPM_f(state->speed_est_rad));
    float filter_const = 1.0f;
    if(abs_rpm < 10000.0f) {
        filter_const = utils_map(abs_rpm, 0.0f, 10000.0f, 0.01f, 1.0f);
    }

    UTILS_LP_FAST(state->mod_alpha_filter, mod_alpha, filter_const);
    UTILS_LP_FAST(state->mod_beta_filter, mod_beta, filter_const);
    UTILS_NAN_ZERO(state->mod_alpha_filter);
    UTILS_NAN_ZERO(state->mod_beta_filter);

    state->v_alpha = state->mod_alpha_filter * (2.0f / 3.0f) * state->v_bus;
    state->v_beta = state->mod_beta_filter * (2.0f / 3.0f) * state->v_bus;
}

void vesc_control_current(TVescMotorState *state, const TVescConfig *cfg, float dt, uint32_t top,
                          uint32_t *duty1, uint32_t *duty2, uint32_t *duty3)
{
    float s = state->phase_sin;
    float c = state->phase_cos;

    float max_duty = fabsf(state->max_duty);
    utils_truncate_number(&max_duty, 0.0f, cfg->l_max_duty);

    /* Park transform (VESC mcpwm_foc.c:4624-4625). */
    state->id = c * state->i_alpha + s * state->i_beta;
    state->iq = c * state->i_beta - s * state->i_alpha;

    UTILS_LP_FAST(state->id_filter, state->id, cfg->foc_current_filter_const);
    UTILS_LP_FAST(state->iq_filter, state->iq, cfg->foc_current_filter_const);

    float Ierr_d = state->id_target - state->id;
    float Ierr_q = state->iq_target - state->iq;

    float ki = cfg->foc_current_ki;
    state->vd_int += Ierr_d * ki * dt;
    state->vq_int += Ierr_q * ki * dt;

    state->vd = state->vd_int + Ierr_d * cfg->foc_current_kp;
    state->vq = state->vq_int + Ierr_q * cfg->foc_current_kp;

    /* Decoupling: default DISABLED; для compile-time оставлены нули. */
    float dec_vd = 0.0f;
    float dec_vq = 0.0f;
    float dec_bemf = 0.0f;
    (void)cfg->foc_cc_decoupling;
    (void)dec_vd;
    (void)dec_vq;
    (void)dec_bemf;

    state->vd -= dec_vd;
    state->vq += dec_vq + dec_bemf;

    float max_v_mag = ONE_BY_SQRT3 * max_duty * state->v_bus * cfg->foc_overmod_factor;

    utils_truncate_number_abs(&state->vd, max_v_mag * cfg->foc_mag_vd_max);
    utils_truncate_number_abs(&state->vd_int, max_v_mag * cfg->foc_mag_vd_max);

    float max_vq = sqrtf(SQ(max_v_mag) - SQ(state->vd));
    UTILS_NAN_ZERO(max_vq);

    utils_truncate_number_abs(&state->vq, max_vq);
    utils_truncate_number_abs(&state->vq_int, max_vq);

    const float voltage_normalize = 1.5f / state->v_bus;
    state->mod_d = state->vd * voltage_normalize;
    state->mod_q = state->vq * voltage_normalize;
    UTILS_NAN_ZERO(state->mod_q_filter);
    UTILS_LP_FAST(state->mod_q_filter, state->mod_q, 0.2f);

    state->i_abs_filter = NORM2_f(state->id_filter, state->iq_filter);

    /* Inverse Park (VESC mcpwm_foc.c:4721-4722). */
    state->mod_alpha_raw = c * state->mod_d - s * state->mod_q;
    state->mod_beta_raw = c * state->mod_q + s * state->mod_d;

    vesc_update_valpha_vbeta(state, cfg, state->mod_alpha_raw, state->mod_beta_raw,
                             voltage_normalize);

    /* vd/vq для телеметрии (из реконструированных v_alpha/v_beta). */
    state->vd = c * state->v_alpha + s * state->v_beta;
    state->vq = c * state->v_beta - s * state->v_alpha;

    foc_svm(state->mod_alpha_raw, state->mod_beta_raw, cfg->l_max_duty, top, duty1, duty2, duty3,
            &state->svm_sector);
}
