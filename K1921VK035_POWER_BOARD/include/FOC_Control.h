/* FOC_Control.h — порт VESC mcpwm_foc.c::control_current + update_valpha_vbeta
 * под K1946VK035/K1921VK035 (Cortex-M4F, hard-float).
 *
 * Copyright Benjamin Vedder 2016-2024 (VESC firmware, GPLv3).
 * This file is a derivative of ref/bldc/motor/mcpwm_foc.c.
 */
#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include "FOC_Observer.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FOC_CC_DECOUPLING_DISABLED = 0,
    FOC_CC_DECOUPLING_CROSS,
    FOC_CC_DECOUPLING_BEMF,
    FOC_CC_DECOUPLING_CROSS_BEMF
} mc_foc_cc_decoupling_mode;

typedef struct {
    /* Мотор */
    float foc_motor_r;
    float foc_motor_l;
    float foc_motor_flux_linkage;
    float foc_motor_ld_lq_diff;

    /* Наблюдатель / PLL */
    foc_observer_type foc_observer_type;
    float foc_observer_gain;
    float foc_observer_gain_slow;
    float foc_pll_kp;
    float foc_pll_ki;

    /* Токовый ПИД */
    float foc_current_kp;
    float foc_current_ki;
    float foc_current_filter_const;

    /* Dead-time компенсация */
    float foc_dt_us;
    float foc_f_zv;

    /* SVM / modulation */
    float l_max_duty;
    float foc_overmod_factor;
    float foc_mag_vd_max;
    mc_foc_cc_decoupling_mode foc_cc_decoupling;

    /* Saturation / temp comp (прокидываются в observer) */
    SAT_COMP_MODE foc_sat_comp_mode;
    float foc_sat_comp;
    bool foc_temp_comp;
    float l_current_max;
} TVescConfig;

typedef struct {
    /* Входы (заполняет caller) */
    float i_alpha;
    float i_beta;
    float id_target;
    float iq_target;
    float v_bus;
    float phase; /* угол ротора, рад (от observer) */
    float phase_sin;
    float phase_cos;
    float max_duty;

    /* Выходы / внутреннее состояние */
    float id, iq;
    float id_filter, iq_filter;
    float i_abs_filter;
    float vd_int, vq_int;
    float vd, vq;
    float mod_d, mod_q;
    float mod_q_filter;
    float mod_alpha_raw, mod_beta_raw;
    float mod_alpha_filter, mod_beta_filter;
    float mod_alpha_measured, mod_beta_measured;
    float v_alpha, v_beta; /* наблюдатель видит эти v */
    float duty_now;
    float speed_est_rad; /* ω для filter_const в update_valpha_vbeta */
    uint32_t svm_sector;
} TVescMotorState;

/* Инициализация дефолтами VESC (mcconf_default.h). */
extern TVescConfig g_vesc_cfg;
extern TVescMotorState g_vesc_state;

/* Инициализация дефолтами VESC (mcconf_default.h). */
void vesc_config_default(TVescConfig *cfg);

/* Реконструкция v_alpha/v_beta из модуляции + dead-time comp + LP-фильтр.
 * voltage_normalize = 1.5f / v_bus. */
void vesc_update_valpha_vbeta(TVescMotorState *state, const TVescConfig *cfg, float mod_alpha,
                              float mod_beta, float voltage_normalize);

/* Полный цикл токового регулятора VESC: Park→PI→decouple→SVM.
 * Возвращает duty1/2/3 [0..top]. */
void vesc_control_current(TVescMotorState *state, const TVescConfig *cfg, float dt, uint32_t top,
                          uint32_t *duty1, uint32_t *duty2, uint32_t *duty3);

#endif /* FOC_CONTROL_H */
