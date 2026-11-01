/*
 * FOC_Core.c
 *
 * Контур тока 6,25 кГц, запуск и управление скоростью.
 * Наблюдатель, PLL, токовый регулятор и SVPWM перенесены из VESC (GPLv3).
 * Clarke/Park/IPark и I/F-пуск адаптированы из демо НПФ Вектор.
 * PWM0/1/2 соответствуют фазам A/B/C; CMPA = k_pwm - t_on.
 */

#include "FOC_Core.h"
#include "FOC_Control.h"
#include "FOC_Svm.h"
#include "VESC_Math.h"
#include "PWM_Control.h"
#include "FOC_Protection.h"
#include "UART_Debug.h"

extern volatile uint32_t g_tick_100us;

#ifdef FOC_HOST_TEST
extern foc_pwm_stub_t pwm0_stub;
extern foc_pwm_stub_t pwm1_stub;
extern foc_pwm_stub_t pwm2_stub;
#endif

/* Сырые коды SEQ0 — живут в PWM_Control.c */
extern volatile uint16_t g_adc_raw[4];
extern volatile uint16_t g_adc_off[4];

TClarke g_clarke = CLARKE_DEFAULTS;
TPark g_park = PARK_DEFAULTS;
TIPark g_ipark = IPARK_DEFAULTS;
TPidReg3 g_pid_d;
TPidReg3 g_pid_q;
TFOC_PWM g_pwm;

volatile uint8_t g_foc_enable = 0;
float g_foc_angle = 0.0f;
float g_foc_id_ref = 0.0f;
float g_foc_iq_ref = 0.0f;
float g_foc_udc = 0.0f;
volatile uint8_t g_foc_inv_i = 1; /* полярность ОУ/шунта инвертирована */
volatile uint8_t g_foc_inv_ang = 0;
volatile uint8_t g_foc_adc_map = 0; /* Ia=ADC0, Ib=ADC1 (фаза A/B по схеме платы) */
volatile uint8_t g_foc_pwm_map = 0; /* SVM: A->PWM0, B->PWM1, C->PWM2 */
volatile uint8_t g_foc_obs_vrot =
    0; /* 0..2: поворот v-входа наблюдателя на -120°*k (кадровый сдвиг) */
volatile uint8_t g_foc_pwm_swap = 0;   /* 1 = обмен GammaA<->GammaB (зеркало) */
volatile float g_foc_ualpha_dc = 0.0f; /* j-тест R: DC Ualpha, pu */
volatile float g_foc_ubeta_dc = 0.0f;  /* j-тест R: DC Ubeta, pu */
volatile uint8_t g_foc_obs_off = 0; /* VESC: v и i в одной рамке (инвертированы вместе), вычет 0 */

/* I/F */
TRMPCtrl g_ramp = RMP_CTRL_DEFAULTS;
volatile uint8_t g_if_active = 0;
float g_if_speed_ref = FOC_SPEED_DEFAULT;
float g_if_flux_ref = FOC_FLUX_DEFAULT;

/* Sensorless observer + state machine.
 * Параметры моторов (R — фазное, L — фаза, lambda — потокосцепление ротора).
 *
 * БОЛЬШОЙ: 3115 12N14P 450KV 12S. Замер R_phase = 0.19 Ом (j-тест),
 * L = 300 мкГн, lambda = 3500 мкВб.
 * МЕЛКИЙ: BrotherHobby/T-Motor F90 2806.5 1300KV: R 38 мОм, L 15 мкГн,
 * B 700 мкВб, pp 7. */
TMotorParams g_motor = {190, 300, 3500, 7, 1000, 22000};
TFluxObserver g_observer;
volatile foc_ctrl_state_t g_foc_ctrl_state = FOC_CTRL_STOP;
volatile uint32_t g_foc_vector_blend = 0;
TPidReg3 g_pid_speed;

volatile uint32_t g_foc_cycles = 0;
volatile uint32_t g_foc_cycles_max = 0;
volatile uint32_t g_foc_isr_count = 0;

/* Open-loop / удержание вектора . */
volatile float g_ol_current = 0.0f;
volatile float g_ol_freq_hz = 0.0f;
volatile float g_ol_freq_target = 0.0f;
volatile float g_ol_angle = 0.0f;
volatile float g_ol_hold_angle = 0.0f;
static float s_hold_cur = 0.0f;
static float s_hold_tgt = 0.0f;
static float s_hold_rate = 0.0f;

/* Параметры sensorless-пуска и регулятора скорости VESC. */
volatile float g_foc_i_cmd = 4.0f;
volatile float g_foc_openloop_rpm = FOC_SL_OPENLOOP_RPM;
volatile float g_foc_sl_hyst = FOC_SL_OPENLOOP_HYST;
volatile float g_foc_sl_time = FOC_SL_OPENLOOP_TIME;
volatile float g_foc_sl_t_ramp = FOC_SL_OPENLOOP_T_RAMP;
volatile float g_s_pid_kp = 0.004f;
volatile float g_s_pid_ki = 0.004f;
volatile float g_s_pid_kd = 0.0001f;
volatile float g_s_pid_kd_filter = 0.2f;
volatile float g_s_pid_min_erpm = 900.0f;
volatile float g_s_pid_ramp_erpms_s = 25000.0f;
volatile float g_foc_max_erpm = 100000.0f;
volatile float g_foc_cc_tc_us = 1000.0f;
volatile float g_foc_obs_gain = FOC_OBSERVER_GAIN;
volatile uint8_t g_foc_obs_vraw = 0u;
volatile uint8_t g_foc_outputs_on = 0u;
volatile float g_foc_observer_offset = 0.0f;
volatile uint8_t g_cl_phase = 0;

static volatile uint8_t s_vesc_mode = 0u;
static volatile float s_iq_set = 0.0f;
static float s_min_rpm_timer = 0.0f;
static float s_min_rpm_hyst_timer = 0.0f;
static float s_phase_ovr = 0.0f;
static float s_x1_ovr = 0.0f;
static float s_x2_ovr = 0.0f;
static uint8_t s_ovr_active = 0u;
static float s_phase_obs = 0.0f;
static float s_duty_now = 0.0f;
static volatile uint8_t s_spd_mode = 0u;
static volatile float s_spd_cmd_erpm = 0.0f;
static float s_spd_set_erpm = 0.0f;
static float s_spd_i = 0.0f;
static float s_spd_prev_err = 0.0f;
static float s_spd_dfilt = 0.0f;

TVescConfig g_vesc_cfg;
TVescMotorState g_vesc_state;

/* Гейны цепи rev1.0 (float): K_I=1/37.2 А/код; K_V=3.3/4096*16 В/код;
 * K_VPU=1/24 1/В. */
#define FOC_K_I 0.0268817f
#define FOC_K_V 0.012890625f
#define FOC_K_VPU 0.0416667f

static float g_foc_udc_pu = 0.0f;

void foc_init(void)
{
    g_clarke.calc = clarke_calc;
    g_park.calc = park_calc;
    g_ipark.calc = ipark_calc;

    g_pwm.k_pwm = 2000; /* = TBPRD (up-down, /) */
    g_pwm.U_lim = 1.0f;
    g_pwm.UdCompK = 1.0f;
    g_pwm.UdCompEnable = 0; /* компенсация Udc выключена */
    g_pwm.GammaA = 0;
    g_pwm.GammaB = 0;
    g_pwm.GammaC = 0;

    g_ramp.input = 0.0f;
    g_ramp.output = 0.0f;
    g_ramp.Ts = FOC_TS;
    g_ramp.T = FOC_RAMP_T;
    g_ramp.calc = rmp_ctrl_calc;
    g_ramp.slow_calc = rmp_ctrl_slow_calc;
    rmp_ctrl_slow_calc(&g_ramp);
    g_if_speed_ref = FOC_SPEED_DEFAULT;
    g_if_flux_ref = FOC_FLUX_DEFAULT;
    g_if_active = 0;
    g_foc_angle = 0.0f;

    /* VESC current-controller config (mcconf_default.h). */
    vesc_config_default(&g_vesc_cfg);

    /* Observer (VESC): конфиг + состояние. */
    g_observer = (TFluxObserver){0};
    g_observer.cfg.foc_motor_ld_lq_diff = 0.0f;
    g_observer.cfg.foc_sat_comp = 0.0f;
    g_observer.cfg.foc_sat_comp_mode = SAT_COMP_DISABLED;
    g_observer.cfg.l_current_max = FOC_VECTOR_SPEED_MAX_A;
    g_observer.cfg.foc_temp_comp = false;
    g_observer.cfg.foc_observer_type = FOC_OBSERVER_TYPE_DEFAULT;
    g_observer.cfg.foc_pll_kp = FOC_OBS_PLL_KP;
    g_observer.cfg.foc_pll_ki = FOC_OBS_PLL_KI;
    g_observer.motor.m_conf = &g_observer.cfg;
    g_observer.motor.m_gamma_now = 0.0f;
    g_observer.motor.m_res_temp_comp = 1.0f;
    flux_observer_slow_calc(&g_observer);

    g_vesc_state = (TVescMotorState){0};

    g_pid_speed.Kp_reg3 = FOC_VECTOR_SPEED_KP;
    g_pid_speed.Ki_reg3 = FOC_VECTOR_SPEED_KI;
    g_pid_speed.Kc_reg3 = 0.0f;
    g_pid_speed.Kd_reg3 = 0.0f;
    g_pid_speed.pid_out_max = FOC_VECTOR_SPEED_MAX_A;
    g_pid_speed.pid_out_min = -FOC_VECTOR_SPEED_MAX_A;
    g_pid_speed.DeadZone = 0.0f;
    g_pid_speed.Kf_d = 0.0f;
    g_pid_speed.DiffDelim = 1;
    g_pid_speed.DiffCounter = 0;
    g_pid_speed.saterr_reg3Add = 0.0f;
    pid_reg3_reset(&g_pid_speed);

    g_foc_ctrl_state = FOC_CTRL_STOP;
}

/* ENTRY-ветка CTRL_RUN_I2F (SM_Ctrl.c:217-238): сброс интеграторов,
 * rmp.output=0, разрешение записи CMPA. Вызывается из main/UART. */
bool foc_outputs_arm(void)
{
    if(g_fault_flags != 0u || g_foc_state != STATE_NORMAL)
        return false;
    PWM0->CMPA_bit.CMPA = (uint16_t)g_pwm.k_pwm;
    PWM1->CMPA_bit.CMPA = (uint16_t)g_pwm.k_pwm;
    PWM2->CMPA_bit.CMPA = (uint16_t)g_pwm.k_pwm;
#ifndef FOC_HOST_TEST
    pwm_module_on();
    PWM_FaultIrq_Drain();
#endif
    foc_protection_reset();
    g_foc_outputs_on = 1u;
    return true;
}

bool foc_fault_clear(void)
{
    if(g_foc_ctrl_state != FOC_CTRL_STOP || g_foc_enable || g_manual_pwm_active)
        return false;
    /* Аппаратные трипы снимаются только перезапуском. */
    if((g_fault_flags & (uint16_t)~(FAULT_UV | FAULT_OV)) != 0u)
        return false;
    for(unsigned i = 0; i < 3u; ++i) {
        float current = ((float)g_adc_raw[i] - (float)g_adc_off[i]) * FOC_K_I;
        if(fabsf(current) > g_foc_i_trip)
            return false;
    }
    if((float)g_adc_raw[3] * FOC_K_V > g_foc_vbus_max)
        return false;
    foc_protection_reset();
    g_fault_flags = 0u;
    g_foc_state = STATE_NORMAL;
    return true;
}

void foc_if_start(void)
{
    if(!foc_outputs_arm())
        return;
    g_ramp.output = 0.0f;
    g_ramp.input = g_if_speed_ref;
    rmp_ctrl_slow_calc(&g_ramp);
    g_pwm.U_lim = FOC_IF_U_LIM; /* потолок на время I/F */

    /* Начальная theta_est = I/F-угол, чтобы observer стартовал без скачка. */
    g_observer.theta_est = g_foc_angle;
    g_observer.theta_i_prev = g_foc_angle;

    /* Сброс интеграторов VESC current controller. */
    g_vesc_state.vd_int = 0.0f;
    g_vesc_state.vq_int = 0.0f;
    g_vesc_state.id_filter = 0.0f;
    g_vesc_state.iq_filter = 0.0f;
    g_vesc_state.mod_alpha_filter = 0.0f;
    g_vesc_state.mod_beta_filter = 0.0f;
    g_vesc_state.mod_q_filter = 0.0f;

    g_foc_ualpha_dc = 0.0f;
    g_foc_ubeta_dc = 0.0f;
    g_if_active = 1;
    g_foc_enable = 1;
    g_foc_ctrl_state = FOC_CTRL_I2F;
}

/* Стоп: Hi-Z, сброс регуляторов и запрет записи PWM. */
void foc_if_stop(void)
{
#ifndef FOC_HOST_TEST
    pwm_module_off();
#endif
    g_foc_outputs_on = 0u;
    g_manual_pwm_active = 0u;
    g_cl_phase = 0;
    s_vesc_mode = 0u;
    s_spd_mode = 0u;
    s_iq_set = 0.0f;
    s_phase_obs = 0.0f;
    s_phase_ovr = 0.0f;
    s_ovr_active = 0u;
    s_min_rpm_timer = 0.0f;
    s_min_rpm_hyst_timer = 0.0f;
    s_duty_now = 0.0f;
    s_spd_i = 0.0f;
    s_spd_prev_err = 0.0f;
    s_spd_dfilt = 0.0f;
    g_if_active = 0;
    g_foc_enable = 0;
    g_foc_id_ref = 0.0f;
    g_foc_iq_ref = 0.0f;
    g_foc_ctrl_state = FOC_CTRL_STOP;
    g_observer.state.x1 = 0.0f;
    g_observer.state.x2 = 0.0f;
    g_observer.state.i_alpha_last = 0.0f;
    g_observer.state.i_beta_last = 0.0f;
    g_observer.state.lambda_est = g_observer.lambda;
    g_observer.theta_i = 0.0f;
    g_observer.theta_est = 0.0f;
    g_observer.theta_i_prev = 0.0f;
    g_observer.omega_est = 0.0f;
    g_observer.psi_mag = 0.0f;
    g_pwm.U_lim = 1.0f; /* вернуть полный диапазон */

    g_vesc_state.vd_int = 0.0f;
    g_vesc_state.vq_int = 0.0f;
    g_vesc_state.id_filter = 0.0f;
    g_vesc_state.iq_filter = 0.0f;
    g_vesc_state.mod_alpha_filter = 0.0f;
    g_vesc_state.mod_beta_filter = 0.0f;
    g_vesc_state.mod_q_filter = 0.0f;

#ifndef FOC_HOST_TEST
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    PWM0->CMPA_bit.CMPA = (uint16_t)g_pwm.k_pwm;
    PWM1->CMPA_bit.CMPA = (uint16_t)g_pwm.k_pwm;
    PWM2->CMPA_bit.CMPA = (uint16_t)g_pwm.k_pwm;
    __set_PRIMASK(primask);
#else
    pwm0_stub.CMPA_bit.CMPA = (uint32_t)g_pwm.k_pwm;
    pwm1_stub.CMPA_bit.CMPA = (uint32_t)g_pwm.k_pwm;
    pwm2_stub.CMPA_bit.CMPA = (uint32_t)g_pwm.k_pwm;
#endif
}

/* Переход I/F -> VECTOR (PLL observer angle, Id=flux, Iq=0). */
void foc_vector_start(void)
{
    if(g_foc_ctrl_state != FOC_CTRL_I2F)
        return;
    g_if_active = 0;
    g_foc_iq_ref = 0.0f;

    /* В VECTOR отдаём токовому контуру полный диапазон напряжения. */
    g_pwm.U_lim = 1.0f;

    /* Плавный blend I/F -> VECTOR: сразу управляем проверенным I/F-углом. */
    g_foc_vector_blend = FOC_VECTOR_BLEND_CNT;
    g_foc_ualpha_dc = 0.0f; /* страховка: j-тест не должен жить в VECTOR */
    g_foc_ubeta_dc = 0.0f;
    g_foc_ctrl_state = FOC_CTRL_VECTOR;
}

/* Любое состояние -> STOP. */
void foc_ctrl_stop(void)
{
    foc_if_stop();
}

/* Сброс интеграторов/фильтров VESC current controller. */
static void vesc_state_reset(void)
{
    g_vesc_state.vd_int = 0.0f;
    g_vesc_state.vq_int = 0.0f;
    g_vesc_state.id_filter = 0.0f;
    g_vesc_state.iq_filter = 0.0f;
    g_vesc_state.mod_alpha_filter = 0.0f;
    g_vesc_state.mod_beta_filter = 0.0f;
    g_vesc_state.mod_q_filter = 0.0f;
}

/* Разомкнутое вращение: ток Id на вращающемся угле, частота в эл. Гц. */
void foc_openloop_start(float current_a, float freq_hz)
{
    /* Уже вращаемся: меняем только цель и ток, без сброса интеграторов. */
    if(g_foc_ctrl_state == FOC_CTRL_OPENLOOP && g_foc_enable) {
        g_ol_current = current_a;
        g_ol_freq_target = freq_hz;
        return;
    }
    if(g_foc_ctrl_state != FOC_CTRL_STOP)
        foc_if_stop();
    if(!foc_outputs_arm())
        return;
    vesc_state_reset();
    g_observer.state.x1 = 0.0f;
    g_observer.state.x2 = 0.0f;
    g_pwm.U_lim = 1.0f;
    g_ol_current = current_a;
    g_ol_freq_hz = 0.0f; /* разгон с нуля, ротор сначала выравнивается */
    g_ol_freq_target = freq_hz;
    g_foc_ualpha_dc = 0.0f;
    g_foc_ubeta_dc = 0.0f;
    g_if_active = 0;
    g_foc_id_ref = current_a;
    g_foc_iq_ref = 0.0f;
    g_foc_enable = 1;
    g_foc_ctrl_state = FOC_CTRL_OPENLOOP;
}

/* Удержание вектора: ток Id на фиксированном эл. угле (рад). */
void foc_hold_start(float current_a, float angle_rad)
{
    float a = angle_rad;
    if(g_foc_ctrl_state != FOC_CTRL_STOP)
        foc_if_stop();
    utils_norm_angle_rad(&a);
    if(!foc_outputs_arm())
        return;
    vesc_state_reset();
    g_pwm.U_lim = 1.0f;
    g_ol_current = current_a;
    g_ol_hold_angle = a;
    g_ol_angle = a;
    s_hold_cur = a;
    s_hold_tgt = a;
    s_hold_rate = 0.0f;
    g_foc_ualpha_dc = 0.0f;
    g_foc_ubeta_dc = 0.0f;
    g_if_active = 0;
    g_foc_id_ref = current_a;
    g_foc_iq_ref = 0.0f;
    g_foc_enable = 1;
    g_foc_ctrl_state = FOC_CTRL_HOLD;
}

/* Изменение удержания на ходу: ток и (опционально) доворот угла. */
void foc_hold_move(float current_a, float target_rad, float rate_rad_s)
{
    if(g_foc_ctrl_state == FOC_CTRL_HOLD && g_foc_enable) {
        g_ol_current = current_a;
        s_hold_rate = rate_rad_s > 0.0f ? rate_rad_s : 0.0f;
        s_hold_tgt = target_rad;
        if(s_hold_rate <= 0.0f)
            s_hold_cur = target_rad;
        return;
    }
    float a = target_rad;
    utils_norm_angle_rad(&a);
    foc_hold_start(current_a, a);
}

/* Start sensorless from standstill (VESC mc_interface: set_current/set_pid_speed
 * while not RUNNING). Returns false if it cannot start. */
static bool vesc_start_from_stop(void)
{
    if(g_foc_ctrl_state != FOC_CTRL_STOP || g_fault_flags != 0u || g_foc_state != STATE_NORMAL)
        return false;
    if(!foc_outputs_arm())
        return false;
    vesc_state_reset();
    g_observer.state.x1 = 0.0f;
    g_observer.state.x2 = 0.0f;
    g_observer.theta_i = 0.0f;
    g_observer.theta_est = 0.0f;
    g_observer.theta_i_prev = 0.0f;
    g_observer.omega_est = 0.0f;
    g_pwm.U_lim = 1.0f;
    g_foc_ualpha_dc = 0.0f;
    g_foc_ubeta_dc = 0.0f;
    g_if_active = 0;
    g_foc_vector_blend = 0;
    g_foc_id_ref = 0.0f;
    g_foc_iq_ref = 0.0f;
    s_phase_obs = 0.0f;
    s_phase_ovr = 0.0f;
    s_ovr_active = 0u;
    s_min_rpm_timer = 0.0f;
    s_min_rpm_hyst_timer = 0.0f;
    s_duty_now = 0.0f;
    s_spd_set_erpm = 0.0f;
    s_spd_i = 0.0f;
    s_spd_prev_err = 0.0f;
    s_spd_dfilt = 0.0f;
    s_vesc_mode = 1u;
    g_cl_phase = 2u;
    g_foc_enable = 1;
    g_foc_ctrl_state = FOC_CTRL_VECTOR;
    return true;
}

/* mcpwm_foc_set_current(): CONTROL_MODE_CURRENT, id = 0. */
bool foc_closed_request(float iq_a)
{
    if(!(iq_a >= 0.0f) || iq_a > g_foc_i_cmd || g_fault_flags != 0u || g_foc_state != STATE_NORMAL)
        return false;
    s_spd_mode = 0u;
    s_iq_set = iq_a;
    if(s_vesc_mode && g_foc_ctrl_state == FOC_CTRL_VECTOR)
        return true;
    if(iq_a < FOC_CC_MIN_CURRENT)
        return true; /* VESC: below cc_min_current the motor does not start */
    return vesc_start_from_stop();
}

/* mcpwm_foc_set_pid_speed(): ramp enabled (s_pid_ramp_erpms_s > 0). Starting
 * from standstill happens BEFORE the speed mode is entered (the outputs are armed before entering the speed mode). */
bool foc_closed_speed_request(float erpm)
{
    if(!(erpm >= 0.0f) || g_fault_flags != 0u || g_foc_state != STATE_NORMAL)
        return false;
    uint8_t running = (s_vesc_mode && g_foc_ctrl_state == FOC_CTRL_VECTOR) ? 1u : 0u;
    if(!running) {
        if(erpm < g_s_pid_min_erpm)
            return true; /* VESC: below s_pid_min_erpm the motor does not start */
        if(!vesc_start_from_stop())
            return false;
    }
    if(!s_spd_mode)
        s_spd_set_erpm = running ? RADPS2RPM_f(g_observer.omega_est) : 0.0f;
    s_spd_cmd_erpm = erpm;
    s_spd_mode = 1u;
    return true;
}

/* Leave VESC speed/current mode (manual openloop/hold/detect take over). */
void foc_closed_release(void)
{
    s_spd_mode = 0u;
    s_iq_set = 0.0f;
    s_spd_i = 0.0f;
    s_spd_prev_err = 0.0f;
    s_spd_dfilt = 0.0f;
}

/* foc_math.c foc_run_pid_control_speed() — literal, rpm = ERPM. */
static void vesc_run_pid_control_speed(float dt)
{
    if(!s_spd_mode) {
        s_spd_i = 0.0f;
        s_spd_prev_err = 0.0f;
        s_spd_dfilt = 0.0f;
        return;
    }
    utils_step_towards(&s_spd_set_erpm, s_spd_cmd_erpm, g_s_pid_ramp_erpms_s * dt);
    utils_truncate_number(&s_spd_set_erpm, 0.0f, g_foc_max_erpm);

    float rpm = RADPS2RPM_f(g_observer.omega_est);
    float error = s_spd_set_erpm - rpm;

    if(fabsf(s_spd_set_erpm) < g_s_pid_min_erpm) {
        s_spd_i = 0.0f;
        s_spd_prev_err = error;
        s_iq_set = 0.0f;
        return;
    }

    float p_term = error * g_s_pid_kp * (1.0f / FOC_CC_DECOUPLING_FACTOR);
    float d_term = (error - s_spd_prev_err) * (g_s_pid_kd / dt) * (1.0f / FOC_CC_DECOUPLING_FACTOR);
    UTILS_LP_FAST(s_spd_dfilt, d_term, g_s_pid_kd_filter);
    d_term = s_spd_dfilt;
    s_spd_prev_err = error;

    float output = p_term + s_spd_i + d_term;
    utils_truncate_number_abs(&output, 1.0f);

    s_spd_i += error * g_s_pid_ki * dt * (1.0f / FOC_CC_DECOUPLING_FACTOR);
    utils_truncate_number_abs(&s_spd_i, 1.0f);

#if !FOC_S_PID_ALLOW_BRAKING
    if(rpm > 20.0f && output < 0.0f)
        output = 0.0f;
    if(rpm < -20.0f && output > 0.0f)
        output = 0.0f;
#endif
    s_iq_set = output * g_foc_i_cmd;
}

/* mcpwm_foc.c timer_update(): openloop override while the PLL cannot track. */
static void vesc_timer_update(float dt)
{
    const float t_lock = FOC_SL_OPENLOOP_T_LOCK;
    const float t_ramp = g_foc_sl_t_ramp;
    const float t_const = g_foc_sl_time;

    float openloop_current = fabsf(g_vesc_state.iq_filter);
    openloop_current += FOC_SL_OPENLOOP_BOOST_Q;
    if(FOC_SL_OPENLOOP_MAX_Q > 0.0f)
        utils_truncate_number(&openloop_current, 0.0f, FOC_SL_OPENLOOP_MAX_Q);

    float openloop_rpm_max =
        utils_map(openloop_current, 0.0f, g_foc_i_cmd, FOC_SL_OPENLOOP_RPM_LOW * g_foc_openloop_rpm,
                  g_foc_openloop_rpm);
    utils_truncate_number_abs(&openloop_rpm_max, g_foc_openloop_rpm);

    float openloop_rpm = openloop_rpm_max;
    {
        float time_fwd = t_lock + t_ramp + t_const - s_min_rpm_timer;
        if(time_fwd < t_lock)
            openloop_rpm = 0.0f;
        else if(time_fwd < (t_lock + t_ramp))
            openloop_rpm = utils_map(time_fwd, t_lock, t_lock + t_ramp, 0.0f, openloop_rpm);
    }
    utils_truncate_number_abs(&openloop_rpm, openloop_rpm_max);

    float add_min_speed = 0.0f;
    if(s_duty_now > 0.0f)
        add_min_speed = RPM2RADPS_f(openloop_rpm) * dt;
    else
        add_min_speed = -RPM2RADPS_f(openloop_rpm) * dt;

    if(fabsf(g_observer.omega_est) < RPM2RADPS_f(openloop_rpm_max) &&
       s_min_rpm_hyst_timer < g_foc_sl_hyst) {
        s_min_rpm_hyst_timer += dt;
    } else if(s_min_rpm_hyst_timer > 0.0f) {
        s_min_rpm_hyst_timer -= dt;
    }

    if(fabsf(s_duty_now) < 0.001f) {
        s_min_rpm_hyst_timer = 0.0f;
        s_min_rpm_timer = 0.0f;
        s_ovr_active = 0u;
    }

    uint8_t started_now = 0u;
    if(s_min_rpm_hyst_timer >= g_foc_sl_hyst && s_min_rpm_timer <= 0.0001f) {
        s_min_rpm_timer = t_lock + t_ramp + t_const;
        started_now = 1u;
    }

    if(s_min_rpm_timer > 0.0f) {
        s_phase_ovr += add_min_speed;
        /* When the motor gets stuck it tends to be 90 degrees off, so start the
     * open loop sequence by correcting with 60 degrees. */
        if(started_now) {
            if(s_duty_now > 0.0f)
                s_phase_ovr += _PI / 3.0f;
            else
                s_phase_ovr -= _PI / 3.0f;
        }
        utils_norm_angle_rad(&s_phase_ovr);
        s_ovr_active = 1u;
        s_min_rpm_timer -= dt;
        s_min_rpm_hyst_timer = 0.0f;

        /* Set observer state to help it start tracking when leaving open loop. */
        float sn, cs;
        utils_fast_sincos_better(s_phase_ovr + (float)SIGN(s_duty_now) * (_PI / 4.0f), &sn, &cs);
        s_x1_ovr = cs * g_observer.cfg.foc_motor_flux_linkage;
        s_x2_ovr = sn * g_observer.cfg.foc_motor_flux_linkage;
    } else {
        s_phase_ovr = s_phase_obs;
        s_ovr_active = 0u;
    }
}

/* VESC timer_thread (~1 kHz) driven from the main loop: timer_update + speed
 * PID. Runs while a VESC current or speed command is active. */
void foc_vesc_timer_tick(void)
{
    static uint32_t s_last = 0u;
    uint32_t now = g_tick_100us;
    uint32_t delta = now - s_last;
    uint32_t steps = delta / FOC_TS_PER_VESC_TICK;
    if(steps == 0u)
        return;
    s_last += steps * FOC_TS_PER_VESC_TICK;
    if(steps > 10u)
        steps = 10u; /* discard excess backlog instead of replaying stale time */

    if(!(s_vesc_mode && g_foc_ctrl_state == FOC_CTRL_VECTOR))
        return;

    const float dt = (float)steps * 1e-3f;
    vesc_timer_update(dt);
    vesc_run_pid_control_speed(dt);
}

void foc_dwt_init(void)
{
#ifndef FOC_HOST_TEST
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

/* Параметры observer / VESC current controller из g_motor
 * (мОм -> Ом, мкГн -> Гн, мкВб -> Вб). */
void flux_observer_slow_calc(TFluxObserver *p)
{
    p->R = (float)g_motor.R_mOhm * 1e-3f;
    p->L = (float)g_motor.L_uH * 1e-6f;
    p->lambda = (float)g_motor.lambda_uWb * 1e-6f;

    p->cfg.foc_motor_r = p->R;
    p->cfg.foc_motor_l = p->L;
    p->cfg.foc_motor_flux_linkage = p->lambda;
    p->state.lambda_est = p->lambda;

    /* Синхронизируем VESC current-controller. */
    g_vesc_cfg.foc_motor_r = p->R;
    g_vesc_cfg.foc_motor_l = p->L;
    g_vesc_cfg.foc_motor_flux_linkage = p->lambda;
    /* VESC: conf_general_calc_apply_foc_cc_kp_ki_gain. */
    if(p->L > 0.0f && p->R > 0.0f && p->lambda > 0.0f && g_foc_cc_tc_us > 0.0f) {
        float bw = 1.0f / (g_foc_cc_tc_us * 1e-6f);
        g_vesc_cfg.foc_current_kp = p->L * bw;
        g_vesc_cfg.foc_current_ki = p->R * bw;
        g_foc_obs_gain = 1.0e-3f / (p->lambda * p->lambda) * 1e6f;
    }
}

/* VESC foc_svm + запись CMPA. Sat на U_lim (<=0.866), прямая карта PWM0<-A,
 * PWM1<-B, PWM2<-C. CMPA = k_pwm - t_on: наш AQ инвертирован.
 * Используется только для j-теста (DC-вектор); основной путь пишет CMPA
 * напрямую из duty, возвращённых vesc_control_current. */
void foc_svpwm_update(TFOC_PWM *p)
{
    float ualpha = p->UalphaRef;
    float ubeta = p->UbetaRef;
    float max_mod = p->U_lim;

    p->U_mag = NORM2_f(ualpha, ubeta);

    if(max_mod > SQRT3_BY_2)
        max_mod = SQRT3_BY_2; /* за пределами — перемодуляция */

    /* компенсация Udc (по умолчанию выключена). */
    p->UdCorTmp = 1.0f / (1.0f + (g_foc_udc_pu - 1.0f) * p->UdCompK);
    if(p->UdCompEnable & 1) {
        ualpha *= p->UdCorTmp;
        ubeta *= p->UdCorTmp;
    }

    if(utils_saturate_vector_2d(&ualpha, &ubeta, max_mod))
        p->ULimitation = 1;
    else
        p->ULimitation = 0;

    p->UalphaNorm = ualpha;
    p->UbetaNorm = ubeta;

    uint32_t tA, tB, tC, sector;
    foc_svm(ualpha, ubeta, max_mod, p->k_pwm, &tA, &tB, &tC, &sector);
    p->sector = (int16_t)sector;

    p->GammaA = p->k_pwm - tA;
    p->GammaB = p->k_pwm - tB;
    p->GammaC = p->k_pwm - tC;

    /* Циклическая пермутация фаз SVM->PWM (аналог adc-маппинга 'm'):
   * e=0: A->PWM0,B->PWM1,C->PWM2; e=1: A->PWM1,B->PWM2,C->PWM0;
   * e=2: A->PWM2,B->PWM0,C->PWM1. */
    uint32_t ga = p->GammaA, gb = p->GammaB, gc = p->GammaC;
    switch(g_foc_pwm_map) {
        default:
        case 0:
            break;
        case 1:
            p->GammaA = gb;
            p->GammaB = gc;
            p->GammaC = ga;
            break;
        case 2:
            p->GammaA = gc;
            p->GammaB = ga;
            p->GammaC = gb;
            break;
    }

    /* Транспозиция A<->B (зеркало; cyclic-пермутациями m/e/k не выразима). */
    if(g_foc_pwm_swap) {
        uint32_t gswap = p->GammaA;
        p->GammaA = p->GammaB;
        p->GammaB = gswap;
    }

    if(g_foc_enable) {
        DINT;
        PWM0->CMPA_bit.CMPA = (uint16_t)p->GammaA;
        PWM1->CMPA_bit.CMPA = (uint16_t)p->GammaB;
        PWM2->CMPA_bit.CMPA = (uint16_t)p->GammaC;
        EINT;
    }
}

/* Запись duty (tA/tB/tC от vesc_control_current) в CMPA с учётом
 * пермутаций фаз (g_foc_pwm_map / g_foc_pwm_swap). */
static void foc_cmpa_from_duties(uint32_t tA, uint32_t tB, uint32_t tC)
{
    g_pwm.GammaA = g_pwm.k_pwm - tA;
    g_pwm.GammaB = g_pwm.k_pwm - tB;
    g_pwm.GammaC = g_pwm.k_pwm - tC;

    uint32_t ga = g_pwm.GammaA, gb = g_pwm.GammaB, gc = g_pwm.GammaC;
    switch(g_foc_pwm_map) {
        default:
        case 0:
            break;
        case 1:
            g_pwm.GammaA = gb;
            g_pwm.GammaB = gc;
            g_pwm.GammaC = ga;
            break;
        case 2:
            g_pwm.GammaA = gc;
            g_pwm.GammaB = ga;
            g_pwm.GammaC = gb;
            break;
    }

    if(g_foc_pwm_swap) {
        uint32_t gswap = g_pwm.GammaA;
        g_pwm.GammaA = g_pwm.GammaB;
        g_pwm.GammaB = gswap;
    }

    if(g_foc_enable) {
        DINT;
        PWM0->CMPA_bit.CMPA = (uint16_t)g_pwm.GammaA;
        PWM1->CMPA_bit.CMPA = (uint16_t)g_pwm.GammaB;
        PWM2->CMPA_bit.CMPA = (uint16_t)g_pwm.GammaC;
        EINT;
    }
}

/* Быстрое кольцо: ADC_SEQ0_IRQHandler, 6,25 кГц. FPU M4F. */
void foc_fast_calc(void)
{
    float ia, ib;

    g_foc_isr_count++;

    {
        float adc0 = ((float)g_adc_raw[0] - (float)g_adc_off[0]) * FOC_K_I;
        float adc1 = ((float)g_adc_raw[1] - (float)g_adc_off[1]) * FOC_K_I;
        float adc2 = ((float)g_adc_raw[2] - (float)g_adc_off[2]) * FOC_K_I;
        switch(g_foc_adc_map) {
            default:
            case 0:
                ia = adc0;
                ib = adc1;
                break;
            case 1:
                ia = adc1;
                ib = adc2;
                break;
            case 2:
                ia = adc2;
                ib = adc0;
                break;
        }
        if(g_foc_inv_i) {
            ia = -ia;
            ib = -ib;
        }
    }

    g_foc_udc = (float)g_adc_raw[3] * FOC_K_V;
    g_foc_udc_pu = g_foc_udc * FOC_K_VPU;
    /* защита от деления на ноль/переполнения в UdComp. */
    if(g_foc_udc_pu < 0.1f)
        g_foc_udc_pu = 0.1f;
    else if(g_foc_udc_pu > 4.0f)
        g_foc_udc_pu = 4.0f;

    /* I/F-ветка (CTRL_RUN_I2F): рампа скорости, theta += omega_nom*Ts*rmp,
   * Id=i_flux, Iq=0. Id растёт вместе с рампой, не ступенькой. */
    uint8_t run_if_angle =
        g_if_active || (g_foc_ctrl_state == FOC_CTRL_VECTOR && g_foc_vector_blend > 0);
    if(run_if_angle) {
        g_ramp.input = g_if_speed_ref;
        rmp_ctrl_calc(&g_ramp);
        float step = FOC_IF_OMEGA_NOM * FOC_TS * g_ramp.output;
        if(g_foc_inv_ang)
            g_foc_angle -= step;
        else
            g_foc_angle += step;
        utils_norm_angle_rad(&g_foc_angle);
        if(g_if_speed_ref > 0.01f) {
            float frac = g_ramp.output / g_if_speed_ref;
            if(frac > 1.0f)
                frac = 1.0f;
            if(frac < 0.0f)
                frac = 0.0f;
            g_foc_id_ref = g_if_flux_ref * frac;
        } else {
            g_foc_id_ref = g_if_flux_ref; /* w=0: статическое удержание */
        }
        if(g_if_active)
            g_foc_iq_ref = 0.0f;
    }

    g_clarke.as = ia;
    g_clarke.bs = ib;
    clarke_calc(&g_clarke);

    /* Open-loop (вращение) и удержание вектора: ток Id на заданном угле,
   * наблюдатель/PLL не используются . */
    if(g_foc_ctrl_state == FOC_CTRL_OPENLOOP || g_foc_ctrl_state == FOC_CTRL_HOLD) {
        float theta;
        float omega = 0.0f;
        if(g_foc_ctrl_state == FOC_CTRL_OPENLOOP) {
            float df = FOC_OL_ACCEL_HZ_S * FOC_TS;
            if(g_ol_freq_hz + df < g_ol_freq_target)
                g_ol_freq_hz += df;
            else if(g_ol_freq_hz - df > g_ol_freq_target)
                g_ol_freq_hz -= df;
            else
                g_ol_freq_hz = g_ol_freq_target;
            g_ol_angle += 2.0f * _PI * g_ol_freq_hz * FOC_TS;
            float a = g_ol_angle;
            utils_norm_angle_rad(&a);
            g_ol_angle = a;
            theta = a;
            omega = 2.0f * _PI * g_ol_freq_hz;
        } else {
            /* HOLD: плавный ход к s_hold_tgt (каждый такт, без сброса регулятора) */
            if(s_hold_rate > 0.0f)
                utils_step_towards(&s_hold_cur, s_hold_tgt, s_hold_rate * FOC_TS);
            else
                s_hold_cur = s_hold_tgt;
            float a = s_hold_cur;
            utils_norm_angle_rad(&a);
            g_ol_hold_angle = a;
            theta = a;
        }

        g_foc_id_ref = g_ol_current;
        g_foc_iq_ref = 0.0f;
        g_observer.theta_est = theta;
        g_observer.omega_est = omega;
        if(g_motor.pp > 0)
            g_observer.speed_rpm = omega * 60.0f / (2.0f * _PI * (float)g_motor.pp);
        else
            g_observer.speed_rpm = 0.0f;

        g_park.ds = g_clarke.ds;
        g_park.qs = g_clarke.qs;
        g_park.ang = theta;
        park_calc(&g_park);
        g_observer.theta_i = theta;

        float o_sin, o_cos;
        utils_fast_sincos_better(theta, &o_sin, &o_cos);
        g_vesc_state.i_alpha = g_clarke.ds;
        g_vesc_state.i_beta = g_clarke.qs;
        g_vesc_state.id_target = g_foc_id_ref;
        g_vesc_state.iq_target = g_foc_iq_ref;
        g_vesc_state.v_bus = g_foc_udc;
        g_vesc_state.phase = theta;
        g_vesc_state.phase_sin = o_sin;
        g_vesc_state.phase_cos = o_cos;
        g_vesc_state.max_duty = g_pwm.U_lim;
        g_vesc_state.speed_est_rad = omega;

        uint32_t otA, otB, otC;
        vesc_control_current(&g_vesc_state, &g_vesc_cfg, FOC_TS, g_pwm.k_pwm, &otA, &otB, &otC);
        g_pwm.UalphaNorm = g_vesc_state.mod_alpha_raw;
        g_pwm.UbetaNorm = g_vesc_state.mod_beta_raw;
        g_pwm.U_mag = NORM2_f(g_vesc_state.mod_alpha_raw, g_vesc_state.mod_beta_raw);
        g_pwm.sector = (int16_t)g_vesc_state.svm_sector;
        foc_cmpa_from_duties(otA, otB, otC);
        return;
    }

    /* Угол Park: в VECTOR — ПLL theta_est, но в blend-периоде сразу после 'v'
   * используем проверенный I/F-угол. */
    float theta_park = g_foc_angle;
    float phase_observer = theta_park;
    int flux_ok = 0;

    /* Токи в dq для observer / телеметрии. */
    g_park.ds = g_clarke.ds;
    g_park.qs = g_clarke.qs;
    g_park.ang = theta_park;
    park_calc(&g_park);

    /* j-тест R: DC-вектор по Ualpha/Ubeta, всё прочее заморожено. */
    if((g_foc_ualpha_dc != 0.0f || g_foc_ubeta_dc != 0.0f) && g_foc_enable) {
        g_foc_id_ref = 0.0f;
        g_foc_iq_ref = 0.0f;
        g_pwm.UalphaRef = g_foc_ualpha_dc;
        g_pwm.UbetaRef = g_foc_ubeta_dc;
        foc_svpwm_update(&g_pwm);
        return;
    }

    if(g_foc_ctrl_state == FOC_CTRL_VECTOR && g_foc_vector_blend > 0)
        g_foc_vector_blend--;

    /* Observer (VESC foc_observer_update) в alpha-beta. В STOP не считаем. */
    if(g_foc_ctrl_state != FOC_CTRL_STOP) {
        g_observer.motor.m_motor_state.id = g_park.de;
        g_observer.motor.m_motor_state.iq = g_park.qe;
        g_observer.motor.m_motor_state.i_abs_filter = NORM2_f(g_park.de, g_park.qe);

        /* VESC: gamma_now = 4 * map(duty, 0, 40/Vbus, 0, observer_gain), floor. */
        float duty_now = NORM2_f(g_pwm.UalphaNorm, g_pwm.UbetaNorm);
        float vin_max = g_foc_vbus_max > 8.0f ? g_foc_vbus_max : 8.0f;
        float gamma_tmp = utils_map(fabsf(duty_now), 0.0f, 40.0f / vin_max, 0.0f, g_foc_obs_gain);
        float gamma_min = FOC_OBSERVER_GAIN_SLOW * g_foc_obs_gain;
        if(gamma_tmp < gamma_min)
            gamma_tmp = gamma_min;
        g_observer.motor.m_gamma_now = gamma_tmp * 4.0f;

        /* VESC control_current выдаёт v в рамке токов. При g_foc_inv_i=1
     * токи измерены в противофазе; observer должен видеть согласованную
     * пару (v, i), поэтому напряжение тоже инвертируем. */
        float v_inv = (g_foc_inv_i && !g_foc_obs_vraw) ? -1.0f : 1.0f;
        float v_alpha = g_vesc_state.v_alpha * v_inv;
        float v_beta = g_vesc_state.v_beta * v_inv;

        /* Экспериментальный поворот v-входа наблюдателя (кнопка k). */
        static const float vrot_cos[3] = {1.0f, -0.5f, -0.5f};
        static const float vrot_sin[3] = {0.0f, -0.8660254f, 0.8660254f};
        uint8_t vrot = g_foc_obs_vrot;
        if(vrot > 2u)
            vrot = 2u;
        float v_rot_alpha = v_alpha * vrot_cos[vrot] - v_beta * vrot_sin[vrot];
        float v_rot_beta = v_alpha * vrot_sin[vrot] + v_beta * vrot_cos[vrot];
        v_alpha = v_rot_alpha;
        v_beta = v_rot_beta;

        foc_observer_update(v_alpha, v_beta, g_clarke.ds, g_clarke.qs, FOC_TS, &g_observer.state,
                            &g_observer.theta_i, &g_observer.motor);
        utils_norm_angle_rad(&g_observer.theta_i);
        g_observer.psi_mag = NORM2_f(g_observer.state.x1, g_observer.state.x2);

        /* Телеметрия: v, которое видел наблюдатель. */
        g_observer.v_alpha = v_alpha;
        g_observer.v_beta = v_beta;

        /* Прямая оценка λ (независимая): e = v − R·i, λ = |e|/|ω|. */
        float e_a = v_alpha - g_observer.cfg.foc_motor_r * g_clarke.ds;
        float e_b = v_beta - g_observer.cfg.foc_motor_r * g_clarke.qs;
        float omega_abs = fabsf(g_observer.omega_est);
        g_observer.lam_meas = omega_abs > 10.0f ? NORM2_f(e_a, e_b) / omega_abs : 0.0f;
        g_observer.ea = e_a;
        g_observer.eb = e_b;

        /* Константный вычет угла наблюдателя (кнопка o): 0 / −120 / +120°. */
        static const float th_off[3] = {0.0f, -2.0943951f, 2.0943951f};
        g_observer.theta_i += th_off[g_foc_obs_off];
        utils_norm_angle_rad(&g_observer.theta_i);

        /* Реальная скорость ротора: производная theta_i за такт (ISR-side). */
        {
            float dth = g_observer.theta_i - g_observer.theta_i_prev;
            utils_norm_angle_rad(&dth);
            g_observer.w_obs = dth / FOC_TS;
            g_observer.theta_i_prev = g_observer.theta_i;
        }

        /* Диагностика кадра: угол наблюдателя минус I/F-углом, ОДИН такт ISR. */
        g_observer.theta_err = g_observer.theta_i - g_foc_angle;
        utils_norm_angle_rad(&g_observer.theta_err);

        flux_ok = g_observer.psi_mag > (g_observer.lambda * 0.3f);

        /* VESC mcpwm_foc.c: phase from the observer, extrapolated by half a
     * sampling period plus the configured offset. */
        float phase_speed = g_observer.omega_est;
        if(g_foc_ctrl_state == FOC_CTRL_I2F)
            phase_speed = FOC_IF_OMEGA_NOM * g_ramp.output;

        phase_observer = g_observer.theta_i + phase_speed * FOC_TS * (0.5f + g_foc_observer_offset);
        utils_norm_angle_rad(&phase_observer);

        if(g_foc_ctrl_state == FOC_CTRL_I2F) {
            if(flux_ok)
                g_observer.theta_est = phase_observer;
            else
                g_observer.theta_est = g_foc_angle;
            utils_norm_angle_rad(&g_observer.theta_est);
            g_observer.omega_est = phase_speed;
        } else if(g_foc_ctrl_state == FOC_CTRL_VECTOR) {
            if(!flux_ok && g_foc_vector_blend > 0) {
                g_observer.theta_est = g_foc_angle;
                utils_norm_angle_rad(&g_observer.theta_est);
            }
        }
    }

    /* VESC sets control_current.phase from phase_observer, not the PLL phase. */
    if(g_foc_ctrl_state == FOC_CTRL_VECTOR && g_foc_vector_blend == 0)
        theta_park = phase_observer;

    if(s_vesc_mode && g_foc_ctrl_state == FOC_CTRL_VECTOR) {
        /* FOC_SENSOR_MODE_SENSORLESS: override phase and observer state during the
     * openloop start; id = 0, iq = m_iq_set. */
        s_phase_obs = phase_observer;
        if(s_ovr_active) {
            theta_park = s_phase_ovr;
            g_observer.state.x1 = s_x1_ovr;
            g_observer.state.x2 = s_x2_ovr;
        }
        g_foc_id_ref = 0.0f;
        float iq_set = s_iq_set;
        iq_set += FOC_SL_OPENLOOP_BOOST_Q * (s_ovr_active ? (float)SIGN(iq_set) : 0.0f);
        utils_truncate_number_abs(&iq_set, g_foc_i_cmd);
        g_foc_iq_ref = iq_set;
    }

    /* Скорость (телеметрия). */
    /* speed_rpm is computed after the PLL run below. */

    /* VESC current controller: Park→PI→decouple→SVM. */
    float phase_sin, phase_cos;
    utils_fast_sincos_better(theta_park, &phase_sin, &phase_cos);

    g_vesc_state.i_alpha = g_clarke.ds;
    g_vesc_state.i_beta = g_clarke.qs;
    g_vesc_state.id_target = g_foc_id_ref;
    g_vesc_state.iq_target = g_foc_iq_ref;
    g_vesc_state.v_bus = g_foc_udc;
    g_vesc_state.phase = theta_park;
    g_vesc_state.phase_sin = phase_sin;
    g_vesc_state.phase_cos = phase_cos;
    g_vesc_state.max_duty = g_pwm.U_lim;
    g_vesc_state.speed_est_rad = g_observer.omega_est;

    uint32_t tA, tB, tC;
    vesc_control_current(&g_vesc_state, &g_vesc_cfg, FOC_TS, g_pwm.k_pwm, &tA, &tB, &tC);

    /* VESC runs the PLL after the current control and uses its phase on the next
   * cycle; it must not replace phase_observer inside the current loop. */
    if(s_vesc_mode && g_foc_ctrl_state == FOC_CTRL_VECTOR) {
        s_duty_now = (float)SIGN(g_vesc_state.vq) *
                     NORM2_f(g_vesc_state.mod_d, g_vesc_state.mod_q) * (2.0f / 1.7320508f);
        foc_pll_run(theta_park, FOC_TS, &g_observer.theta_est, &g_observer.omega_est,
                    &g_observer.cfg);
    } else if(g_foc_ctrl_state == FOC_CTRL_VECTOR && flux_ok) {
        foc_pll_run(phase_observer, FOC_TS, &g_observer.theta_est, &g_observer.omega_est,
                    &g_observer.cfg);
    }

    if(g_motor.pp > 0)
        g_observer.speed_rpm = g_observer.omega_est * 60.0f / (2.0f * _PI * (float)g_motor.pp);
    else
        g_observer.speed_rpm = 0.0f;

    /* Сохраняем модуляцию для observer gamma в следующем такте. */
    g_pwm.UalphaNorm = g_vesc_state.mod_alpha_raw;
    g_pwm.UbetaNorm = g_vesc_state.mod_beta_raw;
    g_pwm.U_mag = NORM2_f(g_vesc_state.mod_alpha_raw, g_vesc_state.mod_beta_raw);
    g_pwm.sector = (int16_t)g_vesc_state.svm_sector;

    foc_cmpa_from_duties(tA, tB, tC);
}
