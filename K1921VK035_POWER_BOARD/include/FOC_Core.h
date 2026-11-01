/*
 * FOC_Core.h
 *
 * Состояние FOC, параметры мотора и команды управления.
 * Математика VESC — GPLv3; Clarke/Park/IPark — адаптация демо НПФ Вектор.
 */

#ifndef INCLUDE_FOC_CORE_H_
#define INCLUDE_FOC_CORE_H_

#ifdef FOC_HOST_TEST
#include "foc_stub.h"
#else
#include "DSP.h"
#endif
#include "FOC_Math.h"
#include "FOC_Observer.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Рабочая структура ШИМ/SVPWM (float). UalphaRef/UbetaRef — задание в pu
 * базового напряжения (FOC_VECTOR_U_BASE), далее saturate до U_lim и foc_svm.
 * UalphaNorm/UbetaNorm — реально приложенный (после saturate) вектор: он же
 * идёт в реконструкцию напряжения для наблюдателя (v = UalphaNorm*(2/3)*Udc).
 * GammaA/B/C — итоговые значения CMPA, записанные в таймеры. */
typedef struct {
    int16_t Enabled;
    float UalphaRef;
    float UbetaRef;
    float UalphaNorm;
    float UbetaNorm;
    uint32_t k_pwm; /* TBPRD (up-down), peak value PWM counter */
    float U_mag;
    float U_lim;
    uint32_t GammaA;
    uint32_t GammaB;
    uint32_t GammaC;
    int16_t sector;
    float UdCorTmp;
    float UdCompK;
    int32_t UdCompEnable;
    int16_t ULimitation;
    int PDP_Fault;
    int ChargingMode;
} TFOC_PWM;

/* Экземпляры кольца (живут в ISR SEQ0) */
extern TClarke g_clarke;
extern TPark g_park;
extern TIPark g_ipark;
extern TPidReg3 g_pid_d;
extern TPidReg3 g_pid_q;
extern TFOC_PWM g_pwm;

/* Задания и состояние (g_foc_enable меняет только main/UART, читает ISR) */
extern volatile uint8_t g_foc_enable;   /* 0 = CMPA не пишем, 1 = пишем */
extern float g_foc_angle;               /* эл. угол, рад (пила I/F) */
extern float g_foc_id_ref;              /* задание Id, А */
extern float g_foc_iq_ref;              /* задание Iq, А */
extern float g_foc_udc;                 /* Vdc, В (телеметрия) */
extern volatile uint8_t g_foc_inv_i;    /* 1 = инвертировать знак ia/ib */
extern volatile uint8_t g_foc_inv_ang;  /* 1 = инвертировать направление угла I/F */
extern volatile uint8_t g_foc_adc_map;  /* 0..2 циклический сдвиг маппинга ADC */
extern volatile uint8_t g_foc_pwm_map;  /* 0..2 циклическая пермутация SVM->CMPA (фазы A/B/C) */
extern volatile uint8_t g_foc_obs_vrot; /* 0..2 поворот v-входа наблюдателя на -120°*k */
extern volatile uint8_t g_foc_pwm_swap; /* 1 = обмен A<->B (зеркало, ) */
extern volatile float g_foc_ualpha_dc;  /* j-тест: DC Ualpha, N/1000 pu, 0=выкл */
extern volatile float g_foc_ubeta_dc;   /* j-тест: DC Ubeta, N/1000 pu, 0=выкл */
extern volatile uint8_t g_foc_obs_off;  /* 0..2 константный вычет угла наблюдателя 0/−120/+120° */

/* Разомкнутый I/F (порт DO-ветки CTRL_RUN_I2F из SM_Ctrl.c:240-300).
 * Кольцо ~6.25 кГц (Ts=160 мкс), FREQ_NOM=350 Гц эл. (7 пар, 3000 об/мин). */
#define FOC_TS (1.0f / 6250.0f)
#define FOC_FREQ_NOM 350                                    /* Гц эл. */
#define FOC_IF_OMEGA_NOM (2.0f * _PI * (float)FOC_FREQ_NOM) /* рад/с эл. */
#define FOC_RAMP_T 2.0f                                     /* разгон 0->номинал за 2 с */
#define FOC_FLUX_DEFAULT 0.5f  /* ток намагничивания 0.5 А (малый мотор) */
#define FOC_IF_KP 0.005f       /* усиление за такт Kp·Vbase·Ts/L≈0.4 */
#define FOC_IF_KI 0.001f       /* интегратор под Kp. */
#define FOC_IF_KC 0.0f         /* анти-виндап отключён: ui clamp внутри PID */
#define FOC_IF_U_LIM 0.5f      /* I/F: потолок 0.5 pu. */
#define FOC_IQ_MAX 2.0f        /* страховка момента для I/F-пуска */
#define FOC_SPEED_DEFAULT 0.1f /* 10% номинала на старте */

extern TRMPCtrl g_ramp;              /* рампа скорости 0..1 pu */
extern volatile uint8_t g_if_active; /* 1 = I/F крутит угол */
extern float g_if_speed_ref;         /* задание рампе, pu 0..1 */
extern float g_if_flux_ref;          /* задание Id, А */

void foc_if_start(void); /* ENTRY I2F: сброс ui, rmp.out=0, enable=1 */
void foc_if_stop(void);  /* ноль-вектор + enable=0 (мотор обесточен) */

/* -------------------------------------------------------------------------
 * Sensorless FOC (VESC flux observer + PLL)
 * ------------------------------------------------------------------------- */

typedef enum {
    FOC_CTRL_STOP = 0, /* ключи закрыты, мотор обесточен */
    FOC_CTRL_I2F,      /* разомкнутый I/F пуск */
    FOC_CTRL_VECTOR,   /* замкнутый sensorless FOC */
    FOC_CTRL_OPENLOOP, /* разомкнутый: ток Id на вращающемся угле f, Гц */
    FOC_CTRL_HOLD      /* удержание вектора: ток Id на фиксированном угле */
} foc_ctrl_state_t;

/* Open-loop / удержание вектора . */
extern volatile float g_ol_current;     /* задание тока Id, А */
extern volatile float g_ol_freq_hz;     /* текущая эл. частота, Гц */
extern volatile float g_ol_freq_target; /* цель эл. частоты, Гц */
#define FOC_OL_ACCEL_HZ_S 30.0f         /* разгон open-loop, Гц/с */
extern volatile float g_ol_angle;       /* текущий угол вращения, рад */
extern volatile float g_ol_hold_angle;  /* удерживаемый угол, рад */

void foc_openloop_start(float current_a, float freq_hz);
void foc_hold_start(float current_a, float angle_rad);
void foc_hold_move(float current_a, float target_rad, float rate_rad_s);

extern volatile foc_ctrl_state_t g_foc_ctrl_state;

/* Плавный переход I/F -> VECTOR. Сразу после 'v' используем рабочий I/F-угол,
 * пока observer/PLL в фоне готовится принять управление. 6250 тактов ≈ 1 с. */
#define FOC_VECTOR_BLEND_CNT 6250
extern volatile uint32_t g_foc_vector_blend;

/* Sensorless-пуск VESC. Контур тока 6,25 кГц, регулятор скорости 1 кГц. */
#define FOC_SL_OPENLOOP_RPM 1500.0f  /* MCCONF_FOC_OPENLOOP_RPM */
#define FOC_SL_OPENLOOP_HYST 0.1f    /* MCCONF_FOC_SL_OPENLOOP_HYST, s */
#define FOC_SL_OPENLOOP_TIME 0.05f   /* MCCONF_FOC_SL_OPENLOOP_TIME, s */
#define FOC_SL_OPENLOOP_T_RAMP 0.1f  /* MCCONF_FOC_SL_OPENLOOP_TIME_RAMP, s */
#define FOC_SL_OPENLOOP_RPM_LOW 0.0f /* MCCONF_FOC_OPENLOOP_RPM_LOW */
#define FOC_SL_OPENLOOP_T_LOCK 0.0f  /* MCCONF_FOC_SL_OPENLOOP_T_LOCK */
#define FOC_SL_OPENLOOP_BOOST_Q 0.0f /* MCCONF_FOC_SL_OPENLOOP_BOOST_Q */
#define FOC_SL_OPENLOOP_MAX_Q -1.0f  /* MCCONF_FOC_SL_OPENLOOP_MAX_Q */
#define FOC_CC_MIN_CURRENT 0.05f     /* MCCONF_CC_MIN_CURRENT */
#define FOC_S_PID_ALLOW_BRAKING 1    /* MCCONF_S_PID_ALLOW_BRAKING */
#define FOC_CC_DECOUPLING_FACTOR 20.0f
#define FOC_TS_PER_VESC_TICK 10u          /* 100 us * 10 = 1 ms, ~1 kHz VESC timer task */
#define FOC_VESC_TIMER_DIV (1000u / 160u) /* 0.001 / FOC_TS ~= 6 ISR ticks (informational) */

extern volatile float g_foc_i_cmd;        /* limit of Iq command, A */
extern volatile float g_foc_openloop_rpm; /* foc_openloop_rpm, ERPM */
extern volatile float g_foc_sl_hyst;      /* foc_sl_openloop_hyst, s */
extern volatile float g_foc_sl_time;      /* foc_sl_openloop_time, s */
extern volatile float g_foc_sl_t_ramp;    /* foc_sl_openloop_time_ramp, s */
extern volatile float g_s_pid_kp;
extern volatile float g_s_pid_ki;
extern volatile float g_s_pid_kd;
extern volatile float g_s_pid_kd_filter;
extern volatile float g_s_pid_min_erpm;
extern volatile float g_s_pid_ramp_erpms_s;
extern volatile float g_foc_max_erpm;
extern volatile float g_foc_cc_tc_us;
extern volatile float g_foc_observer_offset;
extern volatile uint8_t g_cl_phase; /* 0 stop, 1 first cycle, 2 running */

/* mcpwm_foc_set_current(iq): CONTROL_MODE_CURRENT, id = 0. */
bool foc_closed_request(float iq_a);
/* mcpwm_foc_set_pid_speed(erpm): CONTROL_MODE_SPEED, ramp enabled. */
bool foc_closed_speed_request(float erpm);
/* Leave speed/current VESC mode without stopping the motor. */
void foc_closed_release(void);
/* ~1 kHz VESC timer + speed PID; call from the main loop. */
void foc_vesc_timer_tick(void);

/* Параметры мотора (ручной ввод по UART, позже автоидентификация). */
typedef struct {
    int32_t R_mOhm;     /* сопротивление статора, мОм     (450 -> 0.45 Ом) */
    int32_t L_uH;       /* индуктивность, мкГн             (300 -> 300 мкГн) */
    int32_t lambda_uWb; /* потокосцепление ротора, мкВб    (4300 -> 4.3 мВб) */
    uint8_t pp;         /* пары полюсов */
    int32_t target_rpm; /* целевая скорость, об/мин */
    int32_t max_rpm;    /* максимальная скорость, об/мин */
} TMotorParams;

extern TMotorParams g_motor;

/* Обёртка вокруг VESC observer_state + PLL. state/cfg/motor — поля
 * VESC foc_observer_update/foc_pll_run; остальное — наша телеметрия/гейтинг. */
typedef struct {
    observer_state state;     /* VESC: x1,x2,lambda_est,i_alpha_last,i_beta_last */
    TFocObserverCfg cfg;      /* VESC: R,L,lambda,observer_type,pll_kp,pll_ki... */
    TFLuxObserverMotor motor; /* VESC: m_conf,m_motor_state,m_gamma_now,m_res_temp */
    float theta_i;            /* вход PLL: угол потока, рад */
    float theta_est;          /* оцененный эл. угол, рад */
    float theta_i_prev;       /* предыдущий theta_i (для w_obs) */
    float theta_err;          /* theta_i - theta_if одного такта ISR, рад (wrap ±π) */
    float ea, eb;             /* остаток интегранда (v - R i), В — телеметрия */
    float omega_est;          /* эл. частота оценки, рад/с */
    float psi_mag;            /* |поток| оценки, Вб */
    float w_obs;              /* реальная эл. скорость из dtheta_i/Ts, рад/с */
    float lam_meas;           /* прямая оценка λ = |v−R·i|/|ω|, Вб (независимая) */
    float v_alpha;            /* реконструированное напряжение alpha, В (телеметрия) */
    float v_beta;             /* реконструированное напряжение beta, В (телеметрия) */
    float speed_rpm;          /* мех. скорость, об/мин */
    float R;                  /* сопротивление статора, Ом (телеметрия/гейт) */
    float L;                  /* индуктивность, Гн */
    float lambda;             /* потокосцепление ротора, Вб */
} TFluxObserver;

/* VESC-дефолты наблюдателя (mcconf_default.h): gain 9e5 (≈600/L), slow 0.05;
 * PLL KP=2000 рад/с, KI=30000 рад/с². Наблюдатель — MXLEMMING_LAMBDA_COMP. */
#define FOC_OBSERVER_GAIN 9.0e5f
extern volatile float g_foc_obs_gain;
extern volatile uint8_t g_foc_obs_vraw;
extern volatile uint8_t g_foc_outputs_on;
bool foc_outputs_arm(void);
bool foc_fault_clear(void);
#define FOC_OBSERVER_GAIN_SLOW 0.05f
#define FOC_OBS_PLL_KP 2000.0f
#define FOC_OBS_PLL_KI 30000.0f
#define FOC_OBSERVER_TYPE_DEFAULT FOC_OBSERVER_MXLEMMING_LAMBDA_COMP

extern TFluxObserver g_observer;

/* Заполняет R/L/lambda из g_motor и пересчитывает cfg наблюдателя. */
void flux_observer_slow_calc(TFluxObserver *p);

/* PI скорости (вход — об/мин, выход — Iq_ref, А). */
extern TPidReg3 g_pid_speed;

#define FOC_VECTOR_U_BASE 24.0f /* 1.0 pu Udc = 24 В */
#define FOC_VECTOR_SPEED_KP 0.05f
#define FOC_VECTOR_SPEED_KI 0.001f
#define FOC_VECTOR_SPEED_MAX_A 8.0f /* ограничение Iq, А */

void foc_vector_start(void); /* I2F -> VECTOR: угол из observer */
void foc_ctrl_stop(void);    /* любое состояние -> STOP */

/* DWT-замер ISR: последнее и максимальное число тактов foc_fast_calc */
extern volatile uint32_t g_foc_cycles;
extern volatile uint32_t g_foc_cycles_max;
/* Счётчик вызовов foc_fast_calc (для детектора: нумерация выборок ISR). */
extern volatile uint32_t g_foc_isr_count;
/* Порог диагностики времени выполнения, в тактах процессора. */
#define FOC_CYCLES_BUDGET 2400u

void foc_init(void);
void foc_dwt_init(void);
void foc_fast_calc(void);
void foc_svpwm_update(TFOC_PWM *p);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_FOC_CORE_H_ */
