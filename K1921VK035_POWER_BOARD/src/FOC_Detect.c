/*
 * FOC_Detect.c
 *
 * Измерение параметров мотора через UART-конфигуратор.
 *
 * R: два уровня тока через токовую петлю (HOLD) на фиксированном угле,
 *    R = (vd2 - vd1) / (id2 - id1) — отменяет dead-time и смещение нуля.
 * L: импульс DC-вектора вдоль alpha; три равноотстоящих выборки тока дают
 *    (i3-i2)/(i2-i1) = exp(-dt/tau), L = R * tau.
 * LAMBDA: open-loop вращение 100 Гц эл., lambda = |v - R*i| / omega.
 *
 * Лицензия: GNU GPLv3.
 */

#include "FOC_Detect.h"

#include <math.h>

#include "FOC_Control.h"
#include "FOC_Core.h"
#include "PWM_Control.h"
#include "FOC_Protection.h"
#include "VESC_Math.h"

/* SysTick-счётчик 10 кГц (100 мкс), определён в main.c. */
extern volatile uint32_t g_tick_100us;

/* Тайминги в тиках SysTick (100 мкс). */
#define DETECT_SETTLE_TICKS 3000u /* 300 мс на успокоение тока */
#define DETECT_AVG_TICKS 200u     /* выборок ISR (200 x 160 мкс = 32 мс) */

#define DETECT_I_MIN 0.2f /* минимальный ток для валидного замера, А */

/* L: один импульс напряжения вдоль alpha, ток растёт как
 * i(t) = I_ss * (1 - exp(-t/tau)), tau = L/R. Три равноотстоящих выборки
 * дают (i3-i2)/(i2-i1) = exp(-dt/tau): напряжение, dead-time и момент начала
 * импульса в результат не входят. L = R * tau. */
#define DETECT_L_SETTLE_TICKS 1000u /* 100 мс обнуления тока после импульса */
#define DETECT_L_I_SS 8.0f          /* расчётный ток импульса, А */
#define DETECT_L_PU_MIN 0.005f
#define DETECT_L_PU_MAX 0.08f
#define DETECT_L_SAMPLE_DT 40e-6f
#define DETECT_L_N1 2u /* номера выборок i1, i2, i3 после подачи */
#define DETECT_L_N2 3u
#define DETECT_L_N3 4u

/* lambda: open-loop вращение, lambda = |v - R*i| / omega. */
#define DETECT_LAMBDA_FREQ_HZ 100.0f    /* эл. частота вращения, Гц */
#define DETECT_LAMBDA_RAMP_TICKS 38000u /* 3.8 с: разгон 0->100 Гц + запас */
#define DETECT_LAMBDA_SETTLE_TICKS 3000u
#define DETECT_LAMBDA_AVG_TICKS 1000u /* выборок ISR */

typedef enum {
    D_IDLE = 0,
    D_R_S1_SETTLE,
    D_R_S1_AVG,
    D_R_S2_SETTLE,
    D_R_S2_AVG,
    D_L_ZERO,
    D_L_P1,
    D_L_OFF1,
    D_LAMBDA_RAMP,
    D_LAMBDA_SETTLE,
    D_LAMBDA_AVG,
    D_DONE,
    D_ERROR
} detect_step_t;

volatile foc_detect_type_t g_detect_type = FOC_DETECT_NONE;
volatile foc_detect_state_t g_detect_state = FOC_DETECT_IDLE;
volatile float g_detect_progress = 0.0f;
volatile float g_detect_r = 0.0f;
volatile float g_detect_l = 0.0f;
volatile float g_detect_lambda = 0.0f;

static detect_step_t s_step = D_IDLE;
static uint32_t s_deadline = 0u;
static uint32_t s_avg_cnt = 0u;
static float s_avg_id = 0.0f;
static float s_avg_vd = 0.0f;
static float s_avg_emf = 0.0f;
static float s_i1 = 0.0f;
static float s_v1 = 0.0f;
static float s_i2 = 0.0f;
static float s_i_lo = 0.0f;
static float s_i_hi = 0.0f;
static float s_angle = 0.0f;
static uint32_t s_ramp_t0 = 0u;

/* Индуктивность: три выборки тока импульса и их g_foc_isr_count. */
static uint32_t s_l_cnt = 0u;
static volatile uint8_t s_l_active = 0u;
static volatile uint8_t s_l_done = 0u;
static volatile uint8_t s_l_cancel = 0u;
static volatile uint8_t s_l_failed = 0u;
static uint8_t s_l_started = 0u;
static float s_l_pu = 0.0f;
static volatile uint8_t s_l_error = 0u;
static volatile uint16_t s_l_raw[4];
static uint32_t s_lt[3];
static float s_li[3];
static float s_l_rr = 0.0f;    /* R для L = R * tau, Ом */
static bool s_r_valid = false; /* g_detect_r — свежий замер: его берут L и lambda */

/* R для расчётов L/lambda: измеренный, иначе параметр r. */
static float detect_r_use(void)
{
    return s_r_valid ? g_detect_r : (g_motor.R_mOhm * 1e-3f);
}

/* Разрешение выходов перед импульсным измерением. */
static bool detect_arm(void)
{
    return foc_outputs_arm();
}

static bool detect_faulted(void)
{
    return g_fault_flags != 0u || g_foc_state != STATE_NORMAL;
}

void foc_detect_forget_r(void)
{
    s_r_valid = false;
}

static bool time_up(uint32_t now)
{
    return (int32_t)(now - s_deadline) >= 0;
}

/* true один раз на каждую новую выборку ISR. */
static uint32_t s_seen_isr = 0u;

static bool new_sample(void)
{
    uint32_t c = g_foc_isr_count;
    if(c == s_seen_isr)
        return false;
    s_seen_isr = c;
    return true;
}

/* Снять напряжение (нуль-вектор, enable=0). */
static void l_off(void)
{
    foc_ctrl_stop();
}

/* Подать DC-вектор вдоль alpha (pu) в обход токовой петли (j-ветка). */
static bool l_arm(float v)
{
    if(!detect_arm())
        return false;
    g_foc_ctrl_state = FOC_CTRL_STOP;
    g_foc_enable = 1u;
    g_foc_ualpha_dc = v;
    g_foc_ubeta_dc = 0.0f;
    g_pwm.U_lim = 1.0f;
    return true;
}

static void detect_finish(foc_detect_state_t st)
{
    foc_ctrl_stop();
    g_detect_state = st;
    s_step = (st == FOC_DETECT_DONE) ? D_DONE : D_ERROR;
    g_detect_progress = 1.0f;
}

void foc_detect_start(foc_detect_type_t type, float current_a)
{
    if(s_l_active || (s_step != D_IDLE && s_step != D_DONE && s_step != D_ERROR))
        return; /* уже идёт */

    if(!isfinite(current_a) ||
       (type != FOC_DETECT_R && type != FOC_DETECT_L && type != FOC_DETECT_LAMBDA)) {
        g_detect_state = FOC_DETECT_ERROR;
        s_step = D_ERROR;
        return;
    }
    g_detect_type = type;
    g_detect_progress = 0.0f;
    /* Обнуляем только то, что измеряем заново: прежние результаты остаются
   * (R, замеренный раньше, нужен для L и lambda). */
    if(type == FOC_DETECT_R) {
        g_detect_r = 0.0f;
        s_r_valid = false;
    } else if(type == FOC_DETECT_L) {
        g_detect_l = 0.0f;
    } else if(type == FOC_DETECT_LAMBDA) {
        g_detect_lambda = 0.0f;
    }
    s_seen_isr = g_foc_isr_count;

    if(detect_faulted()) {
        g_detect_state = FOC_DETECT_ERROR;
        s_step = D_ERROR;
        g_detect_progress = 1.0f;
        return;
    }

    if(type == FOC_DETECT_R) {
        if(current_a > FOC_VECTOR_SPEED_MAX_A)
            current_a = FOC_VECTOR_SPEED_MAX_A;
        if(current_a < DETECT_I_MIN)
            current_a = DETECT_I_MIN;
        s_i_lo = 0.5f * current_a;
        s_i_hi = current_a;
        s_angle = 0.0f;
        foc_hold_start(s_i_lo, s_angle);
        s_deadline = g_tick_100us + DETECT_SETTLE_TICKS;
        s_step = D_R_S1_SETTLE;
        g_detect_state = FOC_DETECT_BUSY;
        g_detect_progress = 0.05f;
        return;
    }

    if(type == FOC_DETECT_L) {
        s_angle = 0.0f;
        l_off();
        s_deadline = g_tick_100us + DETECT_L_SETTLE_TICKS;
        s_step = D_L_ZERO;
        g_detect_state = FOC_DETECT_BUSY;
        g_detect_progress = 0.05f;
        return;
    }

    if(type == FOC_DETECT_LAMBDA) {
        if(current_a > FOC_VECTOR_SPEED_MAX_A)
            current_a = FOC_VECTOR_SPEED_MAX_A;
        if(current_a < DETECT_I_MIN)
            current_a = DETECT_I_MIN;
        s_i_hi = current_a;
        foc_openloop_start(s_i_hi, DETECT_LAMBDA_FREQ_HZ);
        s_ramp_t0 = g_tick_100us;
        s_deadline = g_tick_100us + DETECT_LAMBDA_RAMP_TICKS;
        s_step = D_LAMBDA_RAMP;
        g_detect_state = FOC_DETECT_BUSY;
        g_detect_progress = 0.05f;
        return;
    }

    g_detect_state = FOC_DETECT_ERROR;
    s_step = D_ERROR;
    g_detect_progress = 1.0f;
}

void foc_detect_abort(void)
{
    s_l_cancel = 1u;
    foc_ctrl_stop();
    s_step = D_IDLE;
    g_detect_state = FOC_DETECT_IDLE;
    g_detect_type = FOC_DETECT_NONE;
    g_detect_progress = 0.0f;
}

bool foc_detect_busy(void)
{
    return s_l_active || (s_step != D_IDLE && s_step != D_DONE && s_step != D_ERROR);
}

bool foc_detect_l_sample(const uint16_t raw[4])
{
    if(!s_l_active)
        return false;
    for(unsigned i = 0; i < 4u; ++i)
        s_l_raw[i] = raw[i];
    if(s_l_cancel || detect_faulted()) {
        s_l_error = 1u;
        l_off();
        if(s_l_started)
            ADC_FOC_L_Mode(0u);
        s_l_failed = 1u;
        s_l_done = 1u;
        s_l_active = 0u;
        return true;
    }
    if(!s_l_started) {
        ADC_FOC_L_Mode(1u);
        s_l_started = 1u;
        if(!l_arm(s_l_pu)) {
            s_l_error = 2u;
            ADC_FOC_L_Mode(0u);
            s_l_failed = 1u;
            s_l_done = 1u;
            s_l_active = 0u;
            return true;
        }
        g_pwm.UalphaRef = s_l_pu;
        g_pwm.UbetaRef = 0.0f;
        foc_svpwm_update(&g_pwm);
        return true;
    }
    float phase[3];
    for(unsigned i = 0; i < 3u; ++i) {
        phase[i] = ((float)raw[i] - (float)g_adc_off[i]) * 0.0268817f;
        if(raw[i] <= 8u || raw[i] >= 4087u || fabsf(phase[i]) > g_foc_i_trip ||
           fabsf(phase[i]) > 12.0f) {
            s_l_error = 3u;
            l_off();
            ADC_FOC_L_Mode(0u);
            s_l_failed = 1u;
            s_l_done = 1u;
            s_l_active = 0u;
            return true;
        }
    }
    unsigned map = g_foc_adc_map <= 2u ? g_foc_adc_map : 0u;
    float current = g_foc_inv_i ? -phase[map] : phase[map];
    ++s_l_cnt;
    if(s_l_cnt >= DETECT_L_N1 && s_l_cnt <= DETECT_L_N3) {
        unsigned i = s_l_cnt - DETECT_L_N1;
        s_lt[i] = s_l_cnt;
        s_li[i] = current;
    }
    if(s_l_cnt == DETECT_L_N3) {
        l_off();
        ADC_FOC_L_Mode(0u);
        s_l_done = 1u;
        s_l_active = 0u;
    }
    return true;
}

void foc_detect_tick(void)
{
    uint32_t now = g_tick_100us;

    /* Авария посреди замера (OC/OV/UV) — замер недействителен. */
    if(foc_detect_busy() && detect_faulted()) {
        detect_finish(FOC_DETECT_ERROR);
        return;
    }

    switch(s_step) {
        case D_IDLE:
        case D_DONE:
        case D_ERROR:
            return;

        case D_R_S1_SETTLE:
            if(!time_up(now))
                return;
            s_avg_id = 0.0f;
            s_avg_vd = 0.0f;
            s_avg_cnt = 0u;
            s_step = D_R_S1_AVG;
            return;

        case D_R_S1_AVG:
            if(!new_sample())
                return;
            s_avg_id += g_vesc_state.id;
            s_avg_vd += g_vesc_state.vd;
            if(++s_avg_cnt < DETECT_AVG_TICKS)
                return;
            s_i1 = s_avg_id / (float)s_avg_cnt;
            s_v1 = s_avg_vd / (float)s_avg_cnt;
            foc_hold_start(s_i_hi, s_angle);
            s_deadline = now + DETECT_SETTLE_TICKS;
            s_step = D_R_S2_SETTLE;
            g_detect_progress = 0.5f;
            return;

        case D_R_S2_SETTLE:
            if(!time_up(now))
                return;
            s_avg_id = 0.0f;
            s_avg_vd = 0.0f;
            s_avg_cnt = 0u;
            s_step = D_R_S2_AVG;
            return;

        case D_R_S2_AVG: {
            if(!new_sample())
                return;
            s_avg_id += g_vesc_state.id;
            s_avg_vd += g_vesc_state.vd;
            if(++s_avg_cnt < DETECT_AVG_TICKS)
                return;
            s_i2 = s_avg_id / (float)s_avg_cnt;
            float v2 = s_avg_vd / (float)s_avg_cnt;
            float di = s_i2 - s_i1;
            if(fabsf(di) > 0.1f) {
                g_detect_r = (v2 - s_v1) / di;
                s_r_valid = g_detect_r > 0.0f;
                detect_finish(FOC_DETECT_DONE);
            } else {
                detect_finish(FOC_DETECT_ERROR);
            }
            return;
        }

        case D_L_ZERO: {
            if(!time_up(now))
                return;
            s_l_rr = detect_r_use(); /* измеренный R, если был замер; иначе параметр r */
            float udc = (g_foc_udc < 1.0f) ? 1.0f : g_foc_udc;
            float pu = DETECT_L_I_SS * s_l_rr / ((2.0f / 3.0f) * udc);
            if(pu < DETECT_L_PU_MIN)
                pu = DETECT_L_PU_MIN;
            if(pu > DETECT_L_PU_MAX)
                pu = DETECT_L_PU_MAX;
            s_l_pu = pu;
            s_l_cnt = 0u;
            s_l_done = 0u;
            s_l_cancel = 0u;
            s_l_failed = 0u;
            s_l_error = 0u;
            s_l_started = 0u;
            s_step = D_L_P1;
            s_deadline = now + 20u; /* импульс не должен ждать main дольше 2 мс */
            s_l_active = 1u;        /* запуск и выборки выполняет ADC ISR */
            g_detect_progress = 0.3f;
            return;
        }

        case D_L_P1:
            if(!s_l_done) {
                if(time_up(now)) {
                    s_l_error = 4u;
                    s_l_cancel = 1u;
                    detect_finish(FOC_DETECT_ERROR);
                }
                return;
            }
            if(s_l_failed) {
                detect_finish(FOC_DETECT_ERROR);
                return;
            }
            s_deadline = now + DETECT_L_SETTLE_TICKS;
            s_step = D_L_OFF1;
            g_detect_progress = 0.8f;
            return;

        case D_L_OFF1: {
            if(!time_up(now))
                return;
            float d1 = s_li[1] - s_li[0];
            float d2 = s_li[2] - s_li[1];
            uint32_t n1 = s_lt[1] - s_lt[0];
            uint32_t n2 = s_lt[2] - s_lt[1];
            if(n1 == 0u || n1 != n2 || d1 < 0.2f || d2 <= 0.0f || d2 >= d1) {
                detect_finish(FOC_DETECT_ERROR);
                return;
            }
            float dt = (float)n1 * DETECT_L_SAMPLE_DT;
            float tau = -dt / logf(d2 / d1);
            g_detect_l = s_l_rr * tau; /* g_detect_r не трогаем: это результат замера R */
            detect_finish(FOC_DETECT_DONE);
            return;
        }

        case D_LAMBDA_RAMP: /* разгон делает ISR (FOC_OL_ACCEL_HZ_S) */
            if(time_up(now)) {
                s_deadline = now + DETECT_LAMBDA_SETTLE_TICKS;
                s_step = D_LAMBDA_SETTLE;
                return;
            }
            g_detect_progress =
                0.05f + 0.4f * ((float)(now - s_ramp_t0) / (float)DETECT_LAMBDA_RAMP_TICKS);
            return;

        case D_LAMBDA_SETTLE:
            if(!time_up(now))
                return;
            s_avg_emf = 0.0f;
            s_avg_cnt = 0u;
            s_step = D_LAMBDA_AVG;
            g_detect_progress = 0.5f;
            return;

        case D_LAMBDA_AVG: {
            if(!new_sample())
                return;
            float Rr = detect_r_use();
            float va = g_vesc_state.mod_alpha_raw * (2.0f / 3.0f) * g_foc_udc;
            float vb = g_vesc_state.mod_beta_raw * (2.0f / 3.0f) * g_foc_udc;
            float ea = va - Rr * g_clarke.ds;
            float eb = vb - Rr * g_clarke.qs;
            s_avg_emf += NORM2_f(ea, eb);
            if(++s_avg_cnt < DETECT_LAMBDA_AVG_TICKS)
                return;
            float omega = 2.0f * _PI * DETECT_LAMBDA_FREQ_HZ;
            g_detect_lambda = (s_avg_emf / (float)s_avg_cnt) / omega;
            detect_finish(FOC_DETECT_DONE);
            return;
        }

        default:
            s_step = D_ERROR;
            g_detect_state = FOC_DETECT_ERROR;
            return;
    }
}
