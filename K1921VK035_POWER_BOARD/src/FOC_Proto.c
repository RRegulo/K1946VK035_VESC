/*
 * FOC_Proto.c
 *
 * Бинарный протокол UART-конфигуратора: кадры, параметры и команды.
 */
#include "FOC_Proto.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "FOC_Control.h"
#include "FOC_Core.h"
#include "FOC_Detect.h"
#include "FOC_Math.h"
#include "FOC_Params.h"
#include "PWM_Control.h"
#include "UART_Debug.h" /* g_manual_pwm_active */

extern volatile uint16_t g_adc_raw[4];

enum { ST_SOF1 = 0, ST_SOF2, ST_LEN, ST_CMD, ST_PAYLOAD, ST_CRC_LO, ST_CRC_HI };

static uint8_t s_state = ST_SOF1;
static uint8_t s_len;
static uint8_t s_cmd;
static uint8_t s_pos;
static uint8_t s_crc_lo;
static uint8_t s_payload[FOC_PROTO_MAX_PAYLOAD];

void foc_proto_reset(void)
{
    s_state = ST_SOF1;
    s_len = 0;
    s_cmd = 0;
    s_pos = 0;
    s_crc_lo = 0;
}

uint32_t foc_proto_encode(uint8_t *out, uint32_t out_sz, uint8_t cmd, const uint8_t *payload,
                          uint8_t len)
{
    if(out == NULL || len > FOC_PROTO_MAX_PAYLOAD)
        return 0;
    uint32_t need = 4u + (uint32_t)len + 2u;
    if(out_sz < need)
        return 0;
    out[0] = FOC_PROTO_SOF1;
    out[1] = FOC_PROTO_SOF2;
    out[2] = len;
    out[3] = cmd;
    if(len > 0u && payload != NULL)
        memcpy(&out[4], payload, len);
    uint16_t crc = foc_params_crc16(&out[2], (uint32_t)len + 2u);
    out[4u + len] = (uint8_t)(crc & 0xFFu);
    out[5u + len] = (uint8_t)(crc >> 8);
    return need;
}

int foc_proto_feed(uint8_t byte, TFocProtoFrame *frame)
{
    switch(s_state) {
        case ST_SOF1:
            if(frame == NULL)
                return -1;
            if(byte == FOC_PROTO_SOF1) {
                s_state = ST_SOF2;
                return 0;
            }
            return -1;
        case ST_SOF2:
            if(byte == FOC_PROTO_SOF2) {
                s_state = ST_LEN;
                return 0;
            }
            s_state = ST_SOF1;
            return -1;
        case ST_LEN:
            if(byte > FOC_PROTO_MAX_PAYLOAD) {
                s_state = ST_SOF1;
                return -1;
            }
            s_len = byte;
            s_pos = 0;
            s_state = ST_CMD;
            return 0;
        case ST_CMD:
            s_cmd = byte;
            s_state = (s_len == 0u) ? ST_CRC_LO : ST_PAYLOAD;
            return 0;
        case ST_PAYLOAD:
            s_payload[s_pos++] = byte;
            if(s_pos >= s_len)
                s_state = ST_CRC_LO;
            return 0;
        case ST_CRC_LO:
            s_crc_lo = byte;
            s_state = ST_CRC_HI;
            return 0;
        case ST_CRC_HI: {
            uint8_t crcbuf[FOC_PROTO_MAX_PAYLOAD + 2u];
            crcbuf[0] = s_len;
            crcbuf[1] = s_cmd;
            memcpy(&crcbuf[2], s_payload, s_len);
            uint16_t crc = foc_params_crc16(crcbuf, (uint32_t)s_len + 2u);
            uint16_t got = (uint16_t)s_crc_lo | ((uint16_t)byte << 8);
            s_state = ST_SOF1;
            if(crc != got)
                return -1;
            frame->cmd = s_cmd;
            frame->len = s_len;
            memcpy(frame->payload, s_payload, s_len);
            return 1;
        }
        default:
            break;
    }
    s_state = ST_SOF1;
    return -1;
}

static void put_f32(uint8_t *p, float v)
{
    memcpy(p, &v, 4u);
}

static float get_f32(const uint8_t *p)
{
    float v;
    memcpy(&v, p, 4u);
    return v;
}

/* Таблица параметров: id -> поле TFocParams (тип + смещение). */
typedef struct {
    uint8_t id;
    uint8_t type; /* 0 f32, 1 u8, 2 u16 */
    uint8_t off;
} TParamDef;

#define PF(i, f) {(i), 0u, (uint8_t)offsetof(TFocParams, s.f)}
#define PB(i, f) {(i), 1u, (uint8_t)offsetof(TFocParams, s.f)}
#define PW(i, f) {(i), 2u, (uint8_t)offsetof(TFocParams, s.f)}
static const TParamDef s_ptab[] = {
    PB(FOC_PID_PP, pp),
    PF(FOC_PID_R, r_ohm),
    PF(FOC_PID_L, l_h),
    PF(FOC_PID_LD_LQ, ld_lq_diff_h),
    PF(FOC_PID_LAMBDA, lambda_wb),
    PF(FOC_PID_KV, kv),
    PF(FOC_PID_IMAX, current_max_a),
    PF(FOC_PID_ICONT, current_cont_a),
    PF(FOC_PID_VBUS_MIN, vbus_min_v),
    PF(FOC_PID_VBUS_MAX, vbus_max_v),
    PB(FOC_PID_FLAGS, flags),
    PW(FOC_PID_ADC_TRIG, adc_trig),
    PF(FOC_PID_DT_US, dt_us),
    PF(FOC_PID_OL_RPM, ol_rpm),
    PF(FOC_PID_SL_HYST, sl_hyst),
    PF(FOC_PID_SL_TIME, sl_time),
    PF(FOC_PID_SL_RAMP, sl_ramp),
    PF(FOC_PID_CC_TC_US, cc_tc_us),
    PF(FOC_PID_S_KP, s_kp),
    PF(FOC_PID_S_KI, s_ki),
    PF(FOC_PID_S_KD, s_kd),
    PF(FOC_PID_S_KD_FILTER, s_kd_filter),
    PF(FOC_PID_S_MIN_ERPM, s_min_erpm),
    PF(FOC_PID_S_RAMP_ERPMS, s_ramp_erpms),
    PF(FOC_PID_MAX_ERPM, max_erpm),
    PB(FOC_PID_CAN_NODE_ID, can_node_id),
    PB(FOC_PID_CAN_ESC_INDEX, can_esc_index),
    PB(FOC_PID_CAN_RAW_MODE, can_raw_mode),
    PB(FOC_PID_CAN_FLAGS, can_flags),
    PF(FOC_PID_CAN_RAW_RPM_MAX, can_raw_rpm_max),
    PF(FOC_PID_OBS_OFFSET, obs_offset),
    PW(FOC_PID_CAN_BITRATE, can_bitrate_kbps),
    PW(FOC_PID_CAN_STATUS_MS, can_status_ms),
    PW(FOC_PID_CAN_TIMEOUT_MS, can_timeout_ms),
};

static const TParamDef *param_find(uint8_t id)
{
    for(uint32_t i = 0; i < sizeof(s_ptab) / sizeof(s_ptab[0]); i++)
        if(s_ptab[i].id == id)
            return &s_ptab[i];
    return NULL;
}

float foc_param_get_f32(uint8_t id)
{
    switch(id) {
        case FOC_PID_OBS_GAIN:
            return g_foc_obs_gain;
        case FOC_PID_CC_KP:
            return g_vesc_cfg.foc_current_kp;
        case FOC_PID_CC_KI:
            return g_vesc_cfg.foc_current_ki;
        default:
            break;
    }
    const TParamDef *d = param_find(id);
    if(d == NULL)
        return 0.0f;
    const uint8_t *b = &g_params.buffer[d->off];
    if(d->type == 1u)
        return (float)b[0];
    if(d->type == 2u) {
        uint16_t w;
        memcpy(&w, b, 2u);
        return (float)w;
    }
    float f;
    memcpy(&f, b, 4u);
    return f;
}

float foc_param_default_f32(uint8_t id)
{
    const TParamDef *d = param_find(id);
    if(d == NULL)
        return 0.0f;
    TFocParams tmp;
    foc_params_defaults(&tmp);
    const uint8_t *b = &tmp.buffer[d->off];
    if(d->type == 1u)
        return (float)b[0];
    if(d->type == 2u) {
        uint16_t w;
        memcpy(&w, b, 2u);
        return (float)w;
    }
    float f;
    memcpy(&f, b, 4u);
    return f;
}

bool foc_param_idle(void)
{
    return g_foc_ctrl_state == FOC_CTRL_STOP && !g_foc_enable && !g_if_active &&
           !g_manual_pwm_active && !foc_detect_busy();
}

bool foc_param_set_f32(uint8_t id, float v)
{
    /* RAM-перекрытия расчётных значений (не во flash). */
    switch(id) {
        case FOC_PID_OBS_GAIN:
            return false; /* gain рассчитывается по потокосцеплению мотора */
        case FOC_PID_CC_KP:
            if(!(v >= 1e-5f && v <= 0.2f))
                return false;
            g_vesc_cfg.foc_current_kp = v;
            return true;
        case FOC_PID_CC_KI:
            if(!(v >= 0.1f && v <= 5000.0f))
                return false;
            g_vesc_cfg.foc_current_ki = v;
            return true;
        default:
            break;
    }
    const TParamDef *d = param_find(id);
    if(d == NULL)
        return false;
    if(id == FOC_PID_R)
        foc_detect_forget_r(); /* записали r вручную: L/lambda снова берут параметр */
    TFocParams saved = g_params;
    uint8_t *b = &g_params.buffer[d->off];
    if(d->type == 1u || d->type == 2u) {
        if(!(v >= 0.0f && v <= (d->type == 1u ? 255.0f : 65535.0f)))
            return false;
        if(d->type == 1u) {
            b[0] = (uint8_t)v;
        } else {
            uint16_t w = (uint16_t)v;
            memcpy(b, &w, 2u);
        }
    } else {
        memcpy(b, &v, 4u);
    }
    if(!foc_params_sane(&g_params)) {
        g_params = saved; /* мусор не применяем */
        return false;
    }
    foc_params_finalize(&g_params);
    foc_params_apply();
    return true;
}

/* Телеметрия v2, 46 байт — раскладка из foc-configurator
 * (lib/focproto.ts decodeTelemetry). */
static uint8_t telemetry_payload(uint8_t *p)
{
    uint8_t n = 0;
    for(uint8_t i = 0; i < 4u; i++) {
        p[n++] = (uint8_t)(g_adc_raw[i] & 0xFFu);
        p[n++] = (uint8_t)(g_adc_raw[i] >> 8);
    }
    int16_t idm = (int16_t)(g_park.de * 1000.0f);
    int16_t iqm = (int16_t)(g_park.qe * 1000.0f);
    memcpy(&p[n], &idm, 2u);
    n += 2u;
    memcpy(&p[n], &iqm, 2u);
    n += 2u;
    uint16_t udc = (uint16_t)(g_foc_udc * 1000.0f);
    memcpy(&p[n], &udc, 2u);
    n += 2u;

    int32_t th_if = (int32_t)(g_foc_angle * 1000.0f);
    int32_t th_obs = (int32_t)(g_observer.theta_est * 1000.0f);
    int32_t rpm = (int32_t)g_observer.speed_rpm;
    int32_t psi = (int32_t)(g_observer.psi_mag * 1e6f);
    memcpy(&p[n], &th_if, 4u);
    n += 4u;
    memcpy(&p[n], &th_obs, 4u);
    n += 4u;
    memcpy(&p[n], &rpm, 4u);
    n += 4u;
    memcpy(&p[n], &psi, 4u);
    n += 4u;
    p[n++] = (uint8_t)g_foc_ctrl_state;

    /* Фазные токи: кларке уже посчитан в ISR (as/bs), ic = -(ia+ib). */
    int16_t ia_m = (int16_t)(g_clarke.as * 1000.0f);
    int16_t ib_m = (int16_t)(g_clarke.bs * 1000.0f);
    int16_t ic_m = (int16_t)(-(g_clarke.as + g_clarke.bs) * 1000.0f);
    memcpy(&p[n], &ia_m, 2u);
    n += 2u;
    memcpy(&p[n], &ib_m, 2u);
    n += 2u;
    memcpy(&p[n], &ic_m, 2u);
    n += 2u;

    /* v2, offset 37: faults u16, outputs_on u8, isr_hz u16, lat_min/max u16
   * (тики CPU, как g_foc_cycles из DWT). */
    uint16_t flt = g_fault_flags;
    memcpy(&p[n], &flt, 2u);
    n += 2u;
    p[n++] = g_foc_outputs_on;
    uint16_t hz = (uint16_t)(1.0f / FOC_TS);
    uint16_t lmin = (uint16_t)g_foc_cycles;
    uint16_t lmax = (uint16_t)g_foc_cycles_max;
    memcpy(&p[n], &hz, 2u);
    n += 2u;
    memcpy(&p[n], &lmin, 2u);
    n += 2u;
    memcpy(&p[n], &lmax, 2u);
    n += 2u;
    return n;
}

uint32_t foc_proto_handle(const TFocProtoFrame *req, uint8_t *out, uint32_t out_sz)
{
    uint8_t p[FOC_PROTO_MAX_PAYLOAD];
    const uint8_t *rp = NULL;
    uint8_t rn = 0;
    uint8_t rcmd = FOC_CMD_ACK;

    if(req == NULL)
        return 0;

    switch(req->cmd) {
        case FOC_CMD_PING:
            p[0] = FOC_PROTO_VERSION;
            rp = p;
            rn = 1u;
            break;
        case FOC_CMD_GET_INFO: {
            uint16_t pv = (uint16_t)FOC_PARAMS_VERSION;
            uint16_t ps = (uint16_t)sizeof(g_params.buffer);
            p[0] = FOC_PROTO_VERSION;
            p[1] = (uint8_t)(pv & 0xFFu);
            p[2] = (uint8_t)(pv >> 8);
            p[3] = (uint8_t)(ps & 0xFFu);
            p[4] = (uint8_t)(ps >> 8);
            rp = p;
            rn = 5u;
            break;
        }
        case FOC_CMD_GET_PARAM:
            if(req->len < 1u) {
                rcmd = FOC_CMD_NAK;
                break;
            }
            put_f32(p, foc_param_get_f32(req->payload[0]));
            rp = p;
            rn = 4u;
            break;
        case FOC_CMD_SET_PARAM:
            if(!foc_param_idle() || req->len < 5u ||
               !foc_param_set_f32(req->payload[0], get_f32(&req->payload[1])))
                rcmd = FOC_CMD_NAK;
            break;
        case FOC_CMD_READ_CONFIG:
            rp = g_params.buffer;
            rn = (uint8_t)sizeof(g_params.buffer);
            break;
        case FOC_CMD_WRITE_CONFIG:
            if(!foc_param_idle() || req->len != sizeof(g_params.buffer)) {
                rcmd = FOC_CMD_NAK;
                break;
            }
            {
                TFocParams tmp;
                memcpy(tmp.buffer, req->payload, sizeof(tmp.buffer));
                if(!foc_params_valid(&tmp)) { /* целевой g_params не портим */
                    rcmd = FOC_CMD_NAK;
                    break;
                }
                g_params = tmp;
            }
            foc_params_apply();
            break;
        case FOC_CMD_SAVE_PARAMS:
            if(!foc_param_idle() || !foc_params_save())
                rcmd = FOC_CMD_NAK;
            break;
        case FOC_CMD_LOAD_DEFAULTS:
            if(!foc_param_idle()) {
                rcmd = FOC_CMD_NAK;
                break;
            }
            foc_params_defaults(&g_params);
            foc_params_apply();
            break;
        case FOC_CMD_GET_TELEMETRY:
            rn = telemetry_payload(p);
            rp = p;
            break;
        case FOC_CMD_GET_BENCH: {
            uint32_t last = g_foc_cycles;
            uint32_t mx = g_foc_cycles_max;
            uint32_t budget = FOC_CYCLES_BUDGET;
            memcpy(&p[0], &last, 4u);
            memcpy(&p[4], &mx, 4u);
            memcpy(&p[8], &budget, 4u);
            rp = p;
            rn = 12u;
            break;
        }
        case FOC_CMD_SET_OPENLOOP:
            /* payload: ток Id (f32), частота в эл. Гц (f32). */
            if(req->len < 8u || g_foc_state != STATE_NORMAL) {
                rcmd = FOC_CMD_NAK;
                break;
            }
            {
                float i = get_f32(&req->payload[0]);
                float f = get_f32(&req->payload[4]);
                if(!isfinite(i) || !isfinite(f) || f > g_foc_max_erpm / 60.0f ||
                   foc_detect_busy()) {
                    rcmd = FOC_CMD_NAK;
                    break;
                }
                if(i < 0.0f)
                    i = 0.0f;
                if(i > FOC_IQ_MAX)
                    i = FOC_IQ_MAX;
                if(f < 0.0f)
                    f = 0.0f;
                g_manual_pwm_active = 0;
                foc_closed_release();
                foc_openloop_start(i, f);
            }
            break;
        case FOC_CMD_SET_CLOSED:
            /* Sensorless current request can start from STOP, as in the core API. */
            if(req->len < 4u || g_foc_state != STATE_NORMAL || g_fault_flags != 0u ||
               foc_detect_busy()) {
                rcmd = FOC_CMD_NAK;
                break;
            }
            {
                float iq = get_f32(&req->payload[0]);
                if(!isfinite(iq)) {
                    rcmd = FOC_CMD_NAK;
                    break;
                }
                if(iq > FOC_IQ_MAX)
                    iq = FOC_IQ_MAX;
                if(iq < -FOC_IQ_MAX)
                    iq = -FOC_IQ_MAX;
                foc_closed_release(); /* ручной ток: контур скорости больше не трогает Iq */
                if(!foc_closed_request(iq))
                    rcmd = FOC_CMD_NAK;
            }
            break;
        case FOC_CMD_STOP:
            g_manual_pwm_active = 0;
            g_foc_ualpha_dc = 0.0f;
            g_foc_ubeta_dc = 0.0f;
            foc_detect_abort(); /* cancel pending measurement steps as well */
            break;
        case FOC_CMD_START_DETECT:
            /* payload: тип (u8), ток А (f32). Только из полного покоя и без аварий. */
            if(req->len < 5u || !foc_param_idle() || g_foc_state != STATE_NORMAL ||
               g_fault_flags != 0u) {
                rcmd = FOC_CMD_NAK;
                break;
            }
            foc_detect_start((foc_detect_type_t)req->payload[0], get_f32(&req->payload[1]));
            if(g_detect_state == FOC_DETECT_ERROR)
                rcmd = FOC_CMD_NAK;
            break;
        case FOC_CMD_ABORT:
            foc_detect_abort();
            break;
        case FOC_CMD_GET_DETECT:
            rcmd = FOC_CMD_GET_DETECT;
            rn = 18u;
            rp = p;
            p[0] = (uint8_t)g_detect_state;
            p[1] = (uint8_t)g_detect_type;
            put_f32(&p[2], g_detect_r);
            put_f32(&p[6], g_detect_l);
            put_f32(&p[10], g_detect_lambda);
            put_f32(&p[14], g_detect_progress);
            break;
        case FOC_CMD_SET_HOLD:
            /* payload: ток Id (f32), угол (град, эл.); опционально скорость (град/с). */
            if(req->len < 8u || g_foc_state != STATE_NORMAL || foc_detect_busy() ||
               !isfinite(get_f32(&req->payload[0])) || get_f32(&req->payload[0]) < 0.0f ||
               get_f32(&req->payload[0]) > FOC_IQ_MAX || !isfinite(get_f32(&req->payload[4])) ||
               (req->len >= 12u && !isfinite(get_f32(&req->payload[8])))) {
                rcmd = FOC_CMD_NAK;
                break;
            }
            g_manual_pwm_active = 0;
            foc_closed_release(); /* удержание ведёт свой режим, PI скорости не нужен */
            if(req->len >= 12u) {
                const float d2r = (float)(3.14159265358979 / 180.0);
                foc_hold_move(get_f32(&req->payload[0]), get_f32(&req->payload[4]) * d2r,
                              get_f32(&req->payload[8]) * d2r);
            } else {
                foc_hold_start(get_f32(&req->payload[0]),
                               get_f32(&req->payload[4]) * (float)(3.14159265358979 / 180.0));
            }
            break;
        case FOC_CMD_SET_SPEED:
            /* payload: ERPM (f32, эл. об/мин). Контур скорости: I/F-пуск -> VECTOR. */
            if(req->len < 4u || g_foc_state != STATE_NORMAL || g_fault_flags != 0u ||
               foc_detect_busy()) {
                rcmd = FOC_CMD_NAK;
                break;
            }
            g_manual_pwm_active = 0;
            g_foc_ualpha_dc = 0.0f;
            g_foc_ubeta_dc = 0.0f;
            if(!isfinite(get_f32(&req->payload[0])) ||
               !foc_closed_speed_request(get_f32(&req->payload[0])))
                rcmd = FOC_CMD_NAK;
            break;
        case FOC_CMD_CLEAR_FAULT:
            if(!foc_fault_clear())
                rcmd = FOC_CMD_NAK;
            break;
        case FOC_CMD_CAP_ARM:
        case FOC_CMD_CAP_TRIGGER:
        case FOC_CMD_CAP_STATUS:
        case FOC_CMD_CAP_READ:
        case FOC_CMD_CAN_TEST:
        case FOC_CMD_CAN_STATUS:
            /* Осциллограф и CAN в этой сборке не поддерживаются. */
            rcmd = FOC_CMD_NAK;
            break;
        default:
            rcmd = FOC_CMD_NAK;
            break;
    }

    return foc_proto_encode(out, out_sz, rcmd, rp, rn);
}
