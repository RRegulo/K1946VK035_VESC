/*
 * FOC_Params.c
 *
 * Параметры мотора и регулятора, хранение во Flash.
 * Формат блока общий с UART-конфигуратором; поля CAN зарезервированы.
 */
#include "FOC_Params.h"

#include <stddef.h>
#include <string.h>

#include "FOC_Control.h"
#include "FOC_Core.h"
#include "FOC_Protection.h" /* FOC_CURRENT_LIMIT_A / FOC_VBUS_* для дефолтов */
#ifndef FOC_HOST_TEST
#include "K1921VK035.h"
#endif

TFocParams g_params;

/* CRC-16/XMODEM: poly 0x1021, init 0x0000, без отражения.
 * Check("123456789") = 0x31C3. */
uint16_t foc_params_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0u;
    for(uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for(uint8_t b = 0; b < 8u; b++) {
            if(crc & 0x8000u)
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void foc_params_finalize(TFocParams *p)
{
    p->s.size = (uint16_t)sizeof(p->s);
    p->s.crc = foc_params_crc16(p->buffer, (uint32_t)offsetof(TFocParams, s.crc));
}

static bool in_range(float v, float lo, float hi)
{
    return (v >= lo) && (v <= hi); /* NaN -> false */
}

bool foc_params_sane(const TFocParams *p)
{
    if(p->s.pp < 1u || p->s.pp > 30u)
        return false;
    if(p->s.adc_trig != 0u && (p->s.adc_trig < 20u || p->s.adc_trig > 600u))
        return false;
    if(!in_range(p->s.r_ohm, 0.001f, 10.0f))
        return false;
    if(!in_range(p->s.l_h, 1e-6f, 10e-3f))
        return false;
    if(!in_range(p->s.ld_lq_diff_h, -10e-3f, 10e-3f))
        return false;
    if(!in_range(p->s.lambda_wb, 10e-6f, 0.1f))
        return false;
    if(!in_range(p->s.kv, 0.0f, 100000.0f))
        return false;
    if(!in_range(p->s.current_max_a, 0.5f, 30.0f))
        return false;
    if(!in_range(p->s.current_cont_a, 0.1f, p->s.current_max_a))
        return false;
    if(!in_range(p->s.vbus_min_v, 3.0f, 60.0f))
        return false;
    if(!in_range(p->s.vbus_max_v, p->s.vbus_min_v + 1.0f, 60.0f))
        return false;
    if(!in_range(p->s.dt_us, 0.0f, 3.0f) || !in_range(p->s.cc_tc_us, 100.0f, 20000.0f) ||
       !in_range(p->s.ol_rpm, 100.0f, 100000.0f) || !in_range(p->s.sl_hyst, 0.0f, 5.0f) ||
       !in_range(p->s.sl_time, 0.0f, 5.0f) || !in_range(p->s.sl_ramp, 0.0f, 5.0f) ||
       !in_range(p->s.s_kp, 0.0f, 100.0f) || !in_range(p->s.s_ki, 0.0f, 100.0f) ||
       !in_range(p->s.s_kd, 0.0f, 100.0f) || !in_range(p->s.s_kd_filter, 0.0f, 1.0f) ||
       !in_range(p->s.s_min_erpm, 0.0f, 100000.0f) || !in_range(p->s.s_ramp_erpms, 0.0f, 1e6f) ||
       !in_range(p->s.max_erpm, 100.0f, 1e6f) || !in_range(p->s.obs_offset, -3.0f, 3.0f))
        return false;
    if(p->s.can_node_id > 127u || p->s.can_esc_index > 19u || p->s.can_raw_mode > 3u ||
       !in_range(p->s.can_raw_rpm_max, 100.0f, 1e6f) || p->s.can_bitrate_kbps < 100u ||
       p->s.can_bitrate_kbps > 1000u || p->s.can_status_ms > 5000u || p->s.can_timeout_ms > 60000u)
        return false;
    return true;
}

bool foc_params_valid(const TFocParams *p)
{
    if(p->s.magic != FOC_PARAMS_MAGIC)
        return false;
    if(p->s.version != FOC_PARAMS_VERSION)
        return false;
    if(p->s.size != (uint16_t)sizeof(p->s))
        return false;
    if(foc_params_crc16(p->buffer, (uint32_t)offsetof(TFocParams, s.crc)) != p->s.crc)
        return false;
    return foc_params_sane(p);
}

void foc_params_defaults(TFocParams *p)
{
    memset(p, 0, sizeof(*p));
    p->s.magic = FOC_PARAMS_MAGIC;
    p->s.version = FOC_PARAMS_VERSION;
    /* Дефолты = текущие значения этой прошивки (FOC_Core.c g_motor,
   * vesc_config_default, FOC_Protection.h). */
    p->s.pp = 7u;
    p->s.flags = FOC_PARAM_FLAG_INV_I;
    p->s.r_ohm = 0.190f;
    p->s.l_h = 300e-6f;
    p->s.ld_lq_diff_h = 0.0f;
    p->s.lambda_wb = 3500e-6f;
    p->s.kv = 0.0f;
    p->s.adc_trig = 100u;
    p->s.current_max_a = FOC_CURRENT_LIMIT_A;
    p->s.current_cont_a = 4.0f;
    p->s.vbus_min_v = FOC_VBUS_MIN_V;
    p->s.vbus_max_v = FOC_VBUS_MAX_V;
    p->s.dt_us = 0.833f; /* как vesc_config_default (foc_dt_us) */
    p->s.cc_tc_us = 1000.0f;
    p->s.ol_rpm = 1500.0f;
    p->s.sl_hyst = 0.1f;
    p->s.sl_time = 0.05f;
    p->s.sl_ramp = 0.1f;
    p->s.s_kp = 0.004f;
    p->s.s_ki = 0.004f;
    p->s.s_kd = 0.0001f;
    p->s.s_kd_filter = 0.2f;
    p->s.s_min_erpm = 900.0f;
    p->s.s_ramp_erpms = 25000.0f;
    p->s.max_erpm = 100000.0f;
    p->s.can_node_id = 0u; /* CAN в этой прошивке нет */
    p->s.can_esc_index = 0u;
    p->s.can_raw_mode = 3u;
    p->s.can_raw_rpm_max = 40000.0f;
    p->s.can_bitrate_kbps = 1000u;
    p->s.can_status_ms = 50u;
    p->s.can_timeout_ms = 500u;
    p->s.obs_offset = 0.0f;
    foc_params_finalize(p);
}

static int32_t foc_round_i32(float v)
{
    return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

void foc_params_apply(void)
{
    g_foc_cc_tc_us = g_params.s.cc_tc_us;
    /* Мотор -> целочисленный вид, который ждут FOC_Core/observer. */
    g_motor.R_mOhm = foc_round_i32(g_params.s.r_ohm * 1000.0f);
    g_motor.L_uH = foc_round_i32(g_params.s.l_h * 1e6f);
    g_motor.lambda_uWb = foc_round_i32(g_params.s.lambda_wb * 1e6f);
    g_motor.pp = g_params.s.pp;

    flux_observer_slow_calc(&g_observer); /* R/L/lambda -> observer + vesc cfg */
    g_observer.cfg.foc_motor_ld_lq_diff = g_params.s.ld_lq_diff_h;
    g_vesc_cfg.foc_motor_ld_lq_diff = g_params.s.ld_lq_diff_h;

    /* Kp/Ki пересчитаны по R/L и cc_tc_us, как в VESC. */
    g_vesc_cfg.foc_dt_us = g_params.s.dt_us;
    g_vesc_cfg.l_current_max = g_params.s.current_max_a;
    g_observer.cfg.l_current_max = g_params.s.current_max_a;

    /* Параметры sensorless-пуска и регулятора скорости. */
    g_foc_i_cmd = g_params.s.current_cont_a; /* VESC lo_current_max */
    g_foc_openloop_rpm = g_params.s.ol_rpm;
    g_foc_sl_hyst = g_params.s.sl_hyst;
    g_foc_sl_time = g_params.s.sl_time;
    g_foc_sl_t_ramp = g_params.s.sl_ramp;
    g_s_pid_kp = g_params.s.s_kp;
    g_s_pid_ki = g_params.s.s_ki;
    g_s_pid_kd = g_params.s.s_kd;
    g_s_pid_kd_filter = g_params.s.s_kd_filter;
    g_s_pid_min_erpm = g_params.s.s_min_erpm;
    g_s_pid_ramp_erpms_s = g_params.s.s_ramp_erpms;
    g_foc_max_erpm = g_params.s.max_erpm;
    g_foc_cc_tc_us = g_params.s.cc_tc_us;
    g_foc_observer_offset = g_params.s.obs_offset;

    /* Переключатели полярности/угла. */
    g_foc_inv_i = (g_params.s.flags & FOC_PARAM_FLAG_INV_I) ? 1u : 0u;
    g_foc_inv_ang = (g_params.s.flags & FOC_PARAM_FLAG_INV_ANG) ? 1u : 0u;
    g_foc_obs_vraw = (g_params.s.flags & FOC_PARAM_FLAG_OBS_V_RAW) ? 1u : 0u;
    g_foc_i_trip = g_params.s.current_max_a;
    g_foc_vbus_min = g_params.s.vbus_min_v;
    g_foc_vbus_max = g_params.s.vbus_max_v;

    /* ADC-триггер фиксирован в ADC_FOC_Init; CAN не используется. */
}

bool foc_params_load(void)
{
    TFocParams tmp;
    foc_flash_read(FOC_PARAMS_FLASH_ADDR, tmp.buffer, sizeof(tmp.buffer));
    if(foc_params_valid(&tmp)) {
        g_params = tmp;
        return true;
    }
    return false;
}

bool foc_params_save(void)
{
    foc_params_finalize(&g_params);
    foc_flash_write_page(FOC_PARAMS_FLASH_ADDR, g_params.buffer, sizeof(g_params.buffer));
    TFocParams chk;
    foc_flash_read(FOC_PARAMS_FLASH_ADDR, chk.buffer, sizeof(chk.buffer));
    return foc_params_valid(&chk);
}

void foc_params_init(void)
{
    if(!foc_params_load())
        foc_params_defaults(&g_params); /* SAVE — только по команде */
    foc_params_apply();
}

/* --- main-flash: 8-байтные операции, по plib035_mflash --- */

#ifdef FOC_HOST_TEST
/* Host: RAM-заглушка, чтобы тесты не лезли в регистры. */
static uint8_t s_host_flash[FOC_PARAMS_FLASH_PAGE];

void foc_flash_read(uint32_t addr, uint8_t *dst, uint32_t len)
{
    (void)addr;
    memcpy(dst, s_host_flash, len);
}

void foc_flash_erase_page(uint32_t addr)
{
    (void)addr;
    memset(s_host_flash, 0xFF, sizeof(s_host_flash));
}

void foc_flash_write_page(uint32_t addr, const uint8_t *src, uint32_t len)
{
    (void)addr;
    memset(s_host_flash, 0xFF, sizeof(s_host_flash));
    memcpy(s_host_flash, src, len);
}

#else /* target */

#define FOC_FLASH_CMD_KEY ((uint32_t)MFLASH_CMD_KEY_Access << MFLASH_CMD_KEY_Pos)
#define FOC_FLASH_REGION_MAIN 0u

__attribute__((section(".ramfunc"), noinline, long_call)) static void foc_flash_cmd(uint32_t cmd)
{
    MFLASH->CMD = FOC_FLASH_CMD_KEY | FOC_FLASH_REGION_MAIN | cmd;
}

__attribute__((section(".ramfunc"), noinline, long_call)) static void foc_flash_wait(void)
{
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    while((MFLASH->STAT & MFLASH_STAT_BUSY_Msk) != 0u) {
    }
}

__attribute__((section(".ramfunc"), noinline, long_call)) static void foc_flash_cache_flush(void)
{
    MFLASH->CTRL_bit.IFLUSH = 1;
    while(MFLASH->ICSTAT_bit.BUSY) {
    }
    MFLASH->CTRL_bit.DFLUSH = 1;
    while(MFLASH->DCSTAT_bit.BUSY) {
    }
}

void foc_flash_read(uint32_t addr, uint8_t *dst, uint32_t len)
{
    uint32_t i = 0;
    while(i < len) {
        MFLASH->ADDR = addr + i;
        foc_flash_cmd(MFLASH_CMD_RD_Msk);
        foc_flash_wait();
        uint32_t w0 = MFLASH->DATA[0].DATA;
        uint32_t w1 = MFLASH->DATA[1].DATA;
        uint8_t tmp[8];
        tmp[0] = (uint8_t)(w0);
        tmp[1] = (uint8_t)(w0 >> 8);
        tmp[2] = (uint8_t)(w0 >> 16);
        tmp[3] = (uint8_t)(w0 >> 24);
        tmp[4] = (uint8_t)(w1);
        tmp[5] = (uint8_t)(w1 >> 8);
        tmp[6] = (uint8_t)(w1 >> 16);
        tmp[7] = (uint8_t)(w1 >> 24);
        for(uint32_t j = 0; j < 8u && (i + j) < len; j++)
            dst[i + j] = tmp[j];
        i += 8u;
    }
}

__attribute__((section(".ramfunc"), noinline, long_call)) void foc_flash_erase_page(uint32_t addr)
{
    MFLASH->ADDR = addr;
    foc_flash_cmd(MFLASH_CMD_ERSEC_Msk);
    foc_flash_wait();
    foc_flash_cache_flush();
}

/* Стирание/запись идут из RAM (.ramfunc): выборка кода из стираемой страницы
 * даёт HardFault. WDT ~8.3 мс (LOAD=200000), операция дольше — снимаем RESEN
 * на время и запрещаем прерывания. */
__attribute__((section(".ramfunc"), noinline, long_call)) void
foc_flash_write_page(uint32_t addr, const uint8_t *src, uint32_t len)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t reset_enabled = WDT->CTRL_bit.RESEN;
    __disable_irq();
    WDT->CTRL_bit.RESEN = 0;
    WDT->INTCLR = 1u;
    foc_flash_erase_page(addr);
    uint32_t i = 0;
    while(i < len) {
        uint32_t b[8];
        for(uint32_t k = 0; k < 8u; k++)
            b[k] = ((i + k) < len) ? src[i + k] : 0xFFu;
        MFLASH->ADDR = addr + i;
        MFLASH->DATA[0].DATA = b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24);
        MFLASH->DATA[1].DATA = b[4] | (b[5] << 8) | (b[6] << 16) | (b[7] << 24);
        foc_flash_cmd(MFLASH_CMD_WR_Msk);
        foc_flash_wait();
        i += 8u;
    }
    foc_flash_cache_flush();
    WDT->INTCLR = 1u;
    WDT->CTRL_bit.RESEN = reset_enabled;
    __set_PRIMASK(primask);
}

#endif /* FOC_HOST_TEST */
