/*
 * UART_Debug.c
 *
 * UART1, 115200 8N1, PB8/PB9 (CN3).
 * Приём команд и передача телеметрии из основного цикла.
 */

#include "UART_Debug.h"
#include "FOC_Control.h"
#include "FOC_Core.h"
#include "FOC_Proto.h"
#include "FOC_Detect.h"
#include "VESC_Math.h"
#include "system_K1921VK035.h" /* OSECLK_VAL/OSICLK_VAL под расчёт бодрейта */

/* Расчёт делителей UART по частоте PLL. */

/* TX из main loop (НЕ из ISR) */
static void uart_send(const uint8_t *p, uint32_t n);
volatile uint8_t g_manual_pwm_active = 0;

/* переключатели отладки на ходу */
extern volatile uint8_t g_foc_inv_i;
extern volatile uint8_t g_foc_inv_ang;
extern volatile uint8_t g_foc_adc_map;

void uart_debug_init(void)
{
    /* Тактирование GPIOB и альтернативная функция PB8/PB9. */
    RCU->HCLKCFG_bit.GPIOBEN = 1;
    RCU->HRSTCFG_bit.GPIOBEN = 1;
    GPIOB->DENSET = GPIO_DENSET_PIN8_Msk | GPIO_DENSET_PIN9_Msk;
    GPIOB->ALTFUNCSET = GPIO_ALTFUNCSET_PIN8_Msk | GPIO_ALTFUNCSET_PIN9_Msk;

    /* UART1: тактирование от PLL без делителя, выход из сброса. */
    RCU->UARTCFG[1].UARTCFG_bit.CLKSEL = RCU_UARTCFG_UARTCFG_CLKSEL_PLLCLK;
    RCU->UARTCFG[1].UARTCFG_bit.DIVN = 0;
    RCU->UARTCFG[1].UARTCFG_bit.DIVEN = 0;
    RCU->UARTCFG[1].UARTCFG_bit.CLKEN = 1;
    RCU->UARTCFG[1].UARTCFG_bit.RSTDIS = 1;

    /* Бодрейт — из реальной частоты PLL (регистры, не хардкод): переживает
   * чужой кварц. Формула — UART_AutoBaudConfig из plib (plib035_uart.c),
   * только целочисленно вместо float. */
    {
        uint32_t refclk =
            (RCU->PLLCFG_bit.REFSRC == RCU_PLLCFG_REFSRC_OSICLK) ? OSICLK_VAL : OSECLK_VAL;
        uint32_t pll = (refclk * (uint32_t)RCU->PLLCFG_bit.M) /
                       ((uint32_t)RCU->PLLCFG_bit.N * (1u << (uint32_t)RCU->PLLCFG_bit.OD));
        uint32_t den = 16u * 115200u;
        uint32_t div_int = pll / den;
        uint32_t rem = pll - div_int * den;
        uint32_t div_frac = (rem * 64u + den / 2u) / den;
        if(div_frac > 63u) {
            div_int++;
            div_frac = 0;
        }
        UART1->IBRD = div_int;
        UART1->FBRD = div_frac;
    }

    /* 8N1, FIFO, приём и передача. */
    UART1->LCRH_bit.WLEN = UART_LCRH_WLEN_8bit;
    UART1->LCRH_bit.STP2 = 0;
    UART1->LCRH_bit.FEN = 1;
    UART1->CR_bit.TXE = 1;
    UART1->CR_bit.RXE = 1;
    UART1->CR_bit.UARTEN = 1;
    /* Приёма по прерыванию нет (демо поллит) — IMSC/IFLS/NVIC не трогаем. */
}

/* Блокирующая передача из основного цикла с обслуживанием watchdog. */
static void uart_send(const uint8_t *p, uint32_t n)
{
    for(uint32_t i = 0; i < n; i++) {
        while(UART1->FR_bit.TXFF) {
            /* WDT ≈ 8.3мс (PLL 96МГц / 4, LOAD=200000): полный статус 's'
       * это ~7мс на линии + вытеснения ADC ISR. Без подпитки внутри
       * отправки — ребут посреди ответа. Как в CaptureOffset. */
            WDT->INTCLR = 1u;
        }
        UART1->DR = p[i];
    }
}

static void uart_put_u32(uint32_t v)
{
    uint8_t tmp[10];
    uint8_t n = 0;
    if(v == 0) {
        uint8_t z = '0';
        uart_send(&z, 1);
        return;
    }
    while(v > 0) {
        tmp[n++] = (uint8_t)('0' + (v % 10u));
        v /= 10u;
    }
    while(n > 0)
        uart_send(&tmp[--n], 1);
}

static void uart_put_u16(uint16_t v)
{
    uart_put_u32((uint32_t)v);
}

static void uart_put_i32(int32_t v)
{
    if(v < 0) {
        uint8_t m = '-';
        uart_send(&m, 1);
        v = -v;
    }
    uart_put_u32((uint32_t)v);
}

/* --- RX-команды побайтово, без строк и Enter: каждый принятый байт сразу
 * --- команда (?, s, t — исполняются немедленно; d<число> — цифры копятся
 * --- до первого не-цифры). Символьные команды не требуют перевода строки.
 * --- */

/* --- бинарный кадр по запросу 't': AA 55 Ia Ib Ic Vbat state faults(lo,hi) xor
 * --- 14Б. Потоком не шлём — глушит ответы в терминале. */
static void uart_telemetry(void)
{
    uint8_t f[14];
    f[0] = 0xAA;
    f[1] = 0x55;
    /* g_adc_raw обновляет ISR SEQ0 — читаем как есть, кадр не атомарен,
   * для осциллограммы достаточно */
    f[2] = (uint8_t)g_adc_raw[0];
    f[3] = (uint8_t)(g_adc_raw[0] >> 8);
    f[4] = (uint8_t)g_adc_raw[1];
    f[5] = (uint8_t)(g_adc_raw[1] >> 8);
    f[6] = (uint8_t)g_adc_raw[2];
    f[7] = (uint8_t)(g_adc_raw[2] >> 8);
    f[8] = (uint8_t)g_adc_raw[3];
    f[9] = (uint8_t)(g_adc_raw[3] >> 8);
    f[10] = (uint8_t)g_foc_state;
    f[11] = (uint8_t)g_fault_flags;
    f[12] = (uint8_t)(g_fault_flags >> 8);
    uint8_t xor = 0;
    for(uint8_t i = 2; i < 13; i++)
        xor^= f[i];
    f[13] = xor;
    uart_send(f, 14);
}

/* --- RX-команды строками (\n или \r): ? помощь, s статус, t кадр, d N duty ---
 */
static void uart_cmd_help(void)
{
    static const uint8_t s[] =
        "? help\r\ns status\r\nt frame once\r\nd N duty 0..1999\r\n"
        "r run I/F\r\nx stop\r\nw N speed 0..100 %%\r\nf N flux 0..8000 mA\r\n"
        "q N torque 0..8000 mA (VECTOR only)\r\n"
        "R N Rs 1..10000 mOhm\r\nL N Ls 1..10000 uH\r\nB N flux 1..65000 uWb\r\n"
        "z N dead-time us/1000 (default 833)\r\n"
        "v vector mode (from I/F)\r\n"
        "p invert current polarity\r\n"
        "n invert angle direction\r\n"
        "m cycle ADC mapping 0..2\r\n"
        "e cycle PWM phase mapping 0..2\r\n";
    uart_send(s, sizeof(s) - 1u);
}

static void uart_cmd_status(void)
{
    static const uint8_t p0[] = "raw ";
    uart_send(p0, sizeof(p0) - 1u);
    for(uint8_t ch = 0; ch < 4; ch++) {
        uart_put_u16(g_adc_raw[ch]);
        uint8_t sp = ' ';
        uart_send(&sp, 1);
    }
    static const uint8_t p1[] = "off ";
    uart_send(p1, sizeof(p1) - 1u);
    for(uint8_t ch = 0; ch < 4; ch++) {
        uart_put_u16(g_adc_off[ch]);
        uint8_t sp = ' ';
        uart_send(&sp, 1);
    }
    static const uint8_t p2[] = "state ";
    uart_send(p2, sizeof(p2) - 1u);
    uart_put_u16((uint16_t)g_foc_state);
    static const uint8_t p3[] = " faults ";
    uart_send(p3, sizeof(p3) - 1u);
    uart_put_u16(g_fault_flags);
    /* последнее/макс. время foc_fast_calc в тактах (бюджет 2400) */
    static const uint8_t p4[] = " foc ";
    uart_send(p4, sizeof(p4) - 1u);
    uart_put_u16((uint16_t)g_foc_cycles);
    uint8_t sp2 = ' ';
    uart_send(&sp2, 1);
    uart_put_u16((uint16_t)g_foc_cycles_max);
    /* I/F состояние */
    static const uint8_t p6[] = " if ";
    uart_send(p6, sizeof(p6) - 1u);
    uart_put_u16((uint16_t)g_if_active);
    uint8_t sp3 = ' ';
    uart_send(&sp3, 1);
    uart_put_u16((uint16_t)g_foc_enable);
    /* Уставки I/F для контроля: поток мА, скорость % */
    static const uint8_t p6b[] = " fl ";
    uart_send(p6b, sizeof(p6b) - 1u);
    uart_put_u16((uint16_t)(g_if_flux_ref * 1000.0f)); /* мА */
    static const uint8_t p6c[] = " sp ";
    uart_send(p6c, sizeof(p6c) - 1u);
    uart_put_u16((uint16_t)(g_if_speed_ref * 100.0f)); /* % */
    static const uint8_t p6d[] = " iq ";
    uart_send(p6d, sizeof(p6d) - 1u);
    uart_put_i32((int32_t)(g_foc_iq_ref * 1000.0f)); /* мА */
    static const uint8_t p6f[] = " idm ";
    uart_send(p6f, sizeof(p6f) - 1u);
    uart_put_i32((int32_t)(g_park.de * 1000.0f)); /* мА */
    static const uint8_t p6g[] = " iqm ";
    uart_send(p6g, sizeof(p6g) - 1u);
    uart_put_i32((int32_t)(g_park.qe * 1000.0f)); /* мА */
    static const uint8_t p6e[] = " udc ";
    uart_send(p6e, sizeof(p6e) - 1u);
    uart_put_u16((uint16_t)(g_foc_udc * 1000.0f)); /* мВ */
    /* Углы — миллирадианы (ядро теперь в радианах). */
    static const uint8_t p7[] = " th ";
    uart_send(p7, sizeof(p7) - 1u);
    uart_put_i32((int32_t)(g_foc_angle * 1000.0f));
    static const uint8_t p8[] = " ramp ";
    uart_send(p8, sizeof(p8) - 1u);
    uart_put_u32((uint32_t)(g_ramp.output * 1000.0f));
    /* observer telemetry */
    static const uint8_t p9[] = " vec ";
    uart_send(p9, sizeof(p9) - 1u);
    uart_put_u16((uint16_t)g_foc_ctrl_state);
    static const uint8_t pa[] = " th_if ";
    uart_send(pa, sizeof(pa) - 1u);
    uart_put_i32((int32_t)(g_foc_angle * 1000.0f));
    static const uint8_t pb[] = " th_obs ";
    uart_send(pb, sizeof(pb) - 1u);
    uart_put_i32((int32_t)(g_observer.theta_est * 1000.0f));
    static const uint8_t pi[] = " th_i ";
    uart_send(pi, sizeof(pi) - 1u);
    uart_put_i32((int32_t)(g_observer.theta_i * 1000.0f));
    static const uint8_t pt[] = " terr ";
    uart_send(pt, sizeof(pt) - 1u);
    uart_put_i32((int32_t)(g_observer.theta_err * 1000.0f)); /* ISR-такт, без скью */
    static const uint8_t pj[] = " psi ";
    uart_send(pj, sizeof(pj) - 1u);
    uart_put_i32((int32_t)(g_observer.psi_mag * 1e6f)); /* мкВб */
    static const uint8_t plm[] = " lm ";
    uart_send(plm, sizeof(plm) - 1u);
    uart_put_i32((int32_t)(g_observer.lam_meas * 1e6f)); /* прямая λ, мкВб */
    static const uint8_t pva[] = " va ";
    uart_send(pva, sizeof(pva) - 1u);
    uart_put_i32((int32_t)(g_observer.v_alpha * 1000.0f)); /* мВ */
    static const uint8_t pvb[] = " vb ";
    uart_send(pvb, sizeof(pvb) - 1u);
    uart_put_i32((int32_t)(g_observer.v_beta * 1000.0f)); /* мВ */
    /* H3: адаптация потокосцепления и сектор SVPWM (устойчивость на малых duty).
   */
    static const uint8_t ph[] = " lam_est ";
    uart_send(ph, sizeof(ph) - 1u);
    uart_put_i32((int32_t)(g_observer.state.lambda_est * 1e6f)); /* мкВб */
    static const uint8_t pk[] = " sect ";
    uart_send(pk, sizeof(pk) - 1u);
    uart_put_u16((uint16_t)g_pwm.sector);
    static const uint8_t pc[] = " rpm ";
    uart_send(pc, sizeof(pc) - 1u);
    uart_put_i32((int32_t)g_observer.speed_rpm);
    static const uint8_t pw[] = " wobs ";
    uart_send(pw, sizeof(pw) - 1u);
    uart_put_i32((int32_t)(g_observer.w_obs * 60.0f / (2.0f * _PI * (float)g_motor.pp)));
    static const uint8_t ptx[] = " th_pre ";
    uart_send(ptx, sizeof(ptx) - 1u);
    uart_put_i32((int32_t)(g_observer.state.theta_pre * 1000.0f));
    static const uint8_t pd[] = " inv ";
    uart_send(pd, sizeof(pd) - 1u);
    uart_put_u16((uint16_t)g_foc_inv_i);
    static const uint8_t pe[] = " n ";
    uart_send(pe, sizeof(pe) - 1u);
    uart_put_u16((uint16_t)g_foc_inv_ang);
    static const uint8_t pf[] = " m ";
    uart_send(pf, sizeof(pf) - 1u);
    uart_put_u16((uint16_t)g_foc_adc_map);
    static const uint8_t ph2[] = " e ";
    uart_send(ph2, sizeof(ph2) - 1u);
    uart_put_u16((uint16_t)g_foc_pwm_map);
    static const uint8_t pv[] = " k ";
    uart_send(pv, sizeof(pv) - 1u);
    uart_put_u16((uint16_t)g_foc_obs_vrot);
    static const uint8_t py[] = " y ";
    uart_send(py, sizeof(py) - 1u);
    uart_put_u16((uint16_t)g_foc_pwm_swap);
    static const uint8_t pdj[] = " j ";
    uart_send(pdj, sizeof(pdj) - 1u);
    uart_put_i32((int32_t)(g_foc_ualpha_dc * 1000.0f));
    static const uint8_t pu_[] = " u ";
    uart_send(pu_, sizeof(pu_) - 1u);
    uart_put_i32((int32_t)(g_foc_ubeta_dc * 1000.0f));
    static const uint8_t pz[] = " dtv ";
    uart_send(pz, sizeof(pz) - 1u);
    uart_put_i32((int32_t)(g_vesc_cfg.foc_dt_us * 1000.0f)); /* 0.833 us -> 833 */
    static const uint8_t peav_[] = " eav ";
    uart_send(peav_, sizeof(peav_) - 1u);
    uart_put_i32((int32_t)(g_observer.ea * 1000.0f));
    static const uint8_t peb_[] = " ebv ";
    uart_send(peb_, sizeof(peb_) - 1u);
    uart_put_i32((int32_t)(g_observer.eb * 1000.0f));
    static const uint8_t po_[] = " o ";
    uart_send(po_, sizeof(po_) - 1u);
    uart_put_u16((uint16_t)g_foc_obs_off);
    static const uint8_t pga[] = " ga ";
    uart_send(pga, sizeof(pga) - 1u);
    uart_put_u16((uint16_t)g_pwm.GammaA);
    static const uint8_t pgb[] = " gb ";
    uart_send(pgb, sizeof(pgb) - 1u);
    uart_put_u16((uint16_t)g_pwm.GammaB);
    static const uint8_t pgc[] = " gc ";
    uart_send(pgc, sizeof(pgc) - 1u);
    uart_put_u16((uint16_t)g_pwm.GammaC);
    static const uint8_t pg[] = " blend ";
    uart_send(pg, sizeof(pg) - 1u);
    uart_put_u32(g_foc_vector_blend);
    static const uint8_t p5[] = "\r\n";
    uart_send(p5, sizeof(p5) - 1u);
}

/* Побайтовый приём. После команды с числом цифры копятся до первого
 * нечислового символа; он же обрабатывается как следующая команда. */
/* Счётчик 100 мкс из SysTick: нужен, чтобы текстовый протокол не «съедал»
 * байты бинарных кадров конфигуратора (2 мс тишины после кадра). */
extern volatile uint32_t g_tick_100us;
#define UART_TEXT_LOCK_TICKS 20u

static void uart_rx_poll(void)
{
    static uint32_t s_last_bin = 0;
    static uint8_t s_bin_seen = 0;
    static uint8_t bin_active = 0; /* кадр FOC_Proto начат, ждём хвост */
    static uint8_t acc = 0;        /* 0=нет, 'd'/'w'/'f' = копим число */
    static uint16_t acc_val = 0;
    while(!UART1->FR_bit.RXFE) {
        uint8_t b = (uint8_t)UART1->DR;
        /* Бинарный протокол конфигуратора: кадр начинается с SOF1 (0xAA). */
        if(bin_active || b == FOC_PROTO_SOF1) {
            s_last_bin = g_tick_100us;
            s_bin_seen = 1;
            TFocProtoFrame fr;
            int r = foc_proto_feed(b, &fr);
            if(r == 1) {
                uint8_t resp[FOC_PROTO_MAX_FRAME];
                uint32_t n = foc_proto_handle(&fr, resp, sizeof(resp));
                if(n > 0u)
                    uart_send(resp, n);
                bin_active = 0;
            } else if(r < 0) {
                bin_active = 0; /* ресинхронизация */
            } else {
                bin_active = 1;
            }
            acc = 0;
            acc_val = 0;
            continue;
        }
        if(s_bin_seen && (g_tick_100us - s_last_bin) < UART_TEXT_LOCK_TICKS) {
            acc = 0;
            acc_val = 0;
            continue; /* хвост бинарного кадра: текст игнорим */
        }
        if(acc && b == ' ')
            continue; /* пробел внутри числа — игнор, не конец ("f 2500" = 2500) */
        if(acc && b >= '0' && b <= '9') {
            acc_val = (uint16_t)(acc_val * 10u + (uint16_t)(b - '0'));
            if(acc == 'd' && acc_val > 1999u)
                acc_val = 1999u;
            if(acc == 'w' && acc_val > 100u)
                acc_val = 100u;
            if((acc == 'f' || acc == 'q') && acc_val > 8000u)
                acc_val = 8000u;
            if((acc == 'R' || acc == 'L') && acc_val > 10000u)
                acc_val = 10000u;
            if(acc == 'B' && acc_val > 65000u)
                acc_val = 65000u;
            if(acc == 'z' && acc_val > 2000u)
                acc_val = 2000u;
            continue;
        }
        if(acc) {
            /* конец числа — применить (clamp уже выше) */
            uint8_t cmd = acc;
            uint16_t v = acc_val;
            acc = 0;
            acc_val = 0;
            if(g_foc_state != STATE_NORMAL && (cmd == 'd' || cmd == 'j' || cmd == 'u')) {
                static const uint8_t err[] = "FAULT\r\n";
                uart_send(err, sizeof(err) - 1u);
                continue;
            }
            if(cmd == 'd') {
                /* ручной duty только при выключенном FOC — иначе ISR
         * перезапишет CMPA в следующем такте */
                if(!g_foc_enable) {
                    if(v != 0u && !foc_outputs_arm())
                        continue;
                    g_manual_pwm_active = v != 0;
                    pwm_set_duty_all(v);
                    if(v == 0u)
                        foc_ctrl_stop();
                }
            } else if(cmd == 'w') {
                /* скорость I/F, % номинала -> pu */
                g_if_speed_ref = (float)v / 100.0f;
            } else if(cmd == 'f') {
                /* ток намагничивания, мА -> А */
                g_if_flux_ref = (float)v / 1000.0f;
            } else if(cmd == 'q') {
                /* ток момента, мА -> А; clamp FOC_IQ_MAX до */
                float iq = (float)v / 1000.0f;
                if(iq > FOC_IQ_MAX)
                    iq = FOC_IQ_MAX;
                if(iq < -FOC_IQ_MAX)
                    iq = -FOC_IQ_MAX;
                g_foc_iq_ref = iq;
            } else if(cmd == 'R') {
                g_motor.R_mOhm = v;
                flux_observer_slow_calc(&g_observer);
            } else if(cmd == 'L') {
                g_motor.L_uH = v;
                flux_observer_slow_calc(&g_observer);
            } else if(cmd == 'B') {
                g_motor.lambda_uWb = v;
                flux_observer_slow_calc(&g_observer);
            } else if(cmd == 'z') {
                /* dead-time компенсация VESC, us/1000 (833 -> 0.833 us) */
                g_vesc_cfg.foc_dt_us = (float)v / 1000.0f;
            } else if(cmd == 'j' || cmd == 'u') {
                /* DC-тест: j = Ualpha N/1000 pu, u = Ubeta N/1000 pu */
                if(cmd == 'j')
                    g_foc_ualpha_dc = (float)v / 1000.0f;
                else
                    g_foc_ubeta_dc = (float)v / 1000.0f;
                if(!foc_outputs_arm())
                    continue;
                g_foc_enable = 1;
                g_if_active = 0;
                g_foc_ctrl_state = FOC_CTRL_I2F;
            }
            /* конец числа — применить (clamp уже выше) */
            static const uint8_t ok[] = "OK\r\n";
            uart_send(ok, sizeof(ok) - 1u);
            if(b == '\r' || b == '\n')
                continue;
        }
        switch(b) {
            case '?':
                uart_cmd_help();
                break;
            case 's':
                uart_cmd_status();
                break;
            case 't':
                uart_telemetry(); /* разовый бинарный кадр AA 55 ... xor */
                break;
            case 'd':
            case 'w':
            case 'f':
            case 'q':
            case 'R':
            case 'L':
            case 'B':
            case 'j':
            case 'u':
            case 'z':
                acc = b;
                acc_val = 0;
                break;
            case 'r':
                if(g_foc_state != STATE_NORMAL)
                    break;
                g_manual_pwm_active = 0;
                g_foc_ualpha_dc = 0.0f; /* j-тест гасится, r снова запускает I/F */
                g_foc_ubeta_dc = 0.0f;
                foc_if_start();
                break;
            case 'v':
                if(g_foc_state != STATE_NORMAL)
                    break;
                foc_vector_start();
                break;
            case 'x':
                g_manual_pwm_active = 0;
                g_foc_ualpha_dc = 0.0f;
                g_foc_ubeta_dc = 0.0f;
                foc_detect_abort();
                break;
            case 'p':
                g_foc_inv_i = !g_foc_inv_i;
                break;
            case 'n':
                g_foc_inv_ang = !g_foc_inv_ang;
                break;
            case 'm':
                g_foc_adc_map = (uint8_t)((g_foc_adc_map + 1u) % 3u);
                break;
            case 'e': /* циклическая пермутация SVM->CMPA (A/B/C по PWM0/1/2) */
                g_foc_pwm_map = (uint8_t)((g_foc_pwm_map + 1u) % 3u);
                break;
            case 'k': /* поворот v-входа наблюдателя на -120°*k */
                g_foc_obs_vrot = (uint8_t)((g_foc_obs_vrot + 1u) % 3u);
                break;
            case 'y': /* обмен A<->B в CMPA (зеркало фаз) */
                g_foc_pwm_swap = (uint8_t)!g_foc_pwm_swap;
                break;
            case 'o': /* константный вычет угла наблюдателя: 0/−120°/+120° */
                g_foc_obs_off = (uint8_t)((g_foc_obs_off + 1u) % 3u);
                break;
            default:
                break; /* \r \n и мусор — игнор */
        }
    }
}

void uart_debug_poll(void)
{
    uart_rx_poll(); /* потока нет: UART молчит, пока не спросят */
}
