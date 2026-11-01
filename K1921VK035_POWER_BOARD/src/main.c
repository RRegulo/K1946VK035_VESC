/******************************************************************************
 * @file        main.c
 * @brief       BLDC FOC: initialization, interrupts and fault handling
 * @version     v2.0
 * @date        09.09.2026
 *
 * @note TDM LAB
 *
 * FSM: STATE_NORMAL -> STATE_OVERCURRENT_FAULT (latched until reset)
 * Faults: PWM TZ (ext OC_TRIG), PWM HD, ADC digital comparator DC overcurrent
 ******************************************************************************/

#include "main.h"
#include "FOC_Core.h"
#include "FOC_Detect.h"
#include "FOC_Params.h"
#include "FOC_Protection.h"
#include "PWM_Control.h"
#include "UART_Debug.h"

/* LED1 / PA14 — статус, LED2 / PA15 — авария. */
#define LED_PORT GPIOA
#define LED1_PIN 14
#define LED2_PIN 15

/* Коды аварий общие с UART-конфигуратором. TZ/HD передаются как SW_OC. */
#define FAULT_TZ FAULT_SW_OC       /* PWM trip-zone, ext OC_TRIG (PA7) */
#define FAULT_HD FAULT_SW_OC       /* PWM hardware dead-band detector */
#define FAULT_ADC_DC FAULT_ADC_SAT /* ADC digital comparator overcurrent */

volatile fsm_state_t g_foc_state = STATE_NORMAL;
volatile uint16_t g_fault_flags = 0;

volatile uint32_t g_tick_100us = 0; /* SysTick counter, 10 kHz; extern в UART */

void SysTick_Handler(void)
{
    g_tick_100us++;
    if(g_foc_state == STATE_OVERCURRENT_FAULT) {
        GPIOA->DATAOUTCLR_bit.PIN14 = 1; /* fault: LED1 гаснет — видно сразу */
        return;
    }
    if((g_tick_100us % 5000u) == 0u) /* normal: LED1 1 Hz */
        GPIOA->DATAOUTTGL_bit.PIN14 = 1;
}

/* ------------------------------------------------------------------ */
/* Watchdog */

static void wdt_init(void)
{
    RCU->WDTCFG_bit.CLKEN = 1;
    RCU->WDTCFG_bit.RSTDIS = 1;
    WDT->LOAD = 200000;
    WDT->CTRL_bit.INTEN = 1;
    WDT->CTRL_bit.RESEN = 1;
}

static void wdt_feed(void)
{
    WDT->INTCLR = 1u;
}

/* ------------------------------------------------------------------ */
static void led_init(void)
{
    RCU->HCLKCFG_bit.GPIOAEN = 1; /* enable GPIO clock (SystemInit does not!) */
    RCU->HRSTCFG_bit.GPIOAEN = 1;

    GPIOA->LOCKKEY = 0xADEADBEEu; /* unlock pin config for write */
    GPIOA->LOCKCLR_bit.PIN14 = 1;
    GPIOA->LOCKCLR_bit.PIN15 = 1;

    GPIOA->ALTFUNCCLR_bit.PIN14 = 1;
    GPIOA->ALTFUNCCLR_bit.PIN15 = 1;
    GPIOA->DENSET_bit.PIN14 = 1;
    GPIOA->DENSET_bit.PIN15 = 1;
    GPIOA->INMODE_bit.PIN14 = 0;
    GPIOA->INMODE_bit.PIN15 = 0;
    GPIOA->OUTENSET_bit.PIN14 = 1;
    GPIOA->OUTENSET_bit.PIN15 = 1;
    /* LEDs active-high (схема: PA→1K→LED→GND). Normal: LED1 мигает 1Гц из
   * SysTick, LED2 погашен; fault: LED1 гаснет, LED2 мигает 5Гц. */
    GPIOA->DATAOUTCLR_bit.PIN14 = 1;
    GPIOA->DATAOUTCLR_bit.PIN15 = 1;
}

/* LED handled directly in SysTick_Handler */

/* ------------------------------------------------------------------ */
static void systick_init(void)
{
    /* SysTick 10 кГц -> 100 мкс. Reload пересчитывается от SystemCoreClock
   * после обновления из PLL. */
    SystemCoreClockUpdate();
    SysTick_Config((SystemCoreClock / 10000u) - 1u);
    NVIC_EnableIRQ(SysTick_IRQn);
    NVIC_SetPriority(SysTick_IRQn, 0xF);
}

int16 main(void)
{
    __disable_irq(); /* init под маской: иначе протухший пендинг TZ уводит в
                    * ISR посреди инициализации (PRIMASK=0 после reset) */
    SystemInit();

    systick_init();
    foc_dwt_init(); /* DWT CYCCNT для замера времени выполнения */

    led_init();
    foc_init();
    foc_params_init(); /* параметры из flash, иначе дефолты; применяет их */
    PWM_Module_Init(); /* Ключи закрыты до включения выходов */
    PWM_TZ_Protection_Init();
    PWM_HD_Protection_Init();
    ADC_DC_Overcurrent_Init();
    ADC_FOC_Init();          /* Четыре канала SEQ0 от PWM SOCA */
    ADC_FOC_CaptureOffset(); /* Калибровка на закрытых ключах */
    uart_debug_init();       /* UART1, 115200 бод */
    wdt_init();

    /* После калибровки выходы закрыты до команды запуска. */
    /* Выходы остаются в Hi-Z до команды запуска. */
    PWM_FaultIrq_Drain(); /* протухший пендинг от TZFRC — до __enable_irq */

    __enable_irq();
    EINT;

    while(1) {
        wdt_feed();
        uart_debug_poll();     /* Приём команд по UART */
        foc_detect_tick();     /* Измерение R/L/lambda (неблокирующее) */
        foc_vesc_timer_tick(); /* Контур скорости (неблокирующий) */
        /* LED2: 5Гц при latched fault (active-high, шарится с SW3) */
        if(g_foc_state == STATE_OVERCURRENT_FAULT) {
            if((g_tick_100us % 2000u) < 1000u)
                GPIOA->DATAOUTCLR_bit.PIN15 = 1;
            else
                GPIOA->DATAOUTSET_bit.PIN15 = 1;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Fault ISRs: latch fault, clear INT only.
 * TZ/HD OST flags are NOT cleared -> PWM outputs stay forced off until reset.
 */

static void fault_set(uint16_t bits)
{
    foc_ctrl_stop();
    g_fault_flags |= bits;
    g_foc_state = STATE_OVERCURRENT_FAULT;
}

/* One-shot: чистим защёлкнутый INT-флаг периферии (TZCLR/HDCLR.INT) +
 * пендинг и МАСИРУЕМ прерывание (TZEINT/HDEINT.OST=0). Иначе удержанный
 * вход трипа даёт шторм ISR → WDT-ресеты. OST-защёлку НЕ трогаем —
 * выходы остаются в HIZ до reset. */
void PWM0_TZ_IRQHandler(void)
{
    fault_set(FAULT_TZ);
    PWM0->TZEINT_bit.OST = 0;
    PWM0->TZCLR_bit.INT = 1;
    PWM0->TZINTCLR_bit.INT = 1;
    NVIC_ClearPendingIRQ(PWM0_TZ_IRQn);
}

void PWM1_TZ_IRQHandler(void)
{
    fault_set(FAULT_TZ);
    PWM1->TZEINT_bit.OST = 0;
    PWM1->TZCLR_bit.INT = 1;
    PWM1->TZINTCLR_bit.INT = 1;
    NVIC_ClearPendingIRQ(PWM1_TZ_IRQn);
}

void PWM2_TZ_IRQHandler(void)
{
    fault_set(FAULT_TZ);
    PWM2->TZEINT_bit.OST = 0;
    PWM2->TZCLR_bit.INT = 1;
    PWM2->TZINTCLR_bit.INT = 1;
    NVIC_ClearPendingIRQ(PWM2_TZ_IRQn);
}

void PWM0_HD_IRQHandler(void)
{
    fault_set(FAULT_HD);
    PWM0->HDEINT_bit.OST = 0;
    PWM0->HDCLR_bit.INT = 1;
    PWM0->HDINTCLR_bit.INT = 1;
    NVIC_ClearPendingIRQ(PWM0_HD_IRQn);
}

void PWM1_HD_IRQHandler(void)
{
    fault_set(FAULT_HD);
    PWM1->HDEINT_bit.OST = 0;
    PWM1->HDCLR_bit.INT = 1;
    PWM1->HDINTCLR_bit.INT = 1;
    NVIC_ClearPendingIRQ(PWM1_HD_IRQn);
}

void PWM2_HD_IRQHandler(void)
{
    fault_set(FAULT_HD);
    PWM2->HDCLR_bit.INT = 1;
    PWM2->HDEINT_bit.OST = 0;
    PWM2->HDINTCLR_bit.INT = 1;
    NVIC_ClearPendingIRQ(PWM2_HD_IRQn);
}

/* Чтение результатов ADC и запуск цикла FOC. */
void ADC_SEQ0_IRQHandler(void)
{
    uint32_t t0 = DWT->CYCCNT; /* Замер времени foc_fast_calc */
    /* IRQ приходит раньше последнего результата. Ожидание — как в эталоне. */
    {
        uint32_t guard = 2000u;
        while(ADC->SEQ[0].SFLOAD_bit.VAL < 4u && guard != 0u)
            guard--;
        if(ADC->SEQ[0].SFLOAD_bit.VAL < 4u) {
            while(ADC->SEQ[0].SFLOAD_bit.VAL)
                (void)ADC->SEQ[0].SFIFO_bit.DATA;
            ADC->IC_bit.SEQIC0 = 1;
            NVIC_ClearPendingIRQ(ADC_SEQ0_IRQn);
            return;
        }
    }
    g_adc_raw[0] = ADC->SEQ[0].SFIFO_bit.DATA;
    g_adc_raw[1] = ADC->SEQ[0].SFIFO_bit.DATA;
    g_adc_raw[2] = ADC->SEQ[0].SFIFO_bit.DATA;
    g_adc_raw[3] = ADC->SEQ[0].SFIFO_bit.DATA;
    while(ADC->SEQ[0].SFLOAD_bit.VAL)
        (void)ADC->SEQ[0].SFIFO_bit.DATA;
    uint16_t raw[4], offset[4];
    for(unsigned i = 0; i < 4; ++i) {
        raw[i] = g_adc_raw[i];
        offset[i] = g_adc_off[i];
    }
    g_foc_udc = (float)raw[3] * 0.012890625f;
    uint16_t faults =
        foc_protection_check(&g_foc_protection, raw, offset, g_foc_enable || g_manual_pwm_active);
    if(faults)
        fault_set(faults);
    bool l_sample = foc_detect_l_sample(raw);
    if(!l_sample && g_foc_state == STATE_NORMAL)
        foc_fast_calc();
    uint32_t dt = DWT->CYCCNT - t0;
    g_foc_cycles = dt;
    if(dt > g_foc_cycles_max)
        g_foc_cycles_max = dt;
    ADC->IC_bit.SEQIC0 = 1;
    NVIC_ClearPendingIRQ(ADC_SEQ0_IRQn);
}

void ADC_DC_IRQHandler(void)
{
    if(ADC->DCTRIG_bit.DCEV0) {
        ADC->DC[0].DCTL_bit.CIE = 0; /* one-shot: маскируем, иначе шторм */
        ADC->IC_bit.DCIC0 = 1;
        fault_set(FAULT_ADC_DC);
    }
    if(ADC->DCTRIG_bit.DCEV1) {
        ADC->DC[1].DCTL_bit.CIE = 0;
        ADC->IC_bit.DCIC1 = 1;
        fault_set(FAULT_ADC_DC);
    }
    if(ADC->DCTRIG_bit.DCEV2) {
        ADC->DC[2].DCTL_bit.CIE = 0;
        ADC->IC_bit.DCIC2 = 1;
        fault_set(FAULT_ADC_DC);
    }
    if(ADC->DCTRIG_bit.DCEV3) {
        ADC->DC[3].DCTL_bit.CIE = 0;
        ADC->IC_bit.DCIC3 = 1;
        fault_set(FAULT_ADC_DC);
    }
}
