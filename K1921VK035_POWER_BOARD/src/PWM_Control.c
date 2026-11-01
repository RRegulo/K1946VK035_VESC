/*
 * PWM_Control.c
 *
 *  Created on: 23 May 2026
 *      Author: TDA
 */
#include "PWM_Control.h"

#define PWM_TZ_PIN 7
#define CONCAT2(a, b) a##b
#define PIN_(n) CONCAT2(PIN, n)

/* Up-down PWM 25 кГц, TBPRD=2000 при TBCLK=100 МГц.
 * B — комплементарный выход A с dead-time на обоих фронтах. */
#define PWM_TIM1_AUTORELOAD 2000u /* 25 kHz @ TBCLK 100 MHz (up-down) */
#define PWM_DEAD_TIME 40u         /* ~0.4 us @ TBCLK 100 MHz */

void PWM_Module_Init(void)
{
    SIU->PWMSYNC_bit.PRESCRST = 0; /* демо V_PWM_Module.c:33 — сброс прескалера */
    GPIOA->DENSET = GPIO_DENSET_PIN8_Msk | GPIO_DENSET_PIN9_Msk | GPIO_DENSET_PIN10_Msk |
                    GPIO_DENSET_PIN11_Msk | GPIO_DENSET_PIN12_Msk | GPIO_DENSET_PIN13_Msk;
    GPIOA->ALTFUNCSET = GPIO_ALTFUNCSET_PIN8_Msk | GPIO_ALTFUNCSET_PIN9_Msk |
                        GPIO_ALTFUNCSET_PIN10_Msk | GPIO_ALTFUNCSET_PIN11_Msk |
                        GPIO_ALTFUNCSET_PIN12_Msk | GPIO_ALTFUNCSET_PIN13_Msk;

    RCU->PCLKCFG_bit.PWM0EN = 1;
    RCU->PRSTCFG_bit.PWM0EN = 1;
    RCU->PCLKCFG_bit.PWM1EN = 1;
    RCU->PRSTCFG_bit.PWM1EN = 1;
    RCU->PCLKCFG_bit.PWM2EN = 1;
    RCU->PRSTCFG_bit.PWM2EN = 1;

    PWM0->TBPRD = PWM_TIM1_AUTORELOAD;
    PWM1->TBPRD = PWM_TIM1_AUTORELOAD;
    PWM2->TBPRD = PWM_TIM1_AUTORELOAD;

    PWM0->TBPHS_bit.TBPHS = 0;
    PWM1->TBPHS_bit.TBPHS = 1; /* демо :125 — компенсация dead-time/sync */
    PWM2->TBPHS_bit.TBPHS = 1;

    PWM0->TBCTR = 0;
    PWM1->TBCTR = 0;
    PWM2->TBCTR = 0;

    /* Синхронизация трёх PWM-блоков. */
    PWM0->TBCTL = (TB_SHADOW << PWM_TBCTL_PRDLD_Pos) | (TB_COUNT_UPDOWN << PWM_TBCTL_CTRMODE_Pos) |
                  (TB_DISABLE << PWM_TBCTL_PHSEN_Pos) | (0 << PWM_TBCTL_HSPCLKDIV_Pos) |
                  (0 << PWM_TBCTL_CLKDIV_Pos) | (TB_CTR_ZERO << PWM_TBCTL_SYNCOSEL_Pos) |
                  (2 << PWM_TBCTL_FREESOFT_Pos) | (1 << PWM_TBCTL_SHDWGLOB_Pos);
    PWM1->TBCTL = (TB_SHADOW << PWM_TBCTL_PRDLD_Pos) | (TB_COUNT_UPDOWN << PWM_TBCTL_CTRMODE_Pos) |
                  (TB_ENABLE << PWM_TBCTL_PHSEN_Pos) | (TB_UP << PWM_TBCTL_PHSDIR_Pos) |
                  (0 << PWM_TBCTL_HSPCLKDIV_Pos) | (0 << PWM_TBCTL_CLKDIV_Pos) |
                  (TB_SYNC_IN << PWM_TBCTL_SYNCOSEL_Pos) | (2 << PWM_TBCTL_FREESOFT_Pos) |
                  (1 << PWM_TBCTL_SHDWGLOB_Pos);
    PWM2->TBCTL = PWM1->TBCTL;

    PWM0->ETSEL_bit.SOCAEN = 1;
    PWM0->ETSEL_bit.SOCASEL = ET_CTR_ZERO;

    PWM0->CMPCTL =
        (CC_SHADOW << PWM_CMPCTL_SHDWAMODE_Pos) | (CC_CTR_ZERO << PWM_CMPCTL_LOADAMODE_Pos) |
        (CC_SHADOW << PWM_CMPCTL_SHDWBMODE_Pos) | (CC_CTR_ZERO << PWM_CMPCTL_LOADBMODE_Pos);
    PWM1->CMPCTL = PWM0->CMPCTL;
    PWM2->CMPCTL = PWM0->CMPCTL;

    /* ZRO: clear, CAU: set, CAD: clear. */
    PWM0->AQCTLA = (AQ_CLEAR << PWM_AQCTLA_ZRO_Pos) | (AQ_SET << PWM_AQCTLA_CAU_Pos) |
                   (AQ_CLEAR << PWM_AQCTLA_CAD_Pos);
    PWM1->AQCTLA = PWM0->AQCTLA;
    PWM2->AQCTLA = PWM0->AQCTLA;
    PWM0->AQCTLB = 0;
    PWM1->AQCTLB = 0;
    PWM2->AQCTLB = 0;

    PWM0->AQSFRC_bit.RLDCSF = 0;
    PWM1->AQSFRC_bit.RLDCSF = 0;
    PWM2->AQSFRC_bit.RLDCSF = 0;

    /* Комплементарные выходы с dead-time. */
    PWM0->DBRED = PWM_DEAD_TIME;
    PWM0->DBFED = PWM_DEAD_TIME;
    PWM0->DBCTL = (DBA_ALL << PWM_DBCTL_INMODE_Pos) | (DB_ACTV_HIC << PWM_DBCTL_POLSEL_Pos) |
                  (DB_FULL_ENABLE << PWM_DBCTL_OUTMODE_Pos);
    PWM1->DBRED = PWM_DEAD_TIME;
    PWM1->DBFED = PWM_DEAD_TIME;
    PWM1->DBCTL = PWM0->DBCTL;
    PWM2->DBRED = PWM_DEAD_TIME;
    PWM2->DBFED = PWM_DEAD_TIME;
    PWM2->DBCTL = PWM0->DBCTL;

    /* Принудительный OST держит выходы в Hi-Z до pwm_module_on(). */
    PWM0->TZCTL = (TZ_HIZ << PWM_TZCTL_TZA_Pos) | (TZ_HIZ << PWM_TZCTL_TZB_Pos);
    PWM1->TZCTL = PWM0->TZCTL;
    PWM2->TZCTL = PWM0->TZCTL;

    PWM0->CMPA_bit.CMPA = 0;
    PWM1->CMPA_bit.CMPA = 0;
    PWM2->CMPA_bit.CMPA = 0;

    PWM0->TZFRC_bit.OST = 1;
    PWM1->TZFRC_bit.OST = 1;
    PWM2->TZFRC_bit.OST = 1;

    SIU->PWMSYNC_bit.PRESCRST = 0x07; /* демо :206 */
}

/* Разрешение силовых выходов. */
void pwm_module_on(void)
{
    PWM0->TZCLR_bit.OST = 1;
    PWM1->TZCLR_bit.OST = 1;
    PWM2->TZCLR_bit.OST = 1;
}

/* Сброс протухших пендингов после pwm_module_on, до __enable_irq.
 * TZFRC.OST из init + включённый TZEINT дают импульс PWM_TZINT в NVIC,
 * который висит до первого __enable_irq и кладёт плату в FAULT на старте
 * (пин при этом в idle-HIGH, реального трипа нет). Демо так не страдает —
 * оно TZEINT не включает вообще. */
void PWM_FaultIrq_Drain(void)
{
    PWM0->TZCLR_bit.INT = 1;
    PWM1->TZCLR_bit.INT = 1;
    PWM2->TZCLR_bit.INT = 1;
    PWM0->TZINTCLR_bit.INT = 1;
    PWM1->TZINTCLR_bit.INT = 1;
    PWM2->TZINTCLR_bit.INT = 1;
    PWM0->HDCLR_bit.INT = 1;
    PWM1->HDCLR_bit.INT = 1;
    PWM2->HDCLR_bit.INT = 1;
    NVIC_ClearPendingIRQ(PWM0_TZ_IRQn);
    NVIC_ClearPendingIRQ(PWM1_TZ_IRQn);
    NVIC_ClearPendingIRQ(PWM2_TZ_IRQn);
    NVIC_ClearPendingIRQ(PWM0_HD_IRQn);
    NVIC_ClearPendingIRQ(PWM1_HD_IRQn);
    NVIC_ClearPendingIRQ(PWM2_HD_IRQn);
    NVIC_ClearPendingIRQ(ADC_DC_IRQn);
    NVIC_ClearPendingIRQ(ADC_SEQ0_IRQn); /* набежал за калибровку */
}

void pwm_module_off(void)
{
    PWM0->TZFRC_bit.OST = 1;
    PWM1->TZFRC_bit.OST = 1;
    PWM2->TZFRC_bit.OST = 1;
}

/* Ручное заполнение всех фаз; CMPB симметричен относительно середины периода. */
void pwm_set_duty_all(uint16_t dc)
{
    PWM0->CMPA_bit.CMPA = dc;
    PWM0->CMPB_bit.CMPB = PWM_TIM1_AUTORELOAD - dc;
    PWM1->CMPA_bit.CMPA = dc;
    PWM1->CMPB_bit.CMPB = PWM_TIM1_AUTORELOAD - dc;
    PWM2->CMPA_bit.CMPA = dc;
    PWM2->CMPB_bit.CMPB = PWM_TIM1_AUTORELOAD - dc;
}

/* Настройки защиты перенесены из проекта платы. Включать после проверки
 * полярности аварийного входа и реакции силовой части на конкретной плате. */
void PWM_TZ_Protection_Init(void)
{
#ifdef ENABLE_EXPERIMENTAL_PROTECTION
    PWM0->TZSEL_bit.OST = 1;
    PWM1->TZSEL_bit.OST = 1;
    PWM2->TZSEL_bit.OST = 1;
    PWM0->TZCTL_bit.TZA = 1;
    PWM0->TZCTL_bit.TZB = 1;
    PWM1->TZCTL_bit.TZA = 1;
    PWM1->TZCTL_bit.TZB = 1;
    PWM2->TZCTL_bit.TZA = 1;
    PWM2->TZCTL_bit.TZB = 1;
    PWM0->TZEINT_bit.OST = 1;
    PWM1->TZEINT_bit.OST = 1;
    PWM2->TZEINT_bit.OST = 1;
    GPIOA->DENSET_bit.PIN_(PWM_TZ_PIN) = 1;
    GPIOA->ALTFUNCSET_bit.PIN_(PWM_TZ_PIN) = 1;
    GPIOA->SYNCCLR_bit.PIN_(PWM_TZ_PIN) = 1;
    GPIOA->QUALSET_bit.PIN_(PWM_TZ_PIN) = 1;
    GPIOA->QUALMODECLR_bit.PIN_(PWM_TZ_PIN) = 1;
    GPIOA->QUALSAMPLE_bit.VAL = 9;
    NVIC_EnableIRQ(PWM0_TZ_IRQn);
    NVIC_EnableIRQ(PWM1_TZ_IRQn);
    NVIC_EnableIRQ(PWM2_TZ_IRQn);
#endif
}

void PWM_HD_Protection_Init(void)
{
#ifdef ENABLE_EXPERIMENTAL_PROTECTION
    PWM0->HDCTL_bit.HDA = PWM_HDCTL_HDA_Clear;
    PWM0->HDCTL_bit.HDB = PWM_HDCTL_HDB_Clear;
    PWM1->HDCTL_bit.HDA = PWM_HDCTL_HDA_Clear;
    PWM1->HDCTL_bit.HDB = PWM_HDCTL_HDB_Clear;
    PWM2->HDCTL_bit.HDA = PWM_HDCTL_HDA_Clear;
    PWM2->HDCTL_bit.HDB = PWM_HDCTL_HDB_Clear;
    PWM0->HDSEL_bit.OST = 1;
    PWM1->HDSEL_bit.OST = 1;
    PWM2->HDSEL_bit.OST = 1;
    PWM0->HDEINT_bit.OST = 1;
    PWM1->HDEINT_bit.OST = 1;
    PWM2->HDEINT_bit.OST = 1;
    NVIC_EnableIRQ(PWM0_HD_IRQn);
    NVIC_EnableIRQ(PWM1_HD_IRQn);
    NVIC_EnableIRQ(PWM2_HD_IRQn);
#endif
}

/* ADC SEQ0 используется FOC. Пороги цифровых компараторов ещё не настроены. */
uint8_t ADC_DC_Overcurrent_Init()
{
    return 1;
}

/* сырые замеры SEQ0 + нули. Читаются из ADC_SEQ0_IRQHandler (main.c),
 * ноль калибруется при старте на закрытых ключах. */
volatile uint16_t g_adc_raw[4] = {0, 0, 0, 0};
volatile uint16_t g_adc_off[4] = {2048, 2048, 2048, 2048};

/* SEQ0 измеряет токи фаз и напряжение питания. PWM SOCA запускает
 * последовательность каждый четвёртый период; усреднение включено. */
void ADC_FOC_Init(void)
{
    RCU->ADCCFG_bit.CLKSEL = RCU_ADCCFG_CLKSEL_PLLCLK; /* ACLK от PLL */
    RCU->ADCCFG_bit.DIVN = 0x1;                        /* N = 2*(DIVN+1) = 4 → ACLK = 24 МГц */
    RCU->ADCCFG_bit.DIVEN = 0x1;
    RCU->ADCCFG_bit.CLKEN = 0x1;
    RCU->ADCCFG_bit.RSTDIS = 0x1;
    ADC->ACTL_bit.ADCEN = 0x1;

    RCU->HCLKCFG_bit.GPIOBEN = 1;
    RCU->HRSTCFG_bit.GPIOBEN = 1;
    GPIOB->DENSET =
        GPIO_DENSET_PIN0_Msk | GPIO_DENSET_PIN1_Msk | GPIO_DENSET_PIN2_Msk | GPIO_DENSET_PIN3_Msk;
    GPIOB->ALTFUNCSET = GPIO_ALTFUNCSET_PIN0_Msk | GPIO_ALTFUNCSET_PIN1_Msk |
                        GPIO_ALTFUNCSET_PIN2_Msk | GPIO_ALTFUNCSET_PIN3_Msk;

    ADC->EMUX_bit.EM0 = ADC_EMUX_EM0_PWM012A; /* триггер от SOCA */
    /* SOCAPRD=3 (0b11) -> каждый 4-й период (~160мкс @25кГц) */
    PWM0->ETPS_bit.SOCAPRD = ET_3RD;
    ADC->SEQSYNC = ADC_SEQSYNC_SYNC0_Msk;
    ADC->SEQ[0].SRQCTL_bit.RQMAX = 0x3; /* 4 замера: RQMAX+1 */
    ADC->SEQ[0].SRQSEL_bit.RQ0 = 0x0;
    ADC->SEQ[0].SRQSEL_bit.RQ1 = 0x1;
    ADC->SEQ[0].SRQSEL_bit.RQ2 = 0x2;
    ADC->SEQ[0].SRQSEL_bit.RQ3 = 0x3;
    ADC->SEQ[0].SCCTL_bit.RCNT = 3;
    /* (TBPRD / AdcDiv) / (RCNT + 1) = 125 */
    ADC->SEQ[0].SRTMR_bit.VAL = 125;
    ADC->SEQ[0].SCCTL_bit.ICNT = 3;   /* повторов ICNT+1 = 4 */
    ADC->SEQ[0].SCCTL_bit.RAVGEN = 1; /* усреднение по 4 */
    ADC->SEQEN_bit.SEQEN0 = 1;

    while(ADC->SEQ[0].SFLOAD_bit.VAL)
        (void)ADC->SEQ[0].SFIFO_bit.DATA;

    while(!ADC->ACTL_bit.ADCRDY)
        ;

    ADC->IM_bit.SEQIM0 = 1;
    NVIC_EnableIRQ(ADC_SEQ0_IRQn);
    NVIC_SetPriority(ADC_SEQ0_IRQn, 2);
}

/* Вызывается после чтения FIFO в ISR, когда последовательность завершена. */
void ADC_FOC_L_Mode(uint8_t enable)
{
    static uint32_t scctl, srtmr, socaprd;
    PWM0->ETSEL_bit.SOCAEN = 0;
    ADC->SEQEN_bit.SEQEN0 = 0;
    if(enable) {
        scctl = ADC->SEQ[0].SCCTL;
        srtmr = ADC->SEQ[0].SRTMR;
        socaprd = PWM0->ETPS_bit.SOCAPRD;
        ADC->SEQ[0].SCCTL = 0;
        ADC->SEQ[0].SRTMR = 0;
        PWM0->ETPS_bit.SOCAPRD = 0; /* выборка каждый период PWM, 40 мкс */
    } else {
        ADC->SEQ[0].SCCTL = scctl;
        ADC->SEQ[0].SRTMR = srtmr;
        PWM0->ETPS_bit.SOCAPRD = socaprd;
    }
    ADC->SEQEN_bit.SEQEN0 = 1;
    PWM0->ETSEL_bit.SOCAEN = 1;
}

/* калибровка нуля. Вызывается ДО pwm_module_on — ключи закрыты,
 * ток фаз физически 0. 256 замеров SEQ0, среднее → g_adc_off. */
void ADC_FOC_CaptureOffset(void)
{
    uint32_t sum[4] = {0, 0, 0, 0};

    for(uint32_t i = 0; i < 256; i++) {
        while(ADC->SEQ[0].SFLOAD_bit.VAL < 4)
            ;
        sum[0] += ADC->SEQ[0].SFIFO_bit.DATA;
        sum[1] += ADC->SEQ[0].SFIFO_bit.DATA;
        sum[2] += ADC->SEQ[0].SFIFO_bit.DATA;
        sum[3] += ADC->SEQ[0].SFIFO_bit.DATA;
        WDT->INTCLR = 1u;
    }

    for(int ch = 0; ch < 4; ch++)
        g_adc_off[ch] = (uint16_t)(sum[ch] / 256u);

    ADC->IC_bit.SEQIC0 = 1;
    NVIC_ClearPendingIRQ(ADC_SEQ0_IRQn);
}
