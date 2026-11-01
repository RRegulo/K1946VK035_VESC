/*
 * PWM_Control.h
 *
 *      Author: TDA
 */

#ifndef INCLUDE_PWM_CONTROL_H_
#define INCLUDE_PWM_CONTROL_H_

#include <stdint.h>
#ifndef FOC_HOST_TEST
#include "EPwm_defines.h"
#include "K1921VK035.h"
#include "main.h"
#include "math.h"
#endif

typedef enum { TZ_MODE_ONE_SHOT, TZ_MODE_CYCLE_BY_CYCLE } tz_mode_t;

typedef struct {
    tz_mode_t mode;
    uint8_t filter_width;
    uint32_t tza_action;
    uint32_t tzb_action;
} tz_config_t;

void PWM_Module_Init(void);
/* Ручное заполнение 0..1999 для всех трёх фаз. */
void pwm_set_duty_all(uint16_t dc);
void pwm_module_on(void);
void pwm_module_off(void);
void PWM_FaultIrq_Drain(void);
void PWM_TZ_Protection_Init(void);
void PWM_HD_Protection_Init(void);
// Set TZ input filter width in nanoseconds for all PWM modules
void PWM_TZ_SetFilter_ns(uint32_t ns);

// ADC with Digital Comparator for overcurrent protection
// Configures PB0, PB1, PB2 as ADC channels 0, 1, 2 with digital comparators
// Triggers on N consecutive samples >= highThreshold (code 4095 = full scale)
// Returns 1 on success, 0 on invalid parameters
uint8_t ADC_DC_Overcurrent_Init();
void ADC_DC_IRQHandler(void);

// ADC FOC: SEQ0, ch0-2 токи фаз, ch3 VBAT, триггер SOCA PWM0 (CTR=0),
// усреднение ×4.
void ADC_FOC_Init(void);
void ADC_FOC_L_Mode(uint8_t enable);
void ADC_FOC_CaptureOffset(void);
extern volatile uint16_t g_adc_raw[4]; /* последний замер SEQ0 */
extern volatile uint16_t g_adc_off[4]; /* ноль с CaptureOffset */

typedef enum { STATE_NORMAL, STATE_OVERCURRENT_FAULT } fsm_state_t;

extern volatile fsm_state_t g_foc_state;
extern volatile uint16_t g_fault_flags; /* биты FAULT_* из main.c */

#endif /* INCLUDE_PWM_CONTROL_H_ */
