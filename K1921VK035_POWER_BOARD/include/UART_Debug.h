/*
 * UART_Debug.h
 *
 * Управление и телеметрия по UART1, 115200 8N1.
 */

#ifndef INCLUDE_UART_DEBUG_H_
#define INCLUDE_UART_DEBUG_H_

#ifndef FOC_HOST_TEST
#include "K1921VK035.h"
#endif
#include "PWM_Control.h" /* g_adc_raw/g_adc_off, g_foc_state, fsm_state_t */

/* Инициализация UART1: 115200 бод, 8N1. */
void uart_debug_init(void);

/* приём поллингом (как демо CANtoRS_Receive), вызывается из main loop.
 * Потоковой телеметрии нет: UART молчит, пока не спросят ('s','t','d','?'). */
void uart_debug_poll(void);
extern volatile uint8_t g_manual_pwm_active;

#endif /* INCLUDE_UART_DEBUG_H_ */
