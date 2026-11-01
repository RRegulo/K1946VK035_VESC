/*
 * foc_stub.h
 *
 * Регистры PWM в памяти для host-тестов FOC.
 */

#ifndef TESTS_FOC_STUB_H_
#define TESTS_FOC_STUB_H_

#include <stdint.h>

typedef struct {
    struct {
        uint32_t CMPA;
    } CMPA_bit;

    uint32_t TBCTR;
} foc_pwm_stub_t;

extern foc_pwm_stub_t pwm0_stub;
extern foc_pwm_stub_t pwm1_stub;
extern foc_pwm_stub_t pwm2_stub;

#define PWM0 (&pwm0_stub)
#define PWM1 (&pwm1_stub)
#define PWM2 (&pwm2_stub)

#define DINT ((void)0)
#define EINT ((void)0)

#endif /* TESTS_FOC_STUB_H_ */
