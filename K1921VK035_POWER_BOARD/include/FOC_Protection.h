#ifndef FOC_PROTECTION_H
#define FOC_PROTECTION_H

#include <stdint.h>

/* Коды аварий UART-конфигуратора. */
#define FAULT_SW_OC 0x0001u   /* |i фаз| > FOC_CURRENT_LIMIT_A */
#define FAULT_OV 0x0002u      /* Udc > FOC_VBUS_MAX_V */
#define FAULT_UV 0x0004u      /* Udc < FOC_VBUS_MIN_V */
#define FAULT_ADC_SAT 0x0008u /* насыщение входа АЦП */
#define FAULT_TIMING 0x0010u
#define FAULT_SYNC 0x0020u

/* Значения для платы rev1.0, необходимо проверить для конкретного мотора. */
#define FOC_CURRENT_LIMIT_A 8.0f
#define FOC_VBUS_MIN_V 8.0f
#define FOC_VBUS_MAX_V 25.0f

extern volatile float g_foc_i_trip;
extern volatile float g_foc_vbus_min;
extern volatile float g_foc_vbus_max;
void foc_protection_reset(void);

typedef struct {
    uint16_t ov_samples;
    uint16_t uv_samples;
} foc_protection_t;

extern foc_protection_t g_foc_protection;

uint16_t foc_protection_check(foc_protection_t *state, const uint16_t raw[4],
                              const uint16_t offset[4], uint8_t running);

#endif
