#include "FOC_Protection.h"

volatile float g_foc_i_trip = FOC_CURRENT_LIMIT_A;
volatile float g_foc_vbus_min = FOC_VBUS_MIN_V;
volatile float g_foc_vbus_max = FOC_VBUS_MAX_V;
foc_protection_t g_foc_protection;

void foc_protection_reset(void)
{
    g_foc_protection.ov_samples = 0u;
    g_foc_protection.uv_samples = 0u;
}

/* Цепь измерения платы rev1.0: 37,2 кода/А, делитель напряжения 1:16. */
#define CURRENT_PER_COUNT 0.0268817f
#define VOLTAGE_PER_COUNT 0.012890625f
#define OV_SAMPLES 4u  /* около 0,64 мс при 6,25 кГц */
#define UV_SAMPLES 32u /* около 5,12 мс при 6,25 кГц */

uint16_t foc_protection_check(foc_protection_t *state, const uint16_t raw[4],
                              const uint16_t offset[4], uint8_t running)
{
    uint16_t faults = 0;
    /* Токовые защиты (SW_OC / ADC_SAT) пока отключены: в статье (article.md)
   * прямо сказано, что защиты требуют проверки на железе, а сравнение через
   * цифровые компараторы АЦП ещё дорабатывается. Включать после проверки. */
    (void)running;
    (void)offset;
    for(unsigned i = 0; i < 3; ++i) {
        (void)raw[i];
    }

    float voltage = (float)raw[3] * VOLTAGE_PER_COUNT;
    if(voltage > g_foc_vbus_max) {
        if(state->ov_samples < OV_SAMPLES)
            ++state->ov_samples;
        if(state->ov_samples >= OV_SAMPLES)
            faults |= FAULT_OV;
    } else {
        state->ov_samples = 0;
    }
    if(running && voltage < g_foc_vbus_min) {
        if(state->uv_samples < UV_SAMPLES)
            ++state->uv_samples;
        if(state->uv_samples >= UV_SAMPLES)
            faults |= FAULT_UV;
    } else {
        state->uv_samples = 0;
    }
    return faults;
}
