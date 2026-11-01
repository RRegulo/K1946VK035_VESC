#include <assert.h>
#include "FOC_Protection.h"

int main(void)
{
    foc_protection_t state = {0};
    uint16_t raw[4] = {2048, 2048, 2048, 1000};
    const uint16_t off[4] = {2048, 2048, 2048, 0};

    assert(foc_protection_check(&state, raw, off, 1) == 0);
    /* Current checks are deliberately disabled in this build. */
    raw[1] = 2400;
    assert(!(foc_protection_check(&state, raw, off, 1) & FAULT_SW_OC));
    raw[1] = 0;
    assert(!(foc_protection_check(&state, raw, off, 1) & FAULT_ADC_SAT));
    assert(!(foc_protection_check(&state, raw, off, 0) & FAULT_ADC_SAT));
    raw[1] = 2048;
    raw[3] = 2000;
    for(int i = 0; i < 3; ++i)
        assert(!(foc_protection_check(&state, raw, off, 0) & FAULT_OV));
    assert(foc_protection_check(&state, raw, off, 0) & FAULT_OV);
    raw[3] = 500;
    for(int i = 0; i < 31; ++i)
        assert(!(foc_protection_check(&state, raw, off, 1) & FAULT_UV));
    assert(foc_protection_check(&state, raw, off, 1) & FAULT_UV);
    return 0;
}
