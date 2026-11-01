/*
        Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se

        This file is part of the VESC firmware.

        The VESC firmware is free software: you can redistribute it and/or
   modify it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

        Ported from VESC (https://github.com/vedderb/bldc) - motor/foc_math.c.
 */

#ifndef FOC_SVM_H_
#define FOC_SVM_H_

#include <stdint.h>

/**
 * @brief svm Space vector modulation. Magnitude must not be larger than
 * sqrt(3)/2, or 0.866 to avoid overmodulation. See
 * https://github.com/vedderb/bldc/pull/372#issuecomment-962499623 for a full
 * description.
 * @param alpha voltage
 * @param beta Park transformed and normalized voltage
 * @param PWMFullDutyCycle is the peak value of the PWM counter.
 * @param tAout PWM duty cycle phase A (0 = off all of the time,
 * PWMFullDutyCycle = on all of the time)
 * @param tBout PWM duty cycle phase B
 * @param tCout PWM duty cycle phase C
 */
void foc_svm(float alpha, float beta, float max_mod, uint32_t PWMFullDutyCycle, uint32_t *tAout,
             uint32_t *tBout, uint32_t *tCout, uint32_t *svm_sector);

#endif /* FOC_SVM_H_ */
