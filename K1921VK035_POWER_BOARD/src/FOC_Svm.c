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

#include "FOC_Svm.h"
#include "VESC_Math.h"

void foc_svm(float alpha, float beta, float max_mod, uint32_t PWMFullDutyCycle, uint32_t *tAout,
             uint32_t *tBout, uint32_t *tCout, uint32_t *svm_sector)
{
    uint32_t sector;

    if(beta >= 0.0f) {
        if(alpha >= 0.0f) {
            // quadrant I
            if(ONE_BY_SQRT3 * beta > alpha) {
                sector = 2;
            } else {
                sector = 1;
            }
        } else {
            // quadrant II
            if(-ONE_BY_SQRT3 * beta > alpha) {
                sector = 3;
            } else {
                sector = 2;
            }
        }
    } else {
        if(alpha >= 0.0f) {
            // quadrant IV5
            if(-ONE_BY_SQRT3 * beta > alpha) {
                sector = 5;
            } else {
                sector = 6;
            }
        } else {
            // quadrant III
            if(ONE_BY_SQRT3 * beta > alpha) {
                sector = 4;
            } else {
                sector = 5;
            }
        }
    }

    // PWM timings
    int tA, tB, tC;

    switch(sector) {

        // sector 1-2
        case 1: {
            // Vector on-times
            int t1 = (alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
            int t2 = (TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;

            // PWM timings
            tA = (PWMFullDutyCycle + t1 + t2) / 2;
            tB = tA - t1;
            tC = tB - t2;

            break;
        }

        // sector 2-3
        case 2: {
            // Vector on-times
            int t2 = (alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
            int t3 = (-alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

            // PWM timings
            tB = (PWMFullDutyCycle + t2 + t3) / 2;
            tA = tB - t3;
            tC = tA - t2;

            break;
        }

        // sector 3-4
        case 3: {
            // Vector on-times
            int t3 = (TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
            int t4 = (-alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

            // PWM timings
            tB = (PWMFullDutyCycle + t3 + t4) / 2;
            tC = tB - t3;
            tA = tC - t4;

            break;
        }

        // sector 4-5
        case 4: {
            // Vector on-times
            int t4 = (-alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
            int t5 = (-TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;

            // PWM timings
            tC = (PWMFullDutyCycle + t4 + t5) / 2;
            tB = tC - t5;
            tA = tB - t4;

            break;
        }

        // sector 5-6
        case 5: {
            // Vector on-times
            int t5 = (-alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;
            int t6 = (alpha - ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

            // PWM timings
            tC = (PWMFullDutyCycle + t5 + t6) / 2;
            tA = tC - t5;
            tB = tA - t6;

            break;
        }

        // sector 6-1
        case 6: {
            // Vector on-times
            int t6 = (-TWO_BY_SQRT3 * beta) * PWMFullDutyCycle;
            int t1 = (alpha + ONE_BY_SQRT3 * beta) * PWMFullDutyCycle;

            // PWM timings
            tA = (PWMFullDutyCycle + t6 + t1) / 2;
            tC = tA - t1;
            tB = tC - t6;

            break;
        }
    }

    int t_max = PWMFullDutyCycle * (1.0 - (1.0 - max_mod) * 0.5);
    utils_truncate_number_int(&tA, 0, t_max);
    utils_truncate_number_int(&tB, 0, t_max);
    utils_truncate_number_int(&tC, 0, t_max);

    *tAout = tA;
    *tBout = tB;
    *tCout = tC;
    *svm_sector = sector;
}
