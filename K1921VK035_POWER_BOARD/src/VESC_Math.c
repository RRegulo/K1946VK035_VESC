/*
        Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

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

        Ported from VESC (https://github.com/vedderb/bldc) - util/utils_math.c.
        Only the helpers used by this project are kept.
 */

#include "VESC_Math.h"

/**
 * Fast atan2 approximation.
 *
 * @param y
 * y
 *
 * @param x
 * x
 *
 * @return
 * The angle in radians
 */
float utils_fast_atan2(float y, float x)
{
    float abs_y = fabsf(y) + 1e-20; // kludge to prevent 0/0 condition
    float angle;

    if(x >= 0) {
        float r = (x - abs_y) / (x + abs_y);
        float rsq = r * r;
        angle = ((0.1963 * rsq) - 0.9817) * r + (M_PI / 4.0);
    } else {
        float r = (x + abs_y) / (abs_y - x);
        float rsq = r * r;
        angle = ((0.1963 * rsq) - 0.9817) * r + (3.0 * M_PI / 4.0);
    }

    UTILS_NAN_ZERO(angle);

    if(y < 0) {
        return (-angle);
    } else {
        return (angle);
    }
}

float utils_fast_sin(float angle)
{
    while(angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    while(angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    float res = 0.0;

    if(angle < 0.0) {
        res = 1.27323954 * angle + 0.405284735 * angle * angle;
    } else {
        res = 1.27323954 * angle - 0.405284735 * angle * angle;
    }

    return res;
}

float utils_fast_cos(float angle)
{
    angle += 0.5 * M_PI;

    while(angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    while(angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    float res = 0.0;

    if(angle < 0.0) {
        res = 1.27323954 * angle + 0.405284735 * angle * angle;
    } else {
        res = 1.27323954 * angle - 0.405284735 * angle * angle;
    }

    return res;
}

/**
 * Fast sine and cosine implementation.
 *
 * See http://lab.polygonal.de/?p=205
 *
 * @param angle
 * The angle in radians
 * WARNING: Don't use too large angles.
 *
 * @param sin
 * A pointer to store the sine value.
 *
 * @param cos
 * A pointer to store the cosine value.
 */
void utils_fast_sincos(float angle, float *sin, float *cos)
{
    // always wrap input angle to -PI..PI
    while(angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    while(angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    // compute sine
    if(angle < 0.0) {
        *sin = 1.27323954 * angle + 0.405284735 * angle * angle;
    } else {
        *sin = 1.27323954 * angle - 0.405284735 * angle * angle;
    }

    // compute cosine: sin(x + PI/2) = cos(x)
    angle += 0.5 * M_PI;

    if(angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    if(angle < 0.0) {
        *cos = 1.27323954 * angle + 0.405284735 * angle * angle;
    } else {
        *cos = 1.27323954 * angle - 0.405284735 * angle * angle;
    }
}

/**
 * Fast sine and cosine implementation.
 *
 * See http://lab.polygonal.de/?p=205
 *
 * @param angle
 * The angle in radians
 * WARNING: Don't use too large angles.
 *
 * @param sin
 * A pointer to store the sine value.
 *
 * @param cos
 * A pointer to store the cosine value.
 */
void utils_fast_sincos_better(float angle, float *sin, float *cos)
{
    // always wrap input angle to -PI..PI
    while(angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    while(angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    // compute sine
    if(angle < 0.0) {
        *sin = 1.27323954 * angle + 0.405284735 * angle * angle;

        if(*sin < 0.0) {
            *sin = 0.225 * (*sin * -*sin - *sin) + *sin;
        } else {
            *sin = 0.225 * (*sin * *sin - *sin) + *sin;
        }
    } else {
        *sin = 1.27323954 * angle - 0.405284735 * angle * angle;

        if(*sin < 0.0) {
            *sin = 0.225 * (*sin * -*sin - *sin) + *sin;
        } else {
            *sin = 0.225 * (*sin * *sin - *sin) + *sin;
        }
    }

    // compute cosine: sin(x + PI/2) = cos(x)
    angle += 0.5 * M_PI;
    if(angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    if(angle < 0.0) {
        *cos = 1.27323954 * angle + 0.405284735 * angle * angle;

        if(*cos < 0.0) {
            *cos = 0.225 * (*cos * -*cos - *cos) + *cos;
        } else {
            *cos = 0.225 * (*cos * *cos - *cos) + *cos;
        }
    } else {
        *cos = 1.27323954 * angle - 0.405284735 * angle * angle;

        if(*cos < 0.0) {
            *cos = 0.225 * (*cos * -*cos - *cos) + *cos;
        } else {
            *cos = 0.225 * (*cos * *cos - *cos) + *cos;
        }
    }
}
