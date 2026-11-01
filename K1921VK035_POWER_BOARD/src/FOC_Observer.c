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
        Only foc_observer_update and foc_pll_run are kept. The VESC
        motor_all_state_t/mc_configuration types are replaced by the minimal
        TFLuxObserverMotor/TFocObserverCfg from FOC_Observer.h; the algorithm
   and its constants are unchanged.
 */

#include "FOC_Observer.h"
#include "VESC_Math.h"
#include <math.h>

// See
// http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
void foc_observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta, float dt,
                         observer_state *state, float *phase, TFLuxObserverMotor *motor)
{

    TFocObserverCfg *conf_now = motor->m_conf;

    float R = conf_now->foc_motor_r;
    float L = conf_now->foc_motor_l;
    float lambda = conf_now->foc_motor_flux_linkage;

    // Saturation compensation
    switch(conf_now->foc_sat_comp_mode) {
        case SAT_COMP_LAMBDA:
            // Here we assume that the inductance drops by the same amount as the flux
            // linkage. I have no idea if this is a valid or even a reasonable
            // assumption.
            if(conf_now->foc_observer_type >= FOC_OBSERVER_ORTEGA_LAMBDA_COMP ||
               conf_now->foc_observer_type >= FOC_OBSERVER_MXLEMMING_LAMBDA_COMP ||
               conf_now->foc_observer_type >= FOC_OBSERVER_MXV_LAMBDA_COMP ||
               conf_now->foc_observer_type >= FOC_OBSERVER_MXV_LAMBDA_COMP_LIN) {
                L = L * (state->lambda_est / lambda);
            }
            break;

        case SAT_COMP_FACTOR: {
            const float comp_fact = conf_now->foc_sat_comp *
                                    (motor->m_motor_state.i_abs_filter / conf_now->l_current_max);
            L -= L * comp_fact;
            lambda -= lambda * comp_fact;
        } break;

        case SAT_COMP_LAMBDA_AND_FACTOR: {
            if(conf_now->foc_observer_type >= FOC_OBSERVER_ORTEGA_LAMBDA_COMP ||
               conf_now->foc_observer_type >= FOC_OBSERVER_MXLEMMING_LAMBDA_COMP ||
               conf_now->foc_observer_type >= FOC_OBSERVER_MXV_LAMBDA_COMP ||
               conf_now->foc_observer_type >= FOC_OBSERVER_MXV_LAMBDA_COMP_LIN) {
                L = L * (state->lambda_est / lambda);
            }
            const float comp_fact = conf_now->foc_sat_comp *
                                    (motor->m_motor_state.i_abs_filter / conf_now->l_current_max);
            L -= L * comp_fact;
        } break;

        default:
            break;
    }

    // Temperature compensation
    if(conf_now->foc_temp_comp) {
        R = motor->m_res_temp_comp;
    }

    float ld_lq_diff = conf_now->foc_motor_ld_lq_diff;
    float id = motor->m_motor_state.id;
    float iq = motor->m_motor_state.iq;

    // Adjust inductance for saliency.
    if(fabsf(id) > 0.1 || fabsf(iq) > 0.1) {
        L = L - ld_lq_diff / 2.0 + ld_lq_diff * SQ(iq) / (SQ(id) + SQ(iq));
    }

    float L_ia = L * i_alpha;
    float L_ib = L * i_beta;
    const float R_ia = R * i_alpha;
    const float R_ib = R * i_beta;
    const float gamma_half = motor->m_gamma_now * 0.5;

    switch(conf_now->foc_observer_type) {
        case FOC_OBSERVER_ORTEGA_ORIGINAL: {
            float err = SQ(lambda) - (SQ(state->x1 - L_ia) + SQ(state->x2 - L_ib));

            // Forcing this term to stay negative helps convergence according to
            //
            // http://cas.ensmp.fr/Publications/Publications/Papers/ObserverPermanentMagnet.pdf
            // and
            // https://arxiv.org/pdf/1905.00833.pdf
            if(err > 0.0) {
                err = 0.0;
            }

            float x1_dot = v_alpha - R_ia + gamma_half * (state->x1 - L_ia) * err;
            float x2_dot = v_beta - R_ib + gamma_half * (state->x2 - L_ib) * err;

            state->x1 += x1_dot * dt;
            state->x2 += x2_dot * dt;
        } break;

        case FOC_OBSERVER_MXLEMMING:
        case FOC_OBSERVER_MXLEMMING_LAMBDA_COMP:
            // LICENCE NOTE:
            // This function deviates slightly from the BSD 3 clause licence.
            // The work here is entirely original to the MESC FOC project, and not based
            // on any appnotes, or borrowed from another project. This work is free to
            // use, as granted in BSD 3 clause, with the exception that this note must
            // be included in where this code is implemented/modified to use your
            // variable names, structures containing variables or other minor
            // rearrangements in place of the original names I have chosen, and credit
            // to David Molony as the original author must be noted.

            state->x1 += (v_alpha - R_ia) * dt - L * (i_alpha - state->i_alpha_last);
            state->x2 += (v_beta - R_ib) * dt - L * (i_beta - state->i_beta_last);

            state->theta_pre = utils_fast_atan2(state->x2, state->x1);

            if(conf_now->foc_observer_type == FOC_OBSERVER_MXLEMMING_LAMBDA_COMP) {
                float err = SQ(state->lambda_est) - (SQ(state->x1) + SQ(state->x2));
                state->lambda_est += 0.1 * gamma_half * state->lambda_est * -err * dt;
                utils_truncate_number(&(state->lambda_est), lambda * 0.3, lambda * 2.5);

                utils_truncate_number_abs(&(state->x1), state->lambda_est);
                utils_truncate_number_abs(&(state->x2), state->lambda_est);
            } else {
                utils_truncate_number_abs(&(state->x1), lambda);
                utils_truncate_number_abs(&(state->x2), lambda);
            }

            // Set these to 0 to allow using the same atan2-code as for Ortega
            L_ia = 0.0;
            L_ib = 0.0;
            break;

        case FOC_OBSERVER_ORTEGA_LAMBDA_COMP: {
            float err = SQ(state->lambda_est) - (SQ(state->x1 - L_ia) + SQ(state->x2 - L_ib));

            // FLux linkage observer. See:
            // https://cas.mines-paristech.fr/~praly/Telechargement/Conferences/2017_IFAC_Bernard-Praly.pdf
            state->lambda_est += 0.2 * gamma_half * state->lambda_est * -err * dt;

            // Clamp the observed flux linkage (not sure if this is needed)
            utils_truncate_number(&(state->lambda_est), lambda * 0.3, lambda * 2.5);

            if(err > 0.0) {
                err = 0.0;
            }

            float x1_dot = v_alpha - R_ia + gamma_half * (state->x1 - L_ia) * err;
            float x2_dot = v_beta - R_ib + gamma_half * (state->x2 - L_ib) * err;

            state->x1 += x1_dot * dt;
            state->x2 += x2_dot * dt;
        } break;

        case FOC_OBSERVER_MXV:
        case FOC_OBSERVER_MXV_LAMBDA_COMP:
        case FOC_OBSERVER_MXV_LAMBDA_COMP_LIN:
            state->x1 += (v_alpha - R_ia) * dt;
            state->x2 += (v_beta - R_ib) * dt;

            if(conf_now->foc_observer_type == FOC_OBSERVER_MXV_LAMBDA_COMP ||
               conf_now->foc_observer_type == FOC_OBSERVER_MXV_LAMBDA_COMP_LIN) {
                if(conf_now->foc_observer_type == FOC_OBSERVER_MXV_LAMBDA_COMP_LIN) {
                    float mag = NORM2_f(state->x1 - L_ia, state->x2 - L_ib);
                    UTILS_LP_FAST(state->lambda_est, mag,
                                  0.1 * gamma_half * dt * SQ(state->lambda_est));
                    utils_truncate_number(&(state->lambda_est), lambda * 0.3, lambda * 2.5);

                    if(mag > state->lambda_est) {
                        state->x1 = (state->x1 / mag) * state->lambda_est;
                        state->x2 = (state->x2 / mag) * state->lambda_est;
                    }
                } else if(conf_now->foc_observer_type == FOC_OBSERVER_MXV_LAMBDA_COMP) {
                    float err =
                        SQ(state->lambda_est) - (SQ(state->x1 - L_ia) + SQ(state->x2 - L_ib));
                    state->lambda_est += 0.2 * gamma_half * state->lambda_est * -err * dt;
                    utils_truncate_number(&(state->lambda_est), lambda * 0.3, lambda * 2.5);

                    float mag = NORM2_f(state->x1 - L_ia, state->x2 - L_ib);
                    if(mag > state->lambda_est) {
                        state->x1 = (state->x1 / mag) * state->lambda_est;
                        state->x2 = (state->x2 / mag) * state->lambda_est;
                    }
                }
            } else {
                float mag = NORM2_f(state->x1 - L_ia, state->x2 - L_ib);
                if(mag > lambda) {
                    state->x1 = (state->x1 / mag) * lambda;
                    state->x2 = (state->x2 / mag) * lambda;
                }
            }
            break;

        default:
            break;
    }

    state->i_alpha_last = i_alpha;
    state->i_beta_last = i_beta;

    UTILS_NAN_ZERO(state->x1);
    UTILS_NAN_ZERO(state->x2);

    // Prevent the magnitude from getting too low, as that makes the angle very
    // unstable.
    float mag = NORM2_f(state->x1, state->x2);
    if(mag < (lambda * 0.5)) {
        state->x1 *= 1.1;
        state->x2 *= 1.1;
    }

    if(phase) {
        *phase = utils_fast_atan2(state->x2 - L_ib, state->x1 - L_ia);
    }

    // Can we clamp the flux in dq with q flux = 0 and d flux is lambda
    // Then the state->x1 and state->x2 (which are the alpha and beta fluxes) are
    // set as lambda*sin and lambda*cos The d flux each time would have a residual
    // after transform from ab to dq. This can be used as an input to the flux
    // estimator
}

void foc_pll_run(float phase, float dt, float *phase_var, float *speed_var, TFocObserverCfg *conf)
{
    UTILS_NAN_ZERO(*phase_var);
    float delta_theta = phase - *phase_var;
    utils_norm_angle_rad(&delta_theta);
    UTILS_NAN_ZERO(*speed_var);
    *phase_var += (*speed_var + conf->foc_pll_kp * delta_theta) * dt;
    utils_norm_angle_rad((float *)phase_var);
    *speed_var += conf->foc_pll_ki * delta_theta * dt;
}
