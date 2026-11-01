/******************************************************************************
 * @file        test_foc.c
 * @brief       Хост-тест FOC-ядра (float). Компилируется хостовым gcc вместе
 *              с РЕАЛЬНЫМИ исходниками (FOC_Math.c, FOC_Core.c, FOC_Observer.c,
 *              FOC_Svm.c, VESC_Math.c) при -DFOC_HOST_TEST. Эталон — double.
 *              Плюс портированные тесты VESC util/utils_math (см. их
 *              tests/utils_math/unittest.cpp). Сборка/прогон:
 *              gcc -DFOC_HOST_TEST -I K1921VK035_POWER_BOARD/include -I tests \
 *                tests/test_foc.c K1921VK035_POWER_BOARD/src/FOC_*.c \
 *                -lm -o /tmp/test_foc && /tmp/test_foc
 ******************************************************************************/

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "PWM_Control.h"
#include "FOC_Params.h"
#include "FOC_Proto.h"
#include "FOC_Detect.h"
#include "FOC_Protection.h"
volatile uint32_t g_tick_100us = 0;
volatile fsm_state_t g_foc_state = STATE_NORMAL;
volatile uint16_t g_fault_flags = 0;
volatile uint8_t g_manual_pwm_active = 0;
static uint8_t l_adc_mode = 0;

void ADC_FOC_L_Mode(uint8_t enable)
{
    l_adc_mode = enable;
}

#include "foc_stub.h"

#include "FOC_Control.h"
#include "FOC_Core.h"
#include "FOC_Math.h"
#include "FOC_Observer.h"
#include "VESC_Math.h"

/* Глобалы прошивки, нужные FOC_Core (на цели живут в PWM_Control.c) */
volatile uint16_t g_adc_raw[4] = {0, 0, 0, 0};
volatile uint16_t g_adc_off[4] = {2048, 2048, 2048, 2048};

/* Стабы PWM-регистров (см. foc_stub.h) */
foc_pwm_stub_t pwm0_stub = {0};
foc_pwm_stub_t pwm1_stub = {0};
foc_pwm_stub_t pwm2_stub = {0};

static int n_fail = 0;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if(!(cond)) {                                                                              \
            printf("FAIL %d: %s\n", __LINE__, #cond);                                              \
            n_fail++;                                                                              \
        }                                                                                          \
    } while(0)

#define CHECK_CLOSE_F(got, want, tol)                                                              \
    do {                                                                                           \
        double _g = (got), _w = (want);                                                            \
        double _d = fabs(_g - _w);                                                                 \
        if(_d > (tol)) {                                                                           \
            printf("FAIL %d: got %.6f want %.6f d=%.6f\n", __LINE__, _g, _w, _d);                  \
            n_fail++;                                                                              \
        }                                                                                          \
    } while(0)

int main(void)
{
    const double PI = 3.14159265358979;

    /* 1. utils_fast_sincos_better: точность vs sin/cos на -pi..pi */
    {
        double maxerr = 0;
        for(int i = 0; i <= 1024; i++) {
            double x = -PI + 2.0 * PI * (double)i / 1024.0;
            float s, c;
            utils_fast_sincos_better((float)x, &s, &c);
            double es = fabs(s - sin(x));
            double ec = fabs(c - cos(x));
            if(es > maxerr)
                maxerr = es;
            if(ec > maxerr)
                maxerr = ec;
        }
        printf("sincos_better maxerr=%.5f\n", maxerr);
        CHECK(maxerr < 0.005);
    }

    /* 2. utils_fast_sin/cos */
    {
        double maxerr = 0;
        for(int i = 0; i <= 512; i++) {
            double x = -PI + 2.0 * PI * (double)i / 512.0;
            double es = fabs(utils_fast_sin((float)x) - sin(x));
            double ec = fabs(utils_fast_cos((float)x) - cos(x));
            if(es > maxerr)
                maxerr = es;
            if(ec > maxerr)
                maxerr = ec;
        }
        printf("fast_sin/cos maxerr=%.5f\n", maxerr);
        CHECK(maxerr < 0.06); /* VESC fast_sin без кубической коррекции */
    }

    /* 3. clarke: сбалансированная тройка ia=1, ib=-0.5 -> ds=1, qs=0 */
    {
        TClarke c = CLARKE_DEFAULTS;
        c.as = 1.0f;
        c.bs = -0.5f;
        clarke_calc(&c);
        CHECK_CLOSE_F(c.ds, 1.0, 1e-5);
        CHECK_CLOSE_F(c.qs, 0.0, 1e-5);
    }

    /* 4. park->ipark roundtrip */
    {
        double maxerr = 0;
        for(int i = 0; i < 64; i++) {
            double ds = (double)(i - 32) / 64.0;
            double qs = (double)(32 - i) / 48.0;
            double ang = -PI + 2.0 * PI * (double)i / 64.0;
            TPark p = PARK_DEFAULTS;
            p.ds = (float)ds;
            p.qs = (float)qs;
            p.ang = (float)ang;
            park_calc(&p);
            TIPark ip = IPARK_DEFAULTS;
            ip.de = p.de;
            ip.qe = p.qe;
            ip.ang = p.ang;
            ipark_calc(&ip);
            double eds = fabs(ip.ds - ds);
            double eqs = fabs(ip.qs - qs);
            if(eds > maxerr)
                maxerr = eds;
            if(eqs > maxerr)
                maxerr = eqs;
        }
        printf("park/ipark roundtrip maxerr=%.6f\n", maxerr);
        CHECK(maxerr < 0.005);
    }

    /* 5. pid: P-only, насыщение */
    {
        TPidReg3 r;
        pid_reg3_reset(&r);
        r.Kp_reg3 = 2.0f;
        r.Ki_reg3 = 0.0f;
        r.Kd_reg3 = 0.0f;
        r.Kc_reg3 = 0.0f;
        r.pid_out_max = 1.0f;
        r.pid_out_min = -1.0f;
        r.DeadZone = 0.0f;
        r.Kf_d = 0.0f;
        r.DiffDelim = 1;
        r.DiffCounter = 0;
        r.pid_ref_reg3 = 0.3f;
        r.pid_fdb_reg3 = 0.1f;
        pid_reg3_calc(&r);
        CHECK_CLOSE_F(r.pid_out_reg3, 0.4, 1e-5);
        r.pid_ref_reg3 = 10.0f;
        pid_reg3_calc(&r);
        CHECK_CLOSE_F(r.pid_out_reg3, 1.0, 1e-5); /* clamp по max */
    }

    /* 6. фильтр: ступенька 1.0, T=0.5 -> сходимость */
    {
        TFilter f = FILTER_DEFAULTS;
        f.T = 0.5f;
        f.input = 1.0f;
        for(int i = 0; i < 40; i++)
            TFilter_Calc(&f);
        CHECK_CLOSE_F(f.output, 1.0, 1e-3);
    }

    /* 7. utils_fast_atan2 vs atan2f по квадрантам */
    {
        double maxerr = 0;
        for(int i = 0; i < 360; i++) {
            double a = (double)i * PI / 180.0;
            double y = sin(a), x = cos(a);
            double got = utils_fast_atan2((float)y, (float)x);
            double want = atan2(y, x);
            double d = fabs(got - want);
            if(d > PI)
                d = 2.0 * PI - d;
            if(d > maxerr)
                maxerr = d;
        }
        printf("fast_atan2 maxerr=%.5f rad\n", maxerr);
        CHECK(maxerr < 0.02);
    }

    /* 8. NORM2_f + utils_saturate_vector_2d */
    {
        CHECK_CLOSE_F(NORM2_f(3.0f, 4.0f), 5.0, 1e-4);
        float x = 2.0f, y = 3.0f;
        CHECK(utils_saturate_vector_2d(&x, &y, 100.0f) == false);
        CHECK_CLOSE_F(x, 2.0, 1e-5);
        CHECK_CLOSE_F(y, 3.0, 1e-5);
    }

    /* 9. Портированные тесты VESC util/utils_math (unittest.cpp).
   *    utils_norm_angle: 0..360; utils_norm_angle_rad: -pi..pi. */
    {
        float a = -370.0f;
        utils_norm_angle(&a);
        CHECK_CLOSE_F(a, 350.0, 1e-4);
        a = 1079.0f;
        utils_norm_angle(&a);
        CHECK_CLOSE_F(a, 359.0, 1e-4);
        a = 90.0f;
        utils_norm_angle(&a);
        CHECK_CLOSE_F(a, 90.0, 1e-4);

        float r = (float)M_PI;
        utils_norm_angle_rad(&r);
        CHECK_CLOSE_F(r, -PI, 1e-6);
        r = (float)(3.5 * M_PI);
        utils_norm_angle_rad(&r);
        CHECK_CLOSE_F(r, -0.5 * PI, 1e-6);
        r = (float)(-3.5 * M_PI);
        utils_norm_angle_rad(&r);
        CHECK_CLOSE_F(r, 0.5 * PI, 1e-6);
        r = 0.5f;
        utils_norm_angle_rad(&r);
        CHECK_CLOSE_F(r, 0.5, 1e-6);

        float sx = 10.0f, sy = 10.0f;
        CHECK(utils_saturate_vector_2d(&sx, &sy, 10.0f) == true);
        CHECK_CLOSE_F(sx, 10.0 / sqrt(2.0), 1e-4);
        CHECK_CLOSE_F(sy, 10.0 / sqrt(2.0), 1e-4);
        sx = 0.0f;
        sy = 0.0f;
        CHECK(utils_saturate_vector_2d(&sx, &sy, 10.0f) == false);
    }

    /* 10. SVPWM через реальный foc_svpwm_update: границы + запись в CMPA */
    {
        foc_init();
        g_foc_enable = 1;
        pwm0_stub.TBCTR = 100;
        g_pwm.UalphaRef = 0.3f;
        g_pwm.UbetaRef = 0.1f;
        foc_svpwm_update(&g_pwm);
        printf("svpwm: sector=%d Ga=%u Gb=%u Gc=%u\n", (int)g_pwm.sector, (unsigned)g_pwm.GammaA,
               (unsigned)g_pwm.GammaB, (unsigned)g_pwm.GammaC);
        CHECK(g_pwm.sector >= 0 && g_pwm.sector <= 5);
        CHECK(g_pwm.GammaA <= 2000u);
        CHECK(g_pwm.GammaB <= 2000u);
        CHECK(g_pwm.GammaC <= 2000u);
        /* плата rev1.0 — прямое соответствие: PWM0<-Ga, PWM1<-Gb,
     * PWM2<-Gc, CMPA = k_pwm - t_on (AQ инвертирован, /). */
        CHECK(pwm0_stub.CMPA_bit.CMPA == (uint32_t)g_pwm.GammaA);
        CHECK(pwm1_stub.CMPA_bit.CMPA == (uint32_t)g_pwm.GammaB);
        CHECK(pwm2_stub.CMPA_bit.CMPA == (uint32_t)g_pwm.GammaC);
    }

    /* 11. SVPWM ноль: tA=tB=tC=k_pwm/2 -> Gamma = k_pwm - 1000 = 1000 */
    {
        g_pwm.UalphaRef = 0.0f;
        g_pwm.UbetaRef = 0.0f;
        foc_svpwm_update(&g_pwm);
        CHECK(g_pwm.GammaA == 1000u);
        CHECK(g_pwm.GammaB == 1000u);
        CHECK(g_pwm.GammaC == 1000u);
    }

    /* 12. fast_calc целиком на нулевых токах: выходы PID ~0, Udc ~17.1 V */
    {
        g_foc_inv_i = 0;
        g_foc_adc_map = 0;
        g_adc_raw[0] = 2048;
        g_adc_raw[1] = 2048;
        g_adc_raw[2] = 2048;
        g_adc_raw[3] = 1327;
        g_adc_off[0] = 2048;
        g_adc_off[1] = 2048;
        g_adc_off[2] = 2048;
        g_foc_enable = 0;
        foc_fast_calc();
        printf("udc=%f V\n", g_foc_udc);
        CHECK_CLOSE_F(g_foc_udc, 17.106, 0.01);
    }

    /* 13. рампа V_RMP_CTRL: step=Ts/T, выход догоняет вход */
    {
        TRMPCtrl r = RMP_CTRL_DEFAULTS;
        rmp_ctrl_slow_calc(&r);
        CHECK_CLOSE_F(r.step, (1.0f / 6250.0f) / 2.0f, 1e-9);
        r.input = 0.2f;
        for(int i = 0; i < 2500; i++)
            rmp_ctrl_calc(&r);
        CHECK_CLOSE_F(r.output, 0.2, 1e-3);
        /* T=0: мгновенно */
        r.T = 0.0f;
        r.input = 0.5f;
        rmp_ctrl_calc(&r);
        CHECK_CLOSE_F(r.output, 0.5, 1e-6);
    }

    /* 14. I/F: старт -> тета крутится, рампа выходит на задание */
    {
        g_foc_inv_i = 0;
        g_foc_adc_map = 0;
        g_adc_raw[0] = 2048;
        g_adc_raw[1] = 2048;
        g_adc_raw[2] = 2048;
        g_adc_raw[3] = 1327;
        g_adc_off[0] = 2048;
        g_adc_off[1] = 2048;
        g_adc_off[2] = 2048;
        foc_init();
        g_foc_enable = 0;
        foc_if_start();
        CHECK(g_if_active == 1);
        CHECK(g_foc_enable == 1);
        CHECK_CLOSE_F(g_ramp.output, 0.0, 1e-6);
        float th0 = g_foc_angle;
        for(int i = 0; i < 3000; i++) {
            pwm0_stub.TBCTR = 100;
            foc_fast_calc();
        }
        printf("if: th0=%f th1=%f ramp=%f speed_ref=%f\n", th0, g_foc_angle, g_ramp.output,
               g_if_speed_ref);
        CHECK_CLOSE_F(g_ramp.output, g_if_speed_ref, 1e-3);
        CHECK(fabsf(g_foc_angle - th0) > 0.01f); /* тета ушла вперёд */
        CHECK_CLOSE_F(g_foc_id_ref, g_if_flux_ref, 1e-4);
        CHECK_CLOSE_F(g_foc_iq_ref, 0.0, 1e-6);
        CHECK(fabsf(g_foc_angle) <= (float)PI + 1e-3f); /* нормализация -pi..pi */
    }

    /* 14a. ток растёт с рампой, не ступенькой */
    {
        g_foc_inv_i = 0;
        g_foc_adc_map = 0;
        foc_init();
        g_foc_enable = 0;
        foc_if_start();
        for(int i = 0; i < 100; i++)
            foc_fast_calc();
        printf("if-ramp: id_ref=%f flux=%f ramp=%f\n", g_foc_id_ref, g_if_flux_ref, g_ramp.output);
        CHECK(g_foc_id_ref > 0.0f);
        CHECK(g_foc_id_ref < g_if_flux_ref); /* середина рампы — частичный ток */
        foc_if_stop();
    }

    /* 15. I/F стоп: ноль-вектор в CMPA, enable=0 */
    {
        foc_if_stop();
        CHECK(g_if_active == 0);
        CHECK(g_foc_enable == 0);
        CHECK(pwm0_stub.CMPA_bit.CMPA == 2000u);
        CHECK(pwm1_stub.CMPA_bit.CMPA == 2000u);
        CHECK(pwm2_stub.CMPA_bit.CMPA == 2000u);
    }

    /* 16. Observer init: R/L/lambda из g_motor */
    {
        g_motor.R_mOhm = 450;
        g_motor.L_uH = 300;
        g_motor.lambda_uWb = 4300;
        foc_init();
        CHECK_CLOSE_F(g_observer.R, 0.45, 1e-6);
        CHECK_CLOSE_F(g_observer.L, 300e-6, 1e-9);
        CHECK_CLOSE_F(g_observer.lambda, 4300e-6, 1e-9);
        CHECK_CLOSE_F(g_observer.cfg.foc_motor_r, 0.45, 1e-6);
        CHECK_CLOSE_F(g_observer.cfg.foc_motor_flux_linkage, 4300e-6, 1e-9);
        printf("obs: R=%f L=%f lambda=%f\n", g_observer.R, g_observer.L, g_observer.lambda);
    }

    /* 17. VESC observer (MXLEMMING_LAMBDA_COMP): theta_i трекает угол ротора.
   * Модель: ротор 70 Гц эл., ток со сдвигом 30°, v = R*i + L*di/dt + back-EMF.
   * m_gamma_now=0 — отключаем адаптацию lambda_est, проверяем голую оценку. */
    {
        g_motor.R_mOhm = 450;
        g_motor.L_uH = 300;
        g_motor.lambda_uWb = 5000;
        g_observer = (TFluxObserver){0};
        flux_observer_slow_calc(&g_observer);
        g_foc_obs_off = 0; /* голый наблюдатель, без -вычета */
        g_observer.cfg.foc_observer_type = FOC_OBSERVER_MXLEMMING_LAMBDA_COMP;
        g_observer.cfg.foc_sat_comp_mode = SAT_COMP_DISABLED;
        g_observer.cfg.foc_temp_comp = false;
        g_observer.motor.m_conf = &g_observer.cfg;
        g_observer.motor.m_gamma_now = 0.0f;
        g_observer.motor.m_res_temp_comp = 1.0f;
        double om = 2.0 * PI * 70.0;
        double th = 0.1;
        double max_err_deg = 0;
        for(int i = 0; i < 6250; i++) {
            th += om * 1.6e-4;
            double i_a = 0.5 * cos(th + 0.5);
            double i_b = 0.5 * sin(th + 0.5);
            double di_a = -om * 0.5 * sin(th + 0.5);
            double di_b = om * 0.5 * cos(th + 0.5);
            double e_a = -om * 0.005 * sin(th);
            double e_b = om * 0.005 * cos(th);
            double v_a = 0.45 * i_a + 0.0003 * di_a + e_a;
            double v_b = 0.45 * i_b + 0.0003 * di_b + e_b;
            foc_observer_update((float)v_a, (float)v_b, (float)i_a, (float)i_b, 1.6e-4f,
                                &g_observer.state, &g_observer.theta_i, &g_observer.motor);
            if(i > 2000) {
                double want = th;
                while(want < -PI)
                    want += 2.0 * PI;
                while(want >= PI)
                    want -= 2.0 * PI;
                double err = fabs(want - g_observer.theta_i);
                if(err > PI)
                    err = 2.0 * PI - err;
                double deg = err * 180.0 / PI;
                if(deg > max_err_deg)
                    max_err_deg = deg;
            }
        }
        printf("obs-vesc: max err %.2f deg |x|=%f\n", max_err_deg,
               NORM2_f(g_observer.state.x1, g_observer.state.x2));
        CHECK(max_err_deg < 5.0);
        CHECK(NORM2_f(g_observer.state.x1, g_observer.state.x2) > 0.001f);
    }

    /* 18. VECTOR переход end-to-end через foc_fast_calc с моделью PMSM.
   * Модель в alpha-beta: di/dt = (v - R*i - e)/L, e = back-EMF ротора
   * (35 Гц эл., lambda=2000uWb). Фазовые токи для АЦП: ia=i_al,
   * ib=(sqrt3*i_be - i_al)/2. Observer следит за th_r в I/F, затем PLL. */
    {
        const double SQ3 = 1.7320508075688772;
        g_foc_inv_i = 0;
        g_foc_adc_map = 0;
        g_foc_obs_off = 0; /* модель честная, без -вычета */
        g_motor.R_mOhm = 450;
        g_motor.L_uH = 300;
        g_motor.lambda_uWb = 2000;
        foc_init();
        g_adc_off[0] = 2048;
        g_adc_off[1] = 2048;
        g_adc_off[2] = 2048;
        g_if_speed_ref = 0.1f;
        g_if_flux_ref = 0.5f;
        foc_if_start();
        double om = 2.0 * PI * 35.0;
        double th_r = 0.3;
        double i_al = 0.0, i_be = 0.0;
        const double Udc = 16.0;

        for(int i = 0; i < 13000; i++) {
            double ia_ph = i_al;
            double ib_ph = (SQ3 * i_be - i_al) * 0.5;
            g_adc_raw[0] = (uint16_t)(2048 + (int)(ia_ph * 37.2));
            g_adc_raw[1] = (uint16_t)(2048 + (int)(ib_ph * 37.2));
            g_adc_raw[2] = (uint16_t)(2048 - (int)((ia_ph + ib_ph) * 37.2));
            g_adc_raw[3] = 1241; /* 16 V */
            foc_fast_calc();
            double va = g_pwm.UalphaNorm * (2.0 / 3.0) * Udc;
            double vb = g_pwm.UbetaNorm * (2.0 / 3.0) * Udc;
            double e_al = -om * 0.002 * sin(th_r);
            double e_be = om * 0.002 * cos(th_r);
            i_al += (va - 0.45 * i_al - e_al) / 0.0003 * 1.6e-4;
            i_be += (vb - 0.45 * i_be - e_be) / 0.0003 * 1.6e-4;
            th_r += om * 1.6e-4;
        }
        {
            double want = th_r;
            while(want < -PI)
                want += 2.0 * PI;
            while(want >= PI)
                want -= 2.0 * PI;
            double err = fabs(want - g_observer.theta_est);
            if(err > PI)
                err = 2.0 * PI - err;
            printf("vec: I/F theta_est err %.2f deg\n", err * 180.0 / PI);
            CHECK(err * 180.0 / PI < 60.0);
        }

        foc_vector_start();

        CHECK(g_foc_ctrl_state == FOC_CTRL_VECTOR);
        CHECK(g_if_active == 0);
        CHECK_CLOSE_F(g_pwm.U_lim, 1.0, 1e-6);
        CHECK(g_vesc_cfg.l_max_duty > 0.9f);
        double max_err_deg = 0;
        for(int i = 0; i < 6250 + 2000; i++) {
            double ia_ph = i_al;
            double ib_ph = (SQ3 * i_be - i_al) * 0.5;
            g_adc_raw[0] = (uint16_t)(2048 + (int)(ia_ph * 37.2));
            g_adc_raw[1] = (uint16_t)(2048 + (int)(ib_ph * 37.2));
            g_adc_raw[2] = (uint16_t)(2048 - (int)((ia_ph + ib_ph) * 37.2));
            g_adc_raw[3] = 1241;
            foc_fast_calc();
            double va = g_pwm.UalphaNorm * (2.0 / 3.0) * Udc;
            double vb = g_pwm.UbetaNorm * (2.0 / 3.0) * Udc;
            double e_al = -om * 0.002 * sin(th_r);
            double e_be = om * 0.002 * cos(th_r);
            i_al += (va - 0.45 * i_al - e_al) / 0.0003 * 1.6e-4;
            i_be += (vb - 0.45 * i_be - e_be) / 0.0003 * 1.6e-4;
            th_r += om * 1.6e-4;
            if(i > 6250) {
                double want = th_r;
                while(want < -PI)
                    want += 2.0 * PI;
                while(want >= PI)
                    want -= 2.0 * PI;
                double err = fabs(want - g_observer.theta_est);
                if(err > PI)
                    err = 2.0 * PI - err;
                double deg = err * 180.0 / PI;
                if(deg > max_err_deg)
                    max_err_deg = deg;
            }
        }
        printf("vec: max theta_est err %.2f deg\n", max_err_deg);
        CHECK(max_err_deg < 60.0);
        foc_if_stop();
        CHECK(g_foc_ctrl_state == FOC_CTRL_STOP);
    }

    /* 18c. Проводка inv_i: при inv=1 измеренный ток в dq меняет знак. */
    {
        g_foc_adc_map = 0;
        g_adc_raw[0] = 2048 + 100; /* ia = +100 кодов */
        g_adc_raw[1] = 2048 - 50;
        g_adc_raw[2] = 2048 - 50;
        g_adc_raw[3] = 1241;
        g_adc_off[0] = 2048;
        g_adc_off[1] = 2048;
        g_adc_off[2] = 2048;
        foc_init();
        g_foc_inv_i = 0;
        foc_if_start();
        foc_fast_calc();
        float id_pos = g_park.de;
        foc_if_stop();
        foc_init();
        g_foc_inv_i = 1;
        foc_if_start();
        foc_fast_calc();
        float id_neg = g_park.de;
        printf("wire: inv id_pos=%.4f id_neg=%.4f\n", id_pos, id_neg);
        CHECK(fabsf(id_neg + id_pos) < 0.02f);
        g_foc_inv_i = 0;
        foc_if_stop();
        CHECK(g_foc_ctrl_state == FOC_CTRL_STOP);
    }

    /* UART regressions: start from STOP, reject invalid floats, cancel detect. */
    {
        foc_ctrl_stop();
        foc_params_init();
        CHECK(foc_params_valid(&g_params));
        g_manual_pwm_active = 1;
        CHECK(!foc_param_idle());
        g_manual_pwm_active = 0;
        CHECK(foc_param_idle());
        CHECK(foc_params_save());
        g_params.s.r_ohm = 0.5f;
        CHECK(foc_params_load());
        CHECK_CLOSE_F(g_params.s.r_ohm, 0.190f, 1e-6);
        TFocProtoFrame req = {0};
        uint8_t response[FOC_PROTO_MAX_FRAME];
        float value = 1.0f;
        req.cmd = FOC_CMD_SET_CLOSED;
        req.len = 4;
        memcpy(req.payload, &value, 4);
        CHECK(foc_proto_handle(&req, response, sizeof(response)) > 0);
        CHECK(response[3] == FOC_CMD_ACK);
        CHECK(g_foc_enable && g_foc_ctrl_state == FOC_CTRL_VECTOR);
        req.cmd = FOC_CMD_STOP;
        req.len = 0;
        foc_proto_handle(&req, response, sizeof(response));
        CHECK(!g_foc_enable);
        req.cmd = FOC_CMD_SET_OPENLOOP;
        req.len = 8;
        value = NAN;
        memcpy(req.payload, &value, 4);
        value = 10.0f;
        memcpy(req.payload + 4, &value, 4);
        foc_proto_handle(&req, response, sizeof(response));
        CHECK(response[3] == FOC_CMD_NAK && !g_foc_enable);
        req.cmd = FOC_CMD_SET_HOLD;
        foc_proto_handle(&req, response, sizeof(response));
        CHECK(response[3] == FOC_CMD_NAK && !g_foc_enable);
        req.cmd = FOC_CMD_SET_SPEED;
        req.len = 4;
        value = INFINITY;
        memcpy(req.payload, &value, 4);
        foc_proto_handle(&req, response, sizeof(response));
        CHECK(response[3] == FOC_CMD_NAK && !g_foc_enable);
        foc_detect_start(FOC_DETECT_R, 1.0f);
        CHECK(foc_detect_busy());
        req.cmd = FOC_CMD_STOP;
        req.len = 0;
        foc_proto_handle(&req, response, sizeof(response));
        CHECK(!foc_detect_busy() && !g_foc_enable);
        g_tick_100us += 10000u;
        foc_detect_tick();
        CHECK(!g_foc_enable && g_foc_ctrl_state == FOC_CTRL_STOP);
    }

    /* Параметры низкоиндуктивного мотора и восстановление после UV. */
    {
        foc_ctrl_stop();
        foc_params_defaults(&g_params);
        g_params.s.r_ohm = 0.041f;
        g_params.s.l_h = 1e-6f;
        g_params.s.lambda_wb = 847e-6f;
        g_params.s.cc_tc_us = 2000.0f;
        g_params.s.flags = FOC_PARAM_FLAG_INV_I | FOC_PARAM_FLAG_OBS_V_RAW;
        g_params.s.vbus_min_v = 9.0f;
        g_params.s.vbus_max_v = 24.0f;
        foc_params_apply();
        CHECK_CLOSE_F(g_vesc_cfg.foc_current_kp, 0.0005f, 1e-7);
        CHECK_CLOSE_F(g_vesc_cfg.foc_current_ki, 20.5f, 1e-4);
        CHECK_CLOSE_F(g_foc_obs_gain, 1e3f / (847e-6f * 847e-6f), 256.0);
        CHECK(g_foc_obs_vraw == 1u);
        CHECK_CLOSE_F(g_foc_vbus_min, 9.0f, 1e-6);
        CHECK_CLOSE_F(g_foc_vbus_max, 24.0f, 1e-6);
        for(unsigned i = 0; i < 3; ++i) {
            g_adc_raw[i] = 2048;
            g_adc_off[i] = 2048;
        }
        g_adc_raw[3] = 1600;
        foc_if_start();
        g_vesc_state.v_alpha = 2.0f;
        g_vesc_state.v_beta = 0.0f;
        foc_fast_calc();
        CHECK_CLOSE_F(g_observer.v_alpha, 2.0f, 1e-6);
        g_foc_obs_vraw = 0u;
        g_vesc_state.v_alpha = 2.0f;
        g_vesc_state.v_beta = 0.0f;
        foc_fast_calc();
        CHECK_CLOSE_F(g_observer.v_alpha, -2.0f, 1e-6);
        foc_ctrl_stop();
        CHECK(!g_foc_outputs_on);
        TFocProtoFrame req = {.cmd = FOC_CMD_CLEAR_FAULT, .len = 0};
        uint8_t response[FOC_PROTO_MAX_FRAME];
        g_foc_state = STATE_OVERCURRENT_FAULT;
        g_fault_flags = FAULT_UV;
        g_foc_protection.uv_samples = 32;
        foc_proto_handle(&req, response, sizeof(response));
        CHECK(response[3] == FOC_CMD_ACK);
        CHECK(g_fault_flags == 0 && g_foc_state == STATE_NORMAL);
        CHECK(!g_foc_enable && !g_foc_outputs_on);
        CHECK(g_foc_protection.uv_samples == 0);
        CHECK(foc_closed_request(1.0f));
        CHECK(g_foc_enable && g_foc_outputs_on);
        foc_proto_handle(&req, response, sizeof(response));
        CHECK(response[3] == FOC_CMD_NAK);
        foc_ctrl_stop();
        g_foc_state = STATE_OVERCURRENT_FAULT;
        g_fault_flags = FAULT_UV;
        g_adc_raw[3] = 2000;
        CHECK(!foc_fault_clear());
        g_adc_raw[3] = 1600;
        g_fault_flags = FAULT_SW_OC;
        CHECK(!foc_fault_clear());
        g_fault_flags = FAULT_UV;
        CHECK(foc_fault_clear());
    }

    /* Короткая серия L не зависит от опроса main и не ускоряет FOC. */
    {
        foc_ctrl_stop();
        foc_params_defaults(&g_params);
        g_params.s.r_ohm = 0.041f;
        g_params.s.l_h = 1.132e-6f;
        foc_params_apply();
        for(unsigned i = 0; i < 3; ++i) {
            g_adc_raw[i] = 2048;
            g_adc_off[i] = 2048;
        }
        g_foc_udc = 20.0f;
        g_adc_raw[3] = 1552;
        uint16_t raw[4] = {2048, 2048, 2048, 1552};
        uint32_t foc_count = g_foc_isr_count;
        foc_detect_start(FOC_DETECT_L, 1.0f);
        g_tick_100us += 1001u;
        foc_detect_tick();
        CHECK(foc_detect_l_sample(raw));
        CHECK(l_adc_mode && g_foc_outputs_on);
        for(unsigned n = 1; n <= 4; ++n) {
            float current = 8.0f * (1.0f - expf(-(float)n * 40e-6f * 0.041f / 1.132e-6f));
            raw[0] = (uint16_t)(2048 - lroundf(current * 37.2f));
            CHECK(foc_detect_l_sample(raw));
        }
        CHECK(!l_adc_mode && !g_foc_outputs_on && !g_foc_enable);
        CHECK(g_foc_isr_count == foc_count);
        foc_detect_tick();
        g_tick_100us += 1001u;
        foc_detect_tick();
        CHECK(g_detect_state == FOC_DETECT_DONE);
        CHECK_CLOSE_F(g_detect_l, 1.132e-6f, 0.2e-6f);
        CHECK_CLOSE_F(FOC_TS, 160e-6f, 1e-9);
        raw[0] = 2048;
        foc_detect_start(FOC_DETECT_L, 1.0f);
        g_tick_100us += 1001u;
        foc_detect_tick();
        CHECK(foc_detect_l_sample(raw));
        foc_detect_abort();
        CHECK(!g_foc_outputs_on);
        CHECK(foc_detect_l_sample(raw));
        CHECK(!l_adc_mode && !foc_detect_busy());
        foc_detect_start(FOC_DETECT_L, 1.0f);
        g_tick_100us += 1001u;
        foc_detect_tick();
        CHECK(foc_detect_l_sample(raw));
        raw[0] = 0;
        CHECK(foc_detect_l_sample(raw));
        CHECK(!l_adc_mode && !g_foc_outputs_on);
        foc_detect_tick();
        CHECK(g_detect_state == FOC_DETECT_ERROR);
        foc_detect_abort();
    }

    if(n_fail == 0)
        printf("ALL PASS\n");
    else
        printf("%d FAILURES\n", n_fail);
    return n_fail ? 1 : 0;
}
