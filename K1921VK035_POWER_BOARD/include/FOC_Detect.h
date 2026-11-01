/*
 * FOC_Detect.h
 *
 * Измерение параметров мотора (bench detection).
 * Неблокирующая state machine, вызывается из main loop (foc_detect_tick).
 * Мотор обесточен между шагами; токи/напряжения читаются из ISR-переменных.
 *
 * R: два уровня DC-вектора вдоль alpha, R = (V2 - V1) / (I2 - I1) — общий
 *    сдвиг (мёртвое время, смещение нуля) вычитается.
 * L: импульс DC-вектора, три равноотстоящих выборки тока: (i3-i2)/(i2-i1) =
 *    exp(-dt/tau), L = R * tau.
 * LAMBDA: разомкнутый I/F на 100 Гц эл., lambda = |v - R*i| / omega.
 *
 * Лицензия: GNU GPLv3.
 */

#ifndef INCLUDE_FOC_DETECT_H_
#define INCLUDE_FOC_DETECT_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FOC_DETECT_NONE = 0,
    FOC_DETECT_R = 1,
    FOC_DETECT_L = 2,
    FOC_DETECT_LAMBDA = 3
} foc_detect_type_t;

typedef enum {
    FOC_DETECT_IDLE = 0,
    FOC_DETECT_BUSY,
    FOC_DETECT_DONE,
    FOC_DETECT_ERROR
} foc_detect_state_t;

extern volatile foc_detect_type_t g_detect_type;
extern volatile foc_detect_state_t g_detect_state;
extern volatile float g_detect_progress; /* 0..1 */
extern volatile float g_detect_r;        /* Ом */
extern volatile float g_detect_l;        /* Гн */
extern volatile float g_detect_lambda;   /* Вб */

/* Запуск измерения. current_a — целевой ток (А) для R/L/lambda. */
void foc_detect_start(foc_detect_type_t type, float current_a);
void foc_detect_abort(void);
/* Забыть замер R (после записи параметра r): L и lambda снова берут параметр. */
void foc_detect_forget_r(void);
bool foc_detect_busy(void);
/* Вызывать из main loop (не блокирует). */
void foc_detect_tick(void);
/* Только короткий импульс L; true — обычный FOC на этой выборке не считать. */
bool foc_detect_l_sample(const uint16_t raw[4]);

#endif /* INCLUDE_FOC_DETECT_H_ */
