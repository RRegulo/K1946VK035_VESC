/*
 * FOC_Params.h
 *
 * Параметры мотора/регулятора: единый блок 128 байт + CRC16, хранение в
 * main-flash (страница 1 КиБ). Раскладка общая с UART-конфигуратором.
 * По проводу любой параметр — float32 LE (см. FOC_Proto.h).
 */
#ifndef FOC_PARAMS_H
#define FOC_PARAMS_H

#include <stdbool.h>
#include <stdint.h>

/* v3: +VESC-параметры (dt, токовый контур, openloop, PID скорости, ERPM)
 * и CAN (DroneCAN); в урезанной прошивке часть из них только хранится. */
#define FOC_PARAMS_VERSION 3u
#define FOC_PARAMS_MAGIC 0x464F4350u /* 'FOCP' */

/* Страница под параметры: main-flash 64 КиБ, код занимает первые ~16 КиБ. */
#define FOC_PARAMS_FLASH_ADDR 0x0000F000u
#define FOC_PARAMS_FLASH_PAGE 1024u

/* s.flags */
#define FOC_PARAM_FLAG_INV_I 0x01u
#define FOC_PARAM_FLAG_INV_ANG 0x02u
#define FOC_PARAM_FLAG_MEASURED 0x04u
#define FOC_PARAM_FLAG_SAVED 0x08u
#define FOC_PARAM_FLAG_OBS_V_RAW 0x10u

typedef union {
    struct {
        uint32_t magic;            /* 0x00 */
        uint16_t version;          /* 0x04 */
        uint16_t size;             /* 0x06 sizeof(struct) */
        uint8_t pp;                /* 0x08 пар полюсов */
        uint8_t flags;             /* 0x09 FOC_PARAM_FLAG_* */
        uint16_t adc_trig;         /* 0x0A тиков TBCLK (0 = по умолчанию 100) */
        float r_ohm;               /* 0x0C сопротивление фазы, Ом */
        float l_h;                 /* 0x10 индуктивность, Гн */
        float ld_lq_diff_h;        /* 0x14 Ld-Lq, Гн */
        float lambda_wb;           /* 0x18 потокосцепление ротора, Вб */
        float kv;                  /* 0x1C об/мин/В (справочно) */
        float current_max_a;       /* 0x20 trip-ток, А */
        float current_cont_a;      /* 0x24 длительный ток, А */
        float vbus_min_v;          /* 0x28 */
        float vbus_max_v;          /* 0x2C */
        float dt_us;               /* 0x30 foc_dt_us */
        float cc_tc_us;            /* 0x34 постоянная токового контура, мкс */
        float ol_rpm;              /* 0x38 foc_openloop_rpm, ERPM */
        float sl_hyst;             /* 0x3C */
        float sl_time;             /* 0x40 */
        float sl_ramp;             /* 0x44 */
        float s_kp;                /* 0x48 */
        float s_ki;                /* 0x4C */
        float s_kd;                /* 0x50 */
        float s_kd_filter;         /* 0x54 */
        float s_min_erpm;          /* 0x58 */
        float s_ramp_erpms;        /* 0x5C */
        float max_erpm;            /* 0x60 */
        uint8_t can_node_id;       /* 0x64 1..127 (0 = CAN выключен) */
        uint8_t can_esc_index;     /* 0x65 */
        uint8_t can_raw_mode;      /* 0x66 */
        uint8_t can_flags;         /* 0x67 */
        float can_raw_rpm_max;     /* 0x68 */
        uint16_t can_bitrate_kbps; /* 0x6C */
        uint16_t can_status_ms;    /* 0x6E */
        uint16_t can_timeout_ms;   /* 0x70 */
        uint8_t pad0[2];           /* 0x72 выравнивание */
        float obs_offset;          /* 0x74 foc_observer_offset */
        uint8_t reserved[6];       /* 0x78..0x7D */
        uint16_t crc;              /* 0x7E CRC16 по 0x00..0x7D */
    } s;

    uint8_t buffer[128]; /* сырой вид для CRC/записи во flash */
} TFocParams;

extern TFocParams g_params;

uint16_t foc_params_crc16(const uint8_t *data, uint32_t len);
void foc_params_defaults(TFocParams *p);
void foc_params_finalize(TFocParams *p); /* пересчитать size + crc */
bool foc_params_valid(const TFocParams *p);
bool foc_params_sane(const TFocParams *p);
void foc_params_init(void);  /* загрузка (или defaults) + apply */
bool foc_params_load(void);  /* flash -> g_params */
bool foc_params_save(void);  /* g_params -> flash (erase+write+verify) */
void foc_params_apply(void); /* g_params -> g_motor / observer / vesc */

/* Прямой доступ к main-flash (8-байтные операции MFLASH). */
void foc_flash_read(uint32_t addr, uint8_t *dst, uint32_t len);
void foc_flash_erase_page(uint32_t addr);
void foc_flash_write_page(uint32_t addr, const uint8_t *src, uint32_t len);

#endif /* FOC_PARAMS_H */
