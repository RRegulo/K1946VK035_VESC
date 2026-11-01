/*
 * FOC_Proto.h
 *
 * Бинарный протокол конфигуратора поверх UART (тот же, что в полной
 * прошивке и UART-конфигураторе).
 * Кадр: AA 55 len cmd payload[len] crc16(lo,hi), CRC-16/XMODEM по
 * len+cmd+payload.
 */
#ifndef INCLUDE_FOC_PROTO_H_
#define INCLUDE_FOC_PROTO_H_

#include <stdbool.h>
#include <stdint.h>

#define FOC_PROTO_SOF1 0xAAu
#define FOC_PROTO_SOF2 0x55u
#define FOC_PROTO_VERSION 3u
#define FOC_PROTO_MAX_PAYLOAD 128u
#define FOC_PROTO_MAX_FRAME (2u + 1u + 1u + FOC_PROTO_MAX_PAYLOAD + 2u)

typedef enum {
    FOC_CMD_PING = 0x01,
    FOC_CMD_GET_INFO = 0x02,
    FOC_CMD_GET_PARAM = 0x03,
    FOC_CMD_SET_PARAM = 0x04,
    FOC_CMD_READ_CONFIG = 0x10,
    FOC_CMD_WRITE_CONFIG = 0x11,
    FOC_CMD_SAVE_PARAMS = 0x12,
    FOC_CMD_LOAD_DEFAULTS = 0x13,
    FOC_CMD_GET_TELEMETRY = 0x20,
    FOC_CMD_GET_BENCH = 0x21,
    FOC_CMD_SET_OPENLOOP = 0x30,
    FOC_CMD_SET_HOLD = 0x31,
    FOC_CMD_STOP = 0x32,
    FOC_CMD_START_DETECT = 0x33,
    FOC_CMD_ABORT = 0x34,
    FOC_CMD_GET_DETECT = 0x35,
    FOC_CMD_CLEAR_FAULT = 0x36,
    FOC_CMD_SET_CLOSED = 0x37,
    FOC_CMD_SET_SPEED = 0x38,
    FOC_CMD_CAP_ARM = 0x40,
    FOC_CMD_CAP_TRIGGER = 0x41,
    FOC_CMD_CAP_STATUS = 0x42,
    FOC_CMD_CAP_READ = 0x43,
    FOC_CMD_CAN_TEST = 0x50,
    FOC_CMD_CAN_STATUS = 0x51,
    FOC_CMD_ACK = 0x80,
    FOC_CMD_NAK = 0x81
} foc_proto_cmd_t;

/* ID параметров: значение на проводе всегда float32 LE; в TFocParams поле
 * может быть u8/u16 (см. таблицу s_ptab в FOC_Proto.c). */
typedef enum {
    FOC_PID_PP = 1,
    FOC_PID_R = 2,
    FOC_PID_L = 3,
    FOC_PID_LD_LQ = 4,
    FOC_PID_LAMBDA = 5,
    FOC_PID_KV = 6,
    FOC_PID_IMAX = 7,
    FOC_PID_ICONT = 8,
    FOC_PID_VBUS_MIN = 9,
    FOC_PID_VBUS_MAX = 10,
    FOC_PID_FLAGS = 11,
    FOC_PID_ADC_TRIG = 12,
    FOC_PID_DT_US = 13,
    FOC_PID_OBS_GAIN = 14, /* RAM: только чтение (константа наблюдателя) */
    FOC_PID_CC_KP = 15,    /* RAM: Kp токового ПИ */
    FOC_PID_CC_KI = 16,    /* RAM: Ki токового ПИ */
    FOC_PID_OL_RPM = 17,
    FOC_PID_SL_HYST = 18,
    FOC_PID_SL_TIME = 19,
    FOC_PID_SL_RAMP = 20,
    FOC_PID_CC_TC_US = 21,
    FOC_PID_S_KP = 22,
    FOC_PID_S_KI = 23,
    FOC_PID_S_KD = 24,
    FOC_PID_S_KD_FILTER = 25,
    FOC_PID_S_MIN_ERPM = 26,
    FOC_PID_S_RAMP_ERPMS = 27,
    FOC_PID_MAX_ERPM = 28,
    FOC_PID_CAN_NODE_ID = 29,
    FOC_PID_CAN_ESC_INDEX = 30,
    FOC_PID_CAN_RAW_MODE = 31,
    FOC_PID_CAN_FLAGS = 32,
    FOC_PID_CAN_RAW_RPM_MAX = 33,
    FOC_PID_CAN_BITRATE = 34,
    FOC_PID_CAN_STATUS_MS = 35,
    FOC_PID_CAN_TIMEOUT_MS = 36,
    FOC_PID_OBS_OFFSET = 37
} foc_param_id_t;

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[FOC_PROTO_MAX_PAYLOAD];
} TFocProtoFrame;

/* Доступ к таблице параметров. */
float foc_param_get_f32(uint8_t id);
bool foc_param_set_f32(uint8_t id, float v);
float foc_param_default_f32(uint8_t id);
bool foc_param_idle(void); /* STOP, нет I/F/VECTOR */

void foc_proto_reset(void);

/* Кодирует кадр в out. Возвращает длину кадра или 0 при ошибке. */
uint32_t foc_proto_encode(uint8_t *out, uint32_t out_sz, uint8_t cmd, const uint8_t *payload,
                          uint8_t len);

/* Побайтовый приём. 0 = нужен ещё байт, 1 = кадр собран, -1 = ошибка/ресинк. */
int foc_proto_feed(uint8_t byte, TFocProtoFrame *frame);

/* Обработка запроса: строит ответный кадр в out. 0 = ответа нет. */
uint32_t foc_proto_handle(const TFocProtoFrame *req, uint8_t *out, uint32_t out_sz);

#endif /* INCLUDE_FOC_PROTO_H_ */
