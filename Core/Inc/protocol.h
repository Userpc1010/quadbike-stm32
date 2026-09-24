#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <string.h>

/* Маркеры */
#define PKT_SOF1            0xAA
#define PKT_SOF2            0x55

/* Команды */
#define PKT_CMD_PCA         0x01
#define PKT_CMD_MPI         0x02
#define PKT_CMD_MODE        0x03
#define PKT_CMD_TELE        0x10
#define PKT_CMD_MODE_ACK    0x11
#define PKT_CMD_PING        0x20
#define PKT_CMD_PONG        0x21

/* Максимальный payload */
#define PKT_MAX_PAYLOAD     64

typedef enum {
    CTRL_MODE_MANUAL = 0,
    CTRL_MODE_MPPI   = 1,
    CTRL_MODE_VPC    = 2
} ctrl_mode_t;

#pragma pack(push, 1)

typedef struct {
    float w_cmd;
    float setpoint;
    uint8_t stop;
} Pkt_PCA_t;

typedef struct {
    float steer_angle;
    float throttle;
    float brake;
} Pkt_MPI_t;

typedef struct {
    float stering_angle;
    float velosity_1d_mps;
} Pkt_Tele_t;

#pragma pack(pop)

static inline uint8_t pkt_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

static inline uint16_t pkt_build(uint8_t *out, uint8_t cmd,
                                 const void *payload, uint8_t len)
{
    out[0] = PKT_SOF1;
    out[1] = PKT_SOF2;
    out[2] = cmd;
    out[3] = len;
    if (len && payload) memcpy(&out[4], payload, len);
    out[4 + len] = pkt_crc8(&out[2], 2 + len);
    return 5 + len;
}

typedef enum {
    PKT_RX_WAIT_SOF1 = 0,
    PKT_RX_WAIT_SOF2,
    PKT_RX_WAIT_CMD,
    PKT_RX_WAIT_LEN,
    PKT_RX_WAIT_PAYLOAD,
    PKT_RX_WAIT_CRC
} pkt_rx_state_t;

typedef struct {
    pkt_rx_state_t state;
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[PKT_MAX_PAYLOAD];
    uint8_t idx;
} pkt_rx_t;

static inline void pkt_rx_init(pkt_rx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
    rx->state = PKT_RX_WAIT_SOF1;
}

static inline int pkt_rx_feed(pkt_rx_t *rx, uint8_t b)
{
    switch (rx->state) {
        case PKT_RX_WAIT_SOF1:
            if (b == PKT_SOF1) rx->state = PKT_RX_WAIT_SOF2;
            break;

        case PKT_RX_WAIT_SOF2:
            if      (b == PKT_SOF2) rx->state = PKT_RX_WAIT_CMD;
            else if (b == PKT_SOF1) rx->state = PKT_RX_WAIT_SOF2;
            else                    rx->state = PKT_RX_WAIT_SOF1;
            break;

        case PKT_RX_WAIT_CMD:
            rx->cmd = b;
            rx->state = PKT_RX_WAIT_LEN;
            break;

        case PKT_RX_WAIT_LEN:
            if (b > PKT_MAX_PAYLOAD) { rx->state = PKT_RX_WAIT_SOF1; break; }
            rx->len = b;
            rx->idx = 0;
            rx->state = (b == 0) ? PKT_RX_WAIT_CRC : PKT_RX_WAIT_PAYLOAD;
            break;

        case PKT_RX_WAIT_PAYLOAD:
            rx->payload[rx->idx++] = b;
            if (rx->idx >= rx->len) rx->state = PKT_RX_WAIT_CRC;
            break;

        case PKT_RX_WAIT_CRC: {
            uint8_t crc_data[2 + PKT_MAX_PAYLOAD];
            crc_data[0] = rx->cmd;
            crc_data[1] = rx->len;
            if (rx->len) memcpy(&crc_data[2], rx->payload, rx->len);
            uint8_t crc_calc = pkt_crc8(crc_data, 2 + rx->len);
            int ok = (crc_calc == b) ? 1 : 0;
            rx->state = PKT_RX_WAIT_SOF1;
            return ok;
        }
    }
    return 0;
}

#endif /* PROTOCOL_H */
