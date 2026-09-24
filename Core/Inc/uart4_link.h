#ifndef UART4_LINK_H
#define UART4_LINK_H

#include <stdint.h>

/* ============ RX ============ */
#define UART4_RX_BUF_SIZE     128
#define UART4_PKT_QUEUE_SIZE  8

/* Временный буфер для прерывания — двойной, чтобы прерывание не перезаписало
   тот, что сейчас обрабатывается */
extern volatile uint8_t  uart4_rx_buf[2][UART4_RX_BUF_SIZE];
extern volatile uint8_t  uart4_rx_active;
extern volatile uint16_t uart4_rx_idx;

/* Очередь готовых пакетов */
extern volatile uint8_t  uart4_pkt_queue[UART4_PKT_QUEUE_SIZE][UART4_RX_BUF_SIZE];
extern volatile uint16_t uart4_pkt_len_queue[UART4_PKT_QUEUE_SIZE];
extern volatile uint8_t  uart4_pkt_head;
extern volatile uint8_t  uart4_pkt_tail;
extern volatile uint32_t uart4_pkt_dropped;

extern volatile uint32_t uart4_total_bytes;
extern volatile uint32_t uart4_total_packets;

/* ============ TX ============ */
#define UART4_TX_BUF_SIZE   512

extern volatile uint8_t  uart4_tx_buf[UART4_TX_BUF_SIZE];
extern volatile uint16_t uart4_tx_head;
extern volatile uint16_t uart4_tx_tail;
extern volatile uint8_t  uart4_tx_busy;

extern volatile uint32_t uart4_tx_total_bytes;
extern volatile uint32_t uart4_tx_overflow;

/* Функция неблокирующей отправки */
uint8_t uart4_send(const uint8_t *data, uint16_t len);

#endif /* UART4_LINK_H */
