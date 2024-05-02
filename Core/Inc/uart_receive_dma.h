/*
 * uart_receive_dma.h
 *
 *  Created on: Feb 8, 2024
 *      Author: Yoo
 */

#ifndef INC_UART_RECEIVE_DMA_H_
#define INC_UART_RECEIVE_DMA_H_

#define QUEUE_BUF_MAX   256

typedef struct queue{
  uint8_t  buf[QUEUE_BUF_MAX];
  uint16_t q_in_index;
  uint16_t q_out_index;
} queue8_t;

uint32_t uart_available(void);
uint8_t uart_q8_read(void);
void uart_init(void);
void read_rfid_number(void);
void transmitData();
#endif /* INC_UART_RECEIVE_DMA_H_ */
