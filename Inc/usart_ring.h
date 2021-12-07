/*
 * usart_ring.h
 *
 *  Created on: 19 авг. 2019 г.
 *      Author: dima
 */

#ifndef USART_RING_H_
#define USART_RING_H_

#include "main.h"

#define UART_RX_BUFFER_SIZE 128

uint16_t uart_available(void);
uint8_t uart_read(void);
void clear_uart_buff();

#endif /* USART_RING_H_ */

















