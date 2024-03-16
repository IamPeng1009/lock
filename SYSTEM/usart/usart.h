#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"  //芯片的所有寄存器定义

struct usart_packet_t
{
	uint8_t rx_buf[64];
	uint8_t rx_len;
};


extern void uart_init(u32 baud);
extern void usart_send_bytes(USART_TypeDef* USARTx,uint8_t *buf,uint32_t len);


#endif


