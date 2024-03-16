#ifndef __USART6_H
#define __USART6_H

struct usart6_packet_t
{
	uint8_t rx_buf[64];
	uint8_t rx_len;
};


extern void uart6_init(u32 baud);
extern void usart_lcd_init(u32 baud);

#endif
