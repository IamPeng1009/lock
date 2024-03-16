#include "includes.h"

int32_t at24c02_write(uint8_t addr,uint8_t *buf,uint32_t len)
{
	uint8_t ack;
	uint8_t *p=buf;
	
	i2c_start();
	i2c_send_byte(0xA0);	
	ack=i2c_wait_ack();
	if(ack)
	{
		printf("device address error\r\n");
		return -1;
	}
	
	i2c_send_byte(addr);
	ack=i2c_wait_ack();
	if(ack)
	{
		printf("word address error\r\n");
		return -2;
	}	
	
	while(len--)
	{
		i2c_send_byte(*p++);	
		ack=i2c_wait_ack();	
		if(ack)
		{
			printf("data write error\r\n");
			return -3;
		}		
	}
	
	i2c_stop();
	printf("write success\r\n");
	
	return 0;
}

int32_t at24c02_read(uint8_t addr,uint8_t *buf,uint32_t len)
{
	uint8_t ack;
	uint8_t *p=buf;	
	i2c_start();
	i2c_send_byte(0xA0);	
	ack=i2c_wait_ack();	
	if(ack)
	{
		printf("device address error\r\n");
		return -1;
	}
	
	i2c_send_byte(addr);
	ack=i2c_wait_ack();	
	if(ack)
	{
		printf("word address error\r\n");
		return -2;
	}
	
	i2c_start();	
	i2c_send_byte(0xA1);	
	ack=i2c_wait_ack();
	if(ack)
	{
		printf("device address error\r\n");
		return -3;
	}	

	while(len--)
	{
		*p++=i2c_recv_byte();
		if(len==0)
		{
			i2c_ack(1);
		}
		else
		{
			i2c_ack(0);
		}
	}
	i2c_stop();
	
	printf("read success\r\n");
	return 0;
}


