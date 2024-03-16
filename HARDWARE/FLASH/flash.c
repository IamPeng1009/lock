#include "includes.h"

void flash_erase_sector(uint32_t FLASH_Sector)
{
	FLASH_Unlock();
	
	FLASH_EraseSector(FLASH_Sector, VoltageRange_3);
	
	FLASH_Lock();
}

void flash_write(flash_t *flash_write_buf )
{
	int i = 0;
	FLASH_Unlock();

	for(i = 0; i<sizeof(flash_t)/4;i++)
	{
		FLASH_ProgramWord(0x08040000+flash_write_buf->offset * sizeof(flash_t) + i*4, *((uint32_t *)flash_write_buf + i));
	}
	
	FLASH_Lock(); 
}

void flash_read(uint32_t num)
{
	int i = 0;
	flash_t flash_read_buf;
	uint8_t buf[32];
	FLASH_Unlock();
	
	for(i=0;i<num;i++)
	{
		flash_read_buf = *((volatile flash_t*)(0x08040000 + i * sizeof(flash_t)));
		
		switch(flash_read_buf.mode)
		{
			case MODE_OPEN_LOCK_KEYBOARD:
				sprintf((char *)buf,"%s","keyboard unlock");
				break;
			case MODE_OPEN_LOCK_BLUE:
				sprintf((char *)buf,"%s","blue unlock");
				break;
			case MODE_OPEN_LOCK_MQTT:
				sprintf((char *)buf,"%s","mqtt unlock");
				break;
			case MODE_OPEN_LOCK_RFID:
				sprintf((char *)buf,"%s","rfid unlock");
				break;
			case MODE_OPEN_LOCK_SFM:
				sprintf((char *)buf,"%s","sfm unlock");
				break;
			default:
				break;
		}
			
		dgb_printf_safe("20%02x/%02x/%02x %02x:%02x:%02x %s\n",
		flash_read_buf.datebuf[0],
		flash_read_buf.datebuf[1],
		flash_read_buf.datebuf[2],
		flash_read_buf.datebuf[3],
		flash_read_buf.datebuf[4],
		flash_read_buf.datebuf[5],
		buf);
	}
	
	FLASH_Lock();
}

uint32_t flash_read_offset(uint32_t *start_addr)
{
	volatile uint32_t *p = start_addr;
	uint32_t count = 0;	
	FLASH_Unlock();
	while(*p != 0xffffffff)
	{
		count++;
		p = (uint32_t *)((uint8_t *)p + sizeof(flash_t));
	}
	FLASH_Lock();
	return count;
}


