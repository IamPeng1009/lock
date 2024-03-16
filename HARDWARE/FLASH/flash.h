#ifndef __FLASH_H
#define __FLASH_H
#include "sys.h"

typedef struct __flash_t
{
#define MODE_OPEN_LOCK_KEYBOARD		0x01
#define MODE_OPEN_LOCK_BLUE			0x02
#define MODE_OPEN_LOCK_MQTT			0x04
#define MODE_OPEN_LOCK_RFID			0x08
#define MODE_OPEN_LOCK_SFM			0x10
	
	uint32_t offset;
	uint8_t mode;
	uint8_t datebuf[16];
}flash_t;

extern void flash_erase_sector(uint32_t FLASH_Sector);
extern void flash_write(flash_t *flash_write_buf);
extern void flash_read(uint32_t num);
extern uint32_t flash_read_offset(uint32_t *start_addr);
#endif
