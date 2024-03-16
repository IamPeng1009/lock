#ifndef __AT24C02_H__
#define	__AT24C02_H__
#include "sys.h"

extern int32_t at24c02_write(uint8_t addr,uint8_t *buf,uint32_t len);
extern int32_t at24c02_read(uint8_t addr,uint8_t *buf,uint32_t len);


#endif
