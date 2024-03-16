#ifndef __W25QXX_H__
#define __W25QXX_H__
#include "sys.h"

extern void w25qxx_init(void);
extern uint16_t w25qxx_read_id(void);
extern void w25qxx_read_data(uint32_t addr,uint8_t *pbuf,uint32_t len);
extern void w25qxx_erase_sector(uint32_t addr);
extern void w25qxx_write_data(uint32_t addr,uint8_t *pbuf,uint32_t len);
extern void w25qxx_write_data_ex(uint32_t addr,uint8_t *pbuf,uint32_t len);

extern int32_t  spi_flash_card_match(uint8_t *card_id);
extern int32_t  spi_flash_card_add(uint8_t *card_id);
extern int32_t  spi_flash_card_times(uint8_t *card_id);
extern uint32_t spi_flash_card_total(void);
extern int32_t  spi_flash_card_list_all(void);
extern int32_t  spi_flash_card_remove_all(void);

#endif

