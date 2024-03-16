#ifndef _KEYBOARD_H_
#define	_KEYBOARD_H_
#include "sys.h"

extern void key_board_init(void);
extern char get_key_board(void);
extern char get_key_board_val(void);
extern int32_t passwd_compare(uint8_t *input_passwd_buf,uint8_t *passwd_buf);

#endif
