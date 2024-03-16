#ifndef __IIC_H__
#define __IIC_H__
#include "sys.h"

#define SCL_W	PBout(8)
#define SDA_W	PBout(9)
#define SDA_R	PBin(9)

extern void i2c_init(void);
extern void i2c_pin_mode(GPIOMode_TypeDef pin_mode);
extern void i2c_start(void);
extern void i2c_stop(void);
extern void i2c_send_byte(uint8_t byte);
extern void i2c_ack(uint8_t ack);
extern uint8_t i2c_wait_ack(void);
extern uint8_t i2c_recv_byte(void);

#endif

