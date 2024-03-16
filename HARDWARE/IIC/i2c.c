#include "includes.h"

static GPIO_InitTypeDef GPIO_InitStructure;

void i2c_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;		//第8 9根引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//设置输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;				//开漏模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			//不需要上拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//只要是输出模式，有初始电平状态
	SCL_W=1;
	SDA_W=1;
}

void i2c_pin_mode(GPIOMode_TypeDef pin_mode)
{
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		//第9根引脚
	GPIO_InitStructure.GPIO_Mode = pin_mode;	//设置输出/输入模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	//开漏模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//不需要上拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void i2c_start(void)
{
	//保证SDA引脚为输出模式
	i2c_pin_mode(GPIO_Mode_OUT);
	
	SCL_W=1;
	SDA_W=1;
	
	delay_us(5);
	SDA_W=0;	
	
	delay_us(5);
	SCL_W=0;;
}

void i2c_stop(void)
{
	//保证SDA引脚为输出模式
	i2c_pin_mode(GPIO_Mode_OUT);
	
	SCL_W=1;
	SDA_W=0;
	
	delay_us(5);
	SDA_W=1;	
}

void i2c_send_byte(uint8_t byte)
{
	int32_t i;
	
	//保证SDA引脚为输出模式
	i2c_pin_mode(GPIO_Mode_OUT);
	
	SCL_W=0;
	SDA_W=0;
	delay_us(5);
	
	for(i=7; i>=0; i--)
	{
		if(byte & (1<<i))
			SDA_W=1;
		else
			SDA_W=0;
	
		delay_us(5);
	
		SCL_W=1;
		delay_us(5);

		SCL_W=0;
		delay_us(5);		
	}
}

void i2c_ack(uint8_t ack)
{
	i2c_pin_mode(GPIO_Mode_OUT);
	
	SDA_W=0;
	SCL_W=0;
	delay_us(5);
	
	if(ack)
		SDA_W=1;
	else
		SDA_W=0;

	delay_us(5);
	
	SCL_W=1;
	delay_us(5);
	
	SCL_W=0;
	delay_us(5);
}

uint8_t i2c_wait_ack(void)
{
	uint8_t ack;
	//保证SDA引脚为输入模式
	i2c_pin_mode(GPIO_Mode_IN);

	SCL_W=1;
	delay_us(5);
	
	if(SDA_R)
		ack=1;
	else
		ack=0;
	
	SCL_W=0;
	delay_us(5);

	return ack;
}

uint8_t i2c_recv_byte(void)
{
	uint8_t d=0;
	int32_t i;
	
	//保证SDA引脚为输入模式
	i2c_pin_mode(GPIO_Mode_IN);
	
	for(i=7; i>=0; i--)
	{
		SCL_W=1;
		delay_us(5);
		
		if(SDA_R)
			d|=1<<i;
		
		SCL_W=0;
		delay_us(5);	
	}

	return d;
}

