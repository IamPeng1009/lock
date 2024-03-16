#include "includes.h"

static GPIO_InitTypeDef GPIO_InitStructure;

void i2c_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;		//��8 9������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//�������ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;				//��©ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			//����Ҫ��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//ֻҪ�����ģʽ���г�ʼ��ƽ״̬
	SCL_W=1;
	SDA_W=1;
}

void i2c_pin_mode(GPIOMode_TypeDef pin_mode)
{
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		//��9������
	GPIO_InitStructure.GPIO_Mode = pin_mode;	//�������/����ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;	//��©ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����Ҫ��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void i2c_start(void)
{
	//��֤SDA����Ϊ���ģʽ
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
	//��֤SDA����Ϊ���ģʽ
	i2c_pin_mode(GPIO_Mode_OUT);
	
	SCL_W=1;
	SDA_W=0;
	
	delay_us(5);
	SDA_W=1;	
}

void i2c_send_byte(uint8_t byte)
{
	int32_t i;
	
	//��֤SDA����Ϊ���ģʽ
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
	//��֤SDA����Ϊ����ģʽ
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
	
	//��֤SDA����Ϊ����ģʽ
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

