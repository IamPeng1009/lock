#include "includes.h"

void motor_init(void)
{
	/* ʹ�ܶ�Ӧ��GPIOD GPIOF ʱ��*/
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_4|GPIO_Pin_14;//��0,4,14������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//����ģʽ��������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			//����Ҫ��������
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;					//��12������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//����Ϊ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//����ģʽ��������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			//����Ҫ��������
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	MOTOR_IN1=MOTOR_IN2=MOTOR_IN3=MOTOR_IN4=0;
}

//��ת
void motor_corotation_open(void)
{
	uint32_t i=0,j=0;
	
	for(i=0; i<32;i++)
	{
		for(j=0;j<8;j++)
		{
			MOTOR_IN4=1;
			MOTOR_IN3=1;
			MOTOR_IN2=0;
			MOTOR_IN1=0;
			delay_ms(2);

			MOTOR_IN4=1;
			MOTOR_IN3=0;
			MOTOR_IN2=0;
			MOTOR_IN1=1;
			delay_ms(2);

			MOTOR_IN4=0;
			MOTOR_IN3=0;
			MOTOR_IN2=1;
			MOTOR_IN1=1;
			delay_ms(2);

			MOTOR_IN4=0;
			MOTOR_IN3=1;
			MOTOR_IN2=1;
			MOTOR_IN1=0;
			delay_ms(2); 
						
		}
	}
	MOTOR_IN4=0;
	MOTOR_IN3=0;
	MOTOR_IN2=0;
	MOTOR_IN1=0;
}

//��ת
void motor_corotation_close(void)
{
	uint32_t i=0,j=0;
	
	for(i=0; i<32;i++)
	{
		for(j=0;j<8;j++)
		{
			MOTOR_IN4=0;
			MOTOR_IN3=1;
			MOTOR_IN2=1;
			MOTOR_IN1=0;
			delay_ms(2);

			MOTOR_IN4=0;
			MOTOR_IN3=0;
			MOTOR_IN2=1;
			MOTOR_IN1=1;
			delay_ms(2);

			MOTOR_IN4=1;
			MOTOR_IN3=0;
			MOTOR_IN2=0;
			MOTOR_IN1=1;
			delay_ms(2);

			MOTOR_IN4=1;
			MOTOR_IN3=1;
			MOTOR_IN2=0;
			MOTOR_IN1=0;
			delay_ms(2); 			
		}
	}
	MOTOR_IN4=0;
	MOTOR_IN3=0;
	MOTOR_IN2=0;
	MOTOR_IN1=0;	
}
