#include "includes.h"

static struct usart6_packet_t g_usart6_packet={0};

//��ʼ��IO ����6 
//bound:������
void uart6_init(u32 baud)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOCʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//ʹ��USART6ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //GPIO6��GPIOA7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PA6��PA7

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = baud; //����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //�շ�ģʽ
	USART_Init(USART6, &USART_InitStructure); //��ʼ������1
	
	USART_Cmd(USART6, ENABLE); //ʹ�ܴ���1 
	
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=6;		//�����ȼ�6
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ�����
}

void usart_lcd_init(u32 baud)
{
	uart6_init(baud);
}

//����6�жϷ������
void USART6_IRQHandler(void)                	
{
	static uint32_t i=0;
	uint8_t d;
	uint32_t ulReturn;

	/* �����ٽ�Σ��ٽ�ο���Ƕ�� */
	ulReturn = taskENTER_CRITICAL_FROM_ISR();	
	
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  
	{
		//���մ�������
		d=USART_ReceiveData(USART6);

		g_usart6_packet.rx_buf[i++]=d;
				
		//��⵽'#'������յ���������ʱ����������
		if(d=='#' || i>=(sizeof(g_usart6_packet.rx_buf)))
		{
			g_usart6_packet.rx_len = i;
			
			xQueueSendFromISR(g_queue_usart6,(void *)g_usart6_packet.rx_buf,NULL);
			
			memset(&g_usart6_packet,0,sizeof(g_usart6_packet));
			
			i=0;
		}			
		
		//��մ��ڽ����жϱ�־λ
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
	} 
	
	/* �˳��ٽ�� */
	taskEXIT_CRITICAL_FROM_ISR(ulReturn );		
} 
