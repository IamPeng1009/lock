#include "includes.h"

void key_board_init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	
    //ʹ�ܶ˿�D
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    //PC13 PB1 PB2 PB15      //PD10 PD8 PE12
	//ʹ��GPIO_Init����������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;			   //ָ����0������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;        //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;       //�������ģʽ,Ĭ�ϵ�
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;     //���٣����ǹ��������
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  //���������������ʹ���������裩
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //D��
	
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_15;            //ָ����7��9��11������
    GPIO_Init(GPIOB,&GPIO_InitStructure);              //E��

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_8; //ָ����1��15������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;         //����ģʽ
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	   //����
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //D��

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;             //ָ����8��10������
    GPIO_Init(GPIOE,&GPIO_InitStructure);  							//E��
}

//PC13 PB1 PB2 PB15      //PD10 PD8 PE12
char get_key_board(void)
{
	//�����һ�б�����      
    PCout(13) = 0;
    PBout(1) = 1;
    PBout(2) = 1;
    PBout(15) = 1;
    delay_ms(2);
	
    //��һ�в�ͬ�ļ�λ������Ӧ�ĺ���
    if( PDin(10) == 0 ) return '1';
    else if( PDin(8) == 0 ) return '2';
    else if( PEin(12) == 0 ) return '3';
    
	//����ڶ��б�����
    PCout(13) = 1;
    PBout(1) = 0;
    PBout(2) = 1;
    PBout(15) = 1;
    delay_ms(2);

    //�ڶ��в�ͬ�ļ�λ������Ӧ�ĺ���
    if( PDin(10) == 0 ) return '4';
    else if( PDin(8) == 0 ) return '5';
    else if( PEin(12) == 0 ) return '6';
    
	//��������б�����
    PCout(13) = 1;
    PBout(1) = 1;
    PBout(2) = 0;
    PBout(15) = 1;
    delay_ms(2);
	
    //�����в�ͬ�ļ�λ������Ӧ�ĺ���
    if( PDin(10) == 0 ) return '7';
    else if( PDin(8) == 0 ) return '8';
    else if( PEin(12) == 0 ) return '9';
    
	//��������б�����
    PCout(13) = 1;
    PBout(1) = 1;
    PBout(2) = 1;
    PBout(15) = 0;
    delay_ms(2);
	
    ///�����в�ͬ�ļ�λ������Ӧ�ĺ���
    if( PDin(10) == 0 ) return '*';
    else if( PDin(8) == 0 ) return '0';
    else if( PEin(12) == 0 ) return '#';
    
    return 'N';
}

char get_key_board_val(void)
{
	static 	char key_sta=0;
	static 	char key_old='N';	
	char key_val='N';
	char key_cur='N';
	
    /* ʹ��״̬��˼��õ�������״̬ */
    switch(key_sta)
    {
        case 0://��ȡ���µİ���
        {
            key_cur = get_key_board();	

            if(key_cur != 'N')
            {
                key_old = key_cur;
                key_sta=1;
            }	
        }break;
        
        
        case 1://ȷ�ϰ��µİ���
        {
			
            key_cur = get_key_board();	
            
            if((key_cur != 'N') && (key_cur == key_old))
            {					
                key_sta=2;				
            }	
            
        }break;
    
        case 2://��ȡ�ͷŵİ���
        {
            
            key_cur = get_key_board();	
                
            if(key_cur == 'N')
            {
				key_val = key_old;			
                key_sta=0;					
                key_old =  'N';
 
            }
        }break;
        
        default:break;
    }

	 return key_val;
}

int32_t passwd_compare(uint8_t *input_passwd_buf,uint8_t *passwd_buf)
{
	if(strstr((char *)input_passwd_buf,(char *)passwd_buf))
		return 1;
	else
		return 0;
}


