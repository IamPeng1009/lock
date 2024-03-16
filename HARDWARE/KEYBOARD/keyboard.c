#include "includes.h"

void key_board_init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	
    //使能端口D
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    //PC13 PB1 PB2 PB15      //PD10 PD8 PE12
	//使用GPIO_Init来配置引脚
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;			   //指定第0根引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;        //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;       //推挽输出模式,默认的
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;     //高速，但是功耗是最高
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;  //无需上下拉（亦可使能下拉电阻）
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //D口
	
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_15;            //指定第7、9、11根引脚
    GPIO_Init(GPIOB,&GPIO_InitStructure);              //E口

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_8; //指定第1、15根引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;         //输入模式
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	   //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //D口

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;             //指定第8、10根引脚
    GPIO_Init(GPIOE,&GPIO_InitStructure);  							//E口
}

//PC13 PB1 PB2 PB15      //PD10 PD8 PE12
char get_key_board(void)
{
	//如果第一行被按下      
    PCout(13) = 0;
    PBout(1) = 1;
    PBout(2) = 1;
    PBout(15) = 1;
    delay_ms(2);
	
    //第一行不同的键位反馈相应的含义
    if( PDin(10) == 0 ) return '1';
    else if( PDin(8) == 0 ) return '2';
    else if( PEin(12) == 0 ) return '3';
    
	//如果第二行被按下
    PCout(13) = 1;
    PBout(1) = 0;
    PBout(2) = 1;
    PBout(15) = 1;
    delay_ms(2);

    //第二行不同的键位反馈相应的含义
    if( PDin(10) == 0 ) return '4';
    else if( PDin(8) == 0 ) return '5';
    else if( PEin(12) == 0 ) return '6';
    
	//如果第三行被按下
    PCout(13) = 1;
    PBout(1) = 1;
    PBout(2) = 0;
    PBout(15) = 1;
    delay_ms(2);
	
    //第三行不同的键位反馈相应的含义
    if( PDin(10) == 0 ) return '7';
    else if( PDin(8) == 0 ) return '8';
    else if( PEin(12) == 0 ) return '9';
    
	//如果第四行被按下
    PCout(13) = 1;
    PBout(1) = 1;
    PBout(2) = 1;
    PBout(15) = 0;
    delay_ms(2);
	
    ///第四行不同的键位反馈相应的含义
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
	
    /* 使用状态机思想得到按键的状态 */
    switch(key_sta)
    {
        case 0://获取按下的按键
        {
            key_cur = get_key_board();	

            if(key_cur != 'N')
            {
                key_old = key_cur;
                key_sta=1;
            }	
        }break;
        
        
        case 1://确认按下的按键
        {
			
            key_cur = get_key_board();	
            
            if((key_cur != 'N') && (key_cur == key_old))
            {					
                key_sta=2;				
            }	
            
        }break;
    
        case 2://获取释放的按键
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


