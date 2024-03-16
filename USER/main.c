#include "includes.h"

/* 任务句柄 */ 
static TaskHandle_t app_task_init_handle			= NULL;
static TaskHandle_t app_task_key_handle				= NULL;
static TaskHandle_t app_task_dht_handle 			= NULL;
static TaskHandle_t app_task_usart_handle 			= NULL;
static TaskHandle_t app_task_rtc_handle 			= NULL;
static TaskHandle_t app_task_led_handle 			= NULL;
static TaskHandle_t app_task_beep_handle 			= NULL;
static TaskHandle_t app_task_sfmreal_handle 		= NULL;
static TaskHandle_t app_task_sfmadmin_handle 		= NULL;
static TaskHandle_t app_task_usartlcd_handle 		= NULL;
static TaskHandle_t app_task_recvlcd_handle 		= NULL;
static TaskHandle_t app_task_motor_handle 			= NULL;
static TaskHandle_t app_task_mfrc522real_handle 	= NULL;
static TaskHandle_t app_task_mfrc522admin_handle	= NULL;
static TaskHandle_t app_task_keyboard_handle		= NULL;
static TaskHandle_t app_task_at24c02_handle			= NULL;
static TaskHandle_t app_task_flash_handle			= NULL;
static TaskHandle_t app_task_mqttheart_handle		= NULL;
static TaskHandle_t app_task_mqtt_handle			= NULL;
static TaskHandle_t app_task_mqttmonitor_handle	 	= NULL;

/* 任务 */ 
static void app_task_init			(void* pvParameters);  
static void app_task_key			(void* pvParameters); 
static void app_task_dht			(void* pvParameters); 
static void app_task_usart			(void* pvParameters); 
static void app_task_rtc			(void* pvParameters);  
static void app_task_led			(void* pvParameters); 
static void app_task_beep			(void* pvParameters);
static void app_task_sfmreal		(void* pvParameters); 
static void app_task_sfmadmin		(void* pvParameters); 
static void app_task_usartlcd		(void* pvParameters);
static void app_task_recvlcd		(void* pvParameters); 
static void app_task_motor			(void* pvParameters); 
static void app_task_mfrc522real	(void* pvParameters); 
static void app_task_mfrc522admin	(void* pvParameters); 
static void app_task_keyboard		(void* pvParameters);
static void app_task_at24c02		(void* pvParameters);
static void app_task_flash			(void* pvParameters);
static void app_task_mqttheart		(void* pvParameters);
static void app_task_mqtt			(void* pvParameters);
static void app_task_mqttmonitor	(void* pvParameters); 

/* 互斥型信号量句柄 */
SemaphoreHandle_t g_mutex_printf;

/* 事件标志组句柄 */
EventGroupHandle_t g_event_group;	

/* 全局变量 */
volatile uint8_t 	g_input_passward[10] = {0};
volatile uint8_t 	g_modify_passward[6] = {0};
volatile uint32_t   g_flash_offset       =  0;
float g_temp=0.0;
float g_humi=0.0;

/* 消息队列句柄 */
QueueHandle_t g_queue_usart;
QueueHandle_t g_queue_usart6;
QueueHandle_t g_queue_led;
QueueHandle_t g_queue_beep;
QueueHandle_t g_queue_usartlcd;
QueueHandle_t g_queue_esp8266;
QueueHandle_t g_queue_flash;

/* 计数型信号量 */
SemaphoreHandle_t g_sem_led;
SemaphoreHandle_t g_sem_beep;

/* 软件定时器句柄 */
static TimerHandle_t soft_timer_Handle =NULL;   

/* 软件定时器 */
static void soft_timer_callback(void* parameter);

//自行增加
static StackType_t  TimerTaskStack[configMINIMAL_STACK_SIZE];
static StaticTask_t TimerTaskTCB;

//定时器任务所需内存
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
	*ppxTimerTaskTCBBuffer=&TimerTaskTCB;
	*ppxTimerTaskStackBuffer=TimerTaskStack; 
	*pulTimerTaskStackSize=configMINIMAL_STACK_SIZE;
}

#define DEBUG_PRINTF_EN	1

// 安全可变长版本printf
void dgb_printf_safe(const char *format, ...)
{
#if DEBUG_PRINTF_EN	

	va_list args;// 声明了一个用于处理可变数量的参数args
	va_start(args, format);// 初始化 args，使其指向参数列表中的第一个可变参数
	
	/* 获取互斥信号量 */
	xSemaphoreTake(g_mutex_printf,portMAX_DELAY);
	
	vprintf(format, args);// 使用可变参数格式输出函数，因为用了va_list
			
	/* 释放互斥信号量 */
	xSemaphoreGive(g_mutex_printf);	

	va_end(args);// 结束使用可变参数列表 args
#else
	(void)0;
#endif
}

/* 任务列表 */
static const task_t task_tbl[]={	
	{app_task_key,			"app_task_key",			512,NULL,5,&app_task_key_handle},
	{app_task_dht,			"app_task_dht",			512,NULL,5,&app_task_dht_handle},
	{app_task_usart,		"app_task_usart",		512,NULL,5,&app_task_usart_handle},
	{app_task_rtc,			"app_task_rtc",			512,NULL,5,&app_task_rtc_handle},
	{app_task_led,			"app_task_led",			512,NULL,5,&app_task_led_handle},
	{app_task_beep,			"app_task_beep",		512,NULL,5,&app_task_beep_handle},
	{app_task_sfmreal,		"app_task_sfmreal",		512,NULL,5,&app_task_sfmreal_handle},
	{app_task_sfmadmin,		"app_task_sfmadmin",	512,NULL,5,&app_task_sfmadmin_handle},
	{app_task_usartlcd,		"app_task_usartlcd",	512,NULL,5,&app_task_usartlcd_handle},
	{app_task_recvlcd,		"app_task_recvlcd",		512,NULL,5,&app_task_recvlcd_handle},
	{app_task_motor,		"app_task_motor",		512,NULL,5,&app_task_motor_handle},
	{app_task_flash,		"app_task_flash",		512,NULL,5,&app_task_flash_handle},
	{app_task_mfrc522real,	"app_task_mfrc522real",	512,NULL,5,&app_task_mfrc522real_handle},
	{app_task_mfrc522admin,	"app_task_mfrc522admin",512,NULL,5,&app_task_mfrc522admin_handle},
	{app_task_keyboard,		"app_task_keyboard",	512,NULL,5,&app_task_keyboard_handle},
	{app_task_at24c02,		"app_task_at24c02",		512,NULL,5,&app_task_at24c02_handle},
	{app_task_mqttheart,	"app_task_mqttheart",	512,NULL,5,&app_task_mqttheart_handle},
	{app_task_mqtt,			"app_task_mqtt",		1024,NULL,5,&app_task_mqtt_handle},
	{app_task_mqttmonitor,	"app_task_mqttmonitor",	1024,NULL,5,&app_task_mqttmonitor_handle},
	{0,0,0,0,0,0}
};

int main(void)
{
	/* 设置系统中断优先级分组4 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	/* 系统定时器中断频率为configTICK_RATE_HZ */
	SysTick_Config(SystemCoreClock/configTICK_RATE_HZ);							
	
	/* 初始化串口1 */
	uart_init(9600);   

	/* 创建 app_task_init任务 */
	xTaskCreate((TaskFunction_t )app_task_init,  		/* 任务入口函数 */
			  (const char*    )	"app_task_init",		/* 任务名字 */
			  (uint16_t       )	512,  					/* 任务栈大小 */
			  (void*          )	NULL,					/* 任务入口函数参数 */
			  (UBaseType_t    )	5, 						/* 任务的优先级 */
			  (TaskHandle_t*  )&app_task_init_handle);	/* 任务控制块指针 */ 	

	/* 开启任务调度 */
	vTaskStartScheduler(); 
			  
	while(1);
	
}

static void app_task_init(void* pvParameters)
{
	uint32_t i=0;
	uint8_t passwd_buf[6]= "666666";
	
	/* 创建互斥型信号量 */	  
	g_mutex_printf=xSemaphoreCreateMutex();
	
	/* 创建计数型信号量 */
	g_sem_led =xSemaphoreCreateCounting(255,0);	
	g_sem_beep=xSemaphoreCreateCounting(255,0);	

	/* 创建事件标志组 */
	g_event_group=xEventGroupCreate();	
			  
	/* 创建消息队列 */
	g_queue_usart 		= xQueueCreate(QUEUE_USART_LEN,sizeof(struct usart_packet_t));
	g_queue_led   		= xQueueCreate(QUEUE_LED_LEN,sizeof(uint8_t));	
	g_queue_beep  		= xQueueCreate(QUEUE_BEEP_LEN,sizeof(beep_t));
	g_queue_usart6		= xQueueCreate(QUEUE_USART6_LEN,sizeof(struct usart6_packet_t));// LCD
	g_queue_usartlcd 	= xQueueCreate(QUEUE_USARTLCD_LEN,sizeof(struct __lcd_t));
	g_queue_esp8266		= xQueueCreate(QUEUE_MQTT_LEN, sizeof(g_esp8266_rx_buf));
	g_queue_flash 		= xQueueCreate(QUEUE_FLASH_LEN,sizeof(flash_t));

	/* led初始化 */
	led_init();
				
	/* beep初始化 */
	beep_init();
			  
	/* 按键初始化 */
	key_init();		

	/* 温湿度模块初始化-------可取消 */
	dht11_init();
			  
	/* rtc初始化 */
	if(RTC_ReadBackupRegister(RTC_BKP_DR0) != 0x1234)// 用于判断备份寄存器，若值不同则初始化
	{
		rtc_init();
		RTC_WriteBackupRegister(RTC_BKP_DR0,0x1234);// 在备份寄存器中写入0x1234
	}
	else
	{
		rtc_resume_init();
	}
										
	//串口屏初始化-------改成tftlcd
	usart_lcd_init(115200);
	
	/*步进电机初始化-------改成舵机*/
	motor_init();
	
	//初始化MFRC522
	MFRC522_Initializtion();

	/* w25qxx初始化 */
	w25qxx_init();
	
	/* 矩阵键盘初始化 */
	key_board_init();
	
	/* I2C初始化*/
	i2c_init();
	at24c02_write(0, passwd_buf, sizeof(passwd_buf));
	
	g_flash_offset = flash_read_offset((uint32_t *)0x08040000);
	dgb_printf_safe("Total number of flash = %d \r\n",g_flash_offset);

	/* mqtt初始化*/
	while(esp8266_mqtt_init())
	{
		dgb_printf_safe("esp8266_mqtt_init ...");
		delay_ms(1000);
	}
		
	while(SFM_ACK_SUCCESS!=sfm_init(115200))
	{
		vTaskDelay(500);
		printf("电容指纹模块连接中 ...\r\n");		
	}
	
	/* 独立看门狗初始化 */		
	iwdg_init();
	
	/* 创建用到的任务 */
	taskDISABLE_INTERRUPTS();//关闭中断
	i=0;
	while(task_tbl[i].pxTaskCode)
	{
		xTaskCreate(task_tbl[i].pxTaskCode,  	/* 任务入口函数 */
			  		task_tbl[i].pcName,			/* 任务名字 */
			   		task_tbl[i].usStackDepth,  	/* 任务栈大小 */
			  		task_tbl[i].pvParameters,	/* 任务入口函数参数 */
			  		task_tbl[i].uxPriority, 	/* 任务的优先级 */
			  		task_tbl[i].pxCreatedTask);	/* 任务控制块指针 */ 
		i++;				  
	}
	taskENABLE_INTERRUPTS();
			  

	/* 创建周期软件定时器 */
	soft_timer_Handle=xTimerCreate(	(const char*		)"AutoReloadTimer",
									(TickType_t			)1000,/* 定时器周期 1000(tick) */
									(UBaseType_t		)pdTRUE,/* 周期模式 */
									(void*				)1,/* 为每个计时器分配一个索引的唯一ID */
									(TimerCallbackFunction_t)soft_timer_callback); 	
	/* 开启周期软件定时器 */
	xTimerStart(soft_timer_Handle,0);		

	/* 删除任务自身 */
	vTaskDelete(NULL);		  		  
}   

static void app_task_keyboard(void* pvParameters)
{
	char 		key_val	= 'N';
	uint32_t 	cnt 	=  0 ;
	EventBits_t EventValue;
	BaseType_t	xReturn;
	lcd_t lcd;
	uint8_t buf[16]={0};
	uint32_t i=0;
	uint32_t led_sta=0;
	while(1)
	{
		EventValue=xEventGroupWaitBits(g_event_group,EVENT_GROUP_KEYBOARD_ON,pdTRUE,pdFALSE,portMAX_DELAY);
		if(EventValue & EVENT_GROUP_KEYBOARD_ON)
		{	
			led_sta=0x22;
			xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
								&led_sta,		/* 发送的消息内容 */
								100);			/* 等待时间 100 Tick */
			xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_keyboard] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
			
			lcd.ctrl = LCD_CTRL_KEYBOARD;
			sprintf((char *)lcd.buf,"page keyboard\xff\xff\xff");
			
			xReturn = xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
									&lcd,			/* 发送的消息内容 */
									1000);			/* 等待时间 100 Tick */			
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_keyboard] xQueueSend error code is %d\r\n",xReturn);
			
			
			lcd.ctrl = LCD_CTRL_KEYBOARD;
			sprintf((char *)lcd.buf,"keyboard.t0.txt=\"%s\"\xff\xff\xff","请你输入密码");
			
			xReturn = xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
									&lcd,			/* 发送的消息内容 */
									1000);			/* 等待时间 100 Tick */			
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_keyboard] xQueueSend error code is %d\r\n",xReturn);
			
			memset(&lcd,0,sizeof(lcd));
	
			memset(buf,0,sizeof(buf));
			dgb_printf_safe("[app_task_keyboard]\r\n");
			cnt = 0;
			while(1)
			{
				key_val=get_key_board_val();
				if(key_val == '#' || cnt > 9)
				{
					goto getpasswd_end;
				}
				if(key_val != 'N')
				{
					if(key_val == '*')
					{
						if(cnt == 0 )
							cnt = 0;
						else		
						{
							cnt--;	
							memset((void *)(g_input_passward+cnt),0,1);
							
							memset(buf,0,sizeof(buf));
							for(i=0;i<cnt;i++)
							{
								buf[i]='*';
							}
							
							lcd.ctrl = LCD_CTRL_PASSWD;
							sprintf((char *)lcd.buf,"keyboard.t0.txt=\"%s\"\xff\xff\xff",buf);
							xReturn = xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
									&lcd,			/* 发送的消息内容 */
									1000);			/* 等待时间 100 Tick */			
							
							if(xReturn != pdPASS)
								dgb_printf_safe("[app_task_keyboard] xQueueSend error code is %d\r\n",xReturn);
							
							memset(&lcd,0,sizeof(lcd));
						}
					}
					else 
					{
						
						g_input_passward[cnt] = (uint8_t )key_val;
						cnt++;
						
						buf[cnt-1]='*';				
						lcd.ctrl = LCD_CTRL_PASSWD;
						sprintf((char *)lcd.buf,"keyboard.t0.txt=\"%s\"\xff\xff\xff",buf);
						xReturn = xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
								&lcd,			/* 发送的消息内容 */
								1000);			/* 等待时间 100 Tick */			
						
						if(xReturn != pdPASS)
							dgb_printf_safe("[app_task_keyboard] xQueueSend error code is %d\r\n",xReturn);
						
						memset(&lcd,0,sizeof(lcd));
					}		
				}	
			}
getpasswd_end:
			key_val='N';
			dgb_printf_safe("[app_task_keyboard] %s \r\n",g_input_passward);
			
			lcd.ctrl = LCD_CTRL_KEYBOARD;
			sprintf((char *)lcd.buf,"keyboard.t0.txt=\"%s\"\xff\xff\xff","请你输入密码");	
			xReturn = xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
									&lcd,			/* 发送的消息内容 */
									1000);			/* 等待时间 100 Tick */			
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_keyboard] xQueueSend error code is %d\r\n",xReturn);
			memset(&lcd,0,sizeof(lcd));
			
			lcd.ctrl = LCD_CTRL_KEYBOARD;
			sprintf((char *)lcd.buf,"page page0\xff\xff\xff");	
			xReturn = xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
									&lcd,			/* 发送的消息内容 */
									1000);			/* 等待时间 100 Tick */			
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_keyboard] xQueueSend error code is %d\r\n",xReturn);
			memset(&lcd,0,sizeof(lcd));
			
			led_sta=0x20;
			xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
								&led_sta,		/* 发送的消息内容 */
								100);			/* 等待时间 100 Tick */
			xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_keyboard] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
			
			xEventGroupSetBits(g_event_group,EVENT_GROUP_KEYBOARD_PASSWD);
		}
	}
}

static void app_task_at24c02(void* pvParameters)
{
	uint8_t passwd_buf[6]={0};
	int32_t rt=0;
	EventBits_t EventValue;
	beep_t beep;
	uint32_t led_sta=0;
	BaseType_t	xReturn;
	flash_t	openlock_flash;
	while(1)
	{
		EventValue=xEventGroupWaitBits(g_event_group,EVENT_GROUP_KEYBOARD_PASSWD | EVENT_GROUP_MODIFY_PASSWD | EVENT_GROUP_BLUE_PASSWD,pdTRUE,pdFALSE,portMAX_DELAY);
		if(EventValue & EVENT_GROUP_KEYBOARD_PASSWD)
		{	
			at24c02_read(0,passwd_buf,sizeof(passwd_buf));
			rt=passwd_compare((uint8_t *)g_input_passward,passwd_buf);
			if(rt)
			{
				xEventGroupSetBits(g_event_group,EVENT_GROUP_OPENLOCK);
						
				openlock_flash.mode=MODE_OPEN_LOCK_KEYBOARD;
				xQueueSend(g_queue_flash,/* 消息队列的句柄 */
							&openlock_flash,		/* 发送的消息内容 */
							100);		/* 等待时间 100 Tick */
				memset(&openlock_flash,0,sizeof(openlock_flash));
				
				beep.sta=1;
				beep.duration=100;
			}
			else
			{
				led_sta=0xFF;
				xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
							&led_sta,		/* 发送的消息内容 */
							100);			/* 等待时间 100 Tick */
				xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
				
				beep.sta=1;
				beep.duration=1500;
			}
			xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
						&beep,			/* 发送的消息内容 */
						100);			/* 等待时间 100 Tick */

			/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
			xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
			if(xReturn != pdPASS)			
				dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	

			led_sta=0xF0;
			xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
						&led_sta,		/* 发送的消息内容 */
						100);			/* 等待时间 100 Tick */
			xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
			
			dgb_printf_safe("[app_task_at24c02] %d \r\n",rt);
			memset((void *)g_input_passward,0,sizeof g_input_passward);
		}
		if(EventValue & EVENT_GROUP_BLUE_PASSWD)
		{	
			at24c02_read(0,passwd_buf,sizeof(passwd_buf));
			rt=passwd_compare((uint8_t *)g_input_passward,passwd_buf);
			if(rt)
			{
				xEventGroupSetBits(g_event_group,EVENT_GROUP_OPENLOCK);
						
				openlock_flash.mode=MODE_OPEN_LOCK_BLUE;
				xQueueSend(g_queue_flash,/* 消息队列的句柄 */
							&openlock_flash,		/* 发送的消息内容 */
							100);		/* 等待时间 100 Tick */
				memset(&openlock_flash,0,sizeof(openlock_flash));
				
				beep.sta=1;
				beep.duration=100;
			}
			else
			{
				led_sta=0xFF;
				xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
							&led_sta,		/* 发送的消息内容 */
							100);			/* 等待时间 100 Tick */
				xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
				
				beep.sta=1;
				beep.duration=1500;
			}
			xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
						&beep,			/* 发送的消息内容 */
						100);			/* 等待时间 100 Tick */

			/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
			xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
			if(xReturn != pdPASS)			
				dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	

			led_sta=0xF0;
			xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
						&led_sta,		/* 发送的消息内容 */
						100);			/* 等待时间 100 Tick */
			xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
			
			dgb_printf_safe("[app_task_at24c02] %d \r\n",rt);
			memset((void *)g_input_passward,0,sizeof g_input_passward);
		}
		if(EventValue & EVENT_GROUP_MODIFY_PASSWD)
		{	
			if(!at24c02_write(0, (uint8_t *)g_modify_passward, sizeof(g_modify_passward)))
			{
				dgb_printf_safe("[app_task_at24c02] modify sucess %s \r\n",g_modify_passward);
			}
			
			beep.sta=1;
			beep.duration=100;
			xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
						&beep,			/* 发送的消息内容 */
						100);			/* 等待时间 100 Tick */
			/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
			xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
			if(xReturn != pdPASS)			
				dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	
			
			memset((void *)g_modify_passward,0,sizeof(g_modify_passward));
		}
		vTaskDelay(500);
	}
}

static void app_task_key(void* pvParameters)
{
	EventBits_t EventValue;
	BaseType_t xReturn=pdFALSE;
	beep_t beep;
	lcd_t lcd;
	for(;;)
	{
		//等待事件组中的相应事件位，或同步
		EventValue=xEventGroupWaitBits((EventGroupHandle_t	)g_event_group,		
									   (EventBits_t			)EVENT_GROUP_KEYALL_DOWN,
									   (BaseType_t			)pdTRUE,				
									   (BaseType_t			)pdFALSE,
									   (TickType_t			)portMAX_DELAY);
		
		if(EventValue & EVENT_GROUP_KEY1_DOWN)
		{
			//禁止EXTI0触发中断
			NVIC_DisableIRQ(EXTI0_IRQn);
			
			//延时消抖
			vTaskDelay(50);
				
			//确认是按下
			if(PAin(0) == 0)
			{
				xEventGroupSetBits(g_event_group,EVENT_GROUP_KEYBOARD_ON);
				
				beep.sta=1;
				beep.duration=100;
				xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
							&beep,			/* 发送的消息内容 */
							100);			/* 等待时间 100 Tick */
							/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	
				
				
				dgb_printf_safe("[app_task_key] S1 Press\r\n");
			}			
			//等待按键释放
			while(PAin(0)==0)
				vTaskDelay(1);
			
			//允许EXTI0触发中断
			NVIC_EnableIRQ(EXTI0_IRQn);					
		}
		
		if(EventValue & EVENT_GROUP_KEY2_DOWN)
		{
			//禁止EXTI2触发中断
			NVIC_DisableIRQ(EXTI2_IRQn);
			
			//延时消抖
			vTaskDelay(50);
				
			if(PEin(2) == 0)
			{
				lcd.ctrl = LCD_CTRL_SLEEP;
				sprintf((char *)lcd.buf,"%s","sleep=1\xff\xff\xff");
			
				xReturn=xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
								&lcd,			/* 发送的消息内容 */
								100);			/* 等待时间 100 Tick */
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_motor] xQueueSend error code is %d\r\n",xReturn);
				
				dgb_printf_safe("[app_task_key] S2 Press\r\n");
			}				
					
			//等待按键释放
			while(PEin(2)==0)
				vTaskDelay(1);
			
			//允许EXTI2触发中断
			NVIC_EnableIRQ(EXTI2_IRQn);	
		}	
		
		if(EventValue & EVENT_GROUP_KEY3_DOWN)
		{
			//禁止EXTI3触发中断
			NVIC_DisableIRQ(EXTI3_IRQn);
			
			//延时消抖
			vTaskDelay(50);
				
			if(PEin(3) == 0)	
				dgb_printf_safe("[app_task_key] S3 Press\r\n");
			
			//等待按键释放
			while(PEin(3)==0)
				vTaskDelay(1);
			
			//允许EXTI3触发中断
			NVIC_EnableIRQ(EXTI3_IRQn);	
		}
		
		if(EventValue & EVENT_GROUP_KEY4_DOWN)
		{
			//禁止EXTI4触发中断
			NVIC_DisableIRQ(EXTI4_IRQn);
			
			//延时消抖
			vTaskDelay(50);
				
			if(PEin(4) == 0)	
				dgb_printf_safe("[app_task_key] S4 Press\r\n");
			
			//等待按键释放
			while(PEin(4)==0)
				vTaskDelay(1);
			
			//允许EXTI4触发中断
			NVIC_EnableIRQ(EXTI4_IRQn);	
		}			
	}
} 

static void app_task_dht(void* pvParameters)
{
	uint8_t buf[5]={0};
	int32_t rt=0;	
	lcd_t lcd;
	BaseType_t	xReturn;
	for(;;)
	{
		rt = dht11_read_data(buf);
				
		if(rt == 0)
		{
			g_temp = (float)buf[2]+(float)buf[3]/10;
			g_humi = (float)buf[0]+(float)buf[1]/10;
			
			dgb_printf_safe("[app_task_dht] T:%d.%d,H:%d.%d\r\n",buf[2],buf[3],buf[0],buf[1]);
			lcd.ctrl = LCD_CTRL_TEMP;
			sprintf((char *)lcd.buf,"page0.t2.txt=\"%d.%d\"\xff\xff\xff",buf[2],buf[3]);
			
			xReturn = xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
									&lcd,			/* 发送的消息内容 */
									1000);			/* 等待时间 100 Tick */
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_dht] xQueueSend error code is %d\r\n",xReturn);
			memset(&lcd,0,sizeof(lcd));
			
			lcd.ctrl = LCD_CTRL_HUMI;
			sprintf((char *)lcd.buf,"page0.t4.txt=\"%d.%d\"\xff\xff\xff",buf[0],buf[1]);
			
			xReturn = xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
									&lcd,			/* 发送的消息内容 */
									1000);			/* 等待时间 100 Tick */
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_dht] xQueueSend error code is %d\r\n",xReturn);
			memset(&lcd,0,sizeof(lcd));
		}	
		else
			dgb_printf_safe("[app_task_dht] dht11 error code = %d\r\n",rt);
		  
		vTaskDelay(6000);
	}
}


static void app_task_usart(void* pvParameters)
{
	struct usart_packet_t usart_packet={0};	
	uint32_t i=0;	
	char *p = NULL;
	RTC_DateTypeDef RTC_DateStructure;
	RTC_TimeTypeDef RTC_TimeStructure;		
	BaseType_t xReturn=pdFALSE;
	beep_t beep;	
	for(;;)
	{
		xReturn=xQueueReceive(g_queue_usart,/* 消息队列的句柄 */
							&usart_packet, /* 得到的消息内容 */
							portMAX_DELAY);/* 等待时间一直等 */
		if(xReturn != pdPASS)
		{
			dgb_printf_safe("[app_task_usart] xQueueReceive usart_packet error code is %d\r\n",xReturn);
			continue;
		}		
		
		dgb_printf_safe("[app_task_usart] xQueueReceive usart_packet %s\r\n",usart_packet.rx_buf);
			
		//设置时间 TIME SET-14-13-50#
		if(strstr((char *)usart_packet.rx_buf,"TIME SET"))
		{
			p = (char *)usart_packet.rx_buf;
			
			//以等号分割字符串
			strtok((char *)p,"-");
			
			//获取时
			p=strtok(NULL,"-");
			i = atoi(p);
			
			
			//通过时，判断是AM还是PM
			if(i<12)
				RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
			else
				RTC_TimeStructure.RTC_H12     = RTC_H12_PM;
				
			//转换为BCD编码
			i= (i/10)*16+i%10;
			RTC_TimeStructure.RTC_Hours   = i;
			
			//获取分
			p=strtok(NULL,"-");
			i = atoi(p);	
			
			//转换为BCD编码
			i= (i/10)*16+i%10;	
			RTC_TimeStructure.RTC_Minutes = i;
			
			//获取秒
			p=strtok(NULL,"-");
			i = atoi(p);	
			
			//转换为BCD编码
			i= (i/10)*16+i%10;					
			RTC_TimeStructure.RTC_Seconds = i; 	
			
			//设置RTC时间
			RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);
						
			dgb_printf_safe("[app_task_usart] rtc set time ok\r\n");				
		}
		//设置时间 DATE SET-2022-12-14-2#
		else if(strstr((char *)usart_packet.rx_buf,"DATE SET"))
		{
			/* 以等号分割字符串 */
			p=strtok((char *)usart_packet.rx_buf,"-");
		
			//获取年
			p=strtok(NULL,"-");

			//2021-2000=21 
			i = atoi(p)-2000;

			//转换为16进制 21 ->0x21
			i= (i/10)*16+i%10;
			RTC_DateStructure.RTC_Year = i;
			
			//获取月
			p=strtok(NULL,"-");

			i=atoi(p);
			//转换为16进制
			i= (i/10)*16+i%10;						
			RTC_DateStructure.RTC_Month=i;


			//获取日
			p=strtok(NULL,"-");

			i=atoi(p);
			//转换为16进制
			i= (i/10)*16+i%10;		
			RTC_DateStructure.RTC_Date = i;
			
			//获取星期
			p=strtok(NULL,"-");

			i=atoi(p);
			//转换为16进制
			i= (i/10)*16+i%10;						
			RTC_DateStructure.RTC_WeekDay = i;
			
			//设置日期
			RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure);
		}			
		else if(strstr((char *)usart_packet.rx_buf,"ADMIN-"))
		{
			//添加指纹ADMIN-1#
			if(usart_packet.rx_buf[6] == '1')
			{
				xEventGroupSetBits(g_event_group,EVENT_GROUP_ADD_SFM);			
			}
			//删除所有指纹ADMIN-2#
			if(usart_packet.rx_buf[6] == '2')
			{
				xEventGroupSetBits(g_event_group,EVENT_GROUP_DELETEALL_SFM);
			}
			//添加rfid   ADMIN-3#
			if(usart_packet.rx_buf[6] == '3')
			{
				xEventGroupSetBits(g_event_group,EVENT_GROUP_ADD_NFC);
			}	
			//删除所有rfid    ADMIN-4#
			if(usart_packet.rx_buf[6] == '4')
			{
				xEventGroupSetBits(g_event_group,EVENT_GROUP_DELETEALL_NFC);
			}
			//修改密码 ADMIN-5-000000#
			if(usart_packet.rx_buf[6] == '5')
			{
				xEventGroupSetBits(g_event_group,EVENT_GROUP_MODIFY_PASSWD);
				for(i=0;i<6;i++)
				{
					g_modify_passward[i]=usart_packet.rx_buf[i+8];
				}
			}
			//蓝牙开锁 ADMIN-6-000000#
			if(usart_packet.rx_buf[6] == '6')
			{
				xEventGroupSetBits(g_event_group,EVENT_GROUP_BLUE_PASSWD);
				for(i=0;i<6;i++)
				{
					g_input_passward[i]=usart_packet.rx_buf[i+8];
				}					
			}
			//获取所有记录 ADMIN-7#
			if(usart_packet.rx_buf[6] == '7')
			{
				flash_read(g_flash_offset);	
				dgb_printf_safe("[flash_read]ok\r\n");
			}
			//删除所有记录 ADMIN-8#
			if(usart_packet.rx_buf[6] == '8')
			{
				flash_erase_sector(FLASH_Sector_6);
				g_flash_offset=flash_read_offset((uint32_t *)0x08040000);
				dgb_printf_safe("[flash_erase_sector]ok\r\n");
			}			
		}
		else
		{
			dgb_printf_safe("[app_task_usart] usart invalid command\r\n");	
		}
		beep.sta=1;
		beep.duration=100;
		xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
					&beep,			/* 发送的消息内容 */
					100);			/* 等待时间 100 Tick */
					/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
		xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
		if(xReturn != pdPASS)
		dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	

		memset(&usart_packet,0,sizeof usart_packet);	
	}
}

static void app_task_flash(void *pvParameters)
{
	flash_t flash_write_buf;
	BaseType_t 	xReturn	=pdFALSE;
	RTC_DateTypeDef  RTC_Date;
	RTC_TimeTypeDef  RTC_Time;
	while(1)
	{
		xReturn = xQueueReceive(g_queue_flash,     		/* 消息队列的句柄 */
								&flash_write_buf,      		/* 接收的消息内容 */
								portMAX_DELAY); 		/* 等待时间一直等 */
		if(xReturn != pdPASS)
			continue;

		dgb_printf_safe("wait get data\r\n");
			
		flash_write_buf.offset=g_flash_offset;
		
		RTC_GetDate(RTC_Format_BCD, &RTC_Date); 		
		RTC_GetTime(RTC_Format_BCD, &RTC_Time); 
		
		dgb_printf_safe("20%02x/%02x/%02x %02x:%02x:%02x %d unlock\n",
		RTC_Date.RTC_Year,
		RTC_Date.RTC_Month,
		RTC_Date.RTC_Date,
		RTC_Time.RTC_Hours,
		RTC_Time.RTC_Minutes,
		RTC_Time.RTC_Seconds,
		flash_write_buf.mode);	
		
		flash_write_buf.datebuf[0]=RTC_Date.RTC_Year;
		flash_write_buf.datebuf[1]=RTC_Date.RTC_Month;
		flash_write_buf.datebuf[2]=RTC_Date.RTC_Date;
		flash_write_buf.datebuf[3]=RTC_Time.RTC_Hours;
		flash_write_buf.datebuf[4]=RTC_Time.RTC_Minutes;
		flash_write_buf.datebuf[5]=RTC_Time.RTC_Seconds;
		
		taskENTER_CRITICAL();
		flash_write(&flash_write_buf);
		taskEXIT_CRITICAL();		
		g_flash_offset++;
	}
}


static void app_task_rtc(void* pvParameters)
{
	char 			buf[16]={0};			
	EventBits_t 	EventValue;
	RTC_DateTypeDef RTC_DateStructure;
	RTC_TimeTypeDef RTC_TimeStructure;		
	BaseType_t xReturn=pdFALSE;	
	lcd_t lcd;
	uint32_t t=0;
	for(;;)
	{
		//等待事件组中的相应事件位，或同步
		EventValue=xEventGroupWaitBits((EventGroupHandle_t	)g_event_group,		
									   (EventBits_t			)EVENT_GROUP_RTC_WAKEUP,
									   (BaseType_t			)pdTRUE,				
									   (BaseType_t			)pdFALSE,
									   (TickType_t			)portMAX_DELAY);
		
		if(EventValue & EVENT_GROUP_RTC_WAKEUP)
		{
			t++;
			//dgb_printf_safe("[app_task_rtc] %s\r\n",buf);			
		}
		if(t>15)
		{
			t=0;
			RTC_GetDate(RTC_Format_BCD, &RTC_DateStructure); 		

			//RTC_GetTime，获取时间
			RTC_GetTime(RTC_Format_BCD, &RTC_TimeStructure); 
				
			//格式化字符串
//			sprintf(buf,"20%02x/%02x/%02x %02x:%02x:%02x",RTC_DateStructure.RTC_Year,RTC_DateStructure.RTC_Month,RTC_DateStructure.RTC_Date,
//			RTC_TimeStructure.RTC_Hours,RTC_TimeStructure.RTC_Minutes,RTC_TimeStructure.RTC_Seconds);
			sprintf(buf,"20%02x/%02x/%02x %02x:%02x",RTC_DateStructure.RTC_Year,RTC_DateStructure.RTC_Month,RTC_DateStructure.RTC_Date,
			RTC_TimeStructure.RTC_Hours,RTC_TimeStructure.RTC_Minutes);					
			
			lcd.ctrl = LCD_CTRL_TIME;
			sprintf((char *)lcd.buf,"page0.t0.txt=\"%s\"\xff\xff\xff",buf);
					
			xReturn = xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
								&lcd,			/* 发送的消息内容 */
								100);			/* 等待时间 100 Tick */
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_rtc] xQueueSend error code is %d\r\n",xReturn);
			
			memset(&lcd,0,sizeof(lcd));
		}
	}
}   

static void app_task_led(void* pvParameters)
{
	uint8_t led_sta=0;	
	BaseType_t xReturn=pdFALSE;	
	
	for(;;)
	{
		xReturn = xQueueReceive( g_queue_led,	/* 消息队列的句柄 */
								&led_sta, 		/* 得到的消息内容 */
								portMAX_DELAY);	/* 等待时间一直等 */
		
		if(xReturn != pdPASS)
			continue;
		
		//检测到控制LED1
		if(led_sta & 0x10)
		{
			if(led_sta & 0x01)
				PFout(9)=0;
			else
				PFout(9)=1;				
		}
		
		//检测到控制LED2
		if(led_sta & 0x20)
		{			
			if(led_sta & 0x02)
				PFout(10)=0;
			else
				PFout(10)=1;				
		}
		
		//检测到控制LED3
		if(led_sta & 0x40)
		{			
			if(led_sta & 0x04)
				PEout(13)=0;
			else
				PEout(13)=1;
		}
		
		//检测到控制LED4
		if(led_sta & 0x80)
		{	
			
			if(led_sta & 0x08)
				PEout(14)=0;
			else
				PEout(14)=1;
		}
		
					
		/* 释放信号量，告诉对方，当前led控制任务已经完成 */
		xSemaphoreGive(g_sem_led);
	}
}   


static void app_task_beep(void* pvParameters)
{
	beep_t beep;	
	BaseType_t xReturn=pdFALSE;	
	for(;;)
	{
		xReturn = xQueueReceive(g_queue_beep,	/* 消息队列的句柄 */
								&beep, 			/* 得到的消息内容 */
								portMAX_DELAY);	/* 等待时间一直等 */
		if(xReturn != pdPASS)
			continue;

		//dgb_printf_safe("[app_task_beep] beep.sta=%d beep.duration=%d\r\n",beep.sta,beep.duration);

		if(beep.duration)
		{
			BEEP(beep.sta);

			while(beep.duration--)
				vTaskDelay(1);

			beep.sta ^=1;
		}
		BEEP(beep.sta);

		xSemaphoreGive(g_sem_beep);		
	}
}



static void app_task_usartlcd(void* pvParameters)
{
	BaseType_t xReturn=pdFALSE;	
	lcd_t lcd;	
	while(1)
	{
		xReturn = xQueueReceive(g_queue_usartlcd, /* 消息队列的句柄 */
								&lcd, 			  /* 得到的消息内容 */
								portMAX_DELAY);   /* 等待时间一直等 */
		if(xReturn != pdPASS)
			continue;
		
		usart_send_bytes(USART6,lcd.buf,strlen(lcd.buf));
		
//		if(lcd.ctrl == LCD_CTRL_LOCK_ON)
//		{
//			usart_send_bytes(USART6,lcd.buf,strlen(lcd.buf));
//			//dgb_printf_safe("[LCD_CTRL_LOCK_ON ] %s\r\n",lcd.buf);
//		}
//		if(lcd.ctrl == LCD_CTRL_LOCK_OFF)
//		{
//			usart_send_bytes(USART6,lcd.buf,strlen(lcd.buf));
//			//dgb_printf_safe("[LCD_CTRL_LOCK_OFF ] %s\r\n",lcd.buf);
//		}
//		if(lcd.ctrl == LCD_CTRL_TIME)
//		{
//			usart_send_bytes(USART6,lcd.buf,strlen(lcd.buf));		
//			//dgb_printf_safe("[LCD_CTRL_TIME] %s\r\n",lcd.buf);
//		}
//		if(lcd.ctrl == LCD_CTRL_TEMP)
//		{
//			usart_send_bytes(USART6,lcd.buf,strlen(lcd.buf));		
//			//dgb_printf_safe("[LCD_CTRL_TEMP] %s\r\n",lcd.buf);
//		}
//		if(lcd.ctrl == LCD_CTRL_HUMI)
//		{
//			usart_send_bytes(USART6,lcd.buf,strlen(lcd.buf));		
//			//dgb_printf_safe("[LCD_CTRL_HUMI] %s\r\n",lcd.buf);
//		}
//		if(lcd.ctrl == LCD_CTRL_KEYBOARD)
//		{
//			usart_send_bytes(USART6,lcd.buf,strlen(lcd.buf));		
//			//dgb_printf_safe("[LCD_CTRL_HUMI] %s\r\n",lcd.buf);
//		}
//		if(lcd.ctrl == LCD_CTRL_PASSWD)
//		{
//			usart_send_bytes(USART6,lcd.buf,strlen(lcd.buf));		
//			//dgb_printf_safe("[LCD_CTRL_HUMI] %s\r\n",lcd.buf);
//		}
  
		memset(&lcd,0,sizeof(lcd));
		vTaskDelay(1000);
	}
}


/*串口屏接收数据任务*/
static void app_task_recvlcd(void* pvParameters)
{
	uint32_t i=0;
	uint8_t buf[64]={0};//lcd-命令-#
	BaseType_t xReturn=pdFALSE;	
	beep_t beep;
	for(;;)
	{
		xReturn = xQueueReceive(g_queue_usart6,&buf,portMAX_DELAY);		
		if(xReturn == pdTRUE)
		{
			dgb_printf_safe("[app_task_lcd] %s\r\n",buf);
			if(buf[4] == '0')
			{
				//添加指纹
				xEventGroupSetBits(g_event_group,EVENT_GROUP_ADD_SFM);
			}		
			if(buf[4] == '1')
			{
				//删除所有指纹
				xEventGroupSetBits(g_event_group,EVENT_GROUP_DELETEALL_SFM);
			}
			if(buf[4] == '2')
			{
				//添加RFID
				xEventGroupSetBits(g_event_group,EVENT_GROUP_ADD_NFC);
				memset(buf,0,sizeof(buf));
			}
			if(buf[4] == '3')
			{
				//删除RFID
				xEventGroupSetBits(g_event_group,EVENT_GROUP_DELETEALL_NFC);
			}
			if(buf[4] == '4')
			{
				//修改密码
				xEventGroupSetBits(g_event_group,EVENT_GROUP_MODIFY_PASSWD);
				for(i=0;i<6;i++)
				{
					g_modify_passward[i]=buf[i+6];
				}
			}
			if(buf[4] == '5')
			{
				//密码解锁
				xEventGroupSetBits(g_event_group,EVENT_GROUP_KEYBOARD_PASSWD);
				for(i=0;i<6;i++)
				{
					g_input_passward[i]=buf[i+6];
				}			
			}
			
			beep.sta=1;
			beep.duration=100;
			xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
						&beep,			/* 发送的消息内容 */
						100);			/* 等待时间 100 Tick */
						/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
			xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	

			memset(buf,0,sizeof(buf));
		}
	}
}  

void app_task_mfrc522real(void *pvParameters)
{
	uint32_t t=0;
	int32_t rt=0;
	uint32_t i=0;
	beep_t beep;
	uint32_t led_sta=0;
	BaseType_t xReturn=pdFALSE;	
	uint32_t sta=STA_CARD_REQUEST;
	uint8_t  card_type[2];
	uint8_t  card_id[5]={0};
	uint8_t  card_key0Abuf[6]={0xff,0xff,0xff,0xff,0xff,0xff};
	uint8_t  card_writebuf[16]={0};
	uint8_t  card_readbuf[18]={0};	
	
	RTC_DateTypeDef  RTC_DateStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;	
	flash_t openlock_flash;
	
	while(1)
	{	
		delay_ms(1);
		if(t++ > 1000)
			t=0;
		if(sta == STA_CARD_REQUEST)
		{
			/* 每500ms寻卡一次 */
			if(t==500)
			{
				MFRC522_Initializtion();
	
				if(MI_OK == MFRC522_Request(0x52, card_type))
				{
					/* 状态变更为找到卡 */
					sta = STA_CARD_FOUND;
					
					dgb_printf_safe("[NOTICE] Card Request success,card type is %02X %02X\r\n",card_type[0],card_type[1]);
					
					continue;
				
				}
			}
		}
		
		/* 检测状态是否找到卡 */
		if(sta == STA_CARD_FOUND)
		{
			if(MI_OK == MFRC522_Anticoll(card_id))
			{
				/* 打开找到的卡 */
				dgb_printf_safe("[NOTICE] Card %02X%02X%02X%02X found\r\n",card_id[0],card_id[1],card_id[2],card_id[3]);
				
				/* 先查询spi flash有没有存在该卡 */
				rt = spi_flash_card_match(card_id);
				
				if(rt < 0)
					/* 识别到无效卡 */
					sta = STA_CARD_INVALID_FOUND;
				else
				{
					xEventGroupSetBits(g_event_group,EVENT_GROUP_OPENLOCK);
					
					openlock_flash.mode=MODE_OPEN_LOCK_RFID;
					xQueueSend(g_queue_flash,/* 消息队列的句柄 */
								&openlock_flash,		/* 发送的消息内容 */
								100);		/* 等待时间 100 Tick */
					memset(&openlock_flash,0,sizeof(openlock_flash));
					
					/* 识别到有效卡 */
					sta = STA_CARD_VALID_FOUND;
				}
			}
		}
	
		if(sta == STA_CARD_VALID_FOUND)
		{			
			/* 更新该卡在spi flash中识别的次数 */
			spi_flash_card_times(card_id);
			
			MFRC522_SelectTag(card_id);
			
			/* 验卡 */
			if(MI_OK!=MFRC522_Auth(0x60, 4, card_key0Abuf, card_id))
			{
				dgb_printf_safe("[ERROR] Card %02X%02X%02X%02X Auth fail\r\n",card_id[0],card_id[1],card_id[2],card_id[3]);
				
				/* 状态变更为寻卡状态 */
				sta = STA_CARD_REQUEST;		
				
				continue;
			}				
			
			/* 读卡 */
			if(MI_OK!=MFRC522_Read(4, card_readbuf))
			{
				dgb_printf_safe("[ERROR] Card %02X%02X%02X%02X read data block 4 fail\r\n",card_id[0],card_id[1],card_id[2],card_id[3]);				
				
				/* 状态变更为寻卡状态 */
				sta = STA_CARD_REQUEST;		
				
				continue;				
			}

			if(card_readbuf[0]=='@')
			{
				/* 打印出卡上一次存储RTC的日期时间 */
				printf("[NOTICE] Card %02X%02X%02X%02X %04X/%02X/%02X Week %x %02X:%02X:%02X last\r\n",
				card_id[0],card_id[1],card_id[2],card_id[3],
				card_readbuf[1]+0x2000,card_readbuf[2],card_readbuf[3],card_readbuf[4],
				card_readbuf[5],card_readbuf[6],card_readbuf[7]);
			}
			else
			{
				dgb_printf_safe("[NOTICE] Card %02X%02X%02X%02X data block 4 is not recorded\r\n",card_id[0],card_id[1],card_id[2],card_id[3]);
			}
				
			/* RTC_GetTime，获取时间 */
			RTC_GetTime(RTC_Format_BCD,&RTC_TimeStructure);
			
			/* RTC_GetDate，获取日期 */
			RTC_GetDate(RTC_Format_BCD,&RTC_DateStructure);
			
			/* 设置要向卡写入的数据格式 */
			card_writebuf[0]='@';
			card_writebuf[1]=RTC_DateStructure.RTC_Year;
			card_writebuf[2]=RTC_DateStructure.RTC_Month;
			card_writebuf[3]=RTC_DateStructure.RTC_Date;
			card_writebuf[4]=RTC_DateStructure.RTC_WeekDay;
			card_writebuf[5]=RTC_TimeStructure.RTC_Hours;
			card_writebuf[6]=RTC_TimeStructure.RTC_Minutes;
			card_writebuf[7]=RTC_TimeStructure.RTC_Seconds;	
			
			/* 写卡 */
			if(MI_OK == MFRC522_Write(4, card_writebuf))
			{
				/* printf打印出日期和时间 */		
				printf("Card %02X%02X%02X%02X  20%02X/%X/%X Week %X %02X:%02X:%02X now\r\n",
				card_id[0],card_id[1],card_id[2],card_id[3],
				RTC_DateStructure.RTC_Year,
				RTC_DateStructure.RTC_Month,
				RTC_DateStructure.RTC_Date,
				RTC_DateStructure.RTC_WeekDay,
				RTC_TimeStructure.RTC_Hours,
				RTC_TimeStructure.RTC_Minutes,
				RTC_TimeStructure.RTC_Seconds);			
			
				/* 蜂鸣器嘀一声示意 */
				beep.sta=1;
				beep.duration=100;
				xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
							&beep,			/* 发送的消息内容 */
							100);			/* 等待时间 100 Tick */
							/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
				if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	
			}
			else
			{
				/* 蜂鸣器嘀两声示意 */
				for(i=0;i<2;i++)
				{
					beep.sta=1;
					beep.duration=100;
					xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
						&beep,			/* 发送的消息内容 */
						100);			/* 等待时间 100 Tick */
						/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
					xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
					if(xReturn != pdPASS)
						dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	
				}			
			}

			/* 一直检测卡是否已经离开 */		
			while(1)
			{
				
				if(MI_OK != MFRC522_Read(4,card_readbuf))
					break;
	
				delay_ms(100);
			}
			
			dgb_printf_safe("[NOTICE ] Card has left\r\n");
					
			/* 状态变更为寻卡状态 */
			sta = STA_CARD_REQUEST;			
		}

		if(sta == STA_CARD_INVALID_FOUND)
		{
			led_sta=0xFF;
			xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
						&led_sta,		/* 发送的消息内容 */
						100);			/* 等待时间 100 Tick */
			xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
			
			/* 蜂鸣器响 */				
			beep.sta=1;
			beep.duration=2000;
			xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
						&beep,			/* 发送的消息内容 */
						100);			/* 等待时间 100 Tick */
						/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
			xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	
			
			dgb_printf_safe("Card %02X%02X%02X%02X is invalid\r\n",card_id[0],card_id[1],card_id[2],card_id[3]);

			/* 一直检测卡是否已经离开 */		
			while(1)
			{
				
				if(MI_OK != MFRC522_Anticoll(card_id))
					break;
					
				delay_ms(100);
			}
			
			led_sta=0xF0;
			xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
						&led_sta,		/* 发送的消息内容 */
						100);			/* 等待时间 100 Tick */
			xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
			
			dgb_printf_safe("[NOTICE ] Card has left\r\n");
											
			/* 状态变更为寻卡状态 */
			sta = STA_CARD_REQUEST;	
		}		
	}
}

void app_task_mfrc522admin(void *pvParameters)
{
	uint32_t t=0;
	int32_t rt=0;
	uint8_t  card_type[2];
	uint8_t  card_id[5]={0};
	BaseType_t xReturn=pdFALSE;
	beep_t beep;	
	EventBits_t EventValue;
	while(1)
	{
		EventValue=xEventGroupWaitBits(g_event_group,
										EVENT_GROUP_ADD_NFC | EVENT_GROUP_DELETEALL_NFC,
										pdTRUE,pdFALSE,
										portMAX_DELAY);
		if(EventValue & EVENT_GROUP_ADD_NFC)
		{
			vTaskSuspend(app_task_mfrc522real_handle);
			dgb_printf_safe("add nfc...\r\n");
			t=0;
			while(1)
			{
				t++;
				/* 卡初始化 */
				MFRC522_Initializtion();
				if(MI_OK == MFRC522_Request(0x52, card_type))
				{
					if(MI_OK == MFRC522_Anticoll(card_id))
					{
						/* 先查询spi flash有没有存在该卡 */
						rt = spi_flash_card_match(card_id);
						
						/* spi flash没有存储该卡 */
						if(rt < 0)
						{
							/* 添加卡 */
							rt = spi_flash_card_add(card_id);
							
							if(rt == 0)
							{
								beep.sta=1;
								beep.duration=100;
								xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
											&beep,			/* 发送的消息内容 */
											100);			/* 等待时间 100 Tick */
											/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
								xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
								if(xReturn != pdPASS)
									dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	
								
								dgb_printf_safe("[NOTICE] Spi flash add card %02X%02X%02X%02X success\r\n",card_id[0],card_id[1],card_id[2],card_id[3]);
								goto mfc_end;
							}
							else
							{
								dgb_printf_safe("[ERROR] Spi flash add card %02X%02X%02X%02X fail\r\n",card_id[0],card_id[1],card_id[2],card_id[3]);
								goto mfc_end;
							}
						}
						/* spi flash 已存储该卡 */
						else
						{
							dgb_printf_safe("[NOTICE] Spi flash card %02X%02X%02X%02X alread exist\r\n",card_id[0],card_id[1],card_id[2],card_id[3]);
							goto mfc_end;
						}
					}
				}
				if(t>10)
				{
					goto mfc_end;
				}
				vTaskDelay(500);
			}
mfc_end:
			vTaskResume(app_task_mfrc522real_handle);
		}
		if(EventValue & EVENT_GROUP_DELETEALL_NFC)
		{
			/* 清空spi flash存储的有效卡记录 */
			spi_flash_card_remove_all();
			
			dgb_printf_safe("[NOTICE] All cards stored in spi flash have been removed\r\n");
		}
		vTaskDelay(1000);
	}
	
}

static void app_task_sfmreal(void* pvParameters)
{
	int32_t rt;
	uint16_t id	=1;
	flash_t	openlock_flash;
	beep_t beep;
	BaseType_t xReturn=pdFALSE;
	uint32_t led_sta=0;
	while(1)
	{
		vTaskDelay(1000);
		sfm_ctrl_led(0x07,0x07,0x60);
		if(sfm_touch_check()==SFM_ACK_SUCCESS)
		{			
			/* 显示蓝色 */
			sfm_ctrl_led(0x06,0x06,0x32);
			dgb_printf_safe("We've got a finger. Now we're brushing...\r\n");	
			rt=sfm_compare_users(&id);
			if(rt == SFM_ACK_SUCCESS)//匹对成功   ->    解锁
			{			
				openlock_flash.mode=MODE_OPEN_LOCK_SFM;
				xQueueSend(g_queue_flash,/* 消息队列的句柄 */
							&openlock_flash,		/* 发送的消息内容 */
							100);		/* 等待时间 100 Tick */
				memset(&openlock_flash,0,sizeof(openlock_flash));
				
				xEventGroupSetBits(g_event_group,EVENT_GROUP_OPENLOCK);
				
				beep.sta=1;
				beep.duration=100;
				xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
							&beep,			/* 发送的消息内容 */
							100);			/* 等待时间 100 Tick */
							/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	
							
				dgb_printf_safe("okokokokok\r\n",sfm_error_code(rt),id);
				sfm_ctrl_led(0x05,0x05,0x32);/* 成功:光圈显示绿色 */
			}
			else
			{
				led_sta=0xFF;
				xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
							&led_sta,		/* 发送的消息内容 */
							100);			/* 等待时间 100 Tick */
				xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
				
				beep.sta=1;
				beep.duration=1500;
				xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
							&beep,			/* 发送的消息内容 */
							100);			/* 等待时间 100 Tick */
							/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	
				
				led_sta=0xF0;
				xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
							&led_sta,		/* 发送的消息内容 */
							100);			/* 等待时间 100 Tick */
				xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
				
				dgb_printf_safe("flase\r\n",sfm_error_code(rt),id);
				sfm_ctrl_led(0x03,0x03,0x32);/* 失败:光圈显示红色 */		
			}
			dgb_printf_safe("Brush a fingerprint %s Identify withid=%d\r\n",sfm_error_code(rt),id);/* 若id为0，则比对不成功的*/
			sfm_ctrl_led(0x00,0x07,0xC8);/* 恢复光圈全亮->全灭，周期2秒 */
		}	
	}
}


static void app_task_sfmadmin(void* pvParameters)
{
	BaseType_t xReturn=pdFALSE;
	EventBits_t EventValue;
	uint32_t timeout=0;
	int32_t rt;
	uint16_t id=1;
	beep_t beep;
	for(;;)
	{
		EventValue=xEventGroupWaitBits(g_event_group,
										EVENT_GROUP_ADD_SFM | EVENT_GROUP_DELETEALL_SFM,
										pdTRUE,pdFALSE,
										portMAX_DELAY);
		
		if(EventValue & EVENT_GROUP_ADD_SFM)
		{
			vTaskSuspend(app_task_sfmreal_handle);
			while(1)
			{
				timeout=0;
				sfm_ctrl_led(0x06,0x06,0x32);/* 显示蓝色 */
				while((sfm_touch_check()!=SFM_ACK_SUCCESS) && (timeout<5))
					timeout++;
				if(timeout>=5)
				{
					dgb_printf_safe("No finger detected, exit operation!\r\n");
					sfm_ctrl_led(0x00,0x07,0xC8);/* 恢复光圈全亮->全灭，周期2秒 */
					goto sfm_error;
				}
				else
				{
					dgb_printf_safe("Fingers detected. Now add prints...\r\n");
					break;
				}
			}
			rt = sfm_get_unused_id(&id);/* 获取未使用的用户id */
			if(rt != SFM_ACK_SUCCESS)
			{	
				dgb_printf_safe("获取未使用的用户id %s\r\n",sfm_error_code(rt));
				sfm_ctrl_led(0x00,0x07,0xC8);	/* 恢复光圈全亮->全灭，周期2秒 */			
				goto sfm_error;
			}
			dgb_printf_safe("get user id: %d\r\n",id);	
			rt=sfm_reg_user(id);
			if(rt == SFM_ACK_SUCCESS)
			{
				beep.sta=1;
				beep.duration=100;
				xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
							&beep,			/* 发送的消息内容 */
							100);			/* 等待时间 100 Tick */

				/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
				
				sfm_ctrl_led(0x05,0x05,0x32);	/* 成功:光圈显示绿色 */		
			}
			else
			{
				beep.sta=1;
				beep.duration=1000;
				xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
							&beep,			/* 发送的消息内容 */
							100);			/* 等待时间 100 Tick */

				/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
				
				sfm_ctrl_led(0x03,0x03,0x32);/* 失败:光圈显示红色 */	
			}
			sfm_ctrl_led(0x00,0x07,0xC8);	/* 恢复光圈全亮->全灭，周期2秒 */
sfm_error:
			if(timeout>=5)
				dgb_printf_safe("add sfm error!\r\n");	
			vTaskResume(app_task_sfmreal_handle);
			vTaskDelay(1000);
		}
		if(EventValue & EVENT_GROUP_DELETEALL_SFM)
		{	
			rt = sfm_del_user_all();
			
			if(rt == SFM_ACK_SUCCESS)
			{
				dgb_printf_safe("delete all sfm sucess!\r\n");			
			}
			vTaskDelay(1000);
		}
	}
}


static void app_task_motor(void* pvParameters)
{
	BaseType_t xReturn=pdFALSE;
	EventBits_t EventValue;
	lcd_t lcd;
	uint32_t led_sta=0;
	while(1)
	{
		EventValue=xEventGroupWaitBits(g_event_group,
										EVENT_GROUP_OPENLOCK,
										pdTRUE,
										pdFALSE,
										portMAX_DELAY);
		if(EventValue & EVENT_GROUP_OPENLOCK)
		{
			lcd.ctrl = LCD_CTRL_LOCK_ON;
			sprintf((char *)lcd.buf,"%s","page0.t6.pic=6\xff\xff\xff");
			
			xReturn = xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
								&lcd,			/* 发送的消息内容 */
								100);			/* 等待时间 100 Tick */
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_motor] xQueueSend error code is %d\r\n",xReturn);	

			led_sta=0x11;
			xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
								&led_sta,		/* 发送的消息内容 */
								100);			/* 等待时间 100 Tick */
			xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
			
			motor_corotation_open();
			dgb_printf_safe("Open the lock......\n");
						
			vTaskDelay(3000);
			
			memset(&lcd,0,sizeof(lcd));
			
			lcd.ctrl = LCD_CTRL_LOCK_OFF;
			sprintf((char *)lcd.buf,"%s","page0.t6.pic=4\xff\xff\xff");
			
			xReturn = xQueueSend(g_queue_usartlcd,	/* 消息队列的句柄 */
								&lcd,			/* 发送的消息内容 */
								100);			/* 等待时间 100 Tick */
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_motor] xQueueSend error code is %d\r\n",xReturn);	
			
			motor_corotation_close();
			dgb_printf_safe("Close the lock......\n");
			
			
			led_sta=0x10;
			xReturn = xQueueSend(g_queue_led,	/* 消息队列的句柄 */
								&led_sta,		/* 发送的消息内容 */
								100);			/* 等待时间 100 Tick */
			xReturn=xSemaphoreTake(g_sem_led,portMAX_DELAY);
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_motor] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);
			
			memset(&lcd,0,sizeof(lcd));
		}
	}
}

static void app_task_mqttmonitor(void* pvParameters)
{
	uint32_t esp8266_rx_cnt=0;
	
	BaseType_t xReturn = pdFALSE;	
	
	dgb_printf_safe("app_task_monitor create success \r\n");
	
	for(;;)
	{	
		esp8266_rx_cnt = g_esp8266_rx_cnt;
		
		vTaskDelay(50);
		
		/* n毫秒后，发现g_esp8266_rx_cnt没有变化，则认为接收数据结束 */
		if(esp8266_rx_cnt && (esp8266_rx_cnt == g_esp8266_rx_cnt))
		{		
			/* 发送消息，如果队列满了，超时时间为1000个节拍，如果1000个节拍都发送失败，函数直接返回 */
			xReturn = xQueueSend(g_queue_esp8266,(void *)g_esp8266_rx_buf,1000);		
			
			if (xReturn != pdPASS)
				dgb_printf_safe("[app_task_monitor] xQueueSend g_queue_esp8266 error code is %d\r\n", xReturn);
			
			g_esp8266_rx_cnt=0;
			memset((void *)g_esp8266_rx_buf,0,sizeof(g_esp8266_rx_buf));
		}	
	}
}

void app_task_mqtt(void *pvParameters)
{
	uint8_t buf[512];
	BaseType_t xReturn = pdFALSE;	
	uint32_t i;
	flash_t openlock_flash;
	beep_t beep;
	for(;;)
	{	
		xReturn = xQueueReceive(g_queue_esp8266,	/* 消息队列的句柄 */
								buf,				/* 得到的消息内容 */
								portMAX_DELAY); 	/* 等待时间一直等 */
		if (xReturn != pdPASS)
		{
			dgb_printf_safe("[app_task_mqtt] xQueueReceive error code is %d\r\n", xReturn);
			continue;
		}	

		for(i=0;i<sizeof(buf);i++)
		{
			//判断的关键字符是否为 1"
			//核心数据，即{"switch_lock_1":1}中的“1”
			if((buf[i]==0x31) && (buf[i+1]==0x22))
			{
					//判断控制变量
					if(buf[i+3]=='1' )
					{
						beep.sta=1;
						beep.duration=100;
						xQueueSend(g_queue_beep,	/* 消息队列的句柄 */
						&beep,			/* 发送的消息内容 */
						100);			/* 等待时间 100 Tick */
						/* 阻塞等待信号量，用于确保任务完成对beep的控制 */
						xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);
						if(xReturn != pdPASS)
							dgb_printf_safe("[app_task_at24c02] xSemaphoreTake g_sem_beep error code is %d\r\n",xReturn);	
						
						openlock_flash.mode=MODE_OPEN_LOCK_MQTT;
						xQueueSend(g_queue_flash,/* 消息队列的句柄 */
									&openlock_flash,		/* 发送的消息内容 */
									100);		/* 等待时间 100 Tick */
						memset(&openlock_flash,0,sizeof(openlock_flash));
						
						xEventGroupSetBits(g_event_group,EVENT_GROUP_OPENLOCK);
					}	
			}			
		}
	}
}

static void app_task_mqttheart(void* pvParameters)
{
	dgb_printf_safe("app_task_mqttheart true......\n");
	for(;;)
	{
		vTaskDelay(1000);
		mqtt_send_heart();
		mqtt_report_devices_status();
//		dgb_printf_safe("heart ok\n");
	}
}


static void soft_timer_callback(void* parameter)
{		
	/* 关闭中断 */
	portDISABLE_INTERRUPTS();
	
	//dgb_printf_safe("[soft_timer_callback] iwdg feed\r\n");
	
	/* 喂狗，刷新自身计数值 */
	IWDG_ReloadCounter();	
	
	/* 打开中断 */
	portENABLE_INTERRUPTS();	
} 


/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */

	/* 进入睡眠模式 */
	__wfi();				
	

}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	printf("[vApplicationStackOverflowHook] %s is StackOverflow\r\n",pcTaskName);
	for( ;; );
}

void vApplicationTickHook( void )
{
	
	
}
