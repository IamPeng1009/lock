#ifndef __INCLUDES_H__
#define __INCLUDES_H__

/* ��׼C��*/
#include <stdio.h>	
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

/* ������� */
#include "stm32f4xx.h"  //оƬ�����мĴ�������
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "dht11.h"
#include "key.h"
#include "rtc.h"
#include "iwdg.h" 
#include "beep.h"
#include "sfm.h"
#include "usart6.h"
#include "motor.h"
#include "mfrc522.h"
#include "w25qxx.h"
#include "keyboard.h"
#include "i2c.h"
#include "at24c02.h"
#include "esp8266_mqtt.h"
#include "esp8266.h"
#include "flash.h"

/* �궨�� */
#define STA_CARD_REQUEST		0
#define STA_CARD_FOUND			1
#define STA_CARD_VALID_FOUND	2
#define STA_CARD_INVALID_FOUND	3
#define STA_CARD_VALID_ADD		4

#define MAX_CARDS				100

#define EVENT_GROUP_KEY1_DOWN		0x01
#define EVENT_GROUP_KEY2_DOWN		0x02
#define EVENT_GROUP_KEY3_DOWN		0x04
#define EVENT_GROUP_KEY4_DOWN		0x08
#define EVENT_GROUP_KEYALL_DOWN		0x0F

#define EVENT_GROUP_RTC_WAKEUP		0x10

#define EVENT_GROUP_ADD_SFM			0x20	//���ָ��
#define EVENT_GROUP_DELETEALL_SFM	0x40	//ɾ������ָ��
#define EVENT_GROUP_ADD_NFC			0x80	//��ӿ���

#define EVENT_GROUP_DELETEALL_NFC	0x100	//��ӿ���
#define EVENT_GROUP_OPENLOCK		0x200	//�򿪵��
#define EVENT_GROUP_KEYBOARD_ON		0x400	//��������
#define EVENT_GROUP_KEYBOARD_PASSWD	0x800	//�ж�����
#define EVENT_GROUP_BLUE_PASSWD		0x1000	//�ж�����

#define EVENT_GROUP_MODIFY_PASSWD	0x2000	//�ж�����

#define QUEUE_USART_LEN    		4   	/* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define QUEUE_USART6_LEN    	4   	/* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define QUEUE_LED_LEN    		4   	/* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define QUEUE_BEEP_LEN    		4   	/* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define QUEUE_USARTLCD_LEN		16		/* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define QUEUE_MQTT_LEN			16		/* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define QUEUE_FLASH_LEN			4		/* ���еĳ��ȣ����ɰ������ٸ���Ϣ */

/* ���� */
extern SemaphoreHandle_t 	g_mutex_printf;
extern EventGroupHandle_t 	g_event_group;	
extern QueueHandle_t 		g_queue_usart;
extern QueueHandle_t 		g_queue_usart6;
extern QueueHandle_t 		g_queue_esp8266;
extern float g_temp;
extern float g_humi;

typedef struct __beep_t
{
	uint32_t sta;				//1-���� 0-ֹͣ
	uint32_t duration;			//����ʱ�䣬��λ����
}beep_t;

#define LCD_CTRL_LOCK_ON        	0x01
#define LCD_CTRL_LOCK_OFF       	0x02
#define LCD_CTRL_LOCK_ERROR        	0x03
#define LCD_CTRL_TEMP              	0x04
#define LCD_CTRL_HUMI             	0x05
#define LCD_CTRL_TIME       		0x06
#define LCD_CTRL_KEYBOARD       	0x07
#define LCD_CTRL_PASSWD   	    	0x08
#define LCD_CTRL_SLEEP   	    	0x09

typedef struct __lcd_t
{
	uint8_t ctrl;
	uint8_t buf[128];
}lcd_t;

typedef struct __task_t
{
	TaskFunction_t pxTaskCode;
	const char * const pcName;		
	const configSTACK_DEPTH_TYPE usStackDepth;
	void * const pvParameters;
	UBaseType_t uxPriority;
	TaskHandle_t * const pxCreatedTask;
}task_t;

extern void dgb_printf_safe(const char *format, ...);

#endif

