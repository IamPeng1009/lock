#include "includes.h"

#define W25QXX_SS			PBout(14)
#define W25QXX_SECTOR_SIZE	4096

static uint8_t g_spi_flash_buf[W25QXX_SECTOR_SIZE];

void w25qxx_init(void)
{
	GPIO_InitTypeDef  	GPIO_InitStructure;	

	SPI_InitTypeDef  	SPI_InitStructure;
	
	/*!< Enable the SPI clock,使能SPI1硬件时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/*!< Enable GPIO clocks，使能GPIOB硬件时钟 */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE);
	
	//SPI1端口配置 PB3 PB4 PB5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; 			//3~5号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;									//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 									//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 									//上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); 	
	
	/*!< Connect SPI1 pins to AF3 AF4 AF5 */  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);

	//初始化片选引脚 PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 										//14号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;									//输出功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 									//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 									//上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); 	
	
	//由于M4芯片还没有真正配置好，先不让外部SPI设备工作
	W25QXX_SS = 1;
	
	/*!< SPI configuration ，SPI的配置*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				//设置SPI为双线双向全双工通信
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;									//配置M4工作在主机模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;								//SPI的发送和接收都是8位数据位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;										//串口时钟线（SCLK）空闲的时候为高电平，这里电平的设置要根据通信的外围设备有关系的	
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;									//串行时钟的第二跳变沿进行数据采样	
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;										//很多时候基于多设备通信，片选引脚都设置为软件控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;				//SPI通信时钟 = 84MHz/16=5.25MHz

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;								//最高有效位优先，根据通信的外围设备有关系的
	SPI_Init(SPI1, &SPI_InitStructure);

	/*!< Enable the sFLASH_SPI  ，使能SPI1硬件*/
	SPI_Cmd(SPI1, ENABLE);
	
}



uint8_t SPI1_SendByte(uint8_t byte)
{
	/*!< Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	/*!< Send byte through the SPI1 peripheral */
	SPI_I2S_SendData(SPI1, byte);

	/*!< Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	/*!< Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI1);
}


uint16_t w25qxx_read_id(void)
{
	uint16_t id=0;
	
	//片选引脚为低电平
	W25QXX_SS = 0;
	
	//发送0x90指令
	SPI1_SendByte(0x90);
	
	//发送24bit地址，全都是0
	SPI1_SendByte(0x00);	
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);

	//读取厂商ID，填写任意参数
	id = SPI1_SendByte(0xFF)<<8;
	
	//读取设备ID，填写任意参数
	id |= SPI1_SendByte(0xFF);

	//片选引脚为高电平
	W25QXX_SS = 1;
	
	delay_us(10);
	
	return id;
}


void w25qxx_read_data(uint32_t addr,uint8_t *pbuf,uint32_t len)
{
	//片选引脚为低电平
	W25QXX_SS = 0;
	
	
	//发送0x03指令
	SPI1_SendByte(0x03);	

	//发送24bit地址
	SPI1_SendByte((addr>>16)&0xFF);	
	SPI1_SendByte((addr>>8)&0xFF);
	SPI1_SendByte( addr&0xFF);
	
	//读取数据
	while(len--)
		*pbuf++ = SPI1_SendByte(0xFF);
	
	//片选引脚为高电平
	W25QXX_SS = 1;
	
	delay_us(10);
}

//解除写保护
void w25qxx_write_enable(void)
{
	//片选引脚为低电平
	W25QXX_SS = 0;
	
	//发送0x06指令
	SPI1_SendByte(0x06);	
	
	//片选引脚为高电平
	W25QXX_SS = 1;
	
	delay_us(10);

}


//开启写保护
void w25qxx_write_disable(void)
{
	//片选引脚为低电平
	W25QXX_SS = 0;
	
	//发送0x04指令
	SPI1_SendByte(0x04);	
	
	//片选引脚为高电平
	W25QXX_SS = 1;
	
	delay_us(10);

}

//读状态寄存器1
uint8_t w25qxx_read_status1(void)
{
	uint8_t status;
	
	//片选引脚为低电平
	W25QXX_SS = 0;
	
	//发送0x05指令
	SPI1_SendByte(0x05);

	//读取状态寄存器1的值
	status = SPI1_SendByte(0xFF);
	
	//片选引脚为高电平
	W25QXX_SS = 1;
	
	delay_us(10);
	
	return status;

}

void w25qxx_erase_sector(uint32_t addr)
{
	uint8_t status;
	
	//解除写保护
	w25qxx_write_enable();
	
	//延时1ms,让W25Q128能够识别到CS引脚电平的变化
	delay_ms(1);
	
	//片选引脚为低电平
	W25QXX_SS = 0;

	//发送0x20指令
	SPI1_SendByte(0x20);
	
	//发送24bit地址
	SPI1_SendByte((addr>>16)&0xFF);	
	SPI1_SendByte((addr>>8)&0xFF);
	SPI1_SendByte( addr&0xFF);	
	
	//片选引脚为高电平
	W25QXX_SS = 1;	
	
	
	while(1)
	{
		//读取状态寄存器1
		status= w25qxx_read_status1();
		
		//若BUSY位为0，则跳出循环
		if((status & 0x01) == 0)
			break;
	
	}
	
	//开启写保护
	w25qxx_write_disable();
}


void w25qxx_write_page(uint32_t addr,uint8_t *pbuf,uint32_t len)
{

	uint8_t status;
	
	//解除写保护
	w25qxx_write_enable();
	
	//延时1ms,让W25Q128能够识别到CS引脚电平的变化
	delay_ms(1);
	
	//片选引脚为低电平
	W25QXX_SS = 0;

	//发送0x02指令
	SPI1_SendByte(0x02);
	
	//发送24bit地址
	SPI1_SendByte((addr>>16)&0xFF);	
	SPI1_SendByte((addr>>8)&0xFF);
	SPI1_SendByte( addr&0xFF);	
	
	//写入数据
	while(len--)
		SPI1_SendByte(*pbuf++);
	
	//片选引脚为高电平
	W25QXX_SS = 1;	
	
	while(1)
	{
		//读取状态寄存器1
		status= w25qxx_read_status1();
		
		//若BUSY位为0，则跳出循环
		if((status & 0x01) == 0)
			break;
	}
	
	//开启写保护
	w25qxx_write_disable();
}

void w25qxx_write_data(uint32_t addr,uint8_t *pbuf,uint32_t len)
{
	uint16_t page_remain;	   
	
	//单页剩余的字节数	
	page_remain=256-addr%256; 
	
	//不大于256个字节
	if(len<=page_remain)
		page_remain=len;
	
	while(1)
	{	   
		//对页写入数据
		w25qxx_write_page(addr,pbuf,page_remain);
		
		//写入结束了
		if(len==page_remain)
			break;
		//len>page_remain
		else 
		{
			pbuf+=page_remain;
			addr+=page_remain;	

			//减去已经写入了的字节数
			len-=page_remain;
			
			//一次可以写入256个字节
			if(len>256)
				page_remain=256; 
			//不够256个字节了
			else 
				page_remain=len; 	    
		}
	}
}

void w25qxx_write_data_ex(uint32_t addr,uint8_t *pbuf,uint32_t len)  
{ 
	uint32_t   secpos;
	uint16_t  secoff;
	uint16_t  secremain;	   
	uint32_t  i;    
	uint8_t  *SPI_FLASH_BUF;	  
	
	SPI_FLASH_BUF=g_spi_flash_buf;
	
	//扇区地址  
	secpos=addr/W25QXX_SECTOR_SIZE;
	
	//在扇区内的偏移
	secoff=addr%W25QXX_SECTOR_SIZE;
	
	//扇区剩余空间大小   
	secremain=W25QXX_SECTOR_SIZE-secoff;

	//不大于4096个字节
	if(len<=secremain)
		secremain=len;
	
	while(1) 
	{	
		//读出整个扇区的内容
		w25qxx_read_data(secpos*W25QXX_SECTOR_SIZE,SPI_FLASH_BUF,W25QXX_SECTOR_SIZE);

		//校验数据
		for(i=0;i<secremain;i++)
		{
			
			//需要擦除  	  
			if(SPI_FLASH_BUF[secoff+i]!=0XFF)
			{
				break;
			}
				
		}
		
		//需要擦除
		if(i<secremain)
		{
			//擦除这个扇区
			w25qxx_erase_sector(secpos*W25QXX_SECTOR_SIZE);
			
			//复制
			for(i=0;i<secremain;i++)	  
			{
				SPI_FLASH_BUF[i+secoff]=pbuf[i];	  
			}
			
			//写入整个扇区  
			w25qxx_write_data(secpos*W25QXX_SECTOR_SIZE,SPI_FLASH_BUF,W25QXX_SECTOR_SIZE);

		} 
		else 
		{
			//写已经擦除了的,直接写入扇区剩余区间. 	
			w25qxx_write_page(addr,pbuf,secremain);
		}		
		
		//写入结束了
		if(len==secremain)
		{
			break;
		}		
		else//写入未结束
		{
			//扇区地址增1
			secpos++;
			
			//偏移位置为0 	 
			secoff=0;
			
			//指针偏移
			pbuf+=secremain; 
			
			//写地址偏移
			addr+=secremain;	  
			
			//字节数递减
			len-=secremain;		
		    
			//下一个扇区还是写不完
			if(len>W25QXX_SECTOR_SIZE)
			{
				secremain=W25QXX_SECTOR_SIZE;	
			}
				
			//下一个扇区可以写完了
			else 
			{
				secremain=len;
			}
							  
		}	 
	}	 
}




int32_t spi_flash_card_match(uint8_t *card_id)
{
	uint8_t  card_id_format[7]={0};	
	uint32_t i;
	
	for(i=0;i<(MAX_CARDS*(sizeof card_id_format));i+=(sizeof card_id_format))
	{
		w25qxx_read_data(i,card_id_format,sizeof card_id_format);
		
		if(card_id_format[0]!='C')
			return -1;
		
		if(0 == memcmp(card_id,&card_id_format[1],4))
		{
			/* 返回该卡在spi flash的存储地址 */
			return i;
		}
	}
	
	return -1;
}


int32_t spi_flash_card_times(uint8_t *card_id)
{
	uint8_t  card_id_format[7]={0};	
	int32_t i;	
	
	i = spi_flash_card_match(card_id);
	
	if(i < 0)
		return -1;
	
	w25qxx_read_data(i,card_id_format,sizeof card_id_format);
	
	/* 该卡识别的次数加1 */
	if(card_id_format[5] < 255)
		card_id_format[5]+=1;

	/* 更新该卡的识别次数 */
	w25qxx_write_data_ex(i+5,&card_id_format[5],1);
	
	return 0;
}

uint32_t spi_flash_card_total(void)
{
	uint8_t  card_id_format[7]={0};	
	uint32_t i;
	uint32_t total=0;	
	
	for(i=0;i<(MAX_CARDS*(sizeof card_id_format));i+=(sizeof card_id_format))
	{
		w25qxx_read_data(i,card_id_format,sizeof card_id_format);
		
		/* 若没有检测到'C'，即没有存储到卡 */
		if(card_id_format[0]!='C')
			break;
		
		total++;
	}
	
	return total;
}


int32_t spi_flash_card_list_all(void)
{
	uint8_t  card_id_format[7]={0};	
	uint32_t i;
	uint32_t total=0;	
	
	printf("\r\n[NOTICE] valid card list:\r\n");
	
	for(i=0;i<(MAX_CARDS*(sizeof card_id_format));i+=(sizeof card_id_format))
	{
		w25qxx_read_data(i,card_id_format,sizeof card_id_format);
		
		/* 若没有检测到'C'，即没有存储到卡 */
		if(card_id_format[0]!='C')
			break;
		
		total++;
		
		printf("[%03d] card %02X%02X%02X%02X %03d\r\n",
				total,
				card_id_format[1],
				card_id_format[2],
				card_id_format[3],
				card_id_format[4],
				card_id_format[5]);		
	}
	
	//printf("card total is %d:\r\n",total);
	
	return total;
}

int32_t spi_flash_card_remove_all(void)
{
	/* 擦除扇区0  */
	w25qxx_erase_sector(0);
	
	return 0;
}

int32_t spi_flash_card_add(uint8_t *card_id)
{
	uint8_t  card_id_format[7]={0};	
	uint32_t i;
	uint32_t total=0;	
	
	for(i=0;i<(MAX_CARDS*(sizeof card_id_format));i+=(sizeof card_id_format))
	{
		w25qxx_read_data(i,card_id_format,sizeof card_id_format);
		
		/* 若没有检测到'C'，即没有存储到卡 */
		if(card_id_format[0]!='C')
			break;
		
		total++;
	}
	
	/* 检测是否存储已满 */
	if(total >= MAX_CARDS)
		return -1;
	
	card_id_format[0]='C';
	card_id_format[1]=card_id[0];
	card_id_format[2]=card_id[1];
	card_id_format[3]=card_id[2];
	card_id_format[4]=card_id[3];
	card_id_format[5]=0;
	card_id_format[6]='#';
	
	/* 向spi flash存储有效卡 */
	w25qxx_write_data_ex(i,card_id_format,sizeof card_id_format);
	
	return 0;
}




