#include "includes.h"

#define W25QXX_SS			PBout(14)
#define W25QXX_SECTOR_SIZE	4096

static uint8_t g_spi_flash_buf[W25QXX_SECTOR_SIZE];

void w25qxx_init(void)
{
	GPIO_InitTypeDef  	GPIO_InitStructure;	

	SPI_InitTypeDef  	SPI_InitStructure;
	
	/*!< Enable the SPI clock,ʹ��SPI1Ӳ��ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/*!< Enable GPIO clocks��ʹ��GPIOBӲ��ʱ�� */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE);
	
	//SPI1�˿����� PB3 PB4 PB5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; 			//3~5������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;									//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 									//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 									//����
	GPIO_Init(GPIOB,&GPIO_InitStructure); 	
	
	/*!< Connect SPI1 pins to AF3 AF4 AF5 */  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);

	//��ʼ��Ƭѡ���� PB14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 										//14������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;									//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 									//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 									//����
	GPIO_Init(GPIOB,&GPIO_InitStructure); 	
	
	//����M4оƬ��û���������úã��Ȳ����ⲿSPI�豸����
	W25QXX_SS = 1;
	
	/*!< SPI configuration ��SPI������*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				//����SPIΪ˫��˫��ȫ˫��ͨ��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;									//����M4����������ģʽ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;								//SPI�ķ��ͺͽ��ն���8λ����λ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;										//����ʱ���ߣ�SCLK�����е�ʱ��Ϊ�ߵ�ƽ�������ƽ������Ҫ����ͨ�ŵ���Χ�豸�й�ϵ��	
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;									//����ʱ�ӵĵڶ������ؽ������ݲ���	
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;										//�ܶ�ʱ����ڶ��豸ͨ�ţ�Ƭѡ���Ŷ�����Ϊ�������
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;				//SPIͨ��ʱ�� = 84MHz/16=5.25MHz

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;								//�����Чλ���ȣ�����ͨ�ŵ���Χ�豸�й�ϵ��
	SPI_Init(SPI1, &SPI_InitStructure);

	/*!< Enable the sFLASH_SPI  ��ʹ��SPI1Ӳ��*/
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
	
	//Ƭѡ����Ϊ�͵�ƽ
	W25QXX_SS = 0;
	
	//����0x90ָ��
	SPI1_SendByte(0x90);
	
	//����24bit��ַ��ȫ����0
	SPI1_SendByte(0x00);	
	SPI1_SendByte(0x00);
	SPI1_SendByte(0x00);

	//��ȡ����ID����д�������
	id = SPI1_SendByte(0xFF)<<8;
	
	//��ȡ�豸ID����д�������
	id |= SPI1_SendByte(0xFF);

	//Ƭѡ����Ϊ�ߵ�ƽ
	W25QXX_SS = 1;
	
	delay_us(10);
	
	return id;
}


void w25qxx_read_data(uint32_t addr,uint8_t *pbuf,uint32_t len)
{
	//Ƭѡ����Ϊ�͵�ƽ
	W25QXX_SS = 0;
	
	
	//����0x03ָ��
	SPI1_SendByte(0x03);	

	//����24bit��ַ
	SPI1_SendByte((addr>>16)&0xFF);	
	SPI1_SendByte((addr>>8)&0xFF);
	SPI1_SendByte( addr&0xFF);
	
	//��ȡ����
	while(len--)
		*pbuf++ = SPI1_SendByte(0xFF);
	
	//Ƭѡ����Ϊ�ߵ�ƽ
	W25QXX_SS = 1;
	
	delay_us(10);
}

//���д����
void w25qxx_write_enable(void)
{
	//Ƭѡ����Ϊ�͵�ƽ
	W25QXX_SS = 0;
	
	//����0x06ָ��
	SPI1_SendByte(0x06);	
	
	//Ƭѡ����Ϊ�ߵ�ƽ
	W25QXX_SS = 1;
	
	delay_us(10);

}


//����д����
void w25qxx_write_disable(void)
{
	//Ƭѡ����Ϊ�͵�ƽ
	W25QXX_SS = 0;
	
	//����0x04ָ��
	SPI1_SendByte(0x04);	
	
	//Ƭѡ����Ϊ�ߵ�ƽ
	W25QXX_SS = 1;
	
	delay_us(10);

}

//��״̬�Ĵ���1
uint8_t w25qxx_read_status1(void)
{
	uint8_t status;
	
	//Ƭѡ����Ϊ�͵�ƽ
	W25QXX_SS = 0;
	
	//����0x05ָ��
	SPI1_SendByte(0x05);

	//��ȡ״̬�Ĵ���1��ֵ
	status = SPI1_SendByte(0xFF);
	
	//Ƭѡ����Ϊ�ߵ�ƽ
	W25QXX_SS = 1;
	
	delay_us(10);
	
	return status;

}

void w25qxx_erase_sector(uint32_t addr)
{
	uint8_t status;
	
	//���д����
	w25qxx_write_enable();
	
	//��ʱ1ms,��W25Q128�ܹ�ʶ��CS���ŵ�ƽ�ı仯
	delay_ms(1);
	
	//Ƭѡ����Ϊ�͵�ƽ
	W25QXX_SS = 0;

	//����0x20ָ��
	SPI1_SendByte(0x20);
	
	//����24bit��ַ
	SPI1_SendByte((addr>>16)&0xFF);	
	SPI1_SendByte((addr>>8)&0xFF);
	SPI1_SendByte( addr&0xFF);	
	
	//Ƭѡ����Ϊ�ߵ�ƽ
	W25QXX_SS = 1;	
	
	
	while(1)
	{
		//��ȡ״̬�Ĵ���1
		status= w25qxx_read_status1();
		
		//��BUSYλΪ0��������ѭ��
		if((status & 0x01) == 0)
			break;
	
	}
	
	//����д����
	w25qxx_write_disable();
}


void w25qxx_write_page(uint32_t addr,uint8_t *pbuf,uint32_t len)
{

	uint8_t status;
	
	//���д����
	w25qxx_write_enable();
	
	//��ʱ1ms,��W25Q128�ܹ�ʶ��CS���ŵ�ƽ�ı仯
	delay_ms(1);
	
	//Ƭѡ����Ϊ�͵�ƽ
	W25QXX_SS = 0;

	//����0x02ָ��
	SPI1_SendByte(0x02);
	
	//����24bit��ַ
	SPI1_SendByte((addr>>16)&0xFF);	
	SPI1_SendByte((addr>>8)&0xFF);
	SPI1_SendByte( addr&0xFF);	
	
	//д������
	while(len--)
		SPI1_SendByte(*pbuf++);
	
	//Ƭѡ����Ϊ�ߵ�ƽ
	W25QXX_SS = 1;	
	
	while(1)
	{
		//��ȡ״̬�Ĵ���1
		status= w25qxx_read_status1();
		
		//��BUSYλΪ0��������ѭ��
		if((status & 0x01) == 0)
			break;
	}
	
	//����д����
	w25qxx_write_disable();
}

void w25qxx_write_data(uint32_t addr,uint8_t *pbuf,uint32_t len)
{
	uint16_t page_remain;	   
	
	//��ҳʣ����ֽ���	
	page_remain=256-addr%256; 
	
	//������256���ֽ�
	if(len<=page_remain)
		page_remain=len;
	
	while(1)
	{	   
		//��ҳд������
		w25qxx_write_page(addr,pbuf,page_remain);
		
		//д�������
		if(len==page_remain)
			break;
		//len>page_remain
		else 
		{
			pbuf+=page_remain;
			addr+=page_remain;	

			//��ȥ�Ѿ�д���˵��ֽ���
			len-=page_remain;
			
			//һ�ο���д��256���ֽ�
			if(len>256)
				page_remain=256; 
			//����256���ֽ���
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
	
	//������ַ  
	secpos=addr/W25QXX_SECTOR_SIZE;
	
	//�������ڵ�ƫ��
	secoff=addr%W25QXX_SECTOR_SIZE;
	
	//����ʣ��ռ��С   
	secremain=W25QXX_SECTOR_SIZE-secoff;

	//������4096���ֽ�
	if(len<=secremain)
		secremain=len;
	
	while(1) 
	{	
		//������������������
		w25qxx_read_data(secpos*W25QXX_SECTOR_SIZE,SPI_FLASH_BUF,W25QXX_SECTOR_SIZE);

		//У������
		for(i=0;i<secremain;i++)
		{
			
			//��Ҫ����  	  
			if(SPI_FLASH_BUF[secoff+i]!=0XFF)
			{
				break;
			}
				
		}
		
		//��Ҫ����
		if(i<secremain)
		{
			//�����������
			w25qxx_erase_sector(secpos*W25QXX_SECTOR_SIZE);
			
			//����
			for(i=0;i<secremain;i++)	  
			{
				SPI_FLASH_BUF[i+secoff]=pbuf[i];	  
			}
			
			//д����������  
			w25qxx_write_data(secpos*W25QXX_SECTOR_SIZE,SPI_FLASH_BUF,W25QXX_SECTOR_SIZE);

		} 
		else 
		{
			//д�Ѿ������˵�,ֱ��д������ʣ������. 	
			w25qxx_write_page(addr,pbuf,secremain);
		}		
		
		//д�������
		if(len==secremain)
		{
			break;
		}		
		else//д��δ����
		{
			//������ַ��1
			secpos++;
			
			//ƫ��λ��Ϊ0 	 
			secoff=0;
			
			//ָ��ƫ��
			pbuf+=secremain; 
			
			//д��ַƫ��
			addr+=secremain;	  
			
			//�ֽ����ݼ�
			len-=secremain;		
		    
			//��һ����������д����
			if(len>W25QXX_SECTOR_SIZE)
			{
				secremain=W25QXX_SECTOR_SIZE;	
			}
				
			//��һ����������д����
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
			/* ���ظÿ���spi flash�Ĵ洢��ַ */
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
	
	/* �ÿ�ʶ��Ĵ�����1 */
	if(card_id_format[5] < 255)
		card_id_format[5]+=1;

	/* ���¸ÿ���ʶ����� */
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
		
		/* ��û�м�⵽'C'����û�д洢���� */
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
		
		/* ��û�м�⵽'C'����û�д洢���� */
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
	/* ��������0  */
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
		
		/* ��û�м�⵽'C'����û�д洢���� */
		if(card_id_format[0]!='C')
			break;
		
		total++;
	}
	
	/* ����Ƿ�洢���� */
	if(total >= MAX_CARDS)
		return -1;
	
	card_id_format[0]='C';
	card_id_format[1]=card_id[0];
	card_id_format[2]=card_id[1];
	card_id_format[3]=card_id[2];
	card_id_format[4]=card_id[3];
	card_id_format[5]=0;
	card_id_format[6]='#';
	
	/* ��spi flash�洢��Ч�� */
	w25qxx_write_data_ex(i,card_id_format,sizeof card_id_format);
	
	return 0;
}




