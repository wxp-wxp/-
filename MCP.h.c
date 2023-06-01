/**********************************************************************************
 * �ļ���  ��MCP2515.c
 * ����    ��MCP2515����������         
 * ʵ��ƽ̨��NiRen_STC/IAP15���İ�(���û�STC15��Ƭ��������) + NiRen_MCP2515 CANģ��    
**********************************************************************************/

//#include "main.h"
#include "MCP2515.h"

//SPI_HandleTypeDef hspi1;


//MCP2515������Ԥ��Ƶ
#define	CAN_10Kbps	0x31
#define CAN_25Kbps	0x13
#define CAN_50Kbps	0x09
#define CAN_100Kbps	0x04
#define CAN_125Kbps	0x03
#define CAN_250Kbps	0x01
#define	CAN_500Kbps	0x00
#define        MCP2515_CS_L  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET)
#define        MCP2515_CS_H  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET)
/*******************************************************************************
* ������  : Delay_Nms
* ����    : ͨ�������ʱԼnms(��׼ȷ)
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : �˷�ʽ��ʱʱ���ǲ�׼ȷ��,׼ȷ��ʱ�����ö�ʱ��
*******************************************************************************/
void Delay_Nms(unsigned int x)
{
	unsigned int y;

	for (;x>0;x--)
		for (y=0;y<100;y++);
}

/*******************************************************************************
* ������  : SPI_ReadByte
* ����    : ͨ��SPI��ȡһ���ֽ�����
* ����    : ��
* ���    : ��
* ����ֵ  : rByte(��ȡ����һ���ֽ�����)
* ˵��    : ��
*******************************************************************************/
unsigned char SPI_ReadByte(void)
{
	unsigned char rByte=0;
	HAL_SPI_Receive(&hspi1, &rByte, 1,10);
	return rByte;
}

/*******************************************************************************
* ������  : SPI_SendByte
* ����    : SPI����һ���ֽ�����
* ����    : dt:�����͵�����
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void SPI_SendByte(unsigned char dt)
{
	 HAL_SPI_Transmit(&hspi1,&dt, 1, 10);
}

/*******************************************************************************
* ������  : MCP2515_WriteByte
* ����    : ͨ��SPI��MCP2515ָ����ַ�Ĵ���д1���ֽ�����
* ����    : addr:MCP2515�Ĵ�����ַ,dat:��д�������
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void MCP2515_WriteByte(unsigned char addr,unsigned char dat)
{
	MCP2515_CS_L;				//��MCP2515��CSΪ�͵�ƽ
	SPI_SendByte(CAN_WRITE);	//����д����
	SPI_SendByte(addr);			//���͵�ַ
	SPI_SendByte(dat);			//д������
	MCP2515_CS_H;				//��MCP2515��CSΪ�ߵ�ƽ 
}

/*******************************************************************************
* ������  : MCP2515_ReadByte
* ����    : ͨ��SPI��MCP2515ָ����ַ������1���ֽ�����
* ����    : addr:MCP2515�Ĵ�����ַ
* ���    : ��
* ����ֵ  : rByte:��ȡ���Ĵ�����1���ֽ�����
* ˵��    : ��
*******************************************************************************/
unsigned char MCP2515_ReadByte(unsigned char addr)
{
	unsigned char rByte;
	
	MCP2515_CS_L;				//��MCP2515��CSΪ�͵�ƽ
	SPI_SendByte(CAN_READ);		//���Ͷ�����
	SPI_SendByte(addr);			//���͵�ַ
	rByte=SPI_ReadByte();		//��ȡ����
  MCP2515_CS_H;				//��MCP2515��CSΪ�ߵ�ƽ
	return rByte;				//���ض�����һ���ֽ�����
}

/*******************************************************************************
* ������  : MCP2515_Reset
* ����    : ���͸�λָ�������λMCP2515
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ���ڲ��Ĵ�����λΪȱʡ״̬,���������趨Ϊ����ģʽ
*******************************************************************************/
void MCP2515_Reset(void)
{
	MCP2515_CS_L;					//��MCP2515��CSΪ�͵�ƽ
	SPI_SendByte(CAN_RESET);	//���ͼĴ�����λ����
	MCP2515_CS_H;				//��MCP2515��CSΪ�ߵ�ƽ
}

/*******************************************************************************
* ������  : MCP2515_Init
* ����    : MCP2515��ʼ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��ʼ�������������λ���������������á���ʶ��������õȡ�
*******************************************************************************/
void MCP2515_Init(void)
{
	uint8_t temp=0;

	MCP2515_Reset();	//���͸�λָ�������λMCP2515
	Delay_Nms(1);		//ͨ�������ʱԼnms(��׼ȷ)

	//���ò�����Ϊ125Kbps
	//set CNF1,SJW=00,����Ϊ1TQ,BRP=49,TQ=[2*(BRP+1)]/Fsoc=2*50/8M=12.5us
	MCP2515_WriteByte(CNF1,CAN_125Kbps);
	//set CNF2,SAM=0,�ڲ���������߽���һ�β�����PHSEG1=(2+1)TQ=3TQ,PRSEG=(0+1)TQ=1TQ
	MCP2515_WriteByte(CNF2,0x80|PHSEG1_3TQ|PRSEG_1TQ);
	//set CNF3,PHSEG2=(2+1)TQ=3TQ,ͬʱ��CANCTRL.CLKEN=1ʱ�趨CLKOUT����Ϊʱ�����ʹ��λ
	MCP2515_WriteByte(CNF3,PHSEG2_3TQ);
	
	MCP2515_WriteByte(TXB0SIDH,0xFF);//���ͻ�����0��׼��ʶ����λ
	MCP2515_WriteByte(TXB0SIDL,0xE0);//���ͻ�����0��׼��ʶ����λ
	
	MCP2515_WriteByte(RXB0SIDH,0x00);//��ս��ջ�����0�ı�׼��ʶ����λ
	MCP2515_WriteByte(RXB0SIDL,0x00);//��ս��ջ�����0�ı�׼��ʶ����λ
	MCP2515_WriteByte(RXB0CTRL,0x20);//�������ձ�׼��ʶ������Ч��Ϣ
	MCP2515_WriteByte(RXB0DLC,DLC_8);//���ý������ݵĳ���Ϊ8���ֽ�
	
	MCP2515_WriteByte(RXF0SIDH,0xFF);//���������˲��Ĵ���n��׼��ʶ����λ
	MCP2515_WriteByte(RXF0SIDL,0xE0);//���������˲��Ĵ���n��׼��ʶ����λ
	MCP2515_WriteByte(RXM0SIDH,0xFF);//�����������μĴ���n��׼��ʶ����λ
	MCP2515_WriteByte(RXM0SIDL,0xE0);//�����������μĴ���n��׼��ʶ����λ
	
	MCP2515_WriteByte(CANINTF,0x00);//���CAN�жϱ�־�Ĵ���������λ(������MCU���)
	MCP2515_WriteByte(CANINTE,0x01);//����CAN�ж�ʹ�ܼĴ����Ľ��ջ�����0���ж�ʹ��,����λ��ֹ�ж�
	
	MCP2515_WriteByte(CANCTRL,REQOP_NORMAL|CLKOUT_ENABLED);//��MCP2515����Ϊ����ģʽ,�˳�����ģʽ
	
	temp=MCP2515_ReadByte(CANSTAT);//��ȡCAN״̬�Ĵ�����ֵ
	if(OPMODE_NORMAL!=(temp&0xE0))//�ж�MCP2515�Ƿ��Ѿ���������ģʽ
	{
		MCP2515_WriteByte(CANCTRL,REQOP_NORMAL|CLKOUT_ENABLED);//�ٴν�MCP2515����Ϊ����ģʽ,�˳�����ģʽ
	}
}

/*******************************************************************************
* ������  : CAN_Send_Buffer
* ����    : CAN����ָ�����ȵ�����
* ����    : *CAN_TX_Buf(���������ݻ�����ָ��),len(���������ݳ���)
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void CAN_Send_Buffer(unsigned char *CAN_TX_Buf,unsigned char len)
{
	unsigned char j,dly,count;

	count=0;
	while(count<len)
	{
		dly=0;
		while((MCP2515_ReadByte(TXB0CTRL)&0x08) && (dly<50))//���ٶ�ĳЩ״ָ̬��,�ȴ�TXREQ��־����
		{
			Delay_Nms(1);//ͨ�������ʱԼnms(��׼ȷ)
			dly++;
		}
				
		for(j=0;j<8;)
		{
			MCP2515_WriteByte(TXB0D0+j,CAN_TX_Buf[count++]);//�������͵�����д�뷢�ͻ���Ĵ���
			j++;
			if(count>=len) break;
		}
		MCP2515_WriteByte(TXB0DLC,j);//����֡�����͵����ݳ���д�뷢�ͻ�����0�ķ��ͳ��ȼĴ���
		MCP2515_CS_L;	
		MCP2515_WriteByte(TXB0CTRL,0x08);//�����ͱ���
		MCP2515_CS_H;	
	}
}

/*******************************************************************************
* ������  : CAN_Receive_Buffer
* ����    : CAN����һ֡����
* ����    : *CAN_TX_Buf(���������ݻ�����ָ��)
* ���    : ��
* ����ֵ  : len(���յ����ݵĳ���,0~8�ֽ�)
* ˵��    : ��
*******************************************************************************/
unsigned char CAN_Receive_Buffer(unsigned char *CAN_RX_Buf)
{
	unsigned char i=0,len=0,temp=0;

	temp = MCP2515_ReadByte(CANINTF);
	if(temp & 0x01)
	{
		len=MCP2515_ReadByte(RXB0DLC);//��ȡ���ջ�����0���յ������ݳ���(0~8���ֽ�)
		while(i<len)
		{	
			CAN_RX_Buf[i]=MCP2515_ReadByte(RXB0D0+i);//��CAN���յ������ݷ���ָ��������
			i++;
		}
	}
	MCP2515_WriteByte(CANINTF,0);//����жϱ�־λ(�жϱ�־�Ĵ���������MCU����)
	return len;
}


