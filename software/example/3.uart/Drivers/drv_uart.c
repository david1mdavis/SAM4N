#include "drv_uart.h"
/************************************************************************************
*�ļ���  ��drv_uart.c																                                *
*�ļ����ã�����ͨ��																	                                *
*����    ��ũ����																	                                  *
*����QQ  ��382421307																                                  *
*�ļ�����ʱ�䣺2013/09/29															  														*
*************************************************************************************/

/* UART �����ʼ���̶���Ƶϵ�� */
#define UART_MCK_DIV             16
/* ������������С��Ƶϵ��*/
#define UART_MCK_DIV_MIN_FACTOR  1
/* ��������������Ƶϵ�� */
#define UART_MCK_DIV_MAX_FACTOR  65535

/*******************************************************************************************
* ��������UART0_Init()
* ����  ��uint32_t buadrate ������
* ����ֵ��void
* ����  ��UART0��ʼ����������ʹ��UART0ǰ�ȵ���
*********************************************************************************************/
void UART0_Init(uint32_t baudrate)
{
  	
  uint32_t Fdiv =0;
	/*��ֹ���������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*ʹ��UART0��PIOAʱ��*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOA) |   
                    (1UL << ID_UART0) );  
	/*ʹ�����������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*����PA9ΪUART0��RXD��PA10ΪUART0��TXD*/	
	PIOA->PIO_IDR=(PIO_PA9A_URXD0|PIO_PA10A_UTXD0);
	PIOA->PIO_PUDR=(PIO_PA9A_URXD0|PIO_PA10A_UTXD0);
	PIOA->PIO_ABCDSR[0]&=~(PIO_PA9A_URXD0|PIO_PA10A_UTXD0);
	PIOA->PIO_ABCDSR[1]&=~(PIO_PA9A_URXD0|PIO_PA10A_UTXD0);
	PIOA->PIO_PDR=(PIO_PA9A_URXD0|PIO_PA10A_UTXD0);
		/* ��λ����ֹUART�ķ��ͺͽ���*/
	UART0->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
			| UART_CR_RXDIS | UART_CR_TXDIS;
	/*����UART0�Ĳ�����*/
	Fdiv = (SystemCoreClock/  baudrate) /UART_MCK_DIV ;
	if (Fdiv < UART_MCK_DIV_MIN_FACTOR || Fdiv > UART_MCK_DIV_MAX_FACTOR)
		return ;
	UART0->UART_BRGR=Fdiv;
	/*��������λΪ8bit��ֹͣλΪ1��У��λΪNONE*/
	UART0->UART_MR = UART_MR_PAR_NO|UART_MR_CHMODE_NORMAL;
	/*��ֹ DMA ͨ�� */
	UART0->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	/*ʹ��UART���պͷ���*/
	UART0->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
	/*ʹ�ܽ����ж�*/
	UART0->UART_IER=UART_IER_RXRDY;
   /*����UART0����ռ���ȼ�Ϊ1�������ȼ�Ϊ1*/
   NVIC_SetPriority(UART0_IRQn, ((0x01<<3)|0x01));
	/*ʹ��UART0���ж�ͨ��*/
    NVIC_EnableIRQ(UART0_IRQn);
}
/*******************************************************************************************
* ��������UART0_Handler()
* ����  ��void
* ����ֵ��void
* ����  ��UART0�жϷ�����
*********************************************************************************************/
void UART0_Handler(void)
 { 
	 uint8_t temp;
	 if((UART0->UART_SR& UART_SR_RXRDY)==1)
	 {                                            //���������ж�
			  temp= UART0->UART_RHR&0xff;		         //����һ���ֽ�
       UART0_SendByte(temp);					         //�����յ����ݷ���
   }
 }
  /*******************************************************************************************
* ��������UART0_SendByte()
* ����  ��uint8_t c  Ҫ�����ַ�
* ����ֵ��void
* ����  ��UART0����һ���ַ�����
*********************************************************************************************/
void UART0_SendByte(uint8_t c)
{
   while((UART0->UART_SR & UART_SR_TXEMPTY) == 0);		//�ȴ����ͻ�����Ϊ��
   UART0->UART_THR=c;							//�������ַ�д�뷢�ͱ��ּĴ���
}
 /*******************************************************************************************
* ��������UART0_SendString()
* ����  ��uint8_t *s  ָ���ַ�����ָ��
* ����ֵ��void
* ����  ��UART0�����ַ�������
*********************************************************************************************/
void UART0_SendString(uint8_t *s)
{
  while(*s)												//�ж��Ƿ��ַ���ĩβ
  {
   UART0_SendByte(*s);							//����ָ�뵱ǰ��ָ���ֽ�
   s++;													//��ַ��1
  }

}
/*******************************************************************************************
* ��������UART1_Init()
* ����  ��uint32_t buadrate ������
* ����ֵ��void
* ����  ��UART1��ʼ����������ʹ��UART1ǰ�ȵ���
*********************************************************************************************/
void UART1_Init(uint32_t baudrate)
{
  	
  uint32_t Fdiv =0;
	/*��ֹ���������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*ʹ��UART1��PIOBʱ��*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOB) |   
                    (1UL << ID_UART1) );  
	/*ʹ�����������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*����PB2ΪUART1��RXD��PB3ΪUART1��TXD*/	
	PIOB->PIO_IDR=(PIO_PB2A_URXD1|PIO_PB3A_UTXD1);
	PIOB->PIO_PUDR=(PIO_PB2A_URXD1|PIO_PB3A_UTXD1);
	PIOB->PIO_ABCDSR[0]&=~(PIO_PB2A_URXD1|PIO_PB3A_UTXD1);
	PIOB->PIO_ABCDSR[1]&=~(PIO_PB2A_URXD1|PIO_PB3A_UTXD1);
	PIOB->PIO_PDR=(PIO_PB2A_URXD1|PIO_PB3A_UTXD1);
		/* ��λ����ֹUART�ķ��ͺͽ���*/
	UART1->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
			| UART_CR_RXDIS | UART_CR_TXDIS;
	/*����UART1�Ĳ�����*/
	Fdiv = (SystemCoreClock/  baudrate) /UART_MCK_DIV ;
	if (Fdiv < UART_MCK_DIV_MIN_FACTOR || Fdiv > UART_MCK_DIV_MAX_FACTOR)
		return ;
	UART1->UART_BRGR=Fdiv;
	/*��������λΪ8bit��ֹͣλΪ1��У��λΪNONE*/
	UART1->UART_MR = UART_MR_PAR_NO|UART_MR_CHMODE_NORMAL;
	/*��ֹ DMA ͨ�� */
	UART1->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	/*ʹ��UART���պͷ���*/
	UART1->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
	/*ʹ�ܽ����ж�*/
	UART1->UART_IER=UART_IER_RXRDY;
   /*����UART1����ռ���ȼ�Ϊ1�������ȼ�Ϊ1*/
   NVIC_SetPriority(UART1_IRQn, ((0x01<<3)|0x01));
	/*ʹ��UART1���ж�ͨ��*/
    NVIC_EnableIRQ(UART1_IRQn);

}
/*******************************************************************************************
* ��������UART1_Handler()
* ����  ��void
* ����ֵ��void
* ����  ��UART1�жϷ�����
*********************************************************************************************/
void UART1_Handler(void)
 { 
	 uint8_t temp;
	 if((UART1->UART_SR& UART_SR_RXRDY)==1)
	 {                                            //���������ж�
			  temp= UART1->UART_RHR&0xff;		         //����һ���ֽ�
       UART1_SendByte(temp);					         //�����յ����ݷ���
   }
 }
  /*******************************************************************************************
* ��������UART1_SendByte()
* ����  ��uint8_t c  Ҫ�����ַ�
* ����ֵ��void
* ����  ��UART1����һ���ַ�����
*********************************************************************************************/
void UART1_SendByte(uint8_t c)
{
   while((UART1->UART_SR & UART_SR_TXEMPTY) == 0);		//�ȴ����ͻ�����Ϊ��
   UART1->UART_THR=c;							//�������ַ�д�뷢�ͱ��ּĴ���
}
 /*******************************************************************************************
* ��������UART1_SendString()
* ����  ��uint8_t *s  ָ���ַ�����ָ��
* ����ֵ��void
* ����  ��UART1�����ַ�������
*********************************************************************************************/
void UART1_SendString(uint8_t *s)
{
  while(*s)												//�ж��Ƿ��ַ���ĩβ
  {
   UART1_SendByte(*s);							//����ָ�뵱ǰ��ָ���ֽ�
   s++;													//��ַ��1
  }

}
/*******************************************************************************************
* ��������UART2_Init()
* ����  ��uint32_t buadrate ������
* ����ֵ��void
* ����  ��UART2��ʼ����������ʹ��UART2ǰ�ȵ���
*********************************************************************************************/
void UART2_Init(uint32_t baudrate)
{
   	
  uint32_t Fdiv =0;
	/*��ֹ���������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*ʹ��UART2��PIOAʱ��*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOA) |   
                    (1UL << ID_UART2) );  
	/*ʹ�����������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*����PA16ΪUART2��RXD��PA15ΪUART2��TXD*/	
	PIOA->PIO_IDR=(PIO_PA16A_URXD2|PIO_PA15A_UTXD2);
	PIOA->PIO_PUDR=(PIO_PA16A_URXD2|PIO_PA15A_UTXD2);
	PIOA->PIO_ABCDSR[0]&=~(PIO_PA16A_URXD2|PIO_PA15A_UTXD2);
	PIOA->PIO_ABCDSR[1]&=~(PIO_PA16A_URXD2|PIO_PA15A_UTXD2);
	PIOA->PIO_PDR=(PIO_PA16A_URXD2|PIO_PA15A_UTXD2);
		/* ��λ����ֹUART�ķ��ͺͽ���*/
	UART2->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
			| UART_CR_RXDIS | UART_CR_TXDIS;
	/*����UART2�Ĳ�����*/
	Fdiv = (SystemCoreClock/  baudrate) /UART_MCK_DIV ;
	if (Fdiv < UART_MCK_DIV_MIN_FACTOR || Fdiv > UART_MCK_DIV_MAX_FACTOR)
		return ;
	UART2->UART_BRGR=Fdiv;
	/*��������λΪ8bit��ֹͣλΪ1��У��λΪNONE*/
	UART2->UART_MR = UART_MR_PAR_NO|UART_MR_CHMODE_NORMAL;
	/*��ֹ DMA ͨ�� */
	UART2->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	/*ʹ��UART���պͷ���*/
	UART2->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
	/*ʹ�ܽ����ж�*/
	UART2->UART_IER=UART_IER_RXRDY;
   /*����UART2����ռ���ȼ�Ϊ1�������ȼ�Ϊ1*/
   NVIC_SetPriority(UART2_IRQn, ((0x01<<3)|0x01));
	/*ʹ��UART2���ж�ͨ��*/
    NVIC_EnableIRQ(UART2_IRQn);
}
/*******************************************************************************************
* ��������UART2_Handler()
* ����  ��void
* ����ֵ��void
* ����  ��UART2�жϷ�����
*********************************************************************************************/
void UART2_Handler(void)
 { 
	 uint8_t temp;
	 if((UART2->UART_SR& UART_SR_RXRDY)==1)
	 {                                            //���������ж�
			  temp= UART2->UART_RHR&0xff;		         //����һ���ֽ�
       UART2_SendByte(temp);					         //�����յ����ݷ���
   }
 }
  /*******************************************************************************************
* ��������UART2_SendByte()
* ����  ��uint8_t c  Ҫ�����ַ�
* ����ֵ��void
* ����  ��UART2����һ���ַ�����
*********************************************************************************************/
void UART2_SendByte(uint8_t c)
{
    while((UART2->UART_SR & UART_SR_TXEMPTY) == 0);		//�ȴ����ͻ�����Ϊ��
   UART2->UART_THR=c;							//�������ַ�д�뷢�ͱ��ּĴ���
}
 /*******************************************************************************************
* ��������UART2_SendString()
* ����  ��uint8_t *s  ָ���ַ�����ָ��
* ����ֵ��void
* ����  ��UART2�����ַ�������
*********************************************************************************************/
void UART2_SendString(uint8_t *s)
{
  while(*s)												  //�ж��Ƿ��ַ���ĩβ
  {
   UART2_SendByte(*s);							//����ָ�뵱ǰ��ָ���ֽ�
   s++;													  //��ַ��1
  }

}
/*******************************************************************************************
* ��������UART3_Init()
* ����  ��uint32_t buadrate ������
* ����ֵ��void
* ����  ��UART3��ʼ����������ʹ��UART1ǰ�ȵ���
*********************************************************************************************/
void UART3_Init(uint32_t baudrate)
{
  	
  uint32_t Fdiv =0;
	/*��ֹ���������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*ʹ��UART3��PIOBʱ��*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOB) |   
                    (1UL << ID_UART3) );  
	/*ʹ�����������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*����PB10ΪUART3��RXD��PB11ΪUART3��TXD*/	
//	PIOB->PIO_WPMR=0x50494F00;
	PIOB->PIO_IDR=(PIO_PB10B_URXD3|PIO_PB11B_UTXD3);
	PIOB->PIO_PUDR=(PIO_PB10B_URXD3|PIO_PB11B_UTXD3);
	PIOB->PIO_ABCDSR[0]|=(PIO_PB10B_URXD3|PIO_PB11B_UTXD3);
	PIOB->PIO_ABCDSR[1]&=~(PIO_PB10B_URXD3|PIO_PB11B_UTXD3);
	PIOB->PIO_PDR=(PIO_PB10B_URXD3|PIO_PB11B_UTXD3);
	//PIOB->PIO_WPMR=0x50494F01;
		/* ��λ����ֹUART�ķ��ͺͽ���*/
	UART3->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
			| UART_CR_RXDIS | UART_CR_TXDIS;
	/*����UART3�Ĳ�����*/
	Fdiv = (SystemCoreClock/  baudrate) /UART_MCK_DIV ;
	if (Fdiv < UART_MCK_DIV_MIN_FACTOR || Fdiv > UART_MCK_DIV_MAX_FACTOR)
		return ;
	UART3->UART_BRGR=Fdiv;
	/*��������λΪ8bit��ֹͣλΪ1��У��λΪNONE*/
	UART3->UART_MR = UART_MR_PAR_NO|UART_MR_CHMODE_NORMAL;
	/*��ֹ DMA ͨ�� */
	UART3->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	/*ʹ��UART���պͷ���*/
	UART3->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
	/*ʹ�ܽ����ж�*/
	UART3->UART_IER=UART_IER_RXRDY;
   /*����UART1����ռ���ȼ�Ϊ1�������ȼ�Ϊ1*/
   NVIC_SetPriority(UART3_IRQn, ((0x01<<3)|0x01));
	/*ʹ��UART3���ж�ͨ��*/
    NVIC_EnableIRQ(UART3_IRQn);

}
/*******************************************************************************************
* ��������UART3_Handler()
* ����  ��void
* ����ֵ��void
* ����  ��UART3�жϷ�����
*********************************************************************************************/
void UART3_Handler(void)
 { 
	 uint8_t temp;
	 if((UART3->UART_SR& UART_SR_RXRDY)==1)
	 {                                            //���������ж�
			  temp= UART3->UART_RHR&0xff;		         //����һ���ֽ�
       UART3_SendByte(temp);					         //�����յ����ݷ���
   }
 }
  /*******************************************************************************************
* ��������UART3_SendByte()
* ����  ��uint8_t c  Ҫ�����ַ�
* ����ֵ��void
* ����  ��UART3����һ���ַ�����
*********************************************************************************************/
void UART3_SendByte(uint8_t c)
{
   while((UART3->UART_SR & UART_SR_TXEMPTY) == 0);		//�ȴ����ͻ�����Ϊ��
   UART3->UART_THR=c;							//�������ַ�д�뷢�ͱ��ּĴ���
}
 /*******************************************************************************************
* ��������UART3_SendString()
* ����  ��uint8_t *s  ָ���ַ�����ָ��
* ����ֵ��void
* ����  ��UART3�����ַ�������
*********************************************************************************************/
void UART3_SendString(uint8_t *s)
{
  while(*s)												//�ж��Ƿ��ַ���ĩβ
  {
   UART3_SendByte(*s);							//����ָ�뵱ǰ��ָ���ֽ�
   s++;													//��ַ��1
  }

}
/********************************************************************************************************************************
*��������fputc()
* ������int ch��FILE *f
* ����ֵ��int
* ���ܣ����¶���stdio.h�е�fputc()������ʹprintf()�����USART1
*********************************************************************************************************************************/
int fputc(int ch,FILE *f)
{
  UART0_SendByte((uint8_t)ch);							//����1���ֽ�
  return ch;																	   //���� ch
}
