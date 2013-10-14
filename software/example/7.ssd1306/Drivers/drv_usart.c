#include "drv_usart.h"
/************************************************************************************
*�ļ���  ��drv_usart.c																	                                  *
*�ļ����ã��첽����ͨ��																	                                *
*����    ��ũ����																	                                  *
*����QQ  ��382421307																                                  *
*�ļ�����ʱ�䣺2012/05/11															  														*
*************************************************************************************/

/*�����ʼ���*/
#define BAUD(b) ((SystemCoreClock + 8*b)/(16*b))

/*******************************************************************************************
* ��������USART0_Init()
* ����  ��uint32_t buadrate ������
* ����ֵ��void
* ����  ��USART0��ʼ����������ʹ��USART0ǰ�ȵ���
*********************************************************************************************/
void USART0_Init(uint32_t baudrate)
{
	/*��ֹ���������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*ʹ��USART1��PIOAʱ��*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOA) |   
                    (1UL << ID_USART0) );  
	/*ʹ�����������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*����PA5ΪUSART0��RXD��PA6ΪUSART0��TXD*/	
	PIOA->PIO_IDR=(PIO_PA5A_RXD0|PIO_PA6A_TXD0);
	PIOA->PIO_PUDR=(PIO_PA5A_RXD0|PIO_PA6A_TXD0);
	PIOA->PIO_ABCDSR[0]&=~(PIO_PA5A_RXD0|PIO_PA6A_TXD0);
	PIOA->PIO_ABCDSR[1]&=~(PIO_PA5A_RXD0|PIO_PA6A_TXD0);
	PIOA->PIO_PDR=(PIO_PA5A_RXD0|PIO_PA6A_TXD0);
		/* ��λ����ֹUSART�ķ��ͺͽ���*/
	USART0->US_CR = US_CR_RSTRX | US_CR_RSTTX
			| US_CR_RXDIS | US_CR_TXDIS;
	/*����USART0�Ĳ�����*/
	USART0->US_BRGR=BAUD(baudrate);
	/*��������λΪ8bit��ֹͣλΪ1��У��λΪNONE*/
	USART0->US_MR = US_MR_USART_MODE_NORMAL|   //��ͨģʽ
	                US_MR_CHRL_8_BIT|          //����λΪ8λ
									US_MR_NBSTOP_1_BIT|        //ֹͣλΪ1λ
									US_MR_PAR_NO|              //У��λΪNONE
									US_MR_CHMODE_NORMAL;       //��ͨͨ��ģʽ
	/*��ֹ DMA ͨ�� */
	USART0->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
	/*ʹ��USART���պͷ���*/
	USART0->US_CR = US_CR_RXEN | US_CR_TXEN;
	/*ʹ�ܽ����ж�*/
	USART0->US_IER=US_IER_RXRDY;
   /*����USART0����ռ���ȼ�Ϊ1�������ȼ�Ϊ1*/
   NVIC_SetPriority(USART0_IRQn, ((0x01<<3)|0x01));
	/*ʹ��USART0���ж�ͨ��*/
    NVIC_EnableIRQ(USART0_IRQn);
}
/*******************************************************************************************
* ��������USART0_Handler()
* ����  ��void
* ����ֵ��void
* ����  ��USART0�жϷ�����
*********************************************************************************************/
void USART0_Handler(void)
 { 
	 uint8_t temp;
	 if((USART0->US_CSR& US_CSR_RXRDY)==1)
	 {                                            //���������ж�
			  temp= USART0->US_RHR&0xff;		         //����һ���ֽ�
       USART0_SendByte(temp);					         //�����յ����ݷ���
   }
 }
  /*******************************************************************************************
* ��������USART0_SendByte()
* ����  ��uint8_t c  Ҫ�����ַ�
* ����ֵ��void
* ����  ��USART0����һ���ַ�����
*********************************************************************************************/
void USART0_SendByte(uint8_t c)
{
   while((USART0->US_CSR & US_CSR_TXEMPTY) == 0);		//�ȴ����ͻ�����Ϊ��
   USART0->US_THR=c;							//�������ַ�д�뷢�ͱ��ּĴ���
}
 /*******************************************************************************************
* ��������USART0_SendString()
* ����  ��uint8_t *s  ָ���ַ�����ָ��
* ����ֵ��void
* ����  ��USART0�����ַ�������
*********************************************************************************************/
void USART0_SendString(uint8_t *s)
{
  while(*s)												//�ж��Ƿ��ַ���ĩβ
  {
   USART0_SendByte(*s);							//����ָ�뵱ǰ��ָ���ֽ�
   s++;													//��ַ��1
  }

}
/*******************************************************************************************
* ��������USART1_Init()
* ����  ��uint32_t buadrate ������
* ����ֵ��void
* ����  ��USART1��ʼ����������ʹ��USART1ǰ�ȵ���
*********************************************************************************************/
void USART1_Init(uint32_t baudrate)
{
	/*��ֹ���������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*ʹ��USART1��PIOBʱ��*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOA) |   
                    (1UL << ID_USART1) );  
	/*ʹ�����������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*����PA21ΪUSART1��RXD��PA22ΪUSART1��TXD*/	
	PIOA->PIO_IDR=(PIO_PA21A_RXD1|PIO_PA22A_TXD1);
	PIOA->PIO_PUDR=(PIO_PA21A_RXD1|PIO_PA22A_TXD1);
	PIOA->PIO_ABCDSR[0]&=~(PIO_PA21A_RXD1|PIO_PA22A_TXD1);
	PIOA->PIO_ABCDSR[1]&=~(PIO_PA21A_RXD1|PIO_PA22A_TXD1);
	PIOA->PIO_PDR=(PIO_PA21A_RXD1|PIO_PA22A_TXD1);
		/* ��λ����ֹUSART�ķ��ͺͽ���*/
	USART1->US_CR = US_CR_RSTRX | US_CR_RSTTX
			| US_CR_RXDIS | US_CR_TXDIS;
	/*����USART1�Ĳ�����*/
	USART1->US_BRGR=BAUD(baudrate);
	/*��������λΪ8bit��ֹͣλΪ1��У��λΪNONE*/
	USART1->US_MR = US_MR_USART_MODE_NORMAL|   //��ͨģʽ
	                US_MR_CHRL_8_BIT|          //����λΪ8λ
									US_MR_NBSTOP_1_BIT|        //ֹͣλΪ1λ
									US_MR_PAR_NO|              //У��λΪNONE
									US_MR_CHMODE_NORMAL;       //��ͨͨ��ģʽ
	/*��ֹ DMA ͨ�� */
	USART1->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
	/*ʹ��USART���պͷ���*/
	USART1->US_CR = US_CR_RXEN | US_CR_TXEN;
	/*ʹ�ܽ����ж�*/
	USART1->US_IER=US_IER_RXRDY;
   /*����USART1����ռ���ȼ�Ϊ1�������ȼ�Ϊ1*/
   NVIC_SetPriority(USART1_IRQn, ((0x01<<3)|0x01));
	/*ʹ��USART1���ж�ͨ��*/
    NVIC_EnableIRQ(USART1_IRQn);

}
/*******************************************************************************************
* ��������USART1_Handler()
* ����  ��void
* ����ֵ��void
* ����  ��USART1�жϷ�����
*********************************************************************************************/
void USART1_Handler(void)
 { 
	 uint8_t temp;
	 if((USART1->US_CSR& US_CSR_RXRDY)==1)
	 {                                            //���������ж�
			  temp= USART1->US_RHR&0xff;		         //����һ���ֽ�
       USART1_SendByte(temp);					         //�����յ����ݷ���
   }
 }
  /*******************************************************************************************
* ��������USART1_SendByte()
* ����  ��uint8_t c  Ҫ�����ַ�
* ����ֵ��void
* ����  ��USART1����һ���ַ�����
*********************************************************************************************/
void USART1_SendByte(uint8_t c)
{
   while((USART1->US_CSR & US_CSR_TXEMPTY) == 0);		//�ȴ����ͻ�����Ϊ��
   USART1->US_THR=c;							//�������ַ�д�뷢�ͱ��ּĴ���
}
 /*******************************************************************************************
* ��������USART1_SendString()
* ����  ��uint8_t *s  ָ���ַ�����ָ��
* ����ֵ��void
* ����  ��USART1�����ַ�������
*********************************************************************************************/
void USART1_SendString(uint8_t *s)
{
  while(*s)												//�ж��Ƿ��ַ���ĩβ
  {
   USART1_SendByte(*s);							//����ָ�뵱ǰ��ָ���ֽ�
   s++;													//��ַ��1
  }

}
/*******************************************************************************************
* ��������USART2_Init()
* ����  ��uint32_t buadrate ������
* ����ֵ��void
* ����  ��USART2��ʼ����������ʹ��USART2ǰ�ȵ���
*********************************************************************************************/
void USART2_Init(uint32_t baudrate)
{
	/*��ֹ���������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*ʹ��USART2��PIOAʱ��*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOC) |   
                    (1UL << ID_USART2) );  
	/*ʹ�����������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*����PC9ΪUSART2��RXD��PC10ΪUSART2��TXD*/	
	PIOC->PIO_IDR=(PIO_PC9A_RXD2|PIO_PC10A_TXD2);
	PIOC->PIO_PUDR=(PIO_PC9A_RXD2|PIO_PC10A_TXD2);
	PIOC->PIO_ABCDSR[0]&=~(PIO_PC9A_RXD2|PIO_PC10A_TXD2);
	PIOC->PIO_ABCDSR[1]&=~(PIO_PC9A_RXD2|PIO_PC10A_TXD2);
	PIOC->PIO_PDR=(PIO_PC9A_RXD2|PIO_PC10A_TXD2);
		/* ��λ����ֹUSART�ķ��ͺͽ���*/
	USART2->US_CR = US_CR_RSTRX | US_CR_RSTTX
			| US_CR_RXDIS | US_CR_TXDIS;
	/*����USART2�Ĳ�����*/
	USART2->US_BRGR=BAUD(baudrate);
	/*��������λΪ8bit��ֹͣλΪ1��У��λΪNONE*/
	USART2->US_MR = US_MR_USART_MODE_NORMAL|   //��ͨģʽ
	                US_MR_CHRL_8_BIT|          //����λΪ8λ
									US_MR_NBSTOP_1_BIT|        //ֹͣλΪ1λ
									US_MR_PAR_NO|              //У��λΪNONE
									US_MR_CHMODE_NORMAL;       //��ͨͨ��ģʽ
	/*��ֹ DMA ͨ�� */
	USART2->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
	/*ʹ��USART���պͷ���*/
	USART2->US_CR = US_CR_RXEN | US_CR_TXEN;
	/*ʹ�ܽ����ж�*/
	USART2->US_IER=US_IER_RXRDY;
   /*����USART2����ռ���ȼ�Ϊ1�������ȼ�Ϊ1*/
   NVIC_SetPriority(USART2_IRQn, ((0x01<<3)|0x01));
	/*ʹ��USART2���ж�ͨ��*/
    NVIC_EnableIRQ(USART2_IRQn);
}
/*******************************************************************************************
* ��������USART2_Handler()
* ����  ��void
* ����ֵ��void
* ����  ��USART2�жϷ�����
*********************************************************************************************/
void USART2_Handler(void)
 { 
	 uint8_t temp;
	 if((USART2->US_CSR& US_CSR_RXRDY)==1)
	 {                                            //���������ж�
			  temp= USART2->US_RHR&0xff;		         //����һ���ֽ�
       USART2_SendByte(temp);					         //�����յ����ݷ���
   }
 }
  /*******************************************************************************************
* ��������USART2_SendByte()
* ����  ��uint8_t c  Ҫ�����ַ�
* ����ֵ��void
* ����  ��USART2����һ���ַ�����
*********************************************************************************************/
void USART2_SendByte(uint8_t c)
{
    while((USART2->US_CSR & US_CSR_TXEMPTY) == 0);		//�ȴ����ͻ�����Ϊ��
   USART2->US_THR=c;							//�������ַ�д�뷢�ͱ��ּĴ���
}
 /*******************************************************************************************
* ��������USART2_SendString()
* ����  ��uint8_t *s  ָ���ַ�����ָ��
* ����ֵ��void
* ����  ��USART2�����ַ�������
*********************************************************************************************/
void USART2_SendString(uint8_t *s)
{
  while(*s)												  //�ж��Ƿ��ַ���ĩβ
  {
   USART2_SendByte(*s);							//����ָ�뵱ǰ��ָ���ֽ�
   s++;													  //��ַ��1
  }

}
