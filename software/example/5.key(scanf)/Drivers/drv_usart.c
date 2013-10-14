#include "drv_usart.h"
/************************************************************************************
*文件名  ：drv_usart.c																	                                  *
*文件作用：异步串口通信																	                                *
*作者    ：农晓明																	                                  *
*作者QQ  ：382421307																                                  *
*文件创建时间：2012/05/11															  														*
*************************************************************************************/

/*波特率计算*/
#define BAUD(b) ((SystemCoreClock + 8*b)/(16*b))

/*******************************************************************************************
* 函数名：USART0_Init()
* 参数  ：uint32_t buadrate 波特率
* 返回值：void
* 描述  ：USART0初始化函数，在使用USART0前先调用
*********************************************************************************************/
void USART0_Init(uint32_t baudrate)
{
	/*禁止外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*使能USART1和PIOA时钟*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOA) |   
                    (1UL << ID_USART0) );  
	/*使能外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*配置PA5为USART0的RXD，PA6为USART0的TXD*/	
	PIOA->PIO_IDR=(PIO_PA5A_RXD0|PIO_PA6A_TXD0);
	PIOA->PIO_PUDR=(PIO_PA5A_RXD0|PIO_PA6A_TXD0);
	PIOA->PIO_ABCDSR[0]&=~(PIO_PA5A_RXD0|PIO_PA6A_TXD0);
	PIOA->PIO_ABCDSR[1]&=~(PIO_PA5A_RXD0|PIO_PA6A_TXD0);
	PIOA->PIO_PDR=(PIO_PA5A_RXD0|PIO_PA6A_TXD0);
		/* 复位并禁止USART的发送和接收*/
	USART0->US_CR = US_CR_RSTRX | US_CR_RSTTX
			| US_CR_RXDIS | US_CR_TXDIS;
	/*配置USART0的波特率*/
	USART0->US_BRGR=BAUD(baudrate);
	/*定义数据位为8bit，停止位为1，校验位为NONE*/
	USART0->US_MR = US_MR_USART_MODE_NORMAL|   //普通模式
	                US_MR_CHRL_8_BIT|          //数据位为8位
									US_MR_NBSTOP_1_BIT|        //停止位为1位
									US_MR_PAR_NO|              //校验位为NONE
									US_MR_CHMODE_NORMAL;       //普通通道模式
	/*禁止 DMA 通道 */
	USART0->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
	/*使能USART接收和发送*/
	USART0->US_CR = US_CR_RXEN | US_CR_TXEN;
	/*使能接收中断*/
	USART0->US_IER=US_IER_RXRDY;
   /*配置USART0的先占优先级为1，从优先级为1*/
   NVIC_SetPriority(USART0_IRQn, ((0x01<<3)|0x01));
	/*使能USART0的中断通道*/
    NVIC_EnableIRQ(USART0_IRQn);
}
/*******************************************************************************************
* 函数名：USART0_Handler()
* 参数  ：void
* 返回值：void
* 描述  ：USART0中断服务函数
*********************************************************************************************/
void USART0_Handler(void)
 { 
	 uint8_t temp;
	 if((USART0->US_CSR& US_CSR_RXRDY)==1)
	 {                                            //接收数据中断
			  temp= USART0->US_RHR&0xff;		         //接收一个字节
       USART0_SendByte(temp);					         //将接收的数据发回
   }
 }
  /*******************************************************************************************
* 函数名：USART0_SendByte()
* 参数  ：uint8_t c  要发送字符
* 返回值：void
* 描述  ：USART0发送一个字符函数
*********************************************************************************************/
void USART0_SendByte(uint8_t c)
{
   while((USART0->US_CSR & US_CSR_TXEMPTY) == 0);		//等待发送缓冲器为空
   USART0->US_THR=c;							//将发送字符写入发送保持寄存器
}
 /*******************************************************************************************
* 函数名：USART0_SendString()
* 参数  ：uint8_t *s  指向字符串的指针
* 返回值：void
* 描述  ：USART0发送字符串函数
*********************************************************************************************/
void USART0_SendString(uint8_t *s)
{
  while(*s)												//判断是否到字符串末尾
  {
   USART0_SendByte(*s);							//发送指针当前所指的字节
   s++;													//地址加1
  }

}
/*******************************************************************************************
* 函数名：USART1_Init()
* 参数  ：uint32_t buadrate 波特率
* 返回值：void
* 描述  ：USART1初始化函数，在使用USART1前先调用
*********************************************************************************************/
void USART1_Init(uint32_t baudrate)
{
	/*禁止外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*使能USART1和PIOB时钟*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOA) |   
                    (1UL << ID_USART1) );  
	/*使能外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*配置PA21为USART1的RXD，PA22为USART1的TXD*/	
	PIOA->PIO_IDR=(PIO_PA21A_RXD1|PIO_PA22A_TXD1);
	PIOA->PIO_PUDR=(PIO_PA21A_RXD1|PIO_PA22A_TXD1);
	PIOA->PIO_ABCDSR[0]&=~(PIO_PA21A_RXD1|PIO_PA22A_TXD1);
	PIOA->PIO_ABCDSR[1]&=~(PIO_PA21A_RXD1|PIO_PA22A_TXD1);
	PIOA->PIO_PDR=(PIO_PA21A_RXD1|PIO_PA22A_TXD1);
		/* 复位并禁止USART的发送和接收*/
	USART1->US_CR = US_CR_RSTRX | US_CR_RSTTX
			| US_CR_RXDIS | US_CR_TXDIS;
	/*配置USART1的波特率*/
	USART1->US_BRGR=BAUD(baudrate);
	/*定义数据位为8bit，停止位为1，校验位为NONE*/
	USART1->US_MR = US_MR_USART_MODE_NORMAL|   //普通模式
	                US_MR_CHRL_8_BIT|          //数据位为8位
									US_MR_NBSTOP_1_BIT|        //停止位为1位
									US_MR_PAR_NO|              //校验位为NONE
									US_MR_CHMODE_NORMAL;       //普通通道模式
	/*禁止 DMA 通道 */
	USART1->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
	/*使能USART接收和发送*/
	USART1->US_CR = US_CR_RXEN | US_CR_TXEN;
	/*使能接收中断*/
	USART1->US_IER=US_IER_RXRDY;
   /*配置USART1的先占优先级为1，从优先级为1*/
   NVIC_SetPriority(USART1_IRQn, ((0x01<<3)|0x01));
	/*使能USART1的中断通道*/
    NVIC_EnableIRQ(USART1_IRQn);

}
/*******************************************************************************************
* 函数名：USART1_Handler()
* 参数  ：void
* 返回值：void
* 描述  ：USART1中断服务函数
*********************************************************************************************/
void USART1_Handler(void)
 { 
	 uint8_t temp;
	 if((USART1->US_CSR& US_CSR_RXRDY)==1)
	 {                                            //接收数据中断
			  temp= USART1->US_RHR&0xff;		         //接收一个字节
       USART1_SendByte(temp);					         //将接收的数据发回
   }
 }
  /*******************************************************************************************
* 函数名：USART1_SendByte()
* 参数  ：uint8_t c  要发送字符
* 返回值：void
* 描述  ：USART1发送一个字符函数
*********************************************************************************************/
void USART1_SendByte(uint8_t c)
{
   while((USART1->US_CSR & US_CSR_TXEMPTY) == 0);		//等待发送缓冲器为空
   USART1->US_THR=c;							//将发送字符写入发送保持寄存器
}
 /*******************************************************************************************
* 函数名：USART1_SendString()
* 参数  ：uint8_t *s  指向字符串的指针
* 返回值：void
* 描述  ：USART1发送字符串函数
*********************************************************************************************/
void USART1_SendString(uint8_t *s)
{
  while(*s)												//判断是否到字符串末尾
  {
   USART1_SendByte(*s);							//发送指针当前所指的字节
   s++;													//地址加1
  }

}
/*******************************************************************************************
* 函数名：USART2_Init()
* 参数  ：uint32_t buadrate 波特率
* 返回值：void
* 描述  ：USART2初始化函数，在使用USART2前先调用
*********************************************************************************************/
void USART2_Init(uint32_t baudrate)
{
	/*禁止外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*使能USART2和PIOA时钟*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOC) |   
                    (1UL << ID_USART2) );  
	/*使能外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*配置PC9为USART2的RXD，PC10为USART2的TXD*/	
	PIOC->PIO_IDR=(PIO_PC9A_RXD2|PIO_PC10A_TXD2);
	PIOC->PIO_PUDR=(PIO_PC9A_RXD2|PIO_PC10A_TXD2);
	PIOC->PIO_ABCDSR[0]&=~(PIO_PC9A_RXD2|PIO_PC10A_TXD2);
	PIOC->PIO_ABCDSR[1]&=~(PIO_PC9A_RXD2|PIO_PC10A_TXD2);
	PIOC->PIO_PDR=(PIO_PC9A_RXD2|PIO_PC10A_TXD2);
		/* 复位并禁止USART的发送和接收*/
	USART2->US_CR = US_CR_RSTRX | US_CR_RSTTX
			| US_CR_RXDIS | US_CR_TXDIS;
	/*配置USART2的波特率*/
	USART2->US_BRGR=BAUD(baudrate);
	/*定义数据位为8bit，停止位为1，校验位为NONE*/
	USART2->US_MR = US_MR_USART_MODE_NORMAL|   //普通模式
	                US_MR_CHRL_8_BIT|          //数据位为8位
									US_MR_NBSTOP_1_BIT|        //停止位为1位
									US_MR_PAR_NO|              //校验位为NONE
									US_MR_CHMODE_NORMAL;       //普通通道模式
	/*禁止 DMA 通道 */
	USART2->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
	/*使能USART接收和发送*/
	USART2->US_CR = US_CR_RXEN | US_CR_TXEN;
	/*使能接收中断*/
	USART2->US_IER=US_IER_RXRDY;
   /*配置USART2的先占优先级为1，从优先级为1*/
   NVIC_SetPriority(USART2_IRQn, ((0x01<<3)|0x01));
	/*使能USART2的中断通道*/
    NVIC_EnableIRQ(USART2_IRQn);
}
/*******************************************************************************************
* 函数名：USART2_Handler()
* 参数  ：void
* 返回值：void
* 描述  ：USART2中断服务函数
*********************************************************************************************/
void USART2_Handler(void)
 { 
	 uint8_t temp;
	 if((USART2->US_CSR& US_CSR_RXRDY)==1)
	 {                                            //接收数据中断
			  temp= USART2->US_RHR&0xff;		         //接收一个字节
       USART2_SendByte(temp);					         //将接收的数据发回
   }
 }
  /*******************************************************************************************
* 函数名：USART2_SendByte()
* 参数  ：uint8_t c  要发送字符
* 返回值：void
* 描述  ：USART2发送一个字符函数
*********************************************************************************************/
void USART2_SendByte(uint8_t c)
{
    while((USART2->US_CSR & US_CSR_TXEMPTY) == 0);		//等待发送缓冲器为空
   USART2->US_THR=c;							//将发送字符写入发送保持寄存器
}
 /*******************************************************************************************
* 函数名：USART2_SendString()
* 参数  ：uint8_t *s  指向字符串的指针
* 返回值：void
* 描述  ：USART2发送字符串函数
*********************************************************************************************/
void USART2_SendString(uint8_t *s)
{
  while(*s)												  //判断是否到字符串末尾
  {
   USART2_SendByte(*s);							//发送指针当前所指的字节
   s++;													  //地址加1
  }

}
