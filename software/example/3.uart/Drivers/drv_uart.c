#include "drv_uart.h"
/************************************************************************************
*文件名  ：drv_uart.c																                                *
*文件作用：串口通信																	                                *
*作者    ：农晓明																	                                  *
*作者QQ  ：382421307																                                  *
*文件创建时间：2013/09/29															  														*
*************************************************************************************/

/* UART 波特率计算固定分频系数 */
#define UART_MCK_DIV             16
/* 波特率设置最小分频系数*/
#define UART_MCK_DIV_MIN_FACTOR  1
/* 波特率设置最大分频系数 */
#define UART_MCK_DIV_MAX_FACTOR  65535

/*******************************************************************************************
* 函数名：UART0_Init()
* 参数  ：uint32_t buadrate 波特率
* 返回值：void
* 描述  ：UART0初始化函数，在使用UART0前先调用
*********************************************************************************************/
void UART0_Init(uint32_t baudrate)
{
  	
  uint32_t Fdiv =0;
	/*禁止外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*使能UART0和PIOA时钟*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOA) |   
                    (1UL << ID_UART0) );  
	/*使能外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*配置PA9为UART0的RXD，PA10为UART0的TXD*/	
	PIOA->PIO_IDR=(PIO_PA9A_URXD0|PIO_PA10A_UTXD0);
	PIOA->PIO_PUDR=(PIO_PA9A_URXD0|PIO_PA10A_UTXD0);
	PIOA->PIO_ABCDSR[0]&=~(PIO_PA9A_URXD0|PIO_PA10A_UTXD0);
	PIOA->PIO_ABCDSR[1]&=~(PIO_PA9A_URXD0|PIO_PA10A_UTXD0);
	PIOA->PIO_PDR=(PIO_PA9A_URXD0|PIO_PA10A_UTXD0);
		/* 复位并禁止UART的发送和接收*/
	UART0->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
			| UART_CR_RXDIS | UART_CR_TXDIS;
	/*配置UART0的波特率*/
	Fdiv = (SystemCoreClock/  baudrate) /UART_MCK_DIV ;
	if (Fdiv < UART_MCK_DIV_MIN_FACTOR || Fdiv > UART_MCK_DIV_MAX_FACTOR)
		return ;
	UART0->UART_BRGR=Fdiv;
	/*定义数据位为8bit，停止位为1，校验位为NONE*/
	UART0->UART_MR = UART_MR_PAR_NO|UART_MR_CHMODE_NORMAL;
	/*禁止 DMA 通道 */
	UART0->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	/*使能UART接收和发送*/
	UART0->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
	/*使能接收中断*/
	UART0->UART_IER=UART_IER_RXRDY;
   /*配置UART0的先占优先级为1，从优先级为1*/
   NVIC_SetPriority(UART0_IRQn, ((0x01<<3)|0x01));
	/*使能UART0的中断通道*/
    NVIC_EnableIRQ(UART0_IRQn);
}
/*******************************************************************************************
* 函数名：UART0_Handler()
* 参数  ：void
* 返回值：void
* 描述  ：UART0中断服务函数
*********************************************************************************************/
void UART0_Handler(void)
 { 
	 uint8_t temp;
	 if((UART0->UART_SR& UART_SR_RXRDY)==1)
	 {                                            //接收数据中断
			  temp= UART0->UART_RHR&0xff;		         //接收一个字节
       UART0_SendByte(temp);					         //将接收的数据发回
   }
 }
  /*******************************************************************************************
* 函数名：UART0_SendByte()
* 参数  ：uint8_t c  要发送字符
* 返回值：void
* 描述  ：UART0发送一个字符函数
*********************************************************************************************/
void UART0_SendByte(uint8_t c)
{
   while((UART0->UART_SR & UART_SR_TXEMPTY) == 0);		//等待发送缓冲器为空
   UART0->UART_THR=c;							//将发送字符写入发送保持寄存器
}
 /*******************************************************************************************
* 函数名：UART0_SendString()
* 参数  ：uint8_t *s  指向字符串的指针
* 返回值：void
* 描述  ：UART0发送字符串函数
*********************************************************************************************/
void UART0_SendString(uint8_t *s)
{
  while(*s)												//判断是否到字符串末尾
  {
   UART0_SendByte(*s);							//发送指针当前所指的字节
   s++;													//地址加1
  }

}
/*******************************************************************************************
* 函数名：UART1_Init()
* 参数  ：uint32_t buadrate 波特率
* 返回值：void
* 描述  ：UART1初始化函数，在使用UART1前先调用
*********************************************************************************************/
void UART1_Init(uint32_t baudrate)
{
  	
  uint32_t Fdiv =0;
	/*禁止外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*使能UART1和PIOB时钟*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOB) |   
                    (1UL << ID_UART1) );  
	/*使能外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*配置PB2为UART1的RXD，PB3为UART1的TXD*/	
	PIOB->PIO_IDR=(PIO_PB2A_URXD1|PIO_PB3A_UTXD1);
	PIOB->PIO_PUDR=(PIO_PB2A_URXD1|PIO_PB3A_UTXD1);
	PIOB->PIO_ABCDSR[0]&=~(PIO_PB2A_URXD1|PIO_PB3A_UTXD1);
	PIOB->PIO_ABCDSR[1]&=~(PIO_PB2A_URXD1|PIO_PB3A_UTXD1);
	PIOB->PIO_PDR=(PIO_PB2A_URXD1|PIO_PB3A_UTXD1);
		/* 复位并禁止UART的发送和接收*/
	UART1->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
			| UART_CR_RXDIS | UART_CR_TXDIS;
	/*配置UART1的波特率*/
	Fdiv = (SystemCoreClock/  baudrate) /UART_MCK_DIV ;
	if (Fdiv < UART_MCK_DIV_MIN_FACTOR || Fdiv > UART_MCK_DIV_MAX_FACTOR)
		return ;
	UART1->UART_BRGR=Fdiv;
	/*定义数据位为8bit，停止位为1，校验位为NONE*/
	UART1->UART_MR = UART_MR_PAR_NO|UART_MR_CHMODE_NORMAL;
	/*禁止 DMA 通道 */
	UART1->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	/*使能UART接收和发送*/
	UART1->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
	/*使能接收中断*/
	UART1->UART_IER=UART_IER_RXRDY;
   /*配置UART1的先占优先级为1，从优先级为1*/
   NVIC_SetPriority(UART1_IRQn, ((0x01<<3)|0x01));
	/*使能UART1的中断通道*/
    NVIC_EnableIRQ(UART1_IRQn);

}
/*******************************************************************************************
* 函数名：UART1_Handler()
* 参数  ：void
* 返回值：void
* 描述  ：UART1中断服务函数
*********************************************************************************************/
void UART1_Handler(void)
 { 
	 uint8_t temp;
	 if((UART1->UART_SR& UART_SR_RXRDY)==1)
	 {                                            //接收数据中断
			  temp= UART1->UART_RHR&0xff;		         //接收一个字节
       UART1_SendByte(temp);					         //将接收的数据发回
   }
 }
  /*******************************************************************************************
* 函数名：UART1_SendByte()
* 参数  ：uint8_t c  要发送字符
* 返回值：void
* 描述  ：UART1发送一个字符函数
*********************************************************************************************/
void UART1_SendByte(uint8_t c)
{
   while((UART1->UART_SR & UART_SR_TXEMPTY) == 0);		//等待发送缓冲器为空
   UART1->UART_THR=c;							//将发送字符写入发送保持寄存器
}
 /*******************************************************************************************
* 函数名：UART1_SendString()
* 参数  ：uint8_t *s  指向字符串的指针
* 返回值：void
* 描述  ：UART1发送字符串函数
*********************************************************************************************/
void UART1_SendString(uint8_t *s)
{
  while(*s)												//判断是否到字符串末尾
  {
   UART1_SendByte(*s);							//发送指针当前所指的字节
   s++;													//地址加1
  }

}
/*******************************************************************************************
* 函数名：UART2_Init()
* 参数  ：uint32_t buadrate 波特率
* 返回值：void
* 描述  ：UART2初始化函数，在使用UART2前先调用
*********************************************************************************************/
void UART2_Init(uint32_t baudrate)
{
   	
  uint32_t Fdiv =0;
	/*禁止外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*使能UART2和PIOA时钟*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOA) |   
                    (1UL << ID_UART2) );  
	/*使能外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*配置PA16为UART2的RXD，PA15为UART2的TXD*/	
	PIOA->PIO_IDR=(PIO_PA16A_URXD2|PIO_PA15A_UTXD2);
	PIOA->PIO_PUDR=(PIO_PA16A_URXD2|PIO_PA15A_UTXD2);
	PIOA->PIO_ABCDSR[0]&=~(PIO_PA16A_URXD2|PIO_PA15A_UTXD2);
	PIOA->PIO_ABCDSR[1]&=~(PIO_PA16A_URXD2|PIO_PA15A_UTXD2);
	PIOA->PIO_PDR=(PIO_PA16A_URXD2|PIO_PA15A_UTXD2);
		/* 复位并禁止UART的发送和接收*/
	UART2->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
			| UART_CR_RXDIS | UART_CR_TXDIS;
	/*配置UART2的波特率*/
	Fdiv = (SystemCoreClock/  baudrate) /UART_MCK_DIV ;
	if (Fdiv < UART_MCK_DIV_MIN_FACTOR || Fdiv > UART_MCK_DIV_MAX_FACTOR)
		return ;
	UART2->UART_BRGR=Fdiv;
	/*定义数据位为8bit，停止位为1，校验位为NONE*/
	UART2->UART_MR = UART_MR_PAR_NO|UART_MR_CHMODE_NORMAL;
	/*禁止 DMA 通道 */
	UART2->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	/*使能UART接收和发送*/
	UART2->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
	/*使能接收中断*/
	UART2->UART_IER=UART_IER_RXRDY;
   /*配置UART2的先占优先级为1，从优先级为1*/
   NVIC_SetPriority(UART2_IRQn, ((0x01<<3)|0x01));
	/*使能UART2的中断通道*/
    NVIC_EnableIRQ(UART2_IRQn);
}
/*******************************************************************************************
* 函数名：UART2_Handler()
* 参数  ：void
* 返回值：void
* 描述  ：UART2中断服务函数
*********************************************************************************************/
void UART2_Handler(void)
 { 
	 uint8_t temp;
	 if((UART2->UART_SR& UART_SR_RXRDY)==1)
	 {                                            //接收数据中断
			  temp= UART2->UART_RHR&0xff;		         //接收一个字节
       UART2_SendByte(temp);					         //将接收的数据发回
   }
 }
  /*******************************************************************************************
* 函数名：UART2_SendByte()
* 参数  ：uint8_t c  要发送字符
* 返回值：void
* 描述  ：UART2发送一个字符函数
*********************************************************************************************/
void UART2_SendByte(uint8_t c)
{
    while((UART2->UART_SR & UART_SR_TXEMPTY) == 0);		//等待发送缓冲器为空
   UART2->UART_THR=c;							//将发送字符写入发送保持寄存器
}
 /*******************************************************************************************
* 函数名：UART2_SendString()
* 参数  ：uint8_t *s  指向字符串的指针
* 返回值：void
* 描述  ：UART2发送字符串函数
*********************************************************************************************/
void UART2_SendString(uint8_t *s)
{
  while(*s)												  //判断是否到字符串末尾
  {
   UART2_SendByte(*s);							//发送指针当前所指的字节
   s++;													  //地址加1
  }

}
/*******************************************************************************************
* 函数名：UART3_Init()
* 参数  ：uint32_t buadrate 波特率
* 返回值：void
* 描述  ：UART3初始化函数，在使用UART1前先调用
*********************************************************************************************/
void UART3_Init(uint32_t baudrate)
{
  	
  uint32_t Fdiv =0;
	/*禁止外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4300; 
  /*使能UART3和PIOB时钟*/	
  PMC->PMC_PCER0 = ((1UL << ID_PIOB) |   
                    (1UL << ID_UART3) );  
	/*使能外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4301;  
  /*配置PB10为UART3的RXD，PB11为UART3的TXD*/	
//	PIOB->PIO_WPMR=0x50494F00;
	PIOB->PIO_IDR=(PIO_PB10B_URXD3|PIO_PB11B_UTXD3);
	PIOB->PIO_PUDR=(PIO_PB10B_URXD3|PIO_PB11B_UTXD3);
	PIOB->PIO_ABCDSR[0]|=(PIO_PB10B_URXD3|PIO_PB11B_UTXD3);
	PIOB->PIO_ABCDSR[1]&=~(PIO_PB10B_URXD3|PIO_PB11B_UTXD3);
	PIOB->PIO_PDR=(PIO_PB10B_URXD3|PIO_PB11B_UTXD3);
	//PIOB->PIO_WPMR=0x50494F01;
		/* 复位并禁止UART的发送和接收*/
	UART3->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
			| UART_CR_RXDIS | UART_CR_TXDIS;
	/*配置UART3的波特率*/
	Fdiv = (SystemCoreClock/  baudrate) /UART_MCK_DIV ;
	if (Fdiv < UART_MCK_DIV_MIN_FACTOR || Fdiv > UART_MCK_DIV_MAX_FACTOR)
		return ;
	UART3->UART_BRGR=Fdiv;
	/*定义数据位为8bit，停止位为1，校验位为NONE*/
	UART3->UART_MR = UART_MR_PAR_NO|UART_MR_CHMODE_NORMAL;
	/*禁止 DMA 通道 */
	UART3->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	/*使能UART接收和发送*/
	UART3->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
	/*使能接收中断*/
	UART3->UART_IER=UART_IER_RXRDY;
   /*配置UART1的先占优先级为1，从优先级为1*/
   NVIC_SetPriority(UART3_IRQn, ((0x01<<3)|0x01));
	/*使能UART3的中断通道*/
    NVIC_EnableIRQ(UART3_IRQn);

}
/*******************************************************************************************
* 函数名：UART3_Handler()
* 参数  ：void
* 返回值：void
* 描述  ：UART3中断服务函数
*********************************************************************************************/
void UART3_Handler(void)
 { 
	 uint8_t temp;
	 if((UART3->UART_SR& UART_SR_RXRDY)==1)
	 {                                            //接收数据中断
			  temp= UART3->UART_RHR&0xff;		         //接收一个字节
       UART3_SendByte(temp);					         //将接收的数据发回
   }
 }
  /*******************************************************************************************
* 函数名：UART3_SendByte()
* 参数  ：uint8_t c  要发送字符
* 返回值：void
* 描述  ：UART3发送一个字符函数
*********************************************************************************************/
void UART3_SendByte(uint8_t c)
{
   while((UART3->UART_SR & UART_SR_TXEMPTY) == 0);		//等待发送缓冲器为空
   UART3->UART_THR=c;							//将发送字符写入发送保持寄存器
}
 /*******************************************************************************************
* 函数名：UART3_SendString()
* 参数  ：uint8_t *s  指向字符串的指针
* 返回值：void
* 描述  ：UART3发送字符串函数
*********************************************************************************************/
void UART3_SendString(uint8_t *s)
{
  while(*s)												//判断是否到字符串末尾
  {
   UART3_SendByte(*s);							//发送指针当前所指的字节
   s++;													//地址加1
  }

}
/********************************************************************************************************************************
*函数名：fputc()
* 参数：int ch，FILE *f
* 返回值：int
* 功能：重新定义stdio.h中的fputc()函数，使printf()输出到USART1
*********************************************************************************************************************************/
int fputc(int ch,FILE *f)
{
  UART0_SendByte((uint8_t)ch);							//发送1个字节
  return ch;																	   //返回 ch
}
