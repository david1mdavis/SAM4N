/*
 * File      : drv_usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-10-04     xiaonong      first version
 */
#include "drv_usart.h"

struct sam4_usart
{
    struct rt_serial_device serial;
    Usart  *usart_device;
	  IRQn_Type usart_irq;
    struct serial_ringbuffer int_rx;
};
/*�����ʼ���*/
#define BAUD(b) ((SystemCoreClock + 8*b)/(16*b))


#ifdef RT_USING_USART0
static struct sam4_usart usart0;

void USART0_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&usart0.serial);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif // RT_USING_USART0

#ifdef RT_USING_USART1
static struct sam4_usart usart1;

void USART1_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&usart1.serial);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif // RT_USING_USART1

#ifdef RT_USING_USART2
static struct sam4_usart usart2;

void USART2_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&usart2.serial);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif // RT_USING_USART2

static rt_err_t sam4_usart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct sam4_usart *usart;

    RT_ASSERT(serial != RT_NULL);
    usart = (struct sam4_usart *)serial->parent.user_data;
    /* ��λ����ֹUSART�ķ��ͺͽ���*/
	usart->usart_device->US_CR = US_CR_RSTRX | US_CR_RSTTX
			| US_CR_RXDIS | US_CR_TXDIS;
	  /*config baudrate*/
    usart->usart_device->US_BRGR = BAUD(cfg->baud_rate);
    /*config databits*/
	
    switch (cfg->data_bits)
    {
    case DATA_BITS_5:
			 usart->usart_device->US_MR &=~US_MR_CHRL_8_BIT;
        usart->usart_device->US_MR |= US_MR_CHRL_5_BIT;
        break;
    case DATA_BITS_6:
			  usart->usart_device->US_MR &=~US_MR_CHRL_8_BIT;
        usart->usart_device->US_MR |= US_MR_CHRL_6_BIT;
        break;
    case DATA_BITS_7:
			  usart->usart_device->US_MR &=~US_MR_CHRL_8_BIT;
        usart->usart_device->US_MR |= US_MR_CHRL_7_BIT;
        break;
    case DATA_BITS_8:
			  usart->usart_device->US_MR &=~US_MR_CHRL_8_BIT;
        usart->usart_device->US_MR |= US_MR_CHRL_8_BIT;
        break;
    default:
			usart->usart_device->US_MR &=~US_MR_CHRL_8_BIT;
        usart->usart_device->US_MR |= US_MR_CHRL_8_BIT;
        break;
    }
    /*config stopbits*/
   
    switch (cfg->stop_bits)
    {
    case STOP_BITS_1:
			  usart->usart_device->US_MR&=~(0x03<<12);
        usart->usart_device->US_MR |= US_MR_NBSTOP_1_BIT;
        break;
    case STOP_BITS_2:
        usart->usart_device->US_MR&=~(0x03<<12);
        usart->usart_device->US_MR |= US_MR_NBSTOP_1_BIT;
        break;
    default:
			usart->usart_device->US_MR&=~(0x03<<12);
        usart->usart_device->US_MR |= US_MR_NBSTOP_1_BIT;
        break;
    }
    /*config parity*/
    switch (cfg->parity)
    {
    case PARITY_NONE:
			  usart->usart_device->US_MR&=~(0x07<<9);
        usart->usart_device->US_MR |= US_MR_PAR_NO;
        break;
    case PARITY_EVEN:
         usart->usart_device->US_MR&=~(0x07<<9);
        usart->usart_device->US_MR |= US_MR_PAR_EVEN;
        break;
    case PARITY_ODD:
         usart->usart_device->US_MR&=~(0x07<<9);
        usart->usart_device->US_MR |= US_MR_PAR_ODD;
        break;
    default:
        usart->usart_device->US_MR&=~(0x07<<9);
        usart->usart_device->US_MR |= US_MR_PAR_NO;
        break;
    }
    /*config transfer direction*/
    switch (cfg->bit_order)
    {
    case BIT_ORDER_LSB:
			  usart->usart_device->US_MR&=~US_MR_MSBF;
        break;
    case BIT_ORDER_MSB:
        usart->usart_device->US_MR|=US_MR_MSBF;
        break;
    default:
        usart->usart_device->US_MR&=~US_MR_MSBF;
        break;
    }
    /*ʹ��USART���պͷ���*/
	USART0->US_CR = US_CR_RXEN | US_CR_TXEN;

    return RT_EOK;
}
static rt_err_t sam4_usart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct sam4_usart *usart;

    RT_ASSERT(serial != RT_NULL);
    usart = (struct sam4_usart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable usart irq */
        NVIC_DisableIRQ(usart->usart_irq);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable usart irq */
        NVIC_EnableIRQ(usart->usart_irq);
        break;
    }

    return RT_EOK;
}
static int sam4_usart_putc(struct rt_serial_device *serial, char c)
{
    struct sam4_usart *usart;

    RT_ASSERT(serial != RT_NULL);
    usart = (struct sam4_usart *)serial->parent.user_data;

    while ((usart->usart_device->US_CSR & US_CSR_TXEMPTY)==0);
    usart->usart_device->US_THR = (c & 0x1FF);

    return 1;
}

static int sam4_usart_getc(struct rt_serial_device *serial)
{
    int ch;
    struct sam4_usart *usart;

    RT_ASSERT(serial != RT_NULL);
    usart = (struct sam4_usart *)serial->parent.user_data;

    ch = -1;
    if (usart->usart_device->US_CSR& US_CSR_RXRDY)
    {
        ch = usart->usart_device->US_RHR & 0xff;
    }

    return ch;
}

static const struct rt_uart_ops sam4_usart_ops =
{
    sam4_usart_configure,
    sam4_usart_control,
    sam4_usart_putc,
    sam4_usart_getc,
};
void rt_hw_usart_init(void)
{
    struct sam4_usart *usart;

#ifdef RT_USING_USART0
    usart = &usart0;
    usart->usart_device = USART0;
    usart->usart_irq = USART0_IRQn;
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
	USART0->US_BRGR=BAUD(BAUD_RATE_115200);
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
	/*ʹ��USART0���ж�ͨ��*/
    NVIC_EnableIRQ(USART0_IRQn);

    usart->serial.config.baud_rate = BAUD_RATE_115200;
    usart->serial.config.bit_order = BIT_ORDER_LSB;
    usart->serial.config.data_bits = DATA_BITS_8;
    usart->serial.config.parity    = PARITY_NONE;
    usart->serial.config.stop_bits = STOP_BITS_1;
    usart->serial.config.invert    = NRZ_NORMAL;

    usart->serial.ops = &sam4_usart_ops;
    usart->serial.int_rx = &usart->int_rx;

    /* register UART1 device */
    rt_hw_serial_register(&usart->serial,
                          "usart0",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          usart);
#endif // RT_USING_USART0

#ifdef RT_USING_USART1
    usart = &usart1;

    usart->usart_device = USART1;
    usart->usart_irq = USART1_IRQn;


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
	USART1->US_BRGR=BAUD(BAUD_RATE_115200);
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
	/*ʹ��USART1���ж�ͨ��*/
    NVIC_EnableIRQ(USART1_IRQn);

    usart->serial.config.baud_rate = BAUD_RATE_115200;
    usart->serial.config.bit_order = BIT_ORDER_LSB;
    usart->serial.config.data_bits = DATA_BITS_8;
    usart->serial.config.parity    = PARITY_NONE;
    usart->serial.config.stop_bits = STOP_BITS_1;
    usart->serial.config.invert    = NRZ_NORMAL;

    usart->serial.ops = &sam4_usart_ops;
    usart->serial.int_rx = &usart->int_rx;

    /* register UART1 device */
    rt_hw_serial_register(&usart->serial,
                          "usart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          usart);
#endif // RT_USING_USART1

#ifdef RT_USING_USART2
    usart = &usart2;

    usart->usart_device = USART2;
    usart->usart_irq = USART2_IRQn;

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
	USART2->US_BRGR=BAUD(BAUD_RATE_115200);
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
	/*ʹ��USART2���ж�ͨ��*/
    NVIC_EnableIRQ(USART2_IRQn);
    usart->serial.config.baud_rate = BAUD_RATE_115200;
    usart->serial.config.bit_order = BIT_ORDER_LSB;
    usart->serial.config.data_bits = DATA_BITS_8;
    usart->serial.config.parity    = PARITY_NONE;
    usart->serial.config.stop_bits = STOP_BITS_1;
    usart->serial.config.invert    = NRZ_NORMAL;

    usart->serial.ops = &sam4_usart_ops;
    usart->serial.int_rx = &usart->int_rx;

    /* register UART2 device */
    rt_hw_serial_register(&usart->serial,
                          "usart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
                          usart);
#endif // RT_USING_USART2


}
