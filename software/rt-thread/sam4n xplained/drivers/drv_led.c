#include "drv_led.h"

void led_hw_init(void)
{
	/*��ֹ���������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4300; 
 /*����PIOBʱ��*/
	PMC->PMC_PCER0=(0x01UL<<ID_PIOB);
	/*ʹ�����������ƼĴ���(PMC)д����*/
  PMC->PMC_WPMR = 0x504D4301; 
 /*ʹ��LED�ܽ�*/
 PIOB->PIO_PER|=(0x01<<LED0_PIN);
	/*ʹ��LED�ܽ����*/
 PIOB->PIO_OER|=(0x01<<LED0_PIN);
	/*����LED�������LED�ܽ�Ϊ�ߵ�ƽ*/
 PIOB->PIO_SODR|=(0x01<<LED0_PIN);
}





