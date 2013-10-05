#include "drv_led.h"

void led_hw_init(void)
{
	/*禁止外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4300; 
 /*开启PIOB时钟*/
	PMC->PMC_PCER0=(0x01UL<<ID_PIOB);
	/*使能外设管理控制寄存器(PMC)写保护*/
  PMC->PMC_WPMR = 0x504D4301; 
 /*使能LED管脚*/
 PIOB->PIO_PER|=(0x01<<LED0_PIN);
	/*使能LED管脚输出*/
 PIOB->PIO_OER|=(0x01<<LED0_PIN);
	/*设置LED输出，即LED管脚为高电平*/
 PIOB->PIO_SODR|=(0x01<<LED0_PIN);
}





