#include "drv_led.h"

void led_hw_init(void)
{
 /*开启PIOB时钟*/
	PMC->PMC_PCER0=(0x01<<ID_PIOB);
 /*使能LED管脚*/
 PIOB->PIO_PER=(0x01<<LED0_PIN);
	/*使能LED管脚输出*/
 PIOB->PIO_OER=(0x01<<LED0_PIN);
	/*清除LED输出，即LED管脚为低电平*/
 PIOB->PIO_CODR=(0x01<<LED0_PIN);
}





