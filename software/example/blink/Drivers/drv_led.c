#include "drv_led.h"

void led_hw_init(void)
{
 /*ʹ��LED�ܽ�*/
 PIOB->PIO_PER|=(0x01<<LED0_PIN);
	/*ʹ��LED�ܽ����*/
 PIOB->PIO_OER|=(0x01<<LED0_PIN);
	/*���LED�������LED�ܽ�Ϊ�͵�ƽ*/
 PIOB->PIO_CODR|=(0x01<<LED0_PIN);
}





