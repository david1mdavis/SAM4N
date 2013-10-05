#ifndef __DRV_LED_H
#define __DRV_LED_H
#include "sam4n.h"

#define LED0_PIN   14
#define led_hw_on() PIOB->PIO_CODR|=(0x01<<LED0_PIN)
#define led_hw_off() PIOB->PIO_SODR|=(0x01<<LED0_PIN)
void led_hw_init(void);

#endif
