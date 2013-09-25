#include "sam4n16c.h"
#include "drv_led.h"
void delay(uint32_t x)
{
	uint32_t a,b;
 for(a=0;a<5000;a++)
	{
   for(b=0;b<x;b++);
  }
}
int main(void)
{
 led_hw_init();
 while(1){
 PIOB->PIO_CODR=(0x01<<LED0_PIN);
 delay(300);
 PIOB->PIO_SODR=(0x01<<LED0_PIN);
 delay(300);
 }

}

