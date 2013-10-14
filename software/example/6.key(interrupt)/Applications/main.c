#include "sam4n16c.h"
#include "drv_led.h"
#include "delay.h"
#include "drv_uart.h"
#include "drv_key.h"

int main(void)
{
 systick_hw_init();
 led_hw_init();
 UART0_Init(115200);
 Key_GPIO_Config();
 UART0_SendString("this is a key test demo!\r\n");
 while(1){
  PIOB->PIO_CODR=(0x01<<LED0_PIN);
	 delay_ms(200);
  PIOB->PIO_SODR=(0x01<<LED0_PIN);
   delay_ms(200);
}
}
