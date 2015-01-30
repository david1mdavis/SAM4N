#ifndef __DRV_USART_H
#define __DRV_USART_H
/************************************************************************************
*文件名  ：drv_usart.h																	 *
*文件作用：异步串口通信																	 *
*作者    ：农晓明																	 *
*作者QQ  ：382421307																 *
*文件创建时间：2013/10/01															 *																					 *
*************************************************************************************/
#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include "sam4n.h"
#include "stdio.h"
#include "board.h"

void rt_hw_usart_init(void);
#endif
