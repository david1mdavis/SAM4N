/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-09-22     Bernard      add board.h to this bsp
 */

// <<< Use Configuration Wizard in Context Menu >>>
#ifndef __BOARD_H__
#define __BOARD_H__

#include "sam4n.h"

/* board configuration */

/* whether use board external SRAM memory */
// <e>Use external SRAM memory on the board
// 	<i>Enable External SRAM memory
#define SAM4N_EXT_SRAM          0
//	<o>Begin Address of External SRAM
//		<i>Default: 0x68000000
#define SAM4N_EXT_SRAM_BEGIN    0x68000000 /* the begining address of external SRAM */
//	<o>End Address of External SRAM
//		<i>Default: 0x68080000
#define SAM4N_EXT_SRAM_END      0x68080000 /* the end address of external SRAM */
// </e>

// <o> Internal SRAM memory size[Kbytes] <8-64>
//	<i>Default: 64
#define SAM4N_SRAM_SIZE         80
#define SAM4N_SRAM_END          (0x20000000 + SAM4N_SRAM_SIZE * 1024)

/* USART driver select. */
//#define RT_USING_USART0
//#define RT_USING_USART1
//#define RT_USING_USART2
/* UART driver select. */
#define RT_USING_UART0
//#define RT_USING_UART1
//#define RT_USING_UART2
#endif /* __BOARD_H__ */

// <<< Use Configuration Wizard in Context Menu >>>
