/*
*********************************************************************************************************
*
*	模块名称 : BSP模块
*	文件名称 : bsp.h
*	版    本 : V1.0
*	说    明 : 这是硬件底层驱动程序的主文件。每个c文件可以 #include "bsp.h" 来包含所有的外设驱动模块。
*			   bsp = Borad surport packet 板级支持包
*
*********************************************************************************************************
*/

#ifndef _BSP_H_
#define _BSP_H_

#include "es32f0654_conf.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* 开关全局中断的宏 */
#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

/* 定义优先级分组 */
#define NVIC_PREEMPT_PRIORITY	4

/* 通过取消注释或者添加注释的方式控制是否包含底层驱动模块 */

#include "bsp_gpio.h"
#include "bsp_key.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
/* 提供给其他C文件调用的函数 */
void bsp_init(void);
void bsp_idle(void);


#endif
