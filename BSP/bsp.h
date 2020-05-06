/*
*********************************************************************************************************
*
*	ģ������ : BSPģ��
*	�ļ����� : bsp.h
*	��    �� : V1.0
*	˵    �� : ����Ӳ���ײ�������������ļ���ÿ��c�ļ����� #include "bsp.h" ���������е���������ģ�顣
*			   bsp = Borad surport packet �弶֧�ְ�
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

/* ����ȫ���жϵĺ� */
#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

/* �������ȼ����� */
#define NVIC_PREEMPT_PRIORITY	4

/* ͨ��ȡ��ע�ͻ������ע�͵ķ�ʽ�����Ƿ�����ײ�����ģ�� */

#include "bsp_gpio.h"
#include "bsp_key.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
/* �ṩ������C�ļ����õĺ��� */
void bsp_init(void);
void bsp_idle(void);


#endif
