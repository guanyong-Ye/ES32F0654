/**
  *********************************************************************************
  *
  * @file    irq.c
  * @brief   Interrupt handler
  *
  * @version V1.0
  * @date    07 Nov 2017
  * @author  AE Team
  * @note
  *
  * Copyright (C) Shanghai Eastsoft Microelectronics Co. Ltd. All rights reserved.
  *
  *********************************************************************************
  */

#include "main.h"
#include "utils.h"
#include "bsp.h"

/** @addtogroup Projects_Examples_ALD
  * @{
  */

/** @addtogroup Examples
  * @{
  */

/**
  * @brief  NMI IRQ handler
  * @retval None
  */
void NMI_Handler(void)
{
	/* Added Emergency operation */
	return;
}

/**
  * @brief  Hardfault IRQ handler
  * @retval None
  */
void HardFault_Handler(void)
{
	/* Added debug information */
	while (1)
		;
}

/**
  * @brief  Supervisor Call IRQ handler
  * @retval None
  */
void SVC_Handler(void)
{
	/* Added system callback */
	return;
}

/**
  * @brief  Debug Monitor IRQ handler
  * @retval None
  */
void DebugMon_Handler(void)
{
	/* Added debug operation */
	return;
}

/**
  * @brief  PendSV IRQ handler
  * @retval None
  */
void PendSV_Handler(void)
{
	/* Added thread switching operation */
	return;
}

/**
  * @brief  SysTick IRQ handler
  * @retval None
  */
//void SysTick_Handler(void)
//{
//	ald_inc_tick();
//	return;
//}

/**
  * @brief  CMU IRQ#4 handler
  * @retval None
  */
void CMU_Handler(void)
{
	ald_cmu_irq_handler();
	return;
}

#ifdef ALD_DMA
/**
  * @brief  DMA IRQ#9 handler
  * @retval None
  */
void DMA_Handler(void)
{
	ald_dma_irq_handler();
}
#endif

/**
  * @brief  External IRQ handler
  * @retval None
  */
#ifdef KEY_INTERUPT
void EXTI0_3_Handler(void)
{	
	/* Handle external interrupt */
	if (ald_gpio_exti_get_flag_status(KEY1_PIN)) 
	{
		ald_gpio_exti_clear_flag_status(KEY1_PIN);
		toggle_pin(RED_GPIO, RED_PIN);
	}

	if (ald_gpio_exti_get_flag_status(KEY2_PIN)) 
	{
		ald_gpio_exti_clear_flag_status(KEY2_PIN);
		toggle_pin(GREEN_GPIO, GREEN_PIN);
	}

	return;
}
#endif
/**
  * @}
  */

/**
  * @}
  */
