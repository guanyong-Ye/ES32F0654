

#include "bsp.h"

/* GPIO和PIN定义 */
static const gpio_out gpio_list[GPIO_OUT_NUM] = {
	{RED_GPIO, RED_PIN},		/* GPIO1 */
	{GREEN_GPIO, GREEN_PIN},		/* GPIO2 */
};	
/*
*********************************************************************************************************
*	函 数 名: gpio_init_hard
*	功能说明: 配置GPIO
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void out_gpio_init(void)
{
	gpio_init_t	gpio_init;
	uint8_t i = 0;
	
	/* 配置所有的按键GPIO输出 */
	gpio_init.mode = GPIO_MODE_OUTPUT;	
	gpio_init.odos = GPIO_PUSH_PULL;
	gpio_init.pupd = GPIO_PUSH_UP;
	gpio_init.odrv = GPIO_OUT_DRIVE_NORMAL;
	gpio_init.flt  = GPIO_FILTER_DISABLE;
	gpio_init.type = GPIO_TYPE_CMOS;
	gpio_init.func = GPIO_FUNC_1;
	
	for (i = 0; i < GPIO_OUT_NUM; i++)
	{
		ald_gpio_init(gpio_list[i].gpio,gpio_list[i].pin, &gpio_init);	
	}
	RED_GPIO->BSRR |= RED_PIN;
	GREEN_GPIO->BSRR |= GREEN_PIN;
}
/*
*********************************************************************************************************
* 函 数 名: set_pin_high
* 功能说明: 将GPIO置高
* 形 参: 无
* 返 回 值: 无
*********************************************************************************************************
*/
void set_pin_high(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIOx->BSRR |= (uint32_t)GPIO_Pin ;
}
/*
*********************************************************************************************************
* 函 数 名: set_pin_low
* 功能说明: 将GPIO置低
* 形 参: 无
* 返 回 值: 无
*********************************************************************************************************
*/
void set_pin_low(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIOx->BSRR |= GPIO_Pin << 16U;
}
/*
*********************************************************************************************************
* 函 数 名: toggle_pin
* 功能说明: 翻转电平
* 形 参: 无
* 返 回 值: 无
*********************************************************************************************************
*/
void toggle_pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	ald_gpio_toggle_pin(GPIOx,GPIO_Pin);
}

