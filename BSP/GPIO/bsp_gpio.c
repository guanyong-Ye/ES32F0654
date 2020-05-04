

#include "bsp.h"

/* GPIO��PIN���� */
static const gpio_out gpio_list[GPIO_OUT_NUM] = {
	{RED_GPIO, RED_PIN},		/* GPIO1 */
	{GREEN_GPIO, GREEN_PIN},		/* GPIO2 */
};	
/*
*********************************************************************************************************
*	�� �� ��: gpio_init_hard
*	����˵��: ����GPIO
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void out_gpio_init(void)
{
	gpio_init_t	gpio_init;
	uint8_t i = 0;
	
	/* �������еİ���GPIO��� */
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
* �� �� ��: set_pin_high
* ����˵��: ��GPIO�ø�
* �� ��: ��
* �� �� ֵ: ��
*********************************************************************************************************
*/
void set_pin_high(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIOx->BSRR |= (uint32_t)GPIO_Pin ;
}
/*
*********************************************************************************************************
* �� �� ��: set_pin_low
* ����˵��: ��GPIO�õ�
* �� ��: ��
* �� �� ֵ: ��
*********************************************************************************************************
*/
void set_pin_low(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIOx->BSRR |= GPIO_Pin << 16U;
}
/*
*********************************************************************************************************
* �� �� ��: toggle_pin
* ����˵��: ��ת��ƽ
* �� ��: ��
* �� �� ֵ: ��
*********************************************************************************************************
*/
void toggle_pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	ald_gpio_toggle_pin(GPIOx,GPIO_Pin);
}

