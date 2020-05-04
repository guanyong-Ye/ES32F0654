

#ifndef __GPIO_H
#define __GPIO_H

#include "es32f0654_conf.h"

#define GPIO_OUT_NUM 2

#define RED_GPIO	GPIOC
#define RED_PIN		GPIO_PIN_8
#define GREEN_GPIO	GPIOC
#define GREEN_PIN	GPIO_PIN_9

/* 依次定义GPIO */
typedef struct
{
	GPIO_TypeDef* gpio;
	uint16_t pin;
}gpio_out;


/* 供外部调用的函数声明 */
void out_gpio_init(void);
void set_pin_high(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void set_pin_low(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void toggle_pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
#endif

