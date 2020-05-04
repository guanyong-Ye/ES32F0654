#include "bsp.h"

int main(void)
{
	uint8_t keycode;		/* 按键代码 */
	bsp_init();
	ald_mcu_irq_config(EXTI0_3_IRQn, 3, ENABLE);
	while(1)
	{
#if 0
keycode = bsp_getkey();	/* 读取键值, 无键按下时返回 KEY_NONE = 0 */
switch(keycode)
{
	case KEY_1_DOWN:
		toggle_pin(RED_GPIO,RED_PIN);
	break;
	case KEY_2_DOWN:
		toggle_pin(GREEN_GPIO,GREEN_PIN);
	break;
}
#endif
	}
}


