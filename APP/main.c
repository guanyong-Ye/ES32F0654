#include "bsp.h"

int main(void)
{
	uint8_t keycode;		/* �������� */
	bsp_init();
	while(1)
	{
#ifdef KEY_ANFULAI
	keycode = bsp_getkey();	/* ��ȡ��ֵ, �޼�����ʱ���� KEY_NONE = 0 */
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


