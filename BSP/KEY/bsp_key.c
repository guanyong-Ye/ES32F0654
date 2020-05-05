
#include "bsp.h"
#ifdef KEY_ANFULAI
static key_t s_tbtn[KEY_COUNT];
static key_fifo_t s_tkey; /* ���� FIFO ����,�ṹ�� */
/* GPIO��PIN���� */
static const x_gpio_t s_gpio_list[HARD_KEY_NUM] = {
	{GPIOF, GPIO_PIN_0, 0},		/* K1 */
	{GPIOF, GPIO_PIN_1, 0},		/* K2 */
};	
/*
*********************************************************************************************************
						����FIFO���
*********************************************************************************************************
*/
/*
*********************************************************************************************************
* �� �� ��: bsp_putkey_fifo
* ����˵��: �� 1 ����ֵѹ�밴�� FIFO ��������������ģ��һ��������
* �� ��: _KeyCode : ��������
* �� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_putkey_fifo(uint8_t keycode)
{
	s_tkey.buf[s_tkey.write] = keycode;
	if(++s_tkey.write >= KEY_FIFO_SIZE)
	{
		s_tkey.write = 0;
	}
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_GetKeyState
*	����˵��: ��ȡ������״̬
*	��    ��:  _ucKeyID : ����ID����0��ʼ
*	�� �� ֵ: 1 ��ʾ���£� 0 ��ʾδ����
*********************************************************************************************************
*/
uint8_t bsp_getkey_state(KEY_ID_E _ucKeyID)
{
	return s_tbtn[_ucKeyID].state;
}
/*
*********************************************************************************************************
* �� �� ��: bsp_GetKey
* ����˵��: �Ӱ��� FIFO ��������ȡһ����ֵ��
* �� ��: ��
* �� �� ֵ: ��������
*********************************************************************************************************
*/
uint8_t bsp_getkey(void)
{
	uint8_t ret;
	if(s_tkey.read == s_tkey.write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tkey.buf[s_tkey.read];
		
		if(++s_tkey.read >= KEY_FIFO_SIZE)
		{
			s_tkey.read = 0;
		}
		return ret;
	}
}
/*
*********************************************************************************************************
* �� �� ��: bsp_GetKey2
* ����˵��: �Ӱ��� FIFO ��������ȡһ����ֵ��
* �� ��: ��
* �� �� ֵ: ��������
*********************************************************************************************************
*/
uint8_t bsp_getkey2(void)
{
	uint8_t ret;
	if(s_tkey.read2 == s_tkey.write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = s_tkey.buf[s_tkey.read2];
		
		if(++s_tkey.read2 >= KEY_FIFO_SIZE)
		{
			s_tkey.read2 = 0;
		}
		return ret;
	}
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_ClearKey
*	����˵��: ��հ���FIFO������
*	��    �Σ���
*	�� �� ֵ: ��������
*********************************************************************************************************
*/
void bsp_clearkey(void)
{
	s_tkey.read = s_tkey.write;
}

/********************************************************************************************************
						��������������
*********************************************************************************************************/

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetKeyParam
*	����˵��: ���ð�������
*	��    �Σ�_ucKeyID : ����ID����0��ʼ
*			_LongTime : �����¼�ʱ��
*			 _RepeatSpeed : �����ٶ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_setkeyparam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t  _RepeatSpeed)
{
	s_tbtn[_ucKeyID].longtime = _LongTime;			/* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
	s_tbtn[_ucKeyID].repeatspeed = _RepeatSpeed;			/* �����������ٶȣ�0��ʾ��֧������ */
	s_tbtn[_ucKeyID].repeatcount = 0;						/* ���������� */
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_initkey_hard
*	����˵��: ���ð�����Ӧ��GPIO
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_initkey_hard(void)
{
	gpio_init_t gpio_init;
	uint8_t i = 0;
	/* �������еİ���GPIO���� */
	gpio_init.mode = GPIO_MODE_INPUT;
	gpio_init.odos = GPIO_PUSH_PULL;
	gpio_init.pupd = GPIO_PUSH_UP;
	gpio_init.odrv = GPIO_OUT_DRIVE_NORMAL;
	gpio_init.flt  = GPIO_FILTER_DISABLE;
	gpio_init.type = GPIO_TYPE_CMOS;
	gpio_init.func = GPIO_FUNC_1;
	
	for (i = 0; i < HARD_KEY_NUM; i++)
	{
		ald_gpio_init(s_gpio_list[i].gpio,s_gpio_list[i].pin, &gpio_init);
	}
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_initkeyvar
*	����˵��: ��ʼ����������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_initkeyvar(void)
{
	uint8_t i;

	/* �԰���FIFO��дָ������ */
	s_tkey.read = 0;
	s_tkey.write = 0;
	s_tkey.read2 = 0;

	/* ��ÿ�������ṹ���Ա������һ��ȱʡֵ */
	for (i = 0; i < KEY_COUNT; i++)
	{
		s_tbtn[i].longtime	= KEY_LONG_TIME;			/* ����ʱ�� 0 ��ʾ����ⳤ�����¼� */
		s_tbtn[i].count    	= KEY_FILTER_TIME / 2;		/* ����������Ϊ�˲�ʱ���һ�� */
		s_tbtn[i].state    	= 0;						/* ����ȱʡ״̬��0Ϊδ���� */
		s_tbtn[i].repeatspeed = 0;						/* �����������ٶȣ�0��ʾ��֧������ */
		s_tbtn[i].repeatcount = 0;						/* ���������� */
	}

	/* �����Ҫ��������ĳ�������Ĳ����������ڴ˵������¸�ֵ */
	
	/* ҡ���������ң�֧�ֳ���1����Զ����� */
//	bsp_setkeyparam(KID_JOY_U, 100, 6);
//	bsp_setkeyparam(KID_JOY_D, 100, 6);
//	bsp_setkeyparam(KID_JOY_L, 100, 6);
//	bsp_setkeyparam(KID_JOY_R, 100, 6);
}

/*
*********************************************************************************************************
* �� �� ��: bsp_initkey
* ����˵��: ��ʼ������. �ú����� bsp_Init() ���á�
* �� ��: ��
* �� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_initkey(void)
{
	bsp_initkeyvar(); /* ��ʼ���������� */
	bsp_initkey_hard(); /* ��ʼ������Ӳ�� */
}

/*
*********************************************************************************************************
*	�� �� ��: keypin_active
*	����˵��: �жϰ����Ƿ���
*	��    ��: ��
*	�� �� ֵ: ����ֵ1 ��ʾ����(��ͨ����0��ʾδ���£��ͷţ�
*********************************************************************************************************
*/
static uint8_t keypin_active(uint8_t _id)
{
	uint8_t level;
	
	if ((s_gpio_list[_id].gpio->DIN & s_gpio_list[_id].pin) == BIT_RESET)
	{
		level = 0;
	}
	else
	{
		level = 1;
	}

	if (level == s_gpio_list[_id].activelevel)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
/*
*********************************************************************************************************
*	�� �� ��: iskeydownfunc
*	����˵��: �жϰ����Ƿ��¡���������ϼ����֡������¼������������������¡�
*	��    ��: ��
*	�� �� ֵ: ����ֵ1 ��ʾ����(��ͨ����0��ʾδ���£��ͷţ�
*********************************************************************************************************
*/
static uint8_t iskeydownfunc(uint8_t id)
{
	uint8_t i = 0;
	uint8_t count = 0;
	uint8_t save = 255;
	
	if(id < HARD_KEY_NUM)
	{
		for(i = 0;i < HARD_KEY_NUM;i++)
		{
			if(keypin_active(i))
			{
				count++;
				save = i;
			}
		}
		if(count == 1 && save == id)
		{
			return 1;	/* ֻ��1��������ʱ����Ч */
		}
		return 0;
	}
	/* ��ϼ� K1K2 */
	if (id == HARD_KEY_NUM + 0)
	{
		if (keypin_active(KID_K1) && keypin_active(KID_K2))
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	return 0;
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_DetectKey
*	����˵��: ���һ��������������״̬�����뱻�����Եĵ��á�
*	��    ��: IO��id�� ��0��ʼ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_detectkey(uint8_t i)
{
	key_t *pbtn;
	pbtn = &s_tbtn[i];
	if(iskeydownfunc(i))
	{
		if(pbtn->count < KEY_FILTER_TIME)
			pbtn->count = KEY_FILTER_TIME;
		else if(pbtn->count < 2 * KEY_FILTER_TIME)
			pbtn->count++;
		else
		{
			/* �̰� */
			if(pbtn->state == 0)
			{
				pbtn->state = 1;
				/* ���Ͱ�ť���µ���Ϣ */
				bsp_putkey_fifo((uint8_t)(3 * i + 1));
			}
			/* ���� */
			if(pbtn->longtime > 0)
			{
				if(pbtn->longcount < pbtn->longtime)
				{
					/* ���Ͱ�ť�������µ���Ϣ */
					if (++pbtn->longcount == pbtn->longtime)
					{
						/* ��ֵ���밴��FIFO */
						bsp_putkey_fifo((uint8_t)(3 * i + 3));
					}
				}
				else
				{
					if (pbtn->repeatspeed > 0)
					{
						if (++pbtn->repeatcount >= pbtn->repeatspeed)
						{
							pbtn->repeatcount = 0;
							/* ��������ÿ��10ms����1������ */
							bsp_putkey_fifo((uint8_t)(3 * i + 1));
						}
					}
				}
			}
		}
	}
	else
	{
		if(pbtn->count > KEY_FILTER_TIME)
		{
			pbtn->count = KEY_FILTER_TIME;
		}
		else if(pbtn->count != 0)
		{
			pbtn->count--;
		}
		else
		{
			if (pbtn->state == 1)
			{
				pbtn->state = 0;

				/* ���Ͱ�ť�������Ϣ */
				bsp_putkey_fifo((uint8_t)(3 * i + 2));
			}
		}

		pbtn->longcount = 0;
		pbtn->repeatcount = 0;
	}
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_KeyScan10ms
*	����˵��: ɨ�����а���������������systick�ж������Եĵ��ã�10msһ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_keyscan10ms(void)
{
	uint8_t i;
	for(i = 0;i < KEY_COUNT;i++)
		bsp_detectkey(i);
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_detect_fastIO
*	����˵��: �����ٵ�����IO. 1msˢ��һ��
*	��    ��: IO��id�� ��0��ʼ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_detect_fastIO(uint8_t i)
{
	key_t *pbtn;
	pbtn = &s_tbtn[i];
	if (iskeydownfunc(i))
	{
		if (pbtn->state == 0)
		{
			pbtn->state = 1;

			/* ���Ͱ�ť���µ���Ϣ */
			bsp_putkey_fifo((uint8_t)(3 * i + 1));
		}

		if (pbtn->longtime > 0)
		{
			if (pbtn->longcount < pbtn->longtime)
			{
				/* ���Ͱ�ť�������µ���Ϣ */
				if (++pbtn->longcount == pbtn->longtime)
				{
					/* ��ֵ���밴��FIFO */
					bsp_putkey_fifo((uint8_t)(3 * i + 3));
				}
			}
			else
			{
				if (pbtn->repeatspeed > 0)
				{
					if (++pbtn->repeatcount >= pbtn->repeatspeed)
					{
						pbtn->repeatcount = 0;
						/* ��������ÿ��10ms����1������ */
						bsp_putkey_fifo((uint8_t)(3 * i + 1));
					}
				}
			}
		}
	}
	else
	{
		if (pbtn->state == 1)
		{
			pbtn->state = 0;

			/* ���Ͱ�ť�������Ϣ */
			bsp_putkey_fifo((uint8_t)(3 * i + 2));
		}

		pbtn->longcount = 0;
		pbtn->repeatcount = 0;
	}
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_KeyScan1ms
*	����˵��: ɨ�����а���������������systick�ж������Եĵ��ã�1msһ��.
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_KeyScan1ms(void)
{
	uint8_t i;

	for (i = 0; i < KEY_COUNT; i++)
	{
		bsp_detect_fastIO(i);
	}
}
#else 
void bsp_initkey(void)
{
	gpio_init_t gpio_init;
	exti_init_t exti_init;

	/* Initialize key exti pin as input */
	gpio_init.mode = GPIO_MODE_INPUT;
	gpio_init.odos = GPIO_PUSH_PULL;
	gpio_init.pupd = GPIO_PUSH_UP;
	gpio_init.odrv = GPIO_OUT_DRIVE_NORMAL;
	gpio_init.flt  = GPIO_FILTER_DISABLE;
	gpio_init.type = GPIO_TYPE_CMOS;
	gpio_init.func = GPIO_FUNC_1;
	ald_gpio_init(GPIOF, GPIO_PIN_0, &gpio_init);
	ald_gpio_init(GPIOF, GPIO_PIN_1, &gpio_init);	

	/* Initialize external interrupt */
	exti_init.filter      = ENABLE;
	exti_init.cks         = EXTI_FILTER_CLOCK_10K;
	exti_init.filter_time = 10;
	ald_gpio_exti_init(GPIOF, GPIO_PIN_0, &exti_init);
	ald_gpio_exti_init(GPIOF, GPIO_PIN_1, &exti_init);

	/* Clear interrupt flag */
	ald_gpio_exti_clear_flag_status(GPIO_PIN_0);
	ald_gpio_exti_clear_flag_status(GPIO_PIN_1);

	/* Configure interrupt */
	ald_gpio_exti_interrupt_config(GPIO_PIN_0, EXTI_TRIGGER_TRAILING_EDGE, ENABLE);	
	ald_gpio_exti_interrupt_config(GPIO_PIN_1, EXTI_TRIGGER_TRAILING_EDGE, ENABLE);	
	
	ald_mcu_irq_config(EXTI0_3_IRQn, 3, ENABLE);
}


#endif

