#include "bsp.h"


/*
*********************************************************************************************************
*
*	ģ������ : �����ж�+FIFO����ģ��
*	�ļ����� : bsp_uart_fifo.c
*	��    �� : V1.8
*	˵    �� : ���ô����ж�+FIFOģʽʵ�ֶ�����ڵ�ͬʱ����
*
*********************************************************************************************************
*/

/* ����ÿ�����ڽṹ����� */
#if UART0_FIFO_EN
	static UART_T g_tuart0;
	static uint8_t g_TxBuf0[UART0_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf0[UART0_RX_BUF_SIZE];		/* ���ջ����� */
#endif
#if UART1_FIFO_EN
	static UART_T g_tuart1;
	static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE];		/* ���ջ����� */
#endif
#if UART2_FIFO_EN
	static UART_T g_tuart2;
	static uint8_t g_TxBuf2[UART2_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf2[UART2_RX_BUF_SIZE];		/* ���ջ����� */
#endif
#if UART3_FIFO_EN
	static UART_T g_tUart3;
	static uint8_t g_TxBuf3[UART3_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf3[UART3_RX_BUF_SIZE];		/* ���ջ����� */
#endif
/*
*********************************************************************************************************
*	�� �� ��: UartVarInit
*	����˵��: ��ʼ��������صı���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void uartvarinit(void)
{
#if UART0_FIFO_EN
	g_tUart0.uart 	= UART0;						/* STM32 �����豸 */
	g_tUart0.ptxbuf = g_TxBuf0;					/* ���ͻ�����ָ�� */
	g_tUart0.prxbuf = g_RxBuf0;					/* ���ջ�����ָ�� */
	g_tUart0.ustxbufsize = UART0_TX_BUF_SIZE;	/* ���ͻ�������С */
	g_tUart0.usrxbufsize = UART0_RX_BUF_SIZE;	/* ���ջ�������С */
	g_tUart0.ustxwrite 	= 0;						/* ����FIFOд���� */
	g_tUart0.ustxread 	= 0;						/* ����FIFO������ */
	g_tUart0.usrxwrite 	= 0;						/* ����FIFOд���� */
	g_tUart0.usrxread 	= 0;						/* ����FIFO������ */
	g_tUart0.usrxcount 	= 0;						/* ���յ��������ݸ��� */
	g_tUart0.ustxcount 	= 0;						/* �����͵����ݸ��� */
	g_tUart0.sendbefor 	= 0;						/* ��������ǰ�Ļص����� */
	g_tUart0.sendover 	= 0;						/* ������Ϻ�Ļص����� */
	g_tUart0.recivenew 	= 0;						/* ���յ������ݺ�Ļص����� */
	g_tUart0.sending 	= 0;						/* ���ڷ����б�־ */
	g_tUart0.read_len = 0;
#endif
#if UART1_FIFO_EN
	g_tUart1.uart 	= UART1;						/* STM31 �����豸 */
	g_tUart1.ptxbuf = g_TxBuf1;					/* ���ͻ�����ָ�� */
	g_tUart1.prxbuf = g_RxBuf1;					/* ���ջ�����ָ�� */
	g_tUart1.ustxbufsize = UART1_TX_BUF_SIZE;	/* ���ͻ�������С */
	g_tUart1.usrxbufsize = UART1_RX_BUF_SIZE;	/* ���ջ�������С */
	g_tUart1.ustxwrite 	= 0;						/* ����FIFOд���� */
	g_tUart1.ustxread 	= 0;						/* ����FIFO������ */
	g_tUart1.usrxwrite 	= 0;						/* ����FIFOд���� */
	g_tUart1.usrxread 	= 0;						/* ����FIFO������ */
	g_tUart1.usrxcount 	= 0;						/* ���յ��������ݸ��� */
	g_tUart1.ustxcount 	= 0;						/* �����͵����ݸ��� */
	g_tUart1.sendbefor 	= 0;						/* ��������ǰ�Ļص����� */
	g_tUart1.sendover 	= 0;						/* ������Ϻ�Ļص����� */
	g_tUart1.recivenew 	= 0;						/* ���յ������ݺ�Ļص����� */
	g_tUart1.sending 	= 0;						/* ���ڷ����б�־ */
#endif
#if UART2_FIFO_EN
	g_tUart2.uart 	= UART2;						/* STM32 �����豸 */
	g_tUart2.ptxbuf = g_TxBuf2;					/* ���ͻ�����ָ�� */
	g_tUart2.prxbuf = g_RxBuf2;					/* ���ջ�����ָ�� */
	g_tUart2.ustxbufsize = UART2_TX_BUF_SIZE;	/* ���ͻ�������С */
	g_tUart2.usrxbufsize = UART2_RX_BUF_SIZE;	/* ���ջ�������С */
	g_tUart2.ustxwrite 	= 0;						/* ����FIFOд���� */
	g_tUart2.ustxread 	= 0;						/* ����FIFO������ */
	g_tUart2.usrxwrite 	= 0;						/* ����FIFOд���� */
	g_tUart2.usrxread 	= 0;						/* ����FIFO������ */
	g_tUart2.usrxcount 	= 0;						/* ���յ��������ݸ��� */
	g_tUart2.ustxcount 	= 0;						/* �����͵����ݸ��� */
	g_tUart2.sendbefor 	= 0;						/* ��������ǰ�Ļص����� */
	g_tUart2.sendover 	= 0;						/* ������Ϻ�Ļص����� */
	g_tUart2.recivenew 	= 0;						/* ���յ������ݺ�Ļص����� */
	g_tUart2.sending 	= 0;						/* ���ڷ����б�־ */
#endif
}
/*
*********************************************************************************************************
*	�� �� ��: bsp_setuartparam
*	����˵��: ���ô��ڵ�Ӳ�������������ʣ�����λ��ֹͣλ����ʼλ��У��λ���ж�ʹ�ܣ��ʺ���STM32- H7������
*	��    ��: Instance   USART_TypeDef���ͽṹ��
*             BaudRate   ������
*             Parity     У�����ͣ���У�����żУ��
*             Mode       ���ͺͽ���ģʽʹ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_setuartparam(UART_TypeDef *instance,  uint32_t baudrate, uint32_t parity, uint32_t mode)
{
	uart_handle_t h_uart;
	memset(&h_uart,0,sizeof(h_uart));
	/*##-1- ���ô���Ӳ������ ######################################*/
	/* �첽����ģʽ (UART Mode) */
	/* ��������:
	  - �ֳ�    = 8 λ
	  - ֹͣλ  = 1 ��ֹͣλ
	  - У��    = ����Parity
	  - ������  = ����BaudRate
	  - Ӳ�������ƹر� (RTS and CTS signals) */

	h_uart.perh        = instance;

	h_uart.init.baud   		= baudrate;
	h_uart.init.word_length = UART_WORD_LENGTH_8B;
	h_uart.init.stop_bits   = UART_STOP_BITS_1;
	h_uart.init.parity     	= parity;
	h_uart.init.fctl  		= UART_HW_FLOW_CTL_DISABLE;
	h_uart.init.mode       	= mode;
//	h_uart.tx_cplt_cbk      = uart_send_complete;
//	h_uart.rx_cplt_cbk      = uart_recv_complete;
//	h_uart.error_cbk        = uart_error;
	ald_uart_init(&h_uart);
}
/*
*********************************************************************************************************
*	�� �� ��: InitHardUart
*	����˵��: ���ô��ڵ�Ӳ�������������ʣ�����λ��ֹͣλ����ʼλ��У��λ���ж�ʹ�ܣ��ʺ���STM32-H7������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void initharduart(void)
{
	gpio_init_t  gpio_init;

#if UART0_FIFO_EN		/* ����1 */
	/* Initialize tx pin */
	gpio_init.mode = GPIO_MODE_OUTPUT;
	gpio_init.odos = GPIO_PUSH_PULL;
	gpio_init.pupd = GPIO_PUSH_UP;
	gpio_init.odrv = GPIO_OUT_DRIVE_NORMAL;
	gpio_init.flt  = GPIO_FILTER_DISABLE;
	gpio_init.type = GPIO_TYPE_TTL;
	gpio_init.func = GPIO_FUNC_3;
	ald_gpio_init(USART0_TX_GPIO_PORT, USART0_TX_PIN, &gpio_init);

	/* Initialize rx pin */
	gpio_init.mode = GPIO_MODE_INPUT;
	ald_gpio_init(USART0_RX_GPIO_PORT, USART0_TX_PIN, &gpio_init);
	
	/* Enable uart interrupt */
	ald_mcu_irq_config(UART0_IRQn, 3, ENABLE);
  
	/* ���ò����ʡ���żУ�� */
	bsp_setuartparam(UART0,UART0_BAUD, UART_PARITY_NONE, UART_MODE_UART);

	SET_BIT(UART0->ICR, UART_ICR_TCIC_MSK);   /* ���TC������ɱ�־ */
    SET_BIT(UART0->ICR, UART_ICR_RXRDIC_MSK); /* ���RXNE���ձ�־ */
	// USART_CR1_PEIE | USART_CR1_RXNEIE
	SET_BIT(UART0->IER, UART_IER_RXRDIE_MSK);	/* ʹ��PE. RX�����ж� */
#endif
#if UART1_FIFO_EN		/* ����1 */
	/* ʹ�� GPIO TX/RX ʱ�� */
	USART1_TX_GPIO_CLK_ENABLE();
	USART1_RX_GPIO_CLK_ENABLE();
	
	/* ʹ�� USARTx ʱ�� */
	USART1_CLK_ENABLE();	

	/* ����TX���� */
	GPIO_InitStruct.Pin       = USART1_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = USART1_TX_AF;
	HAL_GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStruct);	
	
	/* ����RX���� */
	GPIO_InitStruct.Pin = USART1_RX_PIN;
	GPIO_InitStruct.Alternate = USART1_RX_AF;
	HAL_GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStruct);

	/* ����NVIC the NVIC for UART */   
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
  
	/* ���ò����ʡ���żУ�� */
	bsp_setuartparam(USART1,  UART1_BAUD, UART_PARITY_NONE, UART_MODE_TX_RX);

	CLEAR_BIT(USART1->SR, USART_SR_TC);   /* ���TC������ɱ�־ */
    CLEAR_BIT(USART1->SR, USART_SR_RXNE); /* ���RXNE���ձ�־ */
	// USART_CR1_PEIE | USART_CR1_RXNEIE
	SET_BIT(USART1->CR1, USART_CR1_RXNEIE);	/* ʹ��PE. RX�����ж� */
#endif
#if UART2_FIFO_EN		/* ����2 */
		/* Initialize tx pin */
	gpio_init.mode = GPIO_MODE_OUTPUT;
	gpio_init.odos = GPIO_PUSH_PULL;
	gpio_init.pupd = GPIO_PUSH_UP;
	gpio_init.odrv = GPIO_OUT_DRIVE_NORMAL;
	gpio_init.flt  = GPIO_FILTER_DISABLE;
	gpio_init.type = GPIO_TYPE_TTL;
	gpio_init.func = GPIO_FUNC_3;
	ald_gpio_init(USART2_TX_GPIO_PORT, USART2_TX_PIN, &gpio_init);

	/* Initialize rx pin */
	gpio_init.mode = GPIO_MODE_INPUT;
	ald_gpio_init(USART2_RX_GPIO_PORT, USART2_TX_PIN, &gpio_init);
	
	/* Enable uart interrupt */
	ald_mcu_irq_config(BS16T1_UART2_IRQn, 3, ENABLE);
  
	/* ���ò����ʡ���żУ�� */
	bsp_setuartparam(UART2,UART0_BAUD, UART_PARITY_NONE, UART_MODE_UART);

	SET_BIT(UART1->ICR, UART_ICR_TCIC_MSK);   /* ���TC������ɱ�־ */
    SET_BIT(UART1->ICR, UART_ICR_RXRDIC_MSK); /* ���RXNE���ձ�־ */
	// USART_CR1_PEIE | USART_CR1_RXNEIE
	SET_BIT(UART1->IER, UART_IER_RXRDIE_MSK);	/* ʹ��PE. RX�����ж� */
#endif
}

/*
*
*���� FIFO ��ʼ��
*
*/
 void bsp_inituart(void)
{
	uartvarinit();		/* �����ȳ�ʼ��ȫ�ֱ���,������Ӳ�� */

	initharduart();		/* ���ô��ڵ�Ӳ������(�����ʵ�) */

}
/*
*********************************************************************************************************
*	�� �� ��: ComToUart
*	����˵��: ��COM�˿ں�ת��ΪUARTָ��
*	��    ��: _ucport: �˿ں�(COM1 - COM8)
*	�� �� ֵ: uartָ��
*********************************************************************************************************
*/
UART_T *comtouart(COM_PORT_E _ucport)
{
	switch (_ucport)
	{
		case COM0:
			#if UART0_FIFO_EN
				return &g_tuart0;
			#else
				return 0;
			#endif
		case COM1:
			#if UART1_FIFO_EN
				return &g_tuart1;
			#else
				return 0;
			#endif
		case COM2:
			#if UART2_FIFO_EN
				return &g_tuart2;
			#else
				return 0;
			#endif
		default:
			//_Error_Handler(__FILE__, __LINE__);
			return 0;
	}
}
/*
*********************************************************************************************************
*	�� �� ��: ComToUart
*	����˵��: ��COM�˿ں�ת��Ϊ USART_TypeDef* USARTx
*	��    ��: _ucport: �˿ں�(COM1 - COM8)
*	�� �� ֵ: USART_TypeDef*,  USART1, USART2, USART3, UART4, UART5��USART6��UART7��UART8��
*********************************************************************************************************
*/
UART_TypeDef *comtouartx(COM_PORT_E _ucport)
{
	if (_ucport == COM0)
	{
		#if UART0_FIFO_EN
			return UART0;
		#else
			return 0;
		#endif
	}
	else if (_ucport == COM1)
	{
		#if UART1_FIFO_EN
			return UART1;
		#else
			return 0;
		#endif
	}
	else if (_ucport == COM2)
	{
		#if UART2_FIFO_EN == 1
			return UART2;
		#else
			return 0;
		#endif
	}
	else if (_ucport == COM3)
	{
		#if UART3_FIFO_EN
			return UART3;
		#else
			return 0;
		#endif
	}	
	else
	{
		/* �����κδ��� */
		return 0;
	}
}
/*
*********************************************************************************************************
*	�� �� ��: comClearTxFifo
*	����˵��: ���㴮�ڷ��ͻ�����
*	��    ��: _ucport: �˿ں�(COM1 - COM8)
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comcleartxfifo(COM_PORT_E _ucport)
{
	UART_T *puart;

	puart = comtouart(_ucport);
	if (puart == 0)
	{
		return;
	}

	puart->ustxwrite = 0;
	puart->ustxread = 0;
	puart->ustxcount = 0;
}
/*
*********************************************************************************************************
*	�� �� ��: comClearRxFifo
*	����˵��: ���㴮�ڽ��ջ�����
*	��    ��: _ucport: �˿ں�(COM1 - COM8)
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comclearrxfifo(COM_PORT_E _ucport)
{
	UART_T *puart;

	puart = comtouart(_ucport);
	if (puart == 0)
	{
		return;
	}

	puart->usrxwrite = 0;
	puart->usrxread = 0;
	puart->usrxcount = 0;
}
/*
*********************************************************************************************************
*	�� �� ��: comSetBaud
*	����˵��: ���ô��ڵĲ�����. �������̶�����Ϊ��У�飬�շ���ʹ��ģʽ
*	��    ��: _ucport: �˿ں�(COM1 - COM8)
*			  _BaudRate: �����ʣ�8��������  ������.0-12.5Mbps
*                                16�������� ������.0-6.25Mbps
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comsetbaud(COM_PORT_E _ucport, uint32_t _baudrate)
{
	UART_TypeDef* uartx;
	
	uartx = comtouartx(_ucport);
	if (uartx == 0)
	{
		return;
	}
	
	bsp_setuartparam(uartx,_baudrate,UART_PARITY_NONE,UART_MODE_UART);
}

/*
*********************************************************************************************************
*	�� �� ��: UartSend
*	����˵��: ��д���ݵ�UART���ͻ�����,�����������жϡ��жϴ�����������Ϻ��Զ��رշ����ж�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void uartsend(UART_T *_puart, uint8_t *_ucaBuf, uint16_t _usLen)
{
	uint16_t i;

	for (i = 0; i < _usLen; i++)
	{
		/* ������ͻ������Ѿ����ˣ���ȴ��������� */
		while (1)
		{
			__IO uint16_t usCount;

			DISABLE_INT();
			usCount = _puart->ustxcount;
			ENABLE_INT();

			if (usCount < _puart->ustxbufsize)
			{
				break;
			}
			else if(usCount == _puart->ustxbufsize)/* ���������������� */
			{
				if((_puart->uart->CR1 & USART_CR1_TXEIE) == 0)
				{
					SET_BIT(_puart->uart->CR1, USART_CR1_TXEIE);
				}  
			}
		}

		/* �����������뷢�ͻ����� */
		_puart->ptxbuf[_puart->ustxwrite] = _ucaBuf[i];

		DISABLE_INT();
		if (++_puart->ustxwrite >= _puart->ustxbufsize)
		{
			_puart->ustxwrite = 0;
		}
		_puart->ustxcount++;
		ENABLE_INT();
	}

	SET_BIT(_puart->uart->CR1, USART_CR1_TXEIE);	/* ʹ�ܷ����жϣ��������գ� */
}
/*
*********************************************************************************************************
*	�� �� ��: comSendBuf
*	����˵��: �򴮿ڷ���һ�����ݡ����ݷŵ����ͻ��������������أ����жϷ�������ں�̨��ɷ���
*	��    ��: _ucport: �˿ں�(COM1 - COM8)
*			  _ucaBuf: �����͵����ݻ�����
*			  _usLen : ���ݳ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comsendbuf(COM_PORT_E _ucport, uint8_t *_ucaBuf, uint16_t _usLen)
{
	UART_T *pUart;

	pUart = comtouart(_ucport);
	if (pUart == 0)
	{
		return;
	}

	if (pUart->sendbefor != 0)
	{
		pUart->sendbefor();		/* �����RS485ͨ�ţ���������������н�RS485����Ϊ����ģʽ */
	}

	uartsend(pUart, _ucaBuf, _usLen);
}
/*
*********************************************************************************************************
*	�� �� ��: comSendChar
*	����˵��: �򴮿ڷ���1���ֽڡ����ݷŵ����ͻ��������������أ����жϷ�������ں�̨��ɷ���
*	��    ��: _ucport: �˿ں�(COM1 - COM8)
*			  _ucByte: �����͵�����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comsendchar(COM_PORT_E _ucport, uint8_t _ucByte)
{
	comsendbuf(_ucport, &_ucByte, 1);
}

/*
*
*�������ݽ���
*
*/
/*
*********************************************************************************************************
*   �� �� ��: UartTxEmpty
*   ����˵��: �жϷ��ͻ������Ƿ�Ϊ�ա�
*   ��    ��:  _puart : �����豸
*   �� �� ֵ: 1Ϊ�ա�0Ϊ���ա�
*********************************************************************************************************
*/
uint8_t uarttxempty(COM_PORT_E _ucport)
{
   UART_T *pUart;
   uint8_t Sending;
   
   pUart = comtouart(_ucport);
   if (pUart == 0)
   {
      return 0;
   }

   Sending = pUart->sending;

   if (Sending != 0)
   {
      return 0;
   }
   return 1;
}
/*
*********************************************************************************************************
*	�� �� ��: UartGetChar
*	����˵��: �Ӵ��ڽ��ջ�������ȡ1�ֽ����� ��������������ã�
*	��    ��: _puart : �����豸
*			  _pByte : ��Ŷ�ȡ���ݵ�ָ��
*	�� �� ֵ: 0 ��ʾ������  1��ʾ��ȡ������
*********************************************************************************************************
*/
static uint8_t uartgetchar(UART_T *puart, uint8_t *_pByte)
{
	uint16_t usCount;

	/* usRxWrite �������жϺ����б���д���������ȡ�ñ���ʱ����������ٽ������� */
	DISABLE_INT();
	usCount = puart->usrxcount;
	ENABLE_INT();

	/* �������д������ͬ���򷵻�0 */
	//if (_puart->usRxRead == usRxWrite)
	if (usCount == 0)	/* �Ѿ�û������ */
	{
		return 0;
	}
	else
	{
		*_pByte = puart->prxbuf[puart->usrxread];		/* �Ӵ��ڽ���FIFOȡ1������ */

		/* ��дFIFO������ */
		DISABLE_INT();
		if (++puart->usrxread >= puart->usrxbufsize)
		{
			puart->usrxread = 0;
		}
		puart->usrxcount--;
		ENABLE_INT();
		return 1;
	}
}
/*
*********************************************************************************************************
*	�� �� ��: comgetchar
*	����˵��: �ӽ��ջ�������ȡ1�ֽڣ��������������������ݾ��������ء�
*	��    ��: _ucport: �˿ں�(COM1 - COM8)
*			  _pByte: ���յ������ݴ���������ַ
*	�� �� ֵ: 0 ��ʾ������, 1 ��ʾ��ȡ����Ч�ֽ�
*********************************************************************************************************
*/
uint8_t comgetchar(COM_PORT_E _ucport, uint8_t *_pbyte)
{
	UART_T *puart;

	puart = comtouart(_ucport);
	if (puart == 0)
	{
		return 0;
	}

	return uartgetchar(puart, _pbyte);
}
/*
*********************************************************************************************************
*	�� �� ��: UartIRQ
*	����˵��: ���жϷ��������ã�ͨ�ô����жϴ�����
*	��    ��: _puart : �����豸
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void UartIRQ(UART_T *_puart)
{
	uint32_t ifmflags   = READ_REG(_puart->uart->IFM);
	uint32_t cr1its     = READ_REG(_puart->uart->CR1);
	uint32_t cr3its     = READ_REG(_puart->uart->CR3);
	
	/* ��������ж�  */
	if ((ifmflags & UART_IFM_RXRDIM_MSK) != RESET)
	{
		/* �Ӵ��ڽ������ݼĴ�����ȡ���ݴ�ŵ�����FIFO */
		uint8_t ch;

		ch = READ_REG(_puart->uart->RBR);			/* ��ȡ���ڽ��ռĴ��� */
		_puart->prxbuf[_puart->usrxwrite] = ch;		/* ���봮�ڽ���FIFO */
		if (++_puart->usrxwrite >= _puart->usrxbufsize)/* ���� FIFO ��дָ��+1 */
		{
			_puart->usrxwrite = 0;
		}
		if (_puart->usrxcount < _puart->usrxbufsize)/* ͳ��δ������ֽڸ��� */
		{
			_puart->usrxcount++;
		}

		/* �ص�����,֪ͨӦ�ó����յ�������,һ���Ƿ���1����Ϣ��������һ����� */
		if (_puart->recivenew)
		{
			_puart->recivenew(ch); /* ���磬����MODBUS����������ֽ��� */
		}
	}

	/* �����ͻ��������ж� */
	if ( ((ifmflags & UART_IFM_TXSIM_MSK) != RESET) && (cr1its & USART_CR1_TXEIE) != RESET)
	{
		if (_puart->ustxcount == 0)
		{
			/* ���ͻ�������������ȡ��ʱ�� ��ֹ���ͻ��������ж� ��ע�⣺��ʱ���1�����ݻ�δ����������ϣ�*/
			//USART_ITConfig(_puart->uart, USART_IT_TXE, DISABLE);
			CLEAR_BIT(_puart->uart->CR1, USART_CR1_TXEIE);

			/* ʹ�����ݷ�������ж� */
			//USART_ITConfig(_puart->uart, USART_IT_TC, ENABLE);
			SET_BIT(_puart->uart->CR1, USART_CR1_TCIE);
		}
		else
		{
			_puart->sending = 1;
			
			/* �ӷ���FIFOȡ1���ֽ�д�봮�ڷ������ݼĴ��� */
			//USART_SendData(_puart->uart, _puart->pTxBuf[_puart->usTxRead]);
			_puart->uart->DR = _puart->ptxbuf[_puart->ustxread];
			if (++_puart->ustxread >= _puart->ustxbufsize)
			{
				_puart->ustxread = 0;
			}
			_puart->ustxcount--;
		}

	}
	/* ����bitλȫ��������ϵ��ж� */
	if (((ifmflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
	{
		//if (_puart->usTxRead == _puart->usTxWrite)
		if (_puart->ustxcount == 0)
		{
			/* �������FIFO������ȫ��������ϣ���ֹ���ݷ�������ж� */
			//USART_ITConfig(_puart->uart, USART_IT_TC, DISABLE);
			CLEAR_BIT(_puart->uart->CR1, USART_CR1_TCIE);

			/* �ص�����, һ����������RS485ͨ�ţ���RS485оƬ����Ϊ����ģʽ��������ռ���� */
			if (_puart->sendover)
			{
				_puart->sendover();
			}
			
			_puart->sending = 0;
		}
		else
		{
			/* ��������£��������˷�֧ */

			/* �������FIFO�����ݻ�δ��ϣ���ӷ���FIFOȡ1������д�뷢�����ݼĴ��� */
			//USART_SendData(_puart->uart, _puart->pTxBuf[_puart->usTxRead]);
			_puart->uart->DR = _puart->ptxbuf[_puart->ustxread];
			if (++_puart->ustxread >= _puart->ustxbufsize)
			{
				_puart->ustxread = 0;
			}
			_puart->ustxcount--;
		}
	}
}
/*
*********************************************************************************************************
*	�� �� ��: USART1_IRQHandler  USART2_IRQHandler USART3_IRQHandler UART4_IRQHandler UART5_IRQHandler��
*	����˵��: USART�жϷ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
#if UART0_FIFO_EN 
void UART0_Handler(void)
{
	UartIRQ(&g_tuart0);
}
#endif
#if UART1_FIFO_EN 
void UART1_Handler(void)
{
	UartIRQ(&g_tuart1);
}
#endif
#if UART2_FIFO_EN 
void UART2_Handler(void)
{
	UartIRQ(&g_tuart2);
}
#endif
#if UART3_FIFO_EN 
void UART3_Handler(void)
{
	UartIRQ(&g_tuart3);
}
#endif
/*
*********************************************************************************************************
*	�� �� ��: fputc
*	����˵��: �ض���putc��������������ʹ��printf�����Ӵ���1��ӡ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int fputc(int ch, FILE *f)
{
#if 1	/* ����Ҫprintf���ַ�ͨ�������ж�FIFO���ͳ�ȥ��printf�������������� */
	comsendchar(COM0, ch);
	
	return ch;
#else	/* ����������ʽ����ÿ���ַ�,�ȴ����ݷ������ */
	/* дһ���ֽڵ�USART1 */
	USART1->DR = ch;
	
	/* �ȴ����ͽ��� */
	while((USART1->SR & USART_SR_TC) == 0)
	{}
	
	return ch;
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: fgetc
*	����˵��: �ض���getc��������������ʹ��getchar�����Ӵ���1��������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int fgetc(FILE *f)
{

#if 1	/* �Ӵ��ڽ���FIFO��ȡ1������, ֻ��ȡ�����ݲŷ��� */
	uint8_t ucdata;

	while(comgetchar(COM0, &ucdata) == 0);

	return ucdata;
#else
	/* �ȴ����յ����� */
	while((USART1->SR & USART_SR_RXNE) == 0)
	{}

	return (int)USART1->DR;
#endif
}




