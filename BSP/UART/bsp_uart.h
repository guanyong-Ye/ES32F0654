#ifndef _BSP_USART_H_
#define _BSP_USART_H_
#include "es32f0654_conf.h"
/* ����0��GPIO  PH0, PH1 */

#define USART0_TX_GPIO_PORT              GPIOH
#define USART0_TX_PIN                    GPIO_PIN_0

#define USART0_RX_GPIO_PORT              GPIOH
#define USART0_RX_PIN                    GPIO_PIN_1

/* ����1��GPIO --- PC0 PC1  */
#define USART1_TX_GPIO_PORT              GPIOC
#define USART1_TX_PIN                    GPIO_PIN_0

#define USART1_RX_GPIO_PORT              GPIOC
#define USART1_RX_PIN                    GPIO_PIN_12
/* ����2��GPIO --- PC0 PC1  */
#define USART2_TX_GPIO_PORT              GPIOD
#define USART2_TX_PIN                    GPIO_PIN_2

#define USART2_RX_GPIO_PORT              GPIOC
#define USART2_RX_PIN                    GPIO_PIN_1
/* ����3��GPIO --- PC0 PC1  */
#define USART3_TX_GPIO_PORT              GPIOC
#define USART3_TX_PIN                    GPIO_PIN_4

#define USART3_RX_GPIO_PORT              GPIOC
#define USART3_RX_PIN                    GPIO_PIN_5

#define	UART0_FIFO_EN	1
#define	UART1_FIFO_EN	0
#define	UART2_FIFO_EN	1
#define	UART3_FIFO_EN	0

/* ���崮�ڲ����ʺ�FIFO��������С����Ϊ���ͻ������ͽ��ջ�����, ֧��ȫ˫�� */
#if UART0_FIFO_EN
	#define UART0_BAUD			115200
	#define UART0_TX_BUF_SIZE	1*1024
	#define UART0_RX_BUF_SIZE	1*1024
#endif
#if UART1_FIFO_EN
	#define UART1_BAUD			115200
	#define UART1_TX_BUF_SIZE	1*1024
	#define UART1_RX_BUF_SIZE	1*1024
#endif
#if UART2_FIFO_EN
	#define UART2_BAUD			115200
	#define UART2_TX_BUF_SIZE	1*1024
	#define UART2_RX_BUF_SIZE	1*1024
#endif
#if UART3_FIFO_EN
	#define UART3_BAUD			115200
	#define UART3_TX_BUF_SIZE	1*1024
	#define UART3_RX_BUF_SIZE	1*1024
#endif
/* ����˿ں� */
typedef enum
{
	COM0 = 0,	/* USART8 */
	COM1 = 1,	/* USART1 */
	COM2 = 2,	/* USART2 */
	COM3 = 3,	/* USART3 */
//	COM4 = 4,	/* USART4 */
//	COM5 = 5,	/* USART5 */
//	COM6 = 6,	/* USART6 */
//	COM7 = 7	/* USART7 */	
}COM_PORT_E;
/* �����豸�ṹ�� */
typedef struct
{
	UART_TypeDef *uart;		/* STM32�ڲ������豸ָ�� */
	uint8_t *ptxbuf;			/* ���ͻ����� */
	uint8_t *prxbuf;			/* ���ջ����� */
	
	uint16_t ustxbufsize;		/* ���ͻ�������С */
	uint16_t usrxbufsize;		/* ���ջ�������С */
	
	__IO uint16_t ustxwrite;	/* ���ͻ�����дָ�� */
	__IO uint16_t ustxread;		/* ���ͻ�������ָ�� */
	__IO uint16_t ustxcount;	/* �ȴ����͵����ݸ��� */

	__IO uint16_t usrxwrite;	/* ���ջ�����дָ�� */
	__IO uint16_t usrxread;		/* ���ջ�������ָ�� */
	__IO uint16_t usrxcount;	/* ��δ��ȡ�������ݸ��� */
	__IO uint16_t read_len;
	void (*sendbefor)(void); 	/* ��ʼ����֮ǰ�Ļص�����ָ�루��Ҫ����RS485�л�������ģʽ�� */
	void (*sendover)(void); 	/* ������ϵĻص�����ָ�루��Ҫ����RS485������ģʽ�л�Ϊ����ģʽ�� */
	void (*recivenew)(uint8_t _byte);	/* �����յ����ݵĻص�����ָ�� */
	uint8_t sending;			/* ���ڷ����� */
}UART_T;
void bsp_inituart(void);

void comsendbuf(COM_PORT_E _ucport, uint8_t *_ucaBuf, uint16_t _usLen);
void comsendchar(COM_PORT_E _ucport, uint8_t _ucByte);

uint8_t comgetchar(COM_PORT_E _ucport, uint8_t *_pbyte);

void comcleartxfifo(COM_PORT_E _ucport);
void comclearrxfifo(COM_PORT_E _ucport);

void comsetbaud(COM_PORT_E _ucport, uint32_t _BaudRate);
void bsp_setuartparam(UART_TypeDef *Instance,  uint32_t baudrate, uint32_t parity, uint32_t mode);

uint8_t uarttxempty(COM_PORT_E _ucport);

extern UART_T g_tUart1;

#endif
