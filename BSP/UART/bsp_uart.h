#ifndef _BSP_USART_H_
#define _BSP_USART_H_
#include "es32f0654_conf.h"
/* 串口0的GPIO  PH0, PH1 */

#define USART0_TX_GPIO_PORT              GPIOH
#define USART0_TX_PIN                    GPIO_PIN_0

#define USART0_RX_GPIO_PORT              GPIOH
#define USART0_RX_PIN                    GPIO_PIN_1

/* 串口1的GPIO --- PC0 PC1  */
#define USART1_TX_GPIO_PORT              GPIOC
#define USART1_TX_PIN                    GPIO_PIN_0

#define USART1_RX_GPIO_PORT              GPIOC
#define USART1_RX_PIN                    GPIO_PIN_12
/* 串口2的GPIO --- PC0 PC1  */
#define USART2_TX_GPIO_PORT              GPIOD
#define USART2_TX_PIN                    GPIO_PIN_2

#define USART2_RX_GPIO_PORT              GPIOC
#define USART2_RX_PIN                    GPIO_PIN_1
/* 串口3的GPIO --- PC0 PC1  */
#define USART3_TX_GPIO_PORT              GPIOC
#define USART3_TX_PIN                    GPIO_PIN_4

#define USART3_RX_GPIO_PORT              GPIOC
#define USART3_RX_PIN                    GPIO_PIN_5

#define	UART0_FIFO_EN	1
#define	UART1_FIFO_EN	0
#define	UART2_FIFO_EN	1
#define	UART3_FIFO_EN	0

/* 定义串口波特率和FIFO缓冲区大小，分为发送缓冲区和接收缓冲区, 支持全双工 */
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
/* 定义端口号 */
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
/* 串口设备结构体 */
typedef struct
{
	UART_TypeDef *uart;		/* STM32内部串口设备指针 */
	uint8_t *ptxbuf;			/* 发送缓冲区 */
	uint8_t *prxbuf;			/* 接收缓冲区 */
	
	uint16_t ustxbufsize;		/* 发送缓冲区大小 */
	uint16_t usrxbufsize;		/* 接收缓冲区大小 */
	
	__IO uint16_t ustxwrite;	/* 发送缓冲区写指针 */
	__IO uint16_t ustxread;		/* 发送缓冲区读指针 */
	__IO uint16_t ustxcount;	/* 等待发送的数据个数 */

	__IO uint16_t usrxwrite;	/* 接收缓冲区写指针 */
	__IO uint16_t usrxread;		/* 接收缓冲区读指针 */
	__IO uint16_t usrxcount;	/* 还未读取的新数据个数 */
	__IO uint16_t read_len;
	void (*sendbefor)(void); 	/* 开始发送之前的回调函数指针（主要用于RS485切换到发送模式） */
	void (*sendover)(void); 	/* 发送完毕的回调函数指针（主要用于RS485将发送模式切换为接收模式） */
	void (*recivenew)(uint8_t _byte);	/* 串口收到数据的回调函数指针 */
	uint8_t sending;			/* 正在发送中 */
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
