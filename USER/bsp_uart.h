#ifndef  __BSP_UART_H
#define  __BSP_UART_H


typedef void (*Send)(const uint8_t*, uint16_t);

#define FIFO_TYPE_SIZE  sizeof(FIFO_Type)

#define BSP_UART1_RX_SIZE	512

#define BSP_UART1_TX_SIZE	2048



//#define BSP_UART2_RX_SIZE	512

//#define BSP_UART2_TX_SIZE	512



//#define BSP_UART3_RX_SIZE	1024

//#define BSP_UART3_TX_SIZE	512

//#define BSP_UART4_RX_SIZE	512

//#define BSP_UART4_TX_SIZE	512

void Uart_Init(void);


//void USART1_IntHandler(void);

//void USART2_IntHandler(void);

//void USART3_IntHandler(void);
//void UART4_IntHandler(void);

void Uart_Send(USART_TypeDef* uart, const uint8_t* buffer, uint16_t len);

uint16_t Uart_Get(USART_TypeDef* uart, uint8_t *buffer, uint16_t len);

uint16_t Uart_Select(USART_TypeDef* uart, uint16_t delay);

void Uart_Clean(USART_TypeDef* uart);

void Uart_1_Send(const uint8_t* buf, uint16_t len);

#endif /* __BSP_UART_H */
