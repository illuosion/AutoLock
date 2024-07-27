#ifndef __UART2_H
#define __UART2_H
#include <stdio.h>
#include "stm32f4xx.h"
// #include "sys.h"
/************************************************

************************************************/
#define EN_USART3_RX 1 // ʹ�ܣ�1��/��ֹ��0������2����

// DMA���� ���ֲο�https://blog.csdn.net/gdjason/article/details/51019219
// #define	USART2_RXBUFF_SIZE	200
// #define	USART2_TXBUFF_SIZE	200
#define USART3_RXBUFF_SIZE 512
#define USART3_TXBUFF_SIZE 512

/*�궨��Ϊ��д+�»��ߣ�����ΪС�շ�+�»���*/
// extern uint8_t Uart2_RxBuff[DMA_UART2_RX_SIZE];
// extern uint8_t Uart2_TxBuff[DMA_UART2_TX_SIZE];
extern char Usart3_RxBuff[USART3_RXBUFF_SIZE];
extern char Usart3_TxBuff[USART3_TXBUFF_SIZE];
extern uint16_t Usart3_RxCounter; // Usart2 ���յ������ݳ��ȼ�����

void uart3_init(void);
// void uart3_dma_rx_configuration(void);
// void uart3_dma_rxtx_configuration(void);
// uint16_t Uart3_DMA_Send_Data(void *buffer, u16 size);

void USART3_sendonechar(u8 data);
void USART3_sendstring(u8 *data, u16 length);

// Ϊ�˼��ݳ�ά��WIFI������
void u3_printf(char *, ...);

#endif
