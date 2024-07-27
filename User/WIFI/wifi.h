/*-------------------------------------------------*/
/*                                                 */
/*              ����Wifi���ܵ�ͷ�ļ�               */
/*                                                 */
/*-------------------------------------------------*/

#ifndef __WIFI_H
#define __WIFI_H

#include "./UART2/uart2.h" //������Ҫ��ͷ�ļ�
#include "./STRUCTURE/structure.h"
#include "./dwt_delay/core_delay.h"
/* �궨������ */

extern char Connect_flag; // �ⲿ����������ͬ����������״̬  0����û�����ӷ�����  1�������Ϸ�������

#define RESET_IO(x) GPIO_WriteBit(GPIOE, GPIO_Pin_2, (BitAction)x) // PA4����WiFi�ĸ�λ

#define WiFi_printf u3_printf               // ����3���� WiFi
#define WiFi_RxCounter Usart3_RxCounter     // ����3���� WiFi
#define WiFi_RX_BUF Usart3_RxBuff           // ����3���� WiFi
#define WiFi_RXBUFF_SIZE USART3_RXBUFF_SIZE // ����3���� WiFi
#define Delay_Ms Delay_ms

#define SSID "XIAOCHUN01" // ·����SSID����
#define PASS "3118003167" // ·��������

void WiFi_ResetIO_Init(void);
char WiFi_SendCmd(char *cmd, int timeout);
char WiFi_Reset(int timeout);
char WiFi_JoinAP(int timeout);
char WiFi_Connect_Server(int timeout);
char WiFi_Connect(char *serverip, int serverport, int timeout);
char WiFi_Smartconfig(int timeout);
char WiFi_WaitAP(int timeout);
char WiFi_GetIP(int timeout);
char WiFi_Get_LinkSta(void);
char WiFi_Get_Data(char *data, char *len, char *id);
char WiFi_SendData(char id, char *databuff, int data_len, int timeout);
char WiFi_Close(int timeout);
char WiFi_Init(void);
char WiFi_Send(char *sendbuf);
char WiFi_Recv(char *recvbuf, int len, int timeout);

#endif
