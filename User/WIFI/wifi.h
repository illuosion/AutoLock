/*-------------------------------------------------*/
/*                                                 */
/*              操作Wifi功能的头文件               */
/*                                                 */
/*-------------------------------------------------*/

#ifndef __WIFI_H
#define __WIFI_H

#include "./UART2/uart2.h" //包含需要的头文件
#include "./STRUCTURE/structure.h"
#include "./dwt_delay/core_delay.h"
/* 宏定义引脚 */

extern char Connect_flag; // 外部变量声明，同服务器连接状态  0：还没有连接服务器  1：连接上服务器了

#define RESET_IO(x) GPIO_WriteBit(GPIOE, GPIO_Pin_2, (BitAction)x) // PA4控制WiFi的复位

#define WiFi_printf u3_printf               // 串口3控制 WiFi
#define WiFi_RxCounter Usart3_RxCounter     // 串口3控制 WiFi
#define WiFi_RX_BUF Usart3_RxBuff           // 串口3控制 WiFi
#define WiFi_RXBUFF_SIZE USART3_RXBUFF_SIZE // 串口3控制 WiFi
#define Delay_Ms Delay_ms

#define SSID "XIAOCHUN01" // 路由器SSID名称
#define PASS "3118003167" // 路由器密码

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
