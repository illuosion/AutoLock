/**
 ******************************************************************************
 * @file    main.c
 * @author  fire
 * @version V1.0
 * @date    2015-xx-xx
 * @brief   RFID-RC522ģ��ʵ��
 ******************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:Ұ��  STM32 F407 ������
 * ��̳    :http://www.firebbs.cn
 * �Ա�    :https://fire-stm32.taobao.com
 *
 ******************************************************************************
 */

// FreeRTOSͷ�ļ�
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include <stdbool.h>

/* ����ͷ�ļ� */
#include "./usart/bsp_debug_usart.h"
#include "rc522_config.h"
#include "rc522_function.h"
#include "./beep/bsp_beep.h"
#include "./led/bsp_led.h"
#include "./lcd/bsp_ili9806g_lcd.h"
#include "./font/fonts.h"
#include "./systick/bsp_SysTick.h"
#include "./key/bsp_key.h"
#include "./led/bsp_led.h"
#include "./led/bsp_breath_led.h"
#include "DHT11/bsp_dht11.h"
#include "./UART2/uart2.h"
#include "wifi.h"
#include "esp8266_mqtt.h"
#include "timer3.h"
#include "structure.h"

/* ������ */
static TaskHandle_t AppTaskCreate_Handle = NULL; /* ���������� */
static TaskHandle_t GetData_Task_Handle = NULL;  /* LED������ */
static TaskHandle_t KEY_Task_Handle = NULL;      /* KEY������ */
static TaskHandle_t WIFI_Task_Handle = NULL;     /* WIFI������ */
static TaskHandle_t IC_Task_Handle = NULL;       /* IC������ */
// static TaskHandle_t DHT_Task_Handle = NULL;      /* �¿������� */
// static TaskHandle_t ESP_Task_Handle = NULL;      /* WIFI������ */
// static TaskHandle_t Uart_Task_Handle = NULL;     /* USART������ */

/*�ں˶�����*/
SemaphoreHandle_t BinarySem_Handle_W = NULL;  /*д�Ķ�ֵ�ź���*/
SemaphoreHandle_t BinarySem_Handle_R = NULL;  /*���Ķ�ֵ�ź���*/
SemaphoreHandle_t BinarySem_Handle_IC = NULL; /*IC�Ķ�ֵ�ź���*/
SemaphoreHandle_t MutexSem_Handle = NULL;     /*������*/

/*ȫ�ֱ�������*/
extern char Usart_Rx_Buf[USART_RBUFF_SIZE];
#define MQTT_BROKERADDRESS "iot-06z00bcpj7fur8q.mqtt.iothub.aliyuncs.com"
#define MQTT_CLIENTID "k15fb6M8Sxm.NKVmLwH7SG62GPgNXE6F|securemode=2,signmethod=hmacsha256,timestamp=1713757769095|"
#define MQTT_USARNAME "NKVmLwH7SG62GPgNXE6F&k15fb6M8Sxm"
#define MQTT_PASSWD "e49268d49fb1ff5c7814f8d9b9aa438eeaa80f9995a6fdc7331cecb994a8a564"
#define MQTT_PUBLISH_TOPIC "/sys/k15fb6M8Sxm/test01/thing/event/property/post"
#define MQTT_SUBSCRIBE_TOPIC "/sys/k15fb6M8Sxm/test01/thing/service/property/set"

/* �����Ʒ������ĵ�½���� */

DHT11_Data_TypeDef DHT11_Data;
uint8_t KeyValue[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // ��A��Կ
// uint8_t KeyValue[]={'1' ,'2', '3', '4', '5', '6'};   // ��A��Կ
uint32_t writeValue = 100;
uint32_t readValue;
char cStr[30];
char dispBuff[100];
uint8_t ucArray_ID[4];  // �Ⱥ���IC�������ͺ�UID(IC�����к�)
uint8_t ucStatusReturn; // ����״̬
char mqtt_message[300]; // MQTT���ϱ���Ϣ����
// ������IP��ַ�Ͷ˿ں�
char *IP = MQTT_BROKERADDRESS;
const int Port = 1883;
float humidity, temperature;

/*�궨��*/

/*��������*/
static void AppTaskCreate(void);              // ��������
static void GetData_Task(void *pvParameters); /* ��ȡ�ⲿ�������� */
static void KEY_Task(void *pvParameters);     /* KEY_Task */
static void IC_Task(void *pvParameters);      /* IC_Task */
static void WIFI_Task(void *pvParameters);    /* WIFI���� */
static void LCD_Show_Start(void);             /*Һ����Ļ����*/
void ES8266_MQTT_Init(void);                  /*MQTT��ʼ��*/
static void BSP_Init(void);                   /* ��ʼ�����������Դ  */
// static void DataHandle_Task(void *pvParameters); /* ���ݴ������� */
// static void DHT_Task(void *pvParameters); /* �¿����� */
// static void Uart_Task(void *pvParameters); /*  Uart ����ʵ�� */

int main(void)
{
  BaseType_t xReturn = pdPASS; /* ����һ��������Ϣ����ֵ */

  /* Ӳ����ʼ�� */
  BSP_Init();

  printf("FreeRTOSϵͳ����!!\n\n");
  printf("����KEY1��������,����KEY2�ָ�����\n");

  /* ����APPTaskCreate ���� */
  xReturn = xTaskCreate((TaskFunction_t)AppTaskCreate,          /* ������ں��� */
                        (const char *)"AppTaskCreate",          /* ������ */
                        (uint16_t)512,                          /* ջ��С */
                        (void *)NULL,                           /* ������ں������� */
                        (UBaseType_t)1,                         /* �������ȼ� */
                        (TaskHandle_t *)&AppTaskCreate_Handle); /*  ������ƿ�ָ�� */
  if (pdPASS == xReturn)
    vTaskStartScheduler(); /* ��������  ��ʼ����*/
  else
    return -1;

  while (1)
    ; /* ��������ִ�е����� */
}

static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS; // ����һ��������Ϣ����ֵ
  taskENTER_CRITICAL();        // �����ٽ���  ��ȫ�ֱ������в���ʱ�����ٽ�� �ص����жϿ��صĿ���--ϵͳ���Ⱥ��ⲿ�жϿ��Դ���ٽ��

  /* ���� BinarySem */
  // BinarySem_Handle_W = xSemaphoreCreateBinary();
  // if (NULL != BinarySem_Handle_W)
  //   printf("BinarySem_Handle ��ֵ�ź���-д�����ɹ�!\r\n");

  // BinarySem_Handle_R = xSemaphoreCreateBinary();
  // if (NULL != BinarySem_Handle_R)
  //   printf("BinarySem_Handle ��ֵ�ź���-�������ɹ�!\r\n");

  BinarySem_Handle_IC = xSemaphoreCreateBinary();
  if (NULL != BinarySem_Handle_IC)
    printf("BinarySem_Handle ��ֵ�ź���-IC�����ɹ�!\r\n");

  /* ����LCD���� */
  xReturn = xTaskCreate((TaskFunction_t)GetData_Task,          /* ������ں��� */
                        (const char *)"GetData_Task",          /* ������ */
                        (uint16_t)1024,                        /* ����ջ��С */
                        (void *)NULL,                          /* ������ں������� */
                        (UBaseType_t)3,                        /* �������ȼ�  ��ֵԽ�����ȼ�Խ�� */
                        (TaskHandle_t *)&GetData_Task_Handle); /* ������ƿ�ָ�� */

  if (xReturn == pdPASS)
  {
    printf("����GetData_Task����ɹ�\r\n");
  }

  /* ����IC���� */
  xReturn = xTaskCreate((TaskFunction_t)IC_Task,          /* ������ں��� */
                        (const char *)"IC_Task",          /* ������ */
                        (uint16_t)512,                    /* ����ջ��С */
                        (void *)NULL,                     /* ������ں������� */
                        (UBaseType_t)2,                   /* �������ȼ�  ��ֵԽ�����ȼ�Խ�� */
                        (TaskHandle_t *)&IC_Task_Handle); /* ������ƿ�ָ�� */

  if (xReturn == pdPASS)
  {
    printf("����IC_Task����ɹ�\r\n");
  }

  /*����WIFI����*/
  xReturn = xTaskCreate((TaskFunction_t)WIFI_Task,          /* ������ں��� */
                        (const char *)"WIFI_Task",          /* ������ */
                        (uint16_t)512,                      /* ����ջ��С */
                        (void *)NULL,                       /* ������ں������� */
                        (UBaseType_t)2,                     /* �������ȼ�  ��ֵԽ�����ȼ�Խ�� */
                        (TaskHandle_t *)&WIFI_Task_Handle); /* ������ƿ�ָ�� */

  if (xReturn == pdPASS)
  {
    printf("����WIFI_Task����ɹ�\r\n");
  }
  /*����KEY����*/
  xReturn = xTaskCreate((TaskFunction_t)KEY_Task,          /* ������ں��� */
                        (const char *)"KEY_Task",          /* ������ */
                        (uint16_t)512,                     /* ����ջ��С */
                        (void *)NULL,                      /* ������ں������� */
                        (UBaseType_t)4,                    /* �������ȼ�  ��ֵԽ�����ȼ�Խ�� */
                        (TaskHandle_t *)&KEY_Task_Handle); /* ������ƿ�ָ�� */
  if (xReturn == pdPASS)
  {
    printf("����KEY_Task����ɹ�\r\n");
  }

  vTaskDelete(AppTaskCreate_Handle); // ɾ�����񴴽�����
  taskEXIT_CRITICAL();               // �˳��ٽ���
}

/* ��ȡ�ⲿ�������� */
static void GetData_Task(void *pvParameters)
{
  LCD_Show_Start();
  printf("����LCDģ��ɹ�!!!\n");

  PcdReset();
  M500PcdConfigISOType('A'); // ���ù�����ʽ
  printf("����RC522ģ��ɹ�!!!\n");
  while (1)
  {
    ILI9806G_DispStringLine_EN(LINE(12), "Please put the IC card on WF-RC522 antenna area ...");
    ILI9806G_DispStringLine_EN(LINE(15), "DHT11 Data Show:");
    if (Read_DHT11(&DHT11_Data) == SUCCESS)
    {
      /* ��ʾ�¶� */
      sprintf(dispBuff, "Temperature : %d.%d ", DHT11_Data.temp_int, DHT11_Data.temp_deci);
      ILI9806G_ClearLine(LINE(16));
      ILI9806G_DispStringLine_EN(LINE(16), dispBuff);

      /* ��ʾʪ�� */
      sprintf(dispBuff, "Humidity : %d.%d%% ", DHT11_Data.humi_int, DHT11_Data.humi_deci);
      ILI9806G_ClearLine(LINE(17));
      ILI9806G_DispStringLine_EN(LINE(17), dispBuff);
    }
    else
    {
      LCD_ClearLine(LINE(16));
      LCD_ClearLine(LINE(17));
      ILI9806G_DispStringLine_EN(LINE(16), "Read DHT11 ERROR");
      ILI9806G_DispStringLine_EN(LINE(17), "Read DHT11 ERROR");
    }

    if ((ucStatusReturn = PcdRequest(PICC_REQALL, ucArray_ID)) != MI_OK) // Ѱ��
    {
      // printf("Ѱ��ʧ��\n");                                 // ��ʧ���ٴ�Ѱ��
      ucStatusReturn = PcdRequest(PICC_REQALL, ucArray_ID); // PICC_REQALL   PICC_REQIDL
    }
    if (ucStatusReturn == MI_OK)
    {
      // ����ײ�����ж��ſ������д��������Χʱ������ͻ���ƻ������ѡ��һ�Ž��в�����
      if (PcdAnticoll(ucArray_ID) == MI_OK)
      {
        PcdSelect(ucArray_ID);
        PcdAuthState(PICC_AUTHENT1A, 0x11, KeyValue, ucArray_ID); // У������
        WriteAmount(0x11, writeValue);                            // д����
        printf("Ѱ���ɹ�\n");
        xSemaphoreGive(BinarySem_Handle_IC);
      }
    }
    vTaskDelay(1000);
  }
}

/* ���������� */
static void KEY_Task(void *pvParameters)
{
  while (1)
  {
    if (Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
      vTaskSuspend(IC_Task_Handle);
      printf("��� +100\r\n");
      BEEP_TOGGLE;
      LED1_TOGGLE;
      writeValue += 100;
      vTaskResume(IC_Task_Handle);
    }
    // if (Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    // { /* K2 ���� */
    //   printf("�ָ�����\n");
    //   // vTaskResume(LED_Task_Handle); /* �ָ�LED���� */
    //   BEEP_OFF;
    //   LED1_OFF;
    //   // vTaskResume(BEEP_Task_Handle); /*�ָ�����������*/
    //   printf("�ָ�����ɹ�\n");
    // }
    vTaskDelay(20); /* ��ʱ20��tick */
  }
}

// /*UART������*/
// static void Uart_Task(void *parameter)
// {
//   BaseType_t xReturn = pdPASS; /* ����һ����������ֵ */
//   while (1)
//   {
//     // ��ȡ��ֵ�ź��� û��ȡ��ʱһֱ�ȴ�״̬
//     xReturn = xSemaphoreTake(BinarySem_Handle, /* ��ֵ�ź������ */
//                              portMAX_DELAY);   /* �ȴ�ʱ�� */
//     if (pdPASS == xReturn)
//     {
//       LED2_TOGGLE;
//       printf("�յ�����:%s\n", Usart_Rx_Buf);
//       memset(Usart_Rx_Buf, 0, USART_RBUFF_SIZE); /* ���� */
//     }
//   }
// }

/* RCC������ */
static void IC_Task(void *pvParameters)
{
  while (1)
  {
    xSemaphoreTake(BinarySem_Handle_IC, portMAX_DELAY);

    // ������Ϊ0�򱨾�����Ҫ��ֵ
    if (writeValue <= 0)
    {
      BEEP_ON;
      LED1_ON;
      ILI9806G_ClearLine(LINE(12));
      ILI9806G_DispStringLine_EN(LINE(12), "The residual amount is below 0, please recharge");
    }
    if (ReadAmount(0x11, &readValue) == MI_OK) // ��ȡ���
    {
      writeValue -= 10;
      sprintf(cStr, "The Card ID is: %02X%02X%02X%02X", ucArray_ID[0], ucArray_ID[1], ucArray_ID[2], ucArray_ID[3]);
      printf("%s\r\n", cStr); // ��ӡ��ƬID
      ILI9806G_DispStringLine_EN(LINE(2), cStr);
      printf("���Ϊ��%d\r\n", readValue);
      sprintf(cStr, "The residual amount: %d", readValue);
      ILI9806G_DispStringLine_EN(LINE(4), cStr);
      PcdHalt();
    }

    vTaskDelay(100);
  }
}

// WIFI������
static void WIFI_Task(void *pvParameters)
{
  uint8_t pub_cnt = 0, pub_ret;
  uint16_t Counter_MQTT_Heart = 0;
  // char *recv;

  // MQTTЭ���ʼ��
  ES8266_MQTT_Init();

  while (1)
  {
    // ����������
    if (Counter_MQTT_Heart++ > 300)
    {
      Counter_MQTT_Heart = 0;
      MQTT_SentHeart();
    }

    /* �������� */
    pub_cnt++;
    if (0 == pub_cnt % 500) // Լ3S����һ������
    {
      pub_cnt = 0;
      memset(mqtt_message, 0, 300);
      humidity = (float)DHT11_Data.humi_deci / 10 + DHT11_Data.humi_int;
      temperature = (float)DHT11_Data.temp_deci / 10 + DHT11_Data.temp_int;
      // ��װ����
      sprintf(mqtt_message,
              "{\"method\":\"thing.service.property.post\",\"id\":\"1234\",\"params\":{\
			\"RoomHumidity\":%.1f},\"IndoorTemperature\":%.1f},\"version\":\"1.0.0\"}",
              humidity, temperature);
      // ��������
      pub_ret = MQTT_PublishData(MQTT_PUBLISH_TOPIC, mqtt_message, 0);
      if (pub_ret > 0)
      {
        printf("��Ϣ�����ɹ�!!!data=%.1f,%0.1f\r\n", humidity, temperature);
      }
      else
      {
        printf("��Ϣ����ʧ��!!!pub_ret=%d\r\n", pub_ret);
      }
    }
    // �յ�����
    // if ((WifiMsg.U2_RxCompleted == 1) && (Usart3_RxCounter > 1))
    // {
    //   printf("���Է��������ݣ�%d\r\n", Usart3_RxCounter);
    //   recv = strstr(Usart3_RxBuff, "LED");
    //   // �·�����󣬴���2����յ����������ݣ�
    //   //...{"method":"thing.service.property.set","id":"1593428732","params":{"LED":1},"version":"1.0.0"}
    //   if (recv != NULL)
    //   {
    //     // ����strstr������recvָ�����ַ�����LED":0}...
    //     // Ϊ�õ�LED�����״ֵ̬��ָ��ƫ��5���ֽ�
    //     recv = recv + 3 + 2; // LEDռ3���ֽ�  ��:ռ2���ֽ�
    //     printf("LED=%d\r\n", (*recv) - '0');
    //     LED1 = !((*recv) - '0'); // �����·����������PC13����LED��

    //     memset(mqtt_message, 0, 300);
    //     // ��װ����  id 1454479553
    //     sprintf(mqtt_message,
    //             "{\"method\":\"thing.service.property.set\",\"id\":\"5678\",\"params\":{\
    //             \"LED\":%d},\"version\":\"1.0.0\"}",
    //             (*recv) - '0');

    //     // ��������
    //     pub_ret = MQTT_PublishData(MQTT_PUBLISH_TOPIC, mqtt_message, 0);
    //     if (pub_ret > 0)
    //     {
    //       printf("��Ϣ�����ɹ�!!!pub_ret=%d\r\n", pub_ret);
    //     }
    //     else
    //     {
    //       printf("��Ϣ����ʧ��!!!pub_ret=%d\r\n", pub_ret);
    //     }
    //   }
    //   // ����־λ���������
    //   memset(Usart3_RxBuff, 0, sizeof(Usart3_RxBuff));
    //   WifiMsg.U2_RxCompleted = 0;
    //   Usart3_RxCounter = 0;
    // }
    vTaskDelay(100);
  }
}

/* Һ������ʼ������ */
static void LCD_Show_Start(void)
{
  ILI9806G_GramScan(6);
  LCD_SetFont(&Font16x32);
  LCD_SetColors(RED, BLACK);
  ILI9806G_Clear(0, 0, LCD_X_LENGTH, LCD_Y_LENGTH); // ��������ʾȫ��
}

// MQTT��ʼ������
void ES8266_MQTT_Init(void)
{
  uint8_t status = 1;
  char conn = 1;

  // ��λ���ɹ�����Ҫ���¸�λ
  //    if(!WiFi_Init())
  //    {
  //        printf("ESP8266״̬��ʼ������\r\n");		//���������Ϣ
  //        //��ȡWIFI��ǰIP��ַ
  //        WiFi_GetIP(100);
  //        WifiMsg.Mode = 1;							//r_flag��־��λ����ʾ8266״̬���������Լ���������TCP����
  //        status++;
  //    }

  printf("׼����λģ��\r\n"); // ������ʾ����
  if (WiFi_Reset(50))
  {                                   // ��λ��100ms��ʱ��λ���ܼ�5s��ʱʱ��
    printf("��λʧ�ܣ�׼������\r\n"); // ���ط�0ֵ������if��������ʾ����
  }
  else
    printf("��λ�ɹ�\r\n"); // ������ʾ����

  printf("׼������·����\r\n"); // ������ʾ����

  if (WiFi_JoinAP(10))
  {                                         // ����·����,1s��ʱ��λ���ܼ�10s��ʱʱ��
    printf("����·����ʧ�ܣ�׼������\r\n"); // ���ط�0ֵ������if��������ʾ����
  }
  else
    printf("����·�����ɹ�\r\n"); // ������ʾ����
  printf("׼����ȡIP��ַ\r\n");   // ������ʾ����
  if (WiFi_GetIP(50))
  {                                         // ׼����ȡIP��ַ��100ms��ʱ��λ���ܼ�5s��ʱʱ��
    printf("��ȡIP��ַʧ��, ׼������\r\n"); // ���ط�0ֵ������if��������ʾ����
  }
  else
    printf("��ȡIP��ַ�ɹ�\r\n"); // ������ʾ����

  printf("׼������͸��\r\n"); // ������ʾ����
  if (WiFi_SendCmd("AT+CIPMODE=1", 50))
  {                                       // ����͸����100ms��ʱ��λ���ܼ�5s��ʱʱ��
    printf("����͸��ʧ��, ׼������\r\n"); // ���ط�0ֵ������if��������ʾ����
  }
  else
    printf("����͸���ɹ�\r\n"); // ������ʾ����

  printf("׼���رն�·����\r\n"); // ������ʾ����
  if (WiFi_SendCmd("AT+CIPMUX=0", 50))
  {                                           // �رն�·���ӣ�100ms��ʱ��λ���ܼ�5s��ʱʱ��
    printf("�رն�·����ʧ��, ׼������\r\n"); // ���ط�0ֵ������if��������ʾ����
  }
  else
    printf("�رն�·���ӳɹ�\r\n"); // ������ʾ����
  WifiMsg.Mode = 1;                 // r_flag��־��λ����ʾ8266״̬���������Լ���������TCP����
  status++;

  // ���Ӱ�����IOT������
  if (status == 2)
  {
    printf("���ӷ�����:IP=%s,Port=%d\r\n", IP, Port);
    conn = WiFi_Connect(IP, Port, 100);
    printf("���ӽ��conn=%d\r\n", conn);
    status++;
  }
  // �ر�WIFI����
  // printf("�رջ��ԣ�%d\r\n", WiFi_Send("ATE0"));

  // ��½MQTT
  if (status == 3)
  {
    if (MQTT_Connect(MQTT_CLIENTID, MQTT_USARNAME, MQTT_PASSWD) != 0)
    {
      printf("ESP8266������MQTT��½�ɹ�!\r\n");
      status++;
    }
    else
    {
      printf("ESP8266������MQTT��½ʧ��!\r\n");
      status++;
    }
  }

  // ��������
  if (status == 4)
  {
    if (MQTT_SubscribeTopic(MQTT_SUBSCRIBE_TOPIC, 0, 1) != 0)
    {
      printf("ESP8266������MQTT��������ɹ�!\r\n");
    }
    else
    {
      printf("ESP8266������MQTT��������ʧ��!\r\n");
    }
  }
}

static void BSP_Init(void)
{
  /*
   FreeRTOSʹ�����ȼ�����4
   */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  /* ���ڳ�ʼ��	*/
  Debug_USART_Config();

  /*��������ʼ��*/
  BEEP_GPIO_Config();

  /* ������ʼ��	*/
  Key_GPIO_Config();

  /*��ʼ�����*/
  LED_GPIO_Config();

  /* ��ʼ��Һ���� */
  ILI9806G_Init();

  /* ��ʼ��RC 522 */
  RC522_Init();

  /* ��ʼ�������� */
  BreathLED_Config();

  /* ��ʼ��DHT11 */
  DHT11_GPIO_Config();

  /*��ʼ������3*/
  uart3_init();

  /*Tim3��ʱ��������wifi-uart2�Ľ������*/
  Timer3_Configuration(5);

  /*wifi - RST���ų�ʼ��*/
  WiFi_ResetIO_Init();
}
/*********************************************END OF FILE**********************/
