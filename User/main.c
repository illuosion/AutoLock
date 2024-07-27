/**
 ******************************************************************************
 * @file    main.c
 * @author  fire
 * @version V1.0
 * @date    2015-xx-xx
 * @brief   RFID-RC522模块实验
 ******************************************************************************
 * @attention
 *
 * 实验平台:野火  STM32 F407 开发板
 * 论坛    :http://www.firebbs.cn
 * 淘宝    :https://fire-stm32.taobao.com
 *
 ******************************************************************************
 */

// FreeRTOS头文件
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include <stdbool.h>

/* 外设头文件 */
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

/* 任务句柄 */
static TaskHandle_t AppTaskCreate_Handle = NULL; /* 创建任务句柄 */
static TaskHandle_t GetData_Task_Handle = NULL;  /* LED任务句柄 */
static TaskHandle_t KEY_Task_Handle = NULL;      /* KEY任务句柄 */
static TaskHandle_t WIFI_Task_Handle = NULL;     /* WIFI任务句柄 */
static TaskHandle_t IC_Task_Handle = NULL;       /* IC任务句柄 */
// static TaskHandle_t DHT_Task_Handle = NULL;      /* 温控任务句柄 */
// static TaskHandle_t ESP_Task_Handle = NULL;      /* WIFI任务句柄 */
// static TaskHandle_t Uart_Task_Handle = NULL;     /* USART任务句柄 */

/*内核对象句柄*/
SemaphoreHandle_t BinarySem_Handle_W = NULL;  /*写的二值信号量*/
SemaphoreHandle_t BinarySem_Handle_R = NULL;  /*读的二值信号量*/
SemaphoreHandle_t BinarySem_Handle_IC = NULL; /*IC的二值信号量*/
SemaphoreHandle_t MutexSem_Handle = NULL;     /*互斥锁*/

/*全局变量声明*/
extern char Usart_Rx_Buf[USART_RBUFF_SIZE];
#define MQTT_BROKERADDRESS "iot-06z00bcpj7fur8q.mqtt.iothub.aliyuncs.com"
#define MQTT_CLIENTID "k15fb6M8Sxm.NKVmLwH7SG62GPgNXE6F|securemode=2,signmethod=hmacsha256,timestamp=1713757769095|"
#define MQTT_USARNAME "NKVmLwH7SG62GPgNXE6F&k15fb6M8Sxm"
#define MQTT_PASSWD "e49268d49fb1ff5c7814f8d9b9aa438eeaa80f9995a6fdc7331cecb994a8a564"
#define MQTT_PUBLISH_TOPIC "/sys/k15fb6M8Sxm/test01/thing/event/property/post"
#define MQTT_SUBSCRIBE_TOPIC "/sys/k15fb6M8Sxm/test01/thing/service/property/set"

/* 阿里云服务器的登陆设置 */

DHT11_Data_TypeDef DHT11_Data;
uint8_t KeyValue[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 卡A密钥
// uint8_t KeyValue[]={'1' ,'2', '3', '4', '5', '6'};   // 卡A密钥
uint32_t writeValue = 100;
uint32_t readValue;
char cStr[30];
char dispBuff[100];
uint8_t ucArray_ID[4];  // 先后存放IC卡的类型和UID(IC卡序列号)
uint8_t ucStatusReturn; // 返回状态
char mqtt_message[300]; // MQTT的上报消息缓存
// 服务器IP地址和端口号
char *IP = MQTT_BROKERADDRESS;
const int Port = 1883;
float humidity, temperature;

/*宏定义*/

/*函数声明*/
static void AppTaskCreate(void);              // 创建任务
static void GetData_Task(void *pvParameters); /* 获取外部数据任务 */
static void KEY_Task(void *pvParameters);     /* KEY_Task */
static void IC_Task(void *pvParameters);      /* IC_Task */
static void WIFI_Task(void *pvParameters);    /* WIFI任务 */
static void LCD_Show_Start(void);             /*液晶屏幕启动*/
void ES8266_MQTT_Init(void);                  /*MQTT初始化*/
static void BSP_Init(void);                   /* 初始化板载相关资源  */
// static void DataHandle_Task(void *pvParameters); /* 数据处理任务 */
// static void DHT_Task(void *pvParameters); /* 温控任务 */
// static void Uart_Task(void *pvParameters); /*  Uart 任务实现 */

int main(void)
{
  BaseType_t xReturn = pdPASS; /* 定义一个创建信息返回值 */

  /* 硬件初始化 */
  BSP_Init();

  printf("FreeRTOS系统启动!!\n\n");
  printf("按下KEY1挂起任务,按下KEY2恢复任务\n");

  /* 创建APPTaskCreate 任务 */
  xReturn = xTaskCreate((TaskFunction_t)AppTaskCreate,          /* 任务入口函数 */
                        (const char *)"AppTaskCreate",          /* 任务名 */
                        (uint16_t)512,                          /* 栈大小 */
                        (void *)NULL,                           /* 任务入口函数参数 */
                        (UBaseType_t)1,                         /* 任务优先级 */
                        (TaskHandle_t *)&AppTaskCreate_Handle); /*  任务控制块指针 */
  if (pdPASS == xReturn)
    vTaskStartScheduler(); /* 启动任务  开始调度*/
  else
    return -1;

  while (1)
    ; /* 正常不会执行到这里 */
}

static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS; // 定义一个创建信息返回值
  taskENTER_CRITICAL();        // 进入临界区  对全局变量进行操作时进入临界段 回到对中断开关的控制--系统调度和外部中断可以打断临界段

  /* 创建 BinarySem */
  // BinarySem_Handle_W = xSemaphoreCreateBinary();
  // if (NULL != BinarySem_Handle_W)
  //   printf("BinarySem_Handle 二值信号量-写创建成功!\r\n");

  // BinarySem_Handle_R = xSemaphoreCreateBinary();
  // if (NULL != BinarySem_Handle_R)
  //   printf("BinarySem_Handle 二值信号量-读创建成功!\r\n");

  BinarySem_Handle_IC = xSemaphoreCreateBinary();
  if (NULL != BinarySem_Handle_IC)
    printf("BinarySem_Handle 二值信号量-IC创建成功!\r\n");

  /* 创建LCD任务 */
  xReturn = xTaskCreate((TaskFunction_t)GetData_Task,          /* 任务入口函数 */
                        (const char *)"GetData_Task",          /* 任务名 */
                        (uint16_t)1024,                        /* 任务栈大小 */
                        (void *)NULL,                          /* 任务入口函数参数 */
                        (UBaseType_t)3,                        /* 任务优先级  数值越大优先级越高 */
                        (TaskHandle_t *)&GetData_Task_Handle); /* 任务控制块指针 */

  if (xReturn == pdPASS)
  {
    printf("创建GetData_Task任务成功\r\n");
  }

  /* 创建IC任务 */
  xReturn = xTaskCreate((TaskFunction_t)IC_Task,          /* 任务入口函数 */
                        (const char *)"IC_Task",          /* 任务名 */
                        (uint16_t)512,                    /* 任务栈大小 */
                        (void *)NULL,                     /* 任务入口函数参数 */
                        (UBaseType_t)2,                   /* 任务优先级  数值越大优先级越高 */
                        (TaskHandle_t *)&IC_Task_Handle); /* 任务控制块指针 */

  if (xReturn == pdPASS)
  {
    printf("创建IC_Task任务成功\r\n");
  }

  /*创建WIFI任务*/
  xReturn = xTaskCreate((TaskFunction_t)WIFI_Task,          /* 任务入口函数 */
                        (const char *)"WIFI_Task",          /* 任务名 */
                        (uint16_t)512,                      /* 任务栈大小 */
                        (void *)NULL,                       /* 任务入口函数参数 */
                        (UBaseType_t)2,                     /* 任务优先级  数值越大优先级越高 */
                        (TaskHandle_t *)&WIFI_Task_Handle); /* 任务控制块指针 */

  if (xReturn == pdPASS)
  {
    printf("创建WIFI_Task任务成功\r\n");
  }
  /*创建KEY任务*/
  xReturn = xTaskCreate((TaskFunction_t)KEY_Task,          /* 任务入口函数 */
                        (const char *)"KEY_Task",          /* 任务名 */
                        (uint16_t)512,                     /* 任务栈大小 */
                        (void *)NULL,                      /* 任务入口函数参数 */
                        (UBaseType_t)4,                    /* 任务优先级  数值越大优先级越高 */
                        (TaskHandle_t *)&KEY_Task_Handle); /* 任务控制块指针 */
  if (xReturn == pdPASS)
  {
    printf("创建KEY_Task任务成功\r\n");
  }

  vTaskDelete(AppTaskCreate_Handle); // 删除任务创建任务
  taskEXIT_CRITICAL();               // 退出临界区
}

/* 获取外部数据任务 */
static void GetData_Task(void *pvParameters)
{
  LCD_Show_Start();
  printf("启动LCD模块成功!!!\n");

  PcdReset();
  M500PcdConfigISOType('A'); // 设置工作方式
  printf("启动RC522模块成功!!!\n");
  while (1)
  {
    ILI9806G_DispStringLine_EN(LINE(12), "Please put the IC card on WF-RC522 antenna area ...");
    ILI9806G_DispStringLine_EN(LINE(15), "DHT11 Data Show:");
    if (Read_DHT11(&DHT11_Data) == SUCCESS)
    {
      /* 显示温度 */
      sprintf(dispBuff, "Temperature : %d.%d ", DHT11_Data.temp_int, DHT11_Data.temp_deci);
      ILI9806G_ClearLine(LINE(16));
      ILI9806G_DispStringLine_EN(LINE(16), dispBuff);

      /* 显示湿度 */
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

    if ((ucStatusReturn = PcdRequest(PICC_REQALL, ucArray_ID)) != MI_OK) // 寻卡
    {
      // printf("寻卡失败\n");                                 // 若失败再次寻卡
      ucStatusReturn = PcdRequest(PICC_REQALL, ucArray_ID); // PICC_REQALL   PICC_REQIDL
    }
    if (ucStatusReturn == MI_OK)
    {
      // 防冲撞（当有多张卡进入读写器操作范围时，防冲突机制会从其中选择一张进行操作）
      if (PcdAnticoll(ucArray_ID) == MI_OK)
      {
        PcdSelect(ucArray_ID);
        PcdAuthState(PICC_AUTHENT1A, 0x11, KeyValue, ucArray_ID); // 校验密码
        WriteAmount(0x11, writeValue);                            // 写入金额
        printf("寻卡成功\n");
        xSemaphoreGive(BinarySem_Handle_IC);
      }
    }
    vTaskDelay(1000);
  }
}

/* 按键任务函数 */
static void KEY_Task(void *pvParameters)
{
  while (1)
  {
    if (Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
      vTaskSuspend(IC_Task_Handle);
      printf("金额 +100\r\n");
      BEEP_TOGGLE;
      LED1_TOGGLE;
      writeValue += 100;
      vTaskResume(IC_Task_Handle);
    }
    // if (Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    // { /* K2 按下 */
    //   printf("恢复任务\n");
    //   // vTaskResume(LED_Task_Handle); /* 恢复LED任务 */
    //   BEEP_OFF;
    //   LED1_OFF;
    //   // vTaskResume(BEEP_Task_Handle); /*恢复蜂鸣器任务*/
    //   printf("恢复任务成功\n");
    // }
    vTaskDelay(20); /* 延时20个tick */
  }
}

// /*UART任务函数*/
// static void Uart_Task(void *parameter)
// {
//   BaseType_t xReturn = pdPASS; /* 创建一个创建返回值 */
//   while (1)
//   {
//     // 获取二值信号量 没获取到时一直等待状态
//     xReturn = xSemaphoreTake(BinarySem_Handle, /* 二值信号量句柄 */
//                              portMAX_DELAY);   /* 等待时间 */
//     if (pdPASS == xReturn)
//     {
//       LED2_TOGGLE;
//       printf("收到数据:%s\n", Usart_Rx_Buf);
//       memset(Usart_Rx_Buf, 0, USART_RBUFF_SIZE); /* 清零 */
//     }
//   }
// }

/* RCC任务函数 */
static void IC_Task(void *pvParameters)
{
  while (1)
  {
    xSemaphoreTake(BinarySem_Handle_IC, portMAX_DELAY);

    // 如果余额为0则报警，须要充值
    if (writeValue <= 0)
    {
      BEEP_ON;
      LED1_ON;
      ILI9806G_ClearLine(LINE(12));
      ILI9806G_DispStringLine_EN(LINE(12), "The residual amount is below 0, please recharge");
    }
    if (ReadAmount(0x11, &readValue) == MI_OK) // 读取金额
    {
      writeValue -= 10;
      sprintf(cStr, "The Card ID is: %02X%02X%02X%02X", ucArray_ID[0], ucArray_ID[1], ucArray_ID[2], ucArray_ID[3]);
      printf("%s\r\n", cStr); // 打印卡片ID
      ILI9806G_DispStringLine_EN(LINE(2), cStr);
      printf("余额为：%d\r\n", readValue);
      sprintf(cStr, "The residual amount: %d", readValue);
      ILI9806G_DispStringLine_EN(LINE(4), cStr);
      PcdHalt();
    }

    vTaskDelay(100);
  }
}

// WIFI任务函数
static void WIFI_Task(void *pvParameters)
{
  uint8_t pub_cnt = 0, pub_ret;
  uint16_t Counter_MQTT_Heart = 0;
  // char *recv;

  // MQTT协议初始化
  ES8266_MQTT_Init();

  while (1)
  {
    // 心跳包发送
    if (Counter_MQTT_Heart++ > 300)
    {
      Counter_MQTT_Heart = 0;
      MQTT_SentHeart();
    }

    /* 发送数据 */
    pub_cnt++;
    if (0 == pub_cnt % 500) // 约3S发送一次数据
    {
      pub_cnt = 0;
      memset(mqtt_message, 0, 300);
      humidity = (float)DHT11_Data.humi_deci / 10 + DHT11_Data.humi_int;
      temperature = (float)DHT11_Data.temp_deci / 10 + DHT11_Data.temp_int;
      // 组装数据
      sprintf(mqtt_message,
              "{\"method\":\"thing.service.property.post\",\"id\":\"1234\",\"params\":{\
			\"RoomHumidity\":%.1f},\"IndoorTemperature\":%.1f},\"version\":\"1.0.0\"}",
              humidity, temperature);
      // 发布数据
      pub_ret = MQTT_PublishData(MQTT_PUBLISH_TOPIC, mqtt_message, 0);
      if (pub_ret > 0)
      {
        printf("消息发布成功!!!data=%.1f,%0.1f\r\n", humidity, temperature);
      }
      else
      {
        printf("消息发布失败!!!pub_ret=%d\r\n", pub_ret);
      }
    }
    // 收到数据
    // if ((WifiMsg.U2_RxCompleted == 1) && (Usart3_RxCounter > 1))
    // {
    //   printf("来自服务器数据：%d\r\n", Usart3_RxCounter);
    //   recv = strstr(Usart3_RxBuff, "LED");
    //   // 下发命令后，串口2会接收到这样的数据：
    //   //...{"method":"thing.service.property.set","id":"1593428732","params":{"LED":1},"version":"1.0.0"}
    //   if (recv != NULL)
    //   {
    //     // 经过strstr函数后，recv指向了字符串：LED":0}...
    //     // 为拿到LED后面的状态值，指针偏移5个字节
    //     recv = recv + 3 + 2; // LED占3个字节  ”:占2个字节
    //     printf("LED=%d\r\n", (*recv) - '0');
    //     LED1 = !((*recv) - '0'); // 根据下发的命令控制PC13处的LED灯

    //     memset(mqtt_message, 0, 300);
    //     // 组装数据  id 1454479553
    //     sprintf(mqtt_message,
    //             "{\"method\":\"thing.service.property.set\",\"id\":\"5678\",\"params\":{\
    //             \"LED\":%d},\"version\":\"1.0.0\"}",
    //             (*recv) - '0');

    //     // 发布数据
    //     pub_ret = MQTT_PublishData(MQTT_PUBLISH_TOPIC, mqtt_message, 0);
    //     if (pub_ret > 0)
    //     {
    //       printf("消息发布成功!!!pub_ret=%d\r\n", pub_ret);
    //     }
    //     else
    //     {
    //       printf("消息发布失败!!!pub_ret=%d\r\n", pub_ret);
    //     }
    //   }
    //   // 将标志位和数据清空
    //   memset(Usart3_RxBuff, 0, sizeof(Usart3_RxBuff));
    //   WifiMsg.U2_RxCompleted = 0;
    //   Usart3_RxCounter = 0;
    // }
    vTaskDelay(100);
  }
}

/* 液晶屏初始化函数 */
static void LCD_Show_Start(void)
{
  ILI9806G_GramScan(6);
  LCD_SetFont(&Font16x32);
  LCD_SetColors(RED, BLACK);
  ILI9806G_Clear(0, 0, LCD_X_LENGTH, LCD_Y_LENGTH); // 清屏，显示全黑
}

// MQTT初始化函数
void ES8266_MQTT_Init(void)
{
  uint8_t status = 1;
  char conn = 1;

  // 复位不成功，需要重新复位
  //    if(!WiFi_Init())
  //    {
  //        printf("ESP8266状态初始化正常\r\n");		//串口输出信息
  //        //获取WIFI当前IP地址
  //        WiFi_GetIP(100);
  //        WifiMsg.Mode = 1;							//r_flag标志置位，表示8266状态正常，可以继续，进行TCP连接
  //        status++;
  //    }

  printf("准备复位模块\r\n"); // 串口提示数据
  if (WiFi_Reset(50))
  {                                   // 复位，100ms超时单位，总计5s超时时间
    printf("复位失败，准备重启\r\n"); // 返回非0值，进入if，串口提示数据
  }
  else
    printf("复位成功\r\n"); // 串口提示数据

  printf("准备连接路由器\r\n"); // 串口提示数据

  if (WiFi_JoinAP(10))
  {                                         // 连接路由器,1s超时单位，总计10s超时时间
    printf("连接路由器失败，准备重启\r\n"); // 返回非0值，进入if，串口提示数据
  }
  else
    printf("连接路由器成功\r\n"); // 串口提示数据
  printf("准备获取IP地址\r\n");   // 串口提示数据
  if (WiFi_GetIP(50))
  {                                         // 准备获取IP地址，100ms超时单位，总计5s超时时间
    printf("获取IP地址失败, 准备重启\r\n"); // 返回非0值，进入if，串口提示数据
  }
  else
    printf("获取IP地址成功\r\n"); // 串口提示数据

  printf("准备开启透传\r\n"); // 串口提示数据
  if (WiFi_SendCmd("AT+CIPMODE=1", 50))
  {                                       // 开启透传，100ms超时单位，总计5s超时时间
    printf("开启透传失败, 准备重启\r\n"); // 返回非0值，进入if，串口提示数据
  }
  else
    printf("开启透传成功\r\n"); // 串口提示数据

  printf("准备关闭多路连接\r\n"); // 串口提示数据
  if (WiFi_SendCmd("AT+CIPMUX=0", 50))
  {                                           // 关闭多路连接，100ms超时单位，总计5s超时时间
    printf("关闭多路连接失败, 准备重启\r\n"); // 返回非0值，进入if，串口提示数据
  }
  else
    printf("关闭多路连接成功\r\n"); // 串口提示数据
  WifiMsg.Mode = 1;                 // r_flag标志置位，表示8266状态正常，可以继续，进行TCP连接
  status++;

  // 连接阿里云IOT服务器
  if (status == 2)
  {
    printf("连接服务器:IP=%s,Port=%d\r\n", IP, Port);
    conn = WiFi_Connect(IP, Port, 100);
    printf("连接结果conn=%d\r\n", conn);
    status++;
  }
  // 关闭WIFI回显
  // printf("关闭回显：%d\r\n", WiFi_Send("ATE0"));

  // 登陆MQTT
  if (status == 3)
  {
    if (MQTT_Connect(MQTT_CLIENTID, MQTT_USARNAME, MQTT_PASSWD) != 0)
    {
      printf("ESP8266阿里云MQTT登陆成功!\r\n");
      status++;
    }
    else
    {
      printf("ESP8266阿里云MQTT登陆失败!\r\n");
      status++;
    }
  }

  // 订阅主题
  if (status == 4)
  {
    if (MQTT_SubscribeTopic(MQTT_SUBSCRIBE_TOPIC, 0, 1) != 0)
    {
      printf("ESP8266阿里云MQTT订阅主题成功!\r\n");
    }
    else
    {
      printf("ESP8266阿里云MQTT订阅主题失败!\r\n");
    }
  }
}

static void BSP_Init(void)
{
  /*
   FreeRTOS使用优先级分组4
   */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  /* 串口初始化	*/
  Debug_USART_Config();

  /*蜂鸣器初始化*/
  BEEP_GPIO_Config();

  /* 按键初始化	*/
  Key_GPIO_Config();

  /*初始化红灯*/
  LED_GPIO_Config();

  /* 初始化液晶屏 */
  ILI9806G_Init();

  /* 初始化RC 522 */
  RC522_Init();

  /* 初始化呼吸灯 */
  BreathLED_Config();

  /* 初始化DHT11 */
  DHT11_GPIO_Config();

  /*初始化串口3*/
  uart3_init();

  /*Tim3定时器，用于wifi-uart2的接收完成*/
  Timer3_Configuration(5);

  /*wifi - RST引脚初始化*/
  WiFi_ResetIO_Init();
}
/*********************************************END OF FILE**********************/
