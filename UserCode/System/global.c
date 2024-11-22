/*
 * @Author: GYM-png 480609450@qq.com
 * @Date: 2024-10-13 00:22:43
 * @LastEditors: GYM-png 480609450@qq.com
 * @LastEditTime: 2024-10-25 21:53:11
 * @FilePath: \MDK-ARMd:\warehouse\CmdDebug\CmdDebug\UserCode\global\global.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "global.h"
#include "system.h"

Rtc_Time_t rtc_time =  {0, 0, 0};
Rtc_Date_t rtc_date =  {0, 0, 0};
uint32_t systern_run_time = 0;
uint16_t rtc_ms = 0;//系统时间ms rtc只能提供精确到s

Sca_t motor[MOTOR_NUM];
Can_t fdcan1;

Sca_Targte_t sca_target = {
    .velocity = 0,
    .position = 0,
    .current = 0
};
extern void sca_task_init(void);



/**
 * @defgroup start task相关参数
 */
#define START_TASK_PRIO 3  //任务优先级
#define START_TASK_SIZE 200 //任务堆栈大小
TaskHandle_t START_TASK_Handler;//任务句柄
void start_task(void *pvparameters);





void start_task_init(void)
{
    xTaskCreate((TaskFunction_t )start_task,    //任务函数
                (const char *   )"start",   //任务名称
                (uint16_t       )START_TASK_SIZE, //任务堆栈大小
                (void *         )NULL,                //任务参数
                (UBaseType_t    )START_TASK_PRIO, //任务优先级
                (TaskHandle_t * )&START_TASK_Handler); //任务句句柄
}


static void start_task(void * pvparameters)
{
    /*获取系统初始时间*/
    log_i("系统开机成功 ");
    sca_task_init();
    for(;;)
    {
        if(rtc_ms >= 1000)
        {
            rtc_ms = 0;
            rtc_time.Seconds++;
        }
        if(rtc_time.Seconds >= 60)
        {
            rtc_time.Seconds = 0;
            rtc_time.Minutes++;
        }
        if(rtc_time.Minutes >= 60)
        {
            rtc_time.Minutes = 0;
            rtc_time.Hours++;
        }

        // log_i("你好\r\n");
        // log_w("你好\r\n");
        // log_e("你好\r\n");
        vTaskDelay(1002);
    }
}

