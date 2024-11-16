#include "ne30_task.h"
#include "innfos_ne30.h"
#include "global.h"
#include "system.h"
#include "my_fdcan.h"


/**
 * @defgroup start task相关参数
 */
#define SCA_TASK_PRIO 3  //任务优先级
#define SCA_TASK_SIZE 256 //任务堆栈大小
TaskHandle_t SCA_TASK_Handler;//任务句柄
void sca_task(void *pvparameters);

TimerHandle_t SCA_TIMER;
void sca_tim_callback(TimerHandle_t xTimer);

void sca_task_init(void)
{
    can_init(&fdcan1, &hfdcan1);
    ScaInit(&scaMotor[0], 0x01, SCA_Profile_Velocity_Mode, &fdcan1);
    xTaskCreate((TaskFunction_t )sca_task,    //任务函数
                (const char *   )"sca",   //任务名称
                (uint16_t       )SCA_TASK_SIZE, //任务堆栈大小
                (void *         )NULL,                //任务参数
                (UBaseType_t    )SCA_TASK_PRIO, //任务优先级
                (TaskHandle_t * )&SCA_TASK_Handler); //任务句句柄

    SCA_TIMER = xTimerCreate((const char* const)"heartbeat",
                             (TickType_t       )1000,
                             (const UBaseType_t)pdTRUE,
                             (void * const     )NULL,
                             (TimerCallbackFunction_t)sca_tim_callback);
    xTimerStart(SCA_TIMER, 0);
}

static void sca_task(void * pvparameters)
{
    log_i("SCA Task Work! ");
    uint8_t last_mod = SCA_Profile_Velocity_Mode;
    
    for(;;)//上电检测
    {
        static uint8_t err_t = 0;
        err_t++;
        vTaskDelay(1000);
        SCA_Enable(&scaMotor[0]);
        SCA_GetState(&scaMotor[0]);
        if (err_t == 3)//3s后报警一次
        {
            log_e("sca%d is not online", scaMotor[0].id);
        }
        if (scaMotor[0].Power_State == 0)
            continue;
        else
        {
            log_i("sca%d power on and work in %s", scaMotor[0].id, find_name_of_mode(scaMotor[0].Mode));
            SCA_SetMode(&scaMotor[0], SCA_Profile_Velocity_Mode);
            SCA_GetAllInfomation(&scaMotor[0]);
            break;
        }
    }    
    for(;;)
    {       
        if (scaMotor[0].Mode_Target!= scaMotor[1].Mode)
        {
            SCA_SetMode(&scaMotor[0], scaMotor[0].Mode_Target);
        }

        switch (scaMotor[0].Mode)
        {
        case SCA_Current_Mode:
            SCA_SetCurrent(&scaMotor[0]);
            break;
        case SCA_Profile_Position_Mode:
            SCA_SetPosition(&scaMotor[0]);
            break;
        case SCA_Profile_Velocity_Mode:
            SCA_SetVelocity(&scaMotor[0]);
            break;
        default:
            break;
        }
        SCA_GetCVP(&scaMotor[0]);
        vTaskDelay(10);
    }
}

/**
 * @brief 1s callback
 * @param xTimer 
 */
void sca_tim_callback(TimerHandle_t xTimer)
{
    static uint8_t err_t = 0;
    if (scaMotor[0].Online_State == 0)
        err_t++;
    else
    {
        err_t = 0; 
        scaMotor[0].Online_State = 0;
        SCA_HeartBeat(&scaMotor[0]);
    }

    if (err_t >= 3)
    {
        log_e("sca%d is not online", &scaMotor[0].id);
        xTimerStop(SCA_TIMER, 0);
    }   
}
